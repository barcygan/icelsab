// Pomiar z samego czujnika IMU na bucie: skoki i piruety (pomiar-0.1).
// Czysta logika bez DOM — działa w przeglądarce (window.PomiarCore) i w Node (testy).
//
// Model: przy staniu nieruchomo akcelerometr pokazuje +1 g "do góry" → oś pionowa buta u.
//  - prędkość obrotu wokół pionu  wv = (ω − bias) · u          [°/s]
//  - przyspieszenie wzdłuż pionu   av = a · u / g0              [g]
//    Na lodzie av ≈ 1 g. W locie av ≈ 0, bo siła odśrodkowa od obrotu działa POZIOMO
//    (prostopadle do osi obrotu), więc nie psuje wykrywania lotu jak samo |a|.
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.PomiarCore = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';
  const G = 9.80665, R2D = 180 / Math.PI;

  const DEFAULTS = {
    flightAv: 0.35,        // g wzdłuż pionu poniżej → w powietrzu
    flightGapMs: 30,       // krótkie przerwy w locie (szum) łączymy
    flightMinMs: 150,      // krótszy "lot" to szum / podskok
    flightMaxMs: 1000,
    landingMinG: 1.8,      // lądowanie zawsze daje uderzenie
    rotTailDps: 90,        // obrót na lodzie przed wybiciem / po lądowaniu liczymy powyżej tego
    spinOnRps: 1.0,        // piruet: start powyżej 1 obr/s
    spinOffRps: 0.5,       //         koniec poniżej 0,5 obr/s
    spinMinS: 1.5,
    clipDps: 1950,         // czujnik ±2000 °/s
    maxGapMs: 120          // większa dziura w danych (utracone pakiety) przerywa całkowanie
  };
  // Profile czułości skoków. „Dzieci”: niższe, krótsze skoki i miękkie lądowania.
  const PROFILES = {
    kids:     { flightAv: 0.5, flightMinMs: 100, landingMinG: 1.35 },
    standard: { flightAv: 0.35, flightMinMs: 150, landingMinG: 1.8 }
  };

  function parsePacket(str) {
    const p = String(str).split(',');
    if (p.length < 7) return null;
    const t = +p[0];
    if (!isFinite(t)) return null;
    return {
      t,
      a: [p[1] / G, p[2] / G, p[3] / G],                 // g
      w: [p[4] * R2D, p[5] * R2D, p[6] * R2D],           // °/s
      seq: p.length >= 8 ? +p[7] : null,
      rssi: p.length >= 9 ? +p[8] : null
    };
  }

  const dot = (x, y) => x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
  const norm = x => Math.hypot(x[0], x[1], x[2]);

  // Szuka okna ≥ winMs bezruchu (żyro < 25 °/s, stabilne |a|) od początku nagrania.
  // Zwraca kalibrację albo null, gdy jeszcze nie ma takiego okna.
  function findCalibration(samples, winMs = 800) {
    let i0 = 0;
    for (let i = 0; i < samples.length; i++) {
      const s = samples[i];
      const still = norm(s.w) < 25 && Math.abs(norm(s.a) - 1) < 0.08;
      if (!still) { i0 = i + 1; continue; }
      if (s.t - samples[i0].t >= winMs) return calibrateRange(samples, i0, i);
    }
    return null;
  }

  function calibrateRange(samples, i0, i1) {
    const a = [0, 0, 0], w = [0, 0, 0];
    let n = 0;
    for (let i = i0; i <= i1; i++) {
      for (let k = 0; k < 3; k++) { a[k] += samples[i].a[k]; w[k] += samples[i].w[k]; }
      n++;
    }
    for (let k = 0; k < 3; k++) { a[k] /= n; w[k] /= n; }
    const g0 = norm(a);
    return { u: a.map(v => v / g0), bias: w, g0, t: samples[i1].t, still: true };
  }

  // Gdy na starcie nie było bezruchu: pion ze średniej przyspieszenia w chwilach bez obrotu,
  // bez korekty dryfu żyroskopu (średnia z piruetów zafałszowałaby ją).
  function approxCalibration(samples) {
    const a = [0, 0, 0];
    let n = 0;
    for (const s of samples) {
      if (norm(s.w) > 60) continue;
      for (let k = 0; k < 3; k++) a[k] += s.a[k];
      n++;
    }
    if (n < 50) return null;
    const g0 = norm(a) / n;
    if (!(g0 > 0.5)) return null;
    return { u: a.map(v => v / n / g0), bias: [0, 0, 0], g0, t: samples[samples.length - 1].t, still: false };
  }

  // Przebieg pionowy dla całego nagrania.
  function derive(samples, cal) {
    const n = samples.length;
    const t = new Float64Array(n), av = new Float64Array(n), wv = new Float64Array(n),
          amag = new Float64Array(n), wpeak = new Float64Array(n), wh = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      const s = samples[i];
      t[i] = s.t;
      av[i] = dot(s.a, cal.u) / cal.g0;
      wv[i] = (s.w[0] - cal.bias[0]) * cal.u[0] + (s.w[1] - cal.bias[1]) * cal.u[1] + (s.w[2] - cal.bias[2]) * cal.u[2];
      amag[i] = norm(s.a) / cal.g0;
      wpeak[i] = Math.max(Math.abs(s.w[0]), Math.abs(s.w[1]), Math.abs(s.w[2]));
      const w0 = s.w[0] - cal.bias[0], w1 = s.w[1] - cal.bias[1], w2 = s.w[2] - cal.bias[2];
      wh[i] = Math.sqrt(Math.max(0, w0 * w0 + w1 * w1 + w2 * w2 - wv[i] * wv[i]));   // obrót wokół osi poziomej
    }
    return { t, av, wv, amag, wpeak, wh, n };
  }

  // ∫ wv dt w przedziale indeksów [i0, i1] (trapezy, dziury w danych pomijane) → stopnie
  function integrate(d, i0, i1, maxGapMs, abs) {
    let sum = 0;
    for (let i = Math.max(1, i0 + 1); i <= i1; i++) {
      const dt = d.t[i] - d.t[i - 1];
      if (dt <= 0 || dt > maxGapMs) continue;
      const v = (d.wv[i] + d.wv[i - 1]) / 2;
      sum += (abs ? Math.abs(v) : v) * dt / 1000;
    }
    return sum;
  }

  function maxIn(arr, d, tA, tB) {
    let lo = 0, hi = d.n;                       // pierwsza próbka z t >= tA (wyszukiwanie binarne)
    while (lo < hi) { const mid = (lo + hi) >> 1; if (d.t[mid] < tA) lo = mid + 1; else hi = mid; }
    let m = 0;
    for (let i = lo; i < d.n && d.t[i] <= tB; i++) if (arr[i] > m) m = arr[i];
    return m;
  }

  function findJumps(d, o) {
    const out = [];
    let i = 0;
    while (i < d.n) {
      if (d.av[i] >= o.flightAv) { i++; continue; }
      // początek lotu: i; przedłużaj, łącząc krótkie przerwy
      const s = i;
      let e = i, j = i + 1;
      while (j < d.n) {
        if (d.t[j] - d.t[j - 1] > o.maxGapMs) break;
        if (d.av[j] < o.flightAv) { e = j; j++; continue; }
        // przerwa: sprawdź, czy krótka
        let k = j;
        while (k < d.n && d.av[k] >= o.flightAv && d.t[k] - d.t[e] <= o.flightGapMs) k++;
        if (k < d.n && d.av[k] < o.flightAv && d.t[k] - d.t[e] <= o.flightGapMs) { j = k; continue; }
        break;
      }
      i = e + 1;
      if (e >= d.n - 2) break; // lot jeszcze trwa (na żywo) — poczekaj na lądowanie
      const tTake = s > 0 ? (d.t[s - 1] + d.t[s]) / 2 : d.t[s];
      const tLand = (d.t[e] + d.t[e + 1]) / 2;
      const dur = tLand - tTake;
      if (dur < o.flightMinMs || dur > o.flightMaxMs) continue;
      const landG = maxIn(d.amag, d, tLand, tLand + 250);
      if (landG < o.landingMinG) continue;
      if (d.t[d.n - 1] < tLand + 250) { i = s; break; } // lądowanie świeże — dokończ później

      const airDeg = integrate(d, s - 1 < 0 ? 0 : s - 1, e + 1, o.maxGapMs, false);
      // obrót na lodzie: od momentu, gdy obrót przekroczył próg przed wybiciem, do zatrzymania po lądowaniu
      let p = s - 1;
      while (p > 0 && Math.abs(d.wv[p]) > o.rotTailDps && d.t[s] - d.t[p] < 1000) p--;
      let q = e + 1;
      while (q < d.n - 1 && Math.abs(d.wv[q]) > o.rotTailDps && d.t[q] - d.t[e] < 1000) q++;
      const preDeg = integrate(d, p, Math.max(p, s - 1), o.maxGapMs, false);
      const postDeg = integrate(d, Math.min(q, e + 1), q, o.maxGapMs, false);

      let wMax = 0;
      for (let k = s; k <= e; k++) wMax = Math.max(wMax, Math.abs(d.wv[k]));
      const sec = dur / 1000;
      out.push({
        type: 'jump',
        t0: tTake, t1: tLand,
        airtimeS: sec,
        heightCm: 981 * sec * sec / 8,                 // h = g·t²/8
        revAir: Math.abs(airDeg) / 360,
        revPre: Math.abs(preDeg) / 360,                // obrót na lodzie przed wybiciem
        revPost: Math.abs(postDeg) / 360,
        dir: airDeg >= 0 ? 'L' : 'P',                  // L = przeciwnie do zegara (patrząc z góry)
        maxRps: wMax / 360,
        avgRps: Math.abs(airDeg) / 360 / sec,          // średnia prędkość obrotu w powietrzu
        takeoffG: maxIn(d.amag, d, tTake - 300, tTake),
        landingG: landG,
        clipped: maxIn(d.wpeak, d, tTake - 200, tLand + 200) > o.clipDps
      });
    }
    return out;
  }

  function findSpins(d, o) {
    const n = d.n, out = [];
    // wygładzone |wv| (okno ~0,2 s) w obr/s
    const rps = new Float64Array(n);
    let lo = 0, acc = 0;
    for (let i = 0; i < n; i++) {
      acc += Math.abs(d.wv[i]);
      while (d.t[i] - d.t[lo] > 200) { acc -= Math.abs(d.wv[lo]); lo++; }
      rps[i] = acc / (i - lo + 1) / 360;
    }
    let i = 0;
    while (i < n) {
      if (rps[i] < o.spinOnRps) { i++; continue; }
      let s = i;
      while (s > 0 && rps[s - 1] >= o.spinOffRps && d.t[i] - d.t[s - 1] < 2000) s--;
      let e = i, below = null;
      for (let k = i; k < n; k++) {
        if (k > 0 && d.t[k] - d.t[k - 1] > 1000) { e = k - 1; below = d.t[k - 1]; break; }
        if (rps[k] < o.spinOffRps) {
          if (below === null) below = d.t[k];
          if (d.t[k] - below > 250) break;
        } else { below = null; e = k; }
      }
      i = e + 1;
      if (below === null || d.t[n - 1] - d.t[e] < 300) {     // piruet trwa — poczekaj, ale pokaż na żywo
        const durNow = (d.t[n - 1] - d.t[s]) / 1000;
        if (durNow >= 1.0) out.ongoing = { t0: d.t[s], durationS: durNow, revs: integrate(d, s, n - 1, o.maxGapMs, true) / 360 };
        break;
      }
      const dur = (d.t[e] - d.t[s]) / 1000;
      if (dur < o.spinMinS) continue;
      const deg = integrate(d, s, e, o.maxGapMs, true);
      const signed = integrate(d, s, e, o.maxGapMs, false);
      let mx = 0;
      for (let k = s; k <= e; k++) mx = Math.max(mx, rps[k]);
      // tempo co 0,25 s do wykresu
      const tempo = [];
      for (let tA = d.t[s]; tA < d.t[e]; tA += 250) {
        let sm = 0, c = 0;
        for (let k = s; k <= e; k++) if (d.t[k] >= tA && d.t[k] < tA + 250) { sm += rps[k]; c++; }
        tempo.push(c ? sm / c : 0);
      }
      // pochylenie osi obrotu względem pionu (0° = idealnie pionowa oś buta)
      let sV = 0, sH = 0;
      for (let k = s; k <= e; k++) { sV += Math.abs(d.wv[k]); sH += d.wh[k]; }
      const axisTiltDeg = Math.atan2(sH, sV) * 180 / Math.PI;
      // równość tempa: zmienność obr/s bez wejścia i wyjścia (po 0,5 s)
      const core = tempo.slice(2, Math.max(2, tempo.length - 2));
      let tempoVarPct = null;
      if (core.length >= 3) {
        const m = core.reduce((a, b) => a + b, 0) / core.length;
        const sd = Math.sqrt(core.reduce((a, b) => a + (b - m) * (b - m), 0) / core.length);
        tempoVarPct = m > 0 ? sd / m * 100 : null;
      }
      out.push({
        type: 'spin',
        axisTiltDeg, tempoVarPct,
        t0: d.t[s], t1: d.t[e],
        durationS: dur,
        revs: deg / 360,
        avgRps: deg / 360 / dur,
        maxRps: mx,
        dir: signed >= 0 ? 'L' : 'P',
        tempo,
        clipped: maxIn(d.wpeak, d, d.t[s], d.t[e]) > o.clipDps
      });
    }
    return out;
  }

  // Przebiegi do wykresu skoku: g w pionie, uderzenie |a|, obr/s wokół pionu (ze znakiem) w oknie [tA, tB]
  function trace(samples, cal, tA, tB) {
    const out = { t: [], av: [], g: [], rps: [] };
    if (!cal) return out;
    for (const s of samples) {
      if (s.t < tA) continue;
      if (s.t > tB) break;
      out.t.push(s.t);
      out.av.push(dot(s.a, cal.u) / cal.g0);
      out.g.push(norm(s.a) / cal.g0);
      out.rps.push(((s.w[0] - cal.bias[0]) * cal.u[0] + (s.w[1] - cal.bias[1]) * cal.u[1] + (s.w[2] - cal.bias[2]) * cal.u[2]) / 360);
    }
    return out;
  }

  // mode: 'jump' | 'spin' | 'both'
  function analyze(samples, cal, mode, opts) {
    const o = Object.assign({}, DEFAULTS, PROFILES[(opts && opts.profile) || 'kids'], opts || {});
    if (!cal || samples.length < 10) return { events: [], live: null };
    const d = derive(samples, cal);
    let ev = [];
    if (mode !== 'spin') ev = ev.concat(findJumps(d, o));
    let spinNow = null;
    if (mode !== 'jump') { const sp = findSpins(d, o); spinNow = sp.ongoing || null; ev = ev.concat(sp); }
    ev.sort((x, y) => x.t0 - y.t0);
    const k = d.n - 1;
    let r = 0, c = 0;
    for (let i = k; i >= 0 && d.t[k] - d.t[i] <= 200; i--) { r += Math.abs(d.wv[i]); c++; }
    // diagnostyka: co czujnik widział w ostatniej sekundzie
    let avMin = Infinity, gMax = 0;
    for (let i = k; i >= 0 && d.t[k] - d.t[i] <= 1000; i--) { if (d.av[i] < avMin) avMin = d.av[i]; if (d.amag[i] > gMax) gMax = d.amag[i]; }
    return { events: ev, live: { av: d.av[k], rps: r / c / 360, inAir: d.av[k] < o.flightAv, spin: spinNow, avMin1s: avMin, gMax1s: gMax, thr: { flightAv: o.flightAv, landingMinG: o.landingMinG } } };
  }

  // ---- symulator (demo + testy) ----
  // Skrypt: [{kind:'still'|'glide'|'jump'|'spin', ...}] → próbki co dtMs w układzie czujnika
  function simulate(script, opt) {
    const o = Object.assign({ dtMs: 10, up: [-0.17, -0.43, 0.89], seed: 7 }, opt || {});
    let seed = o.seed;
    const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647 - 0.5; };
    const u = o.up.map(v => v / norm(o.up));
    // baza prostopadła do u
    const tmp = Math.abs(u[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
    let e1 = [u[1] * tmp[2] - u[2] * tmp[1], u[2] * tmp[0] - u[0] * tmp[2], u[0] * tmp[1] - u[1] * tmp[0]];
    e1 = e1.map(v => v / norm(e1));
    const e2 = [u[1] * e1[2] - u[2] * e1[1], u[2] * e1[0] - u[0] * e1[2], u[0] * e1[1] - u[1] * e1[0]];
    const out = [];
    let t = 1000, phase = 0;
    const push = (aV, aH, wV, noiseA) => {
      phase += wV / R2D * o.dtMs / 1000;
      const h1 = aH * Math.cos(phase), h2 = aH * Math.sin(phase);
      const a = [0, 1, 2].map(k => (u[k] * aV + e1[k] * h1 + e2[k] * h2 + rnd() * noiseA) * G);
      const w = [0, 1, 2].map(k => (u[k] * wV + e1[k] * rnd() * 6 + e2[k] * rnd() * 6) / R2D);
      // czujnik nasyca się na ±2000 °/s na każdej osi osobno
      out.push({ t, a: a.map(v => v / G), w: w.map(v => Math.max(-2000, Math.min(2000, v * R2D))), seq: out.length, rssi: -60 });
      t += o.dtMs;
    };
    const steps = ms => Math.round(ms / o.dtMs);
    for (const s of script) {
      if (s.kind === 'still') for (let i = 0; i < steps(s.ms); i++) push(1, 0, 0, 0.01);
      else if (s.kind === 'glide') for (let i = 0; i < steps(s.ms); i++) push(1 + rnd() * 0.3, rnd() * 0.3, rnd() * 20, 0.05);
      else if (s.kind === 'jump') {
        // s.revs w powietrzu, s.air [s], s.pre obrót na lodzie przy wybiciu
        const air = s.air, wAir = s.revs * 360 / air, pre = s.pre || 0.25;
        const nPre = steps(250), nAir = steps(air * 1000), nLand = steps(200);
        for (let i = 0; i < nPre; i++) { const f = i / nPre; push(1 + 1.2 * Math.sin(Math.PI * f), 0.2, (pre * 360 / 0.25) * 2 * f * (s.sign || 1), 0.05); }
        for (let i = 0; i < nAir; i++) push((s.airAv || 0) + rnd() * 0.1, 2.5, wAir * (s.sign || 1), 0.05);
        for (let i = 0; i < nLand; i++) { const f = i / nLand; push(1 + (s.impact || 6) * Math.exp(-f * 6), 0.3, wAir * (1 - f) * 0.3 * (s.sign || 1), 0.1); }
      } else if (s.kind === 'spin') {
        const n = steps(s.ms), ramp = steps(400);
        for (let i = 0; i < n; i++) {
          const f = Math.min(1, i / ramp, (n - i) / ramp);
          push(1 + rnd() * 0.15, 0.4, s.rps * 360 * f * (s.sign || 1), 0.05);
        }
      }
    }
    return out;
  }

  function toCSV(samples) {
    const rows = ['t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,seq,rssi'];
    for (const s of samples) rows.push([s.t, ...s.a.map(v => v.toFixed(4)), ...s.w.map(v => v.toFixed(2)), s.seq ?? '', s.rssi ?? ''].join(','));
    return rows.join('\n');
  }

  return { VERSION: 'pomiar-0.3', DEFAULTS, PROFILES, parsePacket, findCalibration, calibrateRange, approxCalibration, analyze, trace, simulate, toCSV };
});
