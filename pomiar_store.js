// Historia pomiarów w telefonie (IndexedDB, bez internetu, bez konta).
// Sesja = wyniki + nazwy od trenera + surowe próbki (spakowane do tablic typowanych).
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.PomiarStore = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';
  const DB = 'opencarv_pomiar', STORE = 'sessions', VER = 1;

  // ~100 próbek/s × 7 liczb: Float32 wystarcza (g z 4 miejscami, °/s z 2), czas w Float64
  function pack(samples) {
    const n = samples.length, t = new Float64Array(n), aw = new Float32Array(n * 6), seq = new Int32Array(n), rssi = new Int16Array(n);
    for (let i = 0; i < n; i++) {
      const s = samples[i];
      t[i] = s.t;
      aw[i * 6] = s.a[0]; aw[i * 6 + 1] = s.a[1]; aw[i * 6 + 2] = s.a[2];
      aw[i * 6 + 3] = s.w[0]; aw[i * 6 + 4] = s.w[1]; aw[i * 6 + 5] = s.w[2];
      seq[i] = s.seq == null || !isFinite(s.seq) ? -1 : s.seq;
      rssi[i] = s.rssi == null || !isFinite(s.rssi) ? 0 : s.rssi;
    }
    return { n, t, aw, seq, rssi };
  }
  function unpack(p) {
    if (!p) return [];
    const out = new Array(p.n);
    for (let i = 0; i < p.n; i++) {
      const o = i * 6;
      out[i] = { t: p.t[i], a: [p.aw[o], p.aw[o + 1], p.aw[o + 2]], w: [p.aw[o + 3], p.aw[o + 4], p.aw[o + 5]],
                 seq: p.seq[i] < 0 ? null : p.seq[i], rssi: p.rssi[i] || null };
    }
    return out;
  }

  // Krótki opis do listy (bez próbek — lista ładuje się szybko)
  function summarize(rec) {
    const ev = rec.events || [];
    return {
      id: rec.id, startedAt: rec.startedAt, endedAt: rec.endedAt, mode: rec.mode,
      jumps: ev.filter(e => e.type === 'jump').length, spins: ev.filter(e => e.type === 'spin').length,
      nSamples: rec.raw ? rec.raw.n : 0, finished: !!rec.finished, title: rec.title || ''
    };
  }

  let dbp = null;
  function open() {
    if (dbp) return dbp;
    dbp = new Promise((res, rej) => {
      if (typeof indexedDB === 'undefined') return rej(new Error('Brak IndexedDB'));
      const r = indexedDB.open(DB, VER);
      r.onupgradeneeded = () => {
        const db = r.result;
        if (!db.objectStoreNames.contains(STORE)) db.createObjectStore(STORE, { keyPath: 'id' });
        if (!db.objectStoreNames.contains('index')) db.createObjectStore('index', { keyPath: 'id' }); // same podsumowania
      };
      r.onsuccess = () => res(r.result);
      r.onerror = () => rej(r.error);
    });
    return dbp;
  }
  function tx(stores, mode, fn) {
    return open().then(db => new Promise((res, rej) => {
      const t = db.transaction(stores, mode);
      let out;
      Promise.resolve(fn(t)).then(v => { out = v; });
      t.oncomplete = () => res(out);
      t.onerror = () => rej(t.error);
      t.onabort = () => rej(t.error || new Error('abort'));
    }));
  }
  const req = r => new Promise((res, rej) => { r.onsuccess = () => res(r.result); r.onerror = () => rej(r.error); });

  function save(rec) {
    return tx([STORE, 'index'], 'readwrite', t => {
      t.objectStore(STORE).put(rec);
      t.objectStore('index').put(summarize(rec));
    });
  }
  function list() {
    return tx(['index'], 'readonly', t => req(t.objectStore('index').getAll()))
      .then(a => (a || []).sort((x, y) => y.startedAt - x.startedAt));
  }
  function load(id) { return tx([STORE], 'readonly', t => req(t.objectStore(STORE).get(id))); }
  function remove(id) {
    return tx([STORE, 'index'], 'readwrite', t => { t.objectStore(STORE).delete(id); t.objectStore('index').delete(id); });
  }
  // Prośba, żeby przeglądarka nie czyściła danych przy braku miejsca (best effort)
  function persist() {
    try { if (navigator.storage && navigator.storage.persist) return navigator.storage.persist(); } catch (e) {}
    return Promise.resolve(false);
  }

  return { pack, unpack, summarize, save, list, load, remove, persist };
});
