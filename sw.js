// Offline cache for the rink (no internet needed after the first visit).
const CACHE = 'opencarv-v18';
const FILES = ['./', './index.html', './zasieg.html', './pomiar.html', './pomiar_core.js', './pomiar_store.js', './manifest.json'];
self.addEventListener('install', e => e.waitUntil(caches.open(CACHE).then(c => c.addAll(FILES)).then(() => self.skipWaiting())));
self.addEventListener('activate', e => e.waitUntil(
  caches.keys().then(ks => Promise.all(ks.filter(k => k !== CACHE).map(k => caches.delete(k)))).then(() => self.clients.claim())));
// Network first (fresh version when online), cache when offline.
self.addEventListener('fetch', e => {
  if (e.request.method !== 'GET') return;
  const url = new URL(e.request.url);
  // własne pliki: zawsze sprawdź u serwera (GitHub Pages trzyma kopię 10 min w cache przeglądarki)
  const net = url.origin === location.origin ? fetch(url.href, { cache: 'no-cache' }) : fetch(e.request);
  e.respondWith(net.then(r => {
    const copy = r.clone(); caches.open(CACHE).then(c => c.put(e.request, copy)); return r;
  }).catch(() => caches.match(e.request, { ignoreSearch: true })));
});
