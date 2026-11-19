# OpenCARV — podgląd czujnika (GitHub Pages)

- `index.html` — podgląd czujnika: osie, obroty, obr/s, przeciążenie, czas lotu.
- `pomiar.html` + `pomiar_core.js` — pomiar z samego czujnika: skoki (obroty w powietrzu, czas lotu, wysokość, uderzenia) i piruety (obroty, tempo). Historia sesji w telefonie (`pomiar_store.js`, IndexedDB): autozapis co 10 s, otwieranie, nazwy, CSV, usuwanie. Demo bez czujnika: `pomiar.html?demo=5`. Testy: `node tests/test_pomiar.js`.
- `zasieg.html` — test zasięgu Bluetooth: sygnał, pakiety/s, straty, punkty z odległością.

Na iPhonie otwierać w przeglądarce **Bluefy** (Safari nie ma Web Bluetooth). Po pierwszym
otwarciu z internetem strony działają offline (service worker `sw.js`).
Źródła: `sports_tech_poc/software/sensor_view.html` i `ble_range_test.html` — po zmianie skopiuj tu
i podbij `CACHE` w `sw.js`.
