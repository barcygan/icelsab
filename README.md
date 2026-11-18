# OpenCARV — podgląd czujnika (GitHub Pages)

- `index.html` — podgląd czujnika: osie, obroty, obr/s, przeciążenie, czas lotu.
- `zasieg.html` — test zasięgu Bluetooth: sygnał, pakiety/s, straty, punkty z odległością.

Na iPhonie otwierać w przeglądarce **Bluefy** (Safari nie ma Web Bluetooth). Po pierwszym
otwarciu z internetem strony działają offline (service worker `sw.js`).
Źródła: `sports_tech_poc/software/sensor_view.html` i `ble_range_test.html` — po zmianie skopiuj tu
i podbij `CACHE` w `sw.js`.
