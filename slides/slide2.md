# Cara Kerja Proyek: Simulasi dan Kontroler

## 1. Pendahuluan

Simulator quadcopter ini dirancang untuk memodelkan dinamika fisik drone dan menguji algoritma kontrol posisi secara virtual. Fokus utama adalah pada dua komponen: simulasi fisika dan kontroler posisi.

## 2. Simulasi Fisika dan Kontroler

- Simulasi fisika memodelkan gaya, torsi, dan dinamika drone secara real-time, termasuk efek gravitasi, thrust, drag, dan friksi tanah.
- Kontroler posisi (PID) digunakan untuk mengarahkan drone menuju waypoint secara otomatis, serta menjaga kestabilan attitude (roll, pitch, yaw).
- Mode manual juga tersedia untuk mengatur kecepatan rotor secara langsung.

## 3. Perekaman Terrain dan Mapping

- Simulator dilengkapi fitur perekaman terrain (mapping) menggunakan kamera virtual yang mensimulasikan sensor pada drone.
- Setiap beberapa detik, drone melakukan scan area di bawahnya dan menghasilkan heightmap (peta ketinggian) menggunakan fungsi `get_camera_image()`.
- Data heightmap ini diakumulasi dan divisualisasikan sebagai wireframe grid di area yang sudah dipetakan.
- Proses perekaman terrain ini dapat digunakan untuk simulasi misi mapping, pengujian algoritma SLAM, atau analisis coverage area.
- Fitur culling diterapkan agar data terrain yang terekam tidak tumpang tindih dan efisien dalam penggunaan memori.

## 4. Relevansi Fitur Lain

- Data posisi, kecepatan, dan hasil perekaman terrain dapat diekspor untuk analisis lebih lanjut.
- Visualisasi thrust, telemetry, dan progress misi tersedia secara real-time di tampilan GUI.
- Simulator ini dapat dikembangkan lebih lanjut untuk integrasi sensor lain (misal LiDAR) atau skenario misi yang lebih kompleks.

---
