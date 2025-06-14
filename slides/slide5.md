# Cara Kerja Kontroler

## 4. Cara Kerja Kontroler

- **Fungsi utama:** `position_controller(state, target, ...)` mengatur drone agar menuju target waypoint dengan stabil.

### Kontrol Posisi (PID)
- Menghitung error posisi (selisih antara posisi drone dan target).
- PID controller digunakan untuk menentukan akselerasi yang dibutuhkan:
  - **Proportional (P):** Merespons error posisi secara langsung.
  - **Derivative (D):** Meredam osilasi dengan memperhitungkan kecepatan.
  - (Integral jarang dipakai untuk drone karena bisa menyebabkan drift.)
- Output utama: akselerasi yang diinginkan pada sumbu x, y, z.
- Komponen z (vertikal) ditambah gravitasi agar drone bisa hover.

### Kontrol Attitude (Roll, Pitch, Yaw)
- Akselerasi x/y diterjemahkan menjadi target roll dan pitch.
- Kontroler attitude (PID) mengatur roll dan pitch agar sesuai target.
- Kontrol yaw:
  - Yaw bisa diarahkan ke arah lintasan berikutnya (lookahead) atau dipertahankan saat hover.
  - Error yaw dihitung dan dikoreksi dengan PID yaw controller.

### Output Kontroler
- Output akhir berupa:
  - Torsi roll, pitch, yaw (tau_x, tau_y, tau_z)
  - Thrust total (gaya angkat utama)
- Nilai-nilai ini dikonversi menjadi thrust masing-masing rotor menggunakan model distribusi gaya pada quadcopter.

### Fitur Tambahan
- Hover indices: waypoint tertentu diulang untuk menstabilkan drone setelah belokan tajam.
- Yaw control dapat diaktifkan/nonaktifkan sesuai kebutuhan misi.
- Parameter gains (KP, KD) dapat di-tuning untuk menyesuaikan respons kontroler.

### Relevansi dan Implementasi
- Struktur kontroler ini mirip dengan yang digunakan pada drone nyata, sehingga hasil simulasi relevan untuk pengembangan algoritma kontrol dunia nyata.
- Semua proses berjalan secara real-time di setiap iterasi simulasi.

---
