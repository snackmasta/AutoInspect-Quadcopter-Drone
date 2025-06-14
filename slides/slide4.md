## 3. Cara Kerja Simulasi

- Setiap iterasi simulasi dijalankan melalui fungsi `step(delta_time)`, yang mengatur seluruh proses update state drone.

### Mode Simulasi
- **Mode Manual:**
  - Pengguna dapat mengatur kecepatan rotor secara langsung (manual_rpms).
  - Cocok untuk eksperimen kontrol manual atau pengujian fisika dasar.
- **Mode Otomatis:**
  - Drone mengikuti waypoint secara otomatis menggunakan kontroler posisi (PID).
  - Kontroler menghitung thrust dan torsi yang dibutuhkan agar drone menuju target.

### Proses Update Fisika
- Fungsi `physics_update(u, dt)` menghitung perubahan posisi, kecepatan, orientasi, dan kecepatan sudut berdasarkan gaya dan torsi:
  - **Gaya thrust:** Dihasilkan oleh keempat rotor, dihitung dari kecepatan putar (RPM) dan koefisien thrust.
  - **Torsi:** Dihitung untuk roll, pitch, dan yaw berdasarkan distribusi thrust antar rotor.
  - **Efek drag:** Simulasi drag udara dan torsi drag pada sumbu rotasi.
  - **Integrasi Euler:** Posisi, kecepatan, dan orientasi diupdate menggunakan metode Euler berdasarkan percepatan dan kecepatan sudut.
  - **Ground friction:** Jika drone menyentuh tanah, gaya friksi diterapkan untuk menghentikan pergerakan lateral.
  - **Batasan fisik:** Posisi z tidak boleh di bawah permukaan tanah.

### Siklus Simulasi Lengkap
1. **Spin-up:**
   - Sebelum takeoff, rotor diputar perlahan hingga mencapai RPM hover (spinup ramp).
2. **Waypoint Navigation:**
   - Drone bergerak menuju waypoint berikutnya, berpindah ke waypoint selanjutnya jika sudah cukup dekat.
   - Jika waypoint berupa hover (stabil di satu titik), kontroler menjaga posisi dan attitude.
3. **Landing:**
   - Jika perintah landing diberikan, drone mengikuti trajectory khusus untuk turun perlahan dan berhenti di tanah.
4. **Sensor & Mapping:**
   - Pada interval tertentu, drone melakukan scan terrain di bawahnya menggunakan kamera virtual.
   - Data heightmap terrain direkam dan divisualisasikan.
5. **Logging & Visualisasi:**
   - Setiap langkah, posisi drone, kecepatan, dan data sensor disimpan ke trajectory untuk analisis dan visualisasi.
   - GUI menampilkan telemetry, progress, thrust, dan hasil mapping secara real-time.

### Interaksi Antar Komponen
- Fungsi `step()` memanggil kontroler, physics update, sensor, dan logging secara terintegrasi.
- Mode dan status drone (takeoff, landing, manual/otomatis) dapat diubah melalui GUI.

---
