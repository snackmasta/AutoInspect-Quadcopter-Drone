# Struktur Simulasi

## 2. Struktur Simulasi

- **Kelas utama:** `QuadcopterSimulation` mengelola seluruh state drone, waypoint, mode kontrol, serta logika fisika dan kontroler.
- **State drone:**
  - Posisi (x, y, z), kecepatan (vx, vy, vz)
  - Orientasi (roll, pitch, yaw), kecepatan sudut (wx, wy, wz)
  - State ini diperbarui setiap iterasi simulasi.
- **Parameter fisik:**
  - Massa drone, gravitasi, momen inersia, panjang lengan, koefisien thrust, dan densitas udara.
  - Parameter ini dapat diubah untuk simulasi berbagai tipe drone.
- **Fungsi utama:**
  - `step()`: mengatur satu langkah simulasi, update state, kontrol, dan trajectory.
  - `physics_update()`: menghitung perubahan posisi, kecepatan, dan orientasi berdasarkan gaya dan torsi.
  - `position_controller()`: PID controller untuk mengarahkan drone ke target waypoint.
- **Waypoint dan Trajectory:**
  - Waypoint adalah daftar titik yang harus diikuti drone.
  - Trajectory menyimpan jejak posisi drone selama simulasi, digunakan untuk visualisasi dan analisis.
- **Environment/Terrain:**
  - Terrain dimodelkan dengan kelas `Environment` menggunakan Perlin Noise untuk menghasilkan kontur permukaan realistis.
  - Fungsi `contour_height(x, y)` menghasilkan ketinggian terrain di titik tertentu.
- **Sensor Kamera & Mapping:**
  - Drone dilengkapi kamera virtual (simulasi sensor) untuk merekam heightmap terrain di bawahnya.
  - Data hasil perekaman disimpan dan divisualisasikan sebagai grid 3D.
- **Renderer & Visualisasi:**
  - Modul `Renderer` menampilkan drone, terrain, waypoint, trajectory, thrust, dan hasil mapping secara real-time.
  - GUI (ImGui) digunakan untuk kontrol, telemetry, dan monitoring misi.
- **Interaksi Antar Modul:**
  - `QuadcopterSimulation` berinteraksi dengan `Renderer` untuk visualisasi, dan dengan `Environment` untuk data terrain.
  - Kontroler dan physics update saling terintegrasi dalam satu siklus simulasi.

---
