# Alur Simulasi Lengkap

## 5. Alur Simulasi

1. **Inisialisasi**
   - State drone di-set ke posisi awal (di tanah), parameter fisik (massa, gravitasi, momen inersia) diatur.
   - Waypoint dan trajectory di-reset, mode kontrol (manual/otomatis) dipilih.
2. **Penentuan Lintasan (Waypoints)**
   - Lintasan utama di-generate (misal: takeoff, hover, lintasan mapping, landing).
   - Waypoint dengan hover diinsert setelah belokan tajam untuk stabilisasi.
3. **Spin-up Rotor**
   - Sebelum takeoff, rotor diputar perlahan hingga mencapai RPM hover (spinup ramp) untuk menghindari lonjakan gaya tiba-tiba.
4. **Siklus Simulasi (Setiap langkah waktu / step):**
   - **a. Mode Kontrol:**
     - Jika manual, kecepatan rotor diatur langsung oleh user.
     - Jika otomatis, kontroler posisi menghitung thrust dan torsi menuju waypoint.
   - **b. Update Fisika:**
     - Physics update menghitung gaya, torsi, drag, friksi tanah, dan update posisi, kecepatan, orientasi.
   - **c. Switching Waypoint:**
     - Jika drone cukup dekat ke waypoint, pindah ke waypoint berikutnya.
     - Jika di waypoint hover, drone menstabilkan posisi dan attitude.
   - **d. Perekaman Sensor:**
     - Pada interval tertentu, kamera virtual merekam heightmap terrain di bawah drone.
     - Data mapping diakumulasi dan divisualisasikan.
   - **e. Logging & Visualisasi:**
     - Posisi, kecepatan, thrust, dan data sensor disimpan ke trajectory/log untuk analisis dan visualisasi real-time.
5. **Landing**
   - Jika perintah landing diberikan, drone mengikuti trajectory khusus untuk turun perlahan dan berhenti di tanah.
6. **Akhir Simulasi**
   - Proses berulang hingga semua waypoint tercapai atau simulasi dihentikan.
   - Data hasil simulasi dapat diekspor untuk analisis lebih lanjut.

---
