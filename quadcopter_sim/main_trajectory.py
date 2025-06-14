import numpy as np

def get_main_trajectory():
    altitude = 6.0
    takeoff = [np.array([0, 0, z]) for z in np.linspace(0, altitude, 8)]
    # Segitiga: tiga titik utama
    p1 = np.array([0, 0, altitude])
    p2 = np.array([20, 0, altitude])
    p3 = np.array([10, 17.32, altitude])  # 17.32 ≈ 20 * sin(60°)
    # Trajectory: p1 -> p2 -> p3 -> p1
    n_points = 40
    leg1 = [p1 + (p2 - p1) * t for t in np.linspace(0, 1, n_points)]
    leg2 = [p2 + (p3 - p2) * t for t in np.linspace(0, 1, n_points)]
    leg3 = [p3 + (p1 - p3) * t for t in np.linspace(0, 1, n_points)]
    wps = takeoff + leg1 + leg2 + leg3
    # Kembali ke home
    wps.append(np.array([0, 0, altitude]))
    return wps
