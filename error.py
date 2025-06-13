import matplotlib.pyplot as plt
import numpy as np

# Waypoint target positions
waypoints = np.array([
    [0, 0, 2],
    [2, 0, 2],
    [2, 2, 3],
    [0, 2, 2],
    [0, 0, 0]
])

# Actual reached positions (from log)
actual_positions = np.array([
    [0.000, 0.000, 1.803],
    [0.207, 0.000, 2.002],
    [0.605, 0.000, 2.014],
    [1.007, 0.000, 2.004],
    [1.408, 0.000, 2.000],
    [1.808, 0.000, 2.000],
    [1.980, 0.228, 2.114],
    [2.006, 0.628, 2.314],
    [2.002, 1.028, 2.514],
    [2.000, 1.429, 2.714],
    [2.000, 1.829, 2.914],
    [1.779, 1.982, 2.880],
    [1.373, 2.005, 2.689],
    [0.972, 2.002, 2.487],
    [0.571, 2.000, 2.286],
    [0.171, 2.000, 2.086],
    [0.009, 1.739, 1.743],
    [-0.005, 1.342, 1.339],
    [-0.001, 0.937, 0.937],
    [-0.000, 0.537, 0.537]
])

# Calculate deviations
min_len = min(len(waypoints), len(actual_positions))
deviations = np.linalg.norm(actual_positions[:min_len] - waypoints[:min_len], axis=1)

# Plotting
plt.figure(figsize=(10, 5))
plt.plot(range(1, min_len + 1), deviations, marker='o', linestyle='-', color='red')
plt.title('Deviation from Waypoints')
plt.xlabel('Waypoint Number')
plt.ylabel('Deviation (Euclidean Distance)')
plt.grid(True)
plt.xticks(range(1, min_len + 1))
plt.tight_layout()
plt.show()

