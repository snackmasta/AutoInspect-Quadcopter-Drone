import cupy as cp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

N = 210
pos = cp.random.rand(N, 3).astype(cp.float32)
vel = (cp.random.rand(N, 3).astype(cp.float32) - 0.5) * 0.01  # smaller velocity for smoother motion

def update_positions(pos, vel):
    pos += vel
    for i in range(3):
        mask_lower = pos[:, i] < 0
        mask_upper = pos[:, i] > 1
        vel[:, i] = cp.where(mask_lower | mask_upper, -vel[:, i], vel[:, i])
        pos[:, i] = cp.clip(pos[:, i], 0, 1)
    return pos, vel

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
scat = ax.scatter([], [], [], s=50, c='blue')

ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_zlim(0, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title(f'{N} Dots Bouncing in a 3D Box')

def init():
    scat._offsets3d = ([], [], [])
    return scat,

def animate(frame):
    global pos, vel
    pos, vel = update_positions(pos, vel)
    pos_cpu = pos.get()
    scat._offsets3d = (pos_cpu[:, 0], pos_cpu[:, 1], pos_cpu[:, 2])
    return scat,

# Reduce interval to 15 ms (around 66 FPS)
ani = FuncAnimation(fig, animate, init_func=init, frames=None, interval=5, blit=False)
plt.show()
