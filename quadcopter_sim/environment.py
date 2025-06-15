import numpy as np
from OpenGL.GL import *
from perlin_noise import PerlinNoise

class Environment:
    def __init__(self, size=250, step=0.3, seed=42):
        self.size = size  # expanded to cover the full mapping trajectory
        self.step = step
        self.seed = seed
        self.noise = PerlinNoise(octaves=5, seed=seed)
        self.scale = 0.02  # Controls terrain feature size

    def contour_height(self, x, y):
        # Procedural terrain using PerlinNoise (pure Python)
        n = self.noise([x * self.scale, y * self.scale])
        return n * 4.0  # scale to get a nice range

    def draw(self, wireframe=True):
        if wireframe:
            # Draw light grey wireframe for unscanned terrain
            glColor3f(0.8, 0.8, 0.8)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
            step = self.step
            x_range = np.arange(-self.size, self.size, step)
            y_range = np.arange(-self.size, self.size, step)
            for x in x_range:
                glBegin(GL_LINE_STRIP)
                for y in y_range:
                    glVertex3f(x, y, self.contour_height(x, y))
                glEnd()
            for y in y_range:
                glBegin(GL_LINE_STRIP)
                for x in x_range:
                    glVertex3f(x, y, self.contour_height(x, y))
                glEnd()
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        else:
            # Solid color ground (for scanned/known terrain, if needed)
            glColor3f(0.3, 0.5, 0.3)
            glBegin(GL_QUADS)
            glVertex3f(-self.size, -self.size, self.contour_height(-self.size, -self.size))
            glVertex3f(self.size, -self.size, self.contour_height(self.size, -self.size))
            glVertex3f(self.size, self.size, self.contour_height(self.size, self.size))
            glVertex3f(-self.size, self.size, self.contour_height(-self.size, self.size))
            glEnd()

    def get_scanned_map(self, drone_pos, fov=60, res=32):
        """
        Simulate a camera scan from the drone's position, returning a heightmap.
        """
        x, y, z = drone_pos
        half_fov = np.radians(fov / 2)
        size = np.tan(half_fov) * z
        xs = np.linspace(x - size, x + size, res)
        ys = np.linspace(y - size, y + size, res)
        img = np.zeros((res, res), dtype=np.float32)
        for i, xi in enumerate(xs):
            for j, yj in enumerate(ys):
                img[j, i] = self.contour_height(xi, yj)
        return img
