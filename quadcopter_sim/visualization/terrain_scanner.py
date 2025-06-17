"""Terrain scanning and camera chunk management."""

import numpy as np


class TerrainScanner:
    """Manages camera chunks for terrain scanning and visualization."""
    
    def __init__(self):
        self.camera_chunks = []  # List of (img, pos) tuples
        self.camera_chunk_counter = 0
        self.show_camera = False  # Camera visualization toggle, default off
    
    def update(self, sim, environment):
        """Update terrain scanning with new camera data."""
        self.camera_chunk_counter += 1
        
        # Assuming ~30 FPS, 3 seconds = 90 frames
        if self.camera_chunk_counter >= 90:
            img = sim.get_camera_image(environment, fov=60, res=12)
            pos = sim.state[:3].copy()
            self.add_camera_chunk_with_partial_culling(img, pos, fov=60)
            self.camera_chunk_counter = 0
    
    def is_chunk_overlapping(self, pos1, pos2, fov=60, altitude=None):
        """Return True if two camera chunks overlap based on their positions and FOV coverage."""
        if altitude is None:
            altitude = pos1[2]
        half_fov = np.radians(fov / 2)
        size = np.tan(half_fov) * (altitude - 0.2)
        # Use a square region for simplicity
        dx = abs(pos1[0] - pos2[0])
        dy = abs(pos1[1] - pos2[1])
        return dx < 2 * size and dy < 2 * size
    
    def mask_overlapping_area(self, img, pos, existing_chunks, fov=60):
        """Mask out overlapping areas in img based on existing chunks."""
        res = img.shape[0]
        half_fov = np.radians(fov / 2)
        size = np.tan(half_fov) * (pos[2] - 0.2)
        xs = np.linspace(pos[0] - size, pos[0] + size, res)
        ys = np.linspace(pos[1] - size, pos[1] + size, res)
        mask = np.ones_like(img, dtype=bool)
        
        for _, old_pos in existing_chunks:
            old_size = np.tan(half_fov) * (old_pos[2] - 0.2)
            x_min, x_max = old_pos[0] - old_size, old_pos[0] + old_size
            y_min, y_max = old_pos[1] - old_size, old_pos[1] + old_size
            
            # Mask out overlapping region
            x_overlap = (xs >= x_min) & (xs <= x_max)
            y_overlap = (ys >= y_min) & (ys <= y_max)
            
            for i in range(res):
                for j in range(res):
                    if x_overlap[j] and y_overlap[i]:
                        mask[i, j] = False
        
        return np.where(mask, img, np.nan), mask
    
    def add_camera_chunk_with_partial_culling(self, img, pos, fov=60):
        """Add only the non-overlapping part of a camera chunk."""
        masked_img, mask = self.mask_overlapping_area(img, pos, self.camera_chunks, fov)
        if np.any(mask):
            self.camera_chunks.append((masked_img, pos))
    
    def add_camera_chunk_with_culling(self, img, pos, fov=60):
        """Add a camera chunk, replacing any overlapping chunk to save memory."""
        new_chunks = []
        replaced = False
        
        for old_img, old_pos in self.camera_chunks:
            if self.is_chunk_overlapping(pos, old_pos, fov, altitude=pos[2]):
                # Replace the old chunk with the new one
                if not replaced:
                    new_chunks.append((img, pos))
                    replaced = True
                # else: skip adding the old chunk
            else:
                new_chunks.append((old_img, old_pos))
        
        if not replaced:
            new_chunks.append((img, pos))
        
        self.camera_chunks = new_chunks
    
    def get_camera_display_data(self, sim, environment):
        """Get camera image data for display in ImGui."""
        if self.show_camera:
            cam_img = sim.get_camera_image(environment, fov=60, res=16)
            return cam_img[cam_img.shape[0]//2]  # Return center row
        return None
    
    def clear_chunks(self):
        """Clear all accumulated camera chunks."""
        self.camera_chunks = []
        self.camera_chunk_counter = 0
