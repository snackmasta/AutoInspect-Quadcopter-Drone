"""Refactored main renderer using modular visualization components."""

import os
# Force use of high-performance GPU on NVIDIA/AMD systems (must be set before OpenGL context creation)
os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
os.environ["QT_DEVICE_PIXEL_RATIO"] = "auto"
# For NVIDIA Optimus (laptops):
os.environ["CUDA_FORCE_PTX_JIT"] = "1"
os.environ["NVIDIA_OPTIMUS_ENABLE_NVML"] = "1"
os.environ["NV_OPTIMUS_ENABLE"] = "1"
# For AMD switchable graphics:
os.environ["AMD_POWERXPRESS_REQUEST_HIGH_PERFORMANCE"] = "1"

import numpy as np
from OpenGL.GL import *
from OpenGL.arrays import vbo
from OpenGL.GLU import *
import imgui
from quadcopter_sim.environment import Environment
from .visualization import CameraController, UIPanels, SceneRenderer, TerrainScanner


class Renderer:
    """Main renderer class that orchestrates all visualization components."""
    
    def __init__(self, sim):
        self.sim = sim
        self.window_width, self.window_height = 1200, 800
        
        # Initialize visualization components
        self.camera_controller = CameraController()
        self.ui_panels = UIPanels(self.window_width, self.window_height)
        self.scene_renderer = SceneRenderer(sim)
        self.terrain_scanner = TerrainScanner()
        
        # Environment
        self.environment = Environment(size=20, step=2.0)  # Increased step for performance
        
        # Legacy compatibility attributes
        self.show_lidar = False  # Toggle for LiDAR visualization (default OFF)
        self.scanned_points = []  # Accumulated scanned terrain points
        self.vbo = None  # Vertex Buffer Object for scanned points
        self.vbo_needs_update = True
    
    @property
    def angle_x(self):
        """Legacy compatibility property."""
        return self.camera_controller.angle_x
    
    @angle_x.setter
    def angle_x(self, value):
        """Legacy compatibility property."""
        self.camera_controller.angle_x = value
    
    @property
    def angle_y(self):
        """Legacy compatibility property."""
        return self.camera_controller.angle_y
    
    @angle_y.setter
    def angle_y(self, value):
        """Legacy compatibility property."""
        self.camera_controller.angle_y = value
    
    @property
    def angle_z(self):
        """Legacy compatibility property."""
        return self.camera_controller.angle_z
    
    @angle_z.setter
    def angle_z(self, value):
        """Legacy compatibility property."""
        self.camera_controller.angle_z = value
    
    @property
    def zoom(self):
        """Legacy compatibility property."""
        return self.camera_controller.zoom
    
    @zoom.setter
    def zoom(self, value):
        """Legacy compatibility property."""
        self.camera_controller.zoom = value
    
    @property
    def show_camera(self):
        """Legacy compatibility property."""
        return self.terrain_scanner.show_camera
    
    @show_camera.setter
    def show_camera(self, value):
        """Legacy compatibility property."""
        self.terrain_scanner.show_camera = value
    
    @property
    def camera_chunks(self):
        """Legacy compatibility property."""
        return self.terrain_scanner.camera_chunks
    
    def handle_mouse(self, window):
        """Handle mouse input for camera controls."""
        self.camera_controller.handle_mouse(window)
    
    def draw_ground_grid(self, size=3, step=0.5):
        """Draw the ground grid (legacy compatibility)."""
        self.environment.draw()
    
    def draw_scene(self, debug_config=None):
        """Main scene drawing method."""
        sim = self.sim
        
        # Update terrain scanning
        self.terrain_scanner.update(sim, self.environment)
        
        # Draw all UI panels with proper layout management
        self.ui_panels.draw_all_panels(sim, self.camera_controller, self.environment, 
                                      debug_config=debug_config, terrain_scanner=self.terrain_scanner)
        
        # 3D Scene rendering
        self.scene_renderer.clear_scene()
        
        # Set up camera view
        center = sim.state[:3]
        self.camera_controller.setup_camera_view(center)
        
        # Render environment
        self.scene_renderer.draw_environment(self.environment)
        
        # Render waypoints and trajectory
        self.scene_renderer.draw_waypoints_and_trajectory(sim)
        
        # Render drone components
        self.scene_renderer.draw_drone_structure(sim)
        self.scene_renderer.draw_rotor_blades(sim)
        self.scene_renderer.draw_drone_body_box(sim)
        self.scene_renderer.draw_rotor_numbers(sim)
        self.scene_renderer.draw_thrust_arrows(sim)
        self.scene_renderer.draw_body_corner_numbers(sim, self.environment)
        
        # Render sensors and targets
        self.scene_renderer.draw_camera_fov(sim, self.environment)
        self.scene_renderer.draw_lookahead_target(sim)
        
        # Render accumulated camera chunks
        self.scene_renderer.draw_camera_chunks(self.terrain_scanner.camera_chunks)
    
    def reshape(self, width, height):
        """Handle window resize."""
        self.window_width, self.window_height = width, height
        self.ui_panels.update_window_size(width, height)
        self.camera_controller.setup_projection(width, height)
    
    # Legacy compatibility methods for terrain scanner
    def is_chunk_overlapping(self, pos1, pos2, fov=60, altitude=None):
        """Legacy compatibility method."""
        return self.terrain_scanner.is_chunk_overlapping(pos1, pos2, fov, altitude)
    
    def mask_overlapping_area(self, img, pos, existing_chunks, fov=60):
        """Legacy compatibility method."""
        return self.terrain_scanner.mask_overlapping_area(img, pos, existing_chunks, fov)
    
    def add_camera_chunk_with_partial_culling(self, img, pos, fov=60):
        """Legacy compatibility method."""
        return self.terrain_scanner.add_camera_chunk_with_partial_culling(img, pos, fov)
    
    def add_camera_chunk_with_culling(self, img, pos, fov=60):
        """Legacy compatibility method."""
        return self.terrain_scanner.add_camera_chunk_with_culling(img, pos, fov)
