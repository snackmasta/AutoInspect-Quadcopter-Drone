"""
Utility functions for quadcopter simulation.
"""
import numpy as np


def get_rotor_positions(state, arm_length):
    """
    Get world positions of the 4 rotors based on current state.
    
    Args:
        state: Current state [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
        arm_length: Arm length of the quadcopter
        
    Returns:
        Array of 4 rotor positions in world coordinates
    """
    x, y, z, _, _, _, roll, pitch, yaw, _, _, _ = state
    
    # Rotor offsets in body frame (swapped FR and RL)
    offsets = np.array([
        [-arm_length/2, arm_length/2, 0],   # FL (0)
        [-arm_length/2, -arm_length/2, 0],  # RL (was 3, now 1)
        [arm_length/2, -arm_length/2, 0],   # RR (2)
        [arm_length/2, arm_length/2, 0]     # FR (was 1, now 3)
    ])
    
    # Rotation matrix from body to world
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])
    
    world_offsets = offsets @ R.T
    return np.array([[x, y, z] + world_offsets[i] for i in range(4)])


def get_camera_image(state, environment, fov=60, res=32, offset=0.2):
    """
    Simulate a downward-facing camera at the bottom of the drone.
    
    Args:
        state: Current drone state
        environment: Environment object
        fov: Field of view in degrees
        res: Resolution of the image
        offset: Camera offset below drone center
        
    Returns:
        Heightmap (grayscale image) of terrain below
    """
    pos = state[:3].copy()
    pos[2] -= offset  # camera is below the drone
    
    half_fov = np.radians(fov / 2)
    size = np.tan(half_fov) * pos[2]
    
    xs = np.linspace(pos[0] - size, pos[0] + size, res)
    ys = np.linspace(pos[1] - size, pos[1] + size, res)
    
    img = np.zeros((res, res), dtype=np.float32)
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            img[j, i] = environment.contour_height(x, y)
    
    return img


def insert_hover_after_sharp_turns(waypoints, hover_steps=50, angle_threshold_deg=30):
    """
    Insert hover waypoints after sharp turns for stability.
    
    Args:
        waypoints: List of waypoint positions
        hover_steps: Number of hover steps to insert
        angle_threshold_deg: Angle threshold for detecting sharp turns
        
    Returns:
        Tuple of (new_waypoints, hover_indices_set)
    """
    new_wps = []
    hover_indices = set()
    
    for i in range(len(waypoints)):
        new_wps.append(waypoints[i])
        
        if 0 < i < len(waypoints) - 1:
            v1 = waypoints[i] - waypoints[i-1]
            v2 = waypoints[i+1] - waypoints[i]
            v1[2] = 0  # ignore z component for turn detection
            v2[2] = 0
            
            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)
            
            if norm1 > 1e-3 and norm2 > 1e-3:
                v1 /= norm1
                v2 /= norm2
                dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
                angle = np.arccos(dot) * 180 / np.pi
                
                if angle > angle_threshold_deg:
                    # Insert hover after this waypoint
                    for _ in range(hover_steps):
                        new_wps.append(waypoints[i].copy())
                        hover_indices.add(len(new_wps) - 1)
    
    return new_wps, hover_indices


def clamp_value(value, min_val, max_val):
    """Clamp a value between min and max."""
    return max(min_val, min(value, max_val))


def normalize_angle(angle):
    """Normalize angle to [-pi, pi] range."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def rotation_matrix_from_euler(roll, pitch, yaw):
    """Create rotation matrix from Euler angles."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])
