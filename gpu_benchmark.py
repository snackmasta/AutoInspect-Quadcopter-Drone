import numpy as np
import time

try:
    import cupy as cp
    print("CuPy available - GPU acceleration enabled")
    USE_GPU = True
except ImportError:
    print("CuPy not available - using CPU only")
    USE_GPU = False

def benchmark_array_operations():
    """Benchmark array operations to show GPU vs CPU performance difference"""
    print("\n=== Performance Benchmark ===")
    
    # Test array size
    size = 1000000
    iterations = 100
    
    print(f"Array size: {size:,}")
    print(f"Iterations: {iterations}")
    
    # CPU benchmark
    print("\n--- CPU Performance ---")
    start_time = time.time()
    for i in range(iterations):
        a = np.random.randn(size)
        b = np.random.randn(size)
        c = a * b + np.sin(a) * np.cos(b)
        result = np.sum(c)
    cpu_time = time.time() - start_time
    print(f"CPU time: {cpu_time:.4f} seconds")
    
    if USE_GPU:
        # GPU benchmark
        print("\n--- GPU Performance ---")
        start_time = time.time()
        for i in range(iterations):
            a = cp.random.randn(size)
            b = cp.random.randn(size)
            c = a * b + cp.sin(a) * cp.cos(b)
            result = cp.sum(c)
            cp.cuda.Stream.null.synchronize()  # Wait for GPU to finish
        gpu_time = time.time() - start_time
        print(f"GPU time: {gpu_time:.4f} seconds")
        
        speedup = cpu_time / gpu_time
        print(f"\nSpeedup: {speedup:.2f}x faster on GPU")
    else:
        print("\nGPU benchmark skipped (CuPy not available)")

def test_drone_physics():
    """Test the specific operations used in the drone simulation"""
    print("\n=== Drone Physics Operations Test ===")
    
    # Simulate drone state operations
    state_size = 6  # position (3) + velocity (3)
    
    if USE_GPU:
        xp = cp
        print("Using GPU arrays (CuPy)")
    else:
        xp = np
        print("Using CPU arrays (NumPy)")
    
    # Create drone state
    state = xp.array([0, 0, 2, 0, 0, 0], dtype=float)
    target = xp.array([2, 2, 3], dtype=float)
    
    print(f"Initial state: {state}")
    print(f"Target: {target}")
    
    # Simulate control calculations
    pos, vel = state[:3], state[3:]
    kp, kd = 6.0, 4.0
    g, m, dt = 9.81, 0.5, 0.02
    
    acc_des = kp * (target - pos) - kd * vel
    acc_des = xp.append(acc_des[:2], acc_des[2] + g)
    
    u = m * acc_des
    acc = u / m
    acc = xp.append(acc[:2], acc[2] - g)
    
    vel_new = vel + acc * dt
    pos_new = pos + vel_new * dt
    
    state_new = xp.hstack((pos_new, vel_new))
    
    print(f"New state: {state_new}")
    print(f"Device: {'GPU' if USE_GPU and hasattr(state_new, 'device') else 'CPU'}")

if __name__ == "__main__":
    benchmark_array_operations()
    test_drone_physics()
