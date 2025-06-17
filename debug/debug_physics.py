"""
Quadcopter Physics Debug Entry Point
"""
import sys
import os
# Add parent directory to path so we can import quadcopter_sim
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from quadcopter_sim.simulation import QuadcopterSimulation
import numpy as np
import time

def print_state(sim, prev_state=None):
    s = sim.state
    # Unpack state
    x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz = s
    # Calculate acceleration (if previous state is available)
    if prev_state is not None:
        ax = (vx - prev_state[3]) / sim.dt
        ay = (vy - prev_state[4]) / sim.dt
        az = (vz - prev_state[5]) / sim.dt
    else:
        ax = ay = az = 0.0
    # Rotor speeds and thrusts
    rotor_speeds = sim.rotor_speeds
    thrusts = getattr(sim, 'rotor_thrusts', np.zeros(4))
    # Print all key parameters
    print(f"Pos: [{x:.3f}, {y:.3f}, {z:.3f}] m | Vel: [{vx:.3f}, {vy:.3f}, {vz:.3f}] m/s | Acc: [{ax:.3f}, {ay:.3f}, {az:.3f}] m/s²")
    print(f"Angles: Roll={np.degrees(roll):.2f}°, Pitch={np.degrees(pitch):.2f}°, Yaw={np.degrees(yaw):.2f}°")
    print(f"Ang Vel: [{wx:.3f}, {wy:.3f}, {wz:.3f}] rad/s")
    print(f"Rotor RPMs: {[int(rpm) for rpm in rotor_speeds]}")
    print(f"Rotor Thrusts: {[f'{t:.3f}' for t in thrusts]} N")
    # Plausibility checks
    if z < -0.01:
        print("[WARN] Altitude below ground!")
    if np.any(np.abs([vx, vy, vz]) > 50):
        print("[WARN] Unphysically high velocity detected!")
    if np.any(np.abs([ax, ay, az]) > 200):
        print("[WARN] Unphysically high acceleration detected!")
    if np.any(rotor_speeds < 0):
        print("[WARN] Negative rotor RPM!")
    if np.any(thrusts < 0):
        print("[WARN] Negative rotor thrust!")
    print("---")

def main():
    sim = QuadcopterSimulation()
    print("Physics debug mode. Press Ctrl+C to exit.")
    step = 0
    prev_state = None
    try:
        while True:
            sim.step(sim.dt)
            print(f"Step {step}")
            print_state(sim, prev_state)
            prev_state = sim.state.copy()
            step += 1
            time.sleep(sim.dt)
    except KeyboardInterrupt:
        print("\nExiting physics debug mode.")

if __name__ == "__main__":
    main()
