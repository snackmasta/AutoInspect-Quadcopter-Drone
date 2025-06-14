import numpy as np
import time
from quadcopter_sim.simulation import QuadcopterSimulation

# Example: parameters to tune (expand as needed)
PARAM_GRID = {
    'lqr_gain': [0.8, 1.0, 1.2],
    'max_rpm': [4000, 5000, 6000],
}

# Number of iterations per parameter set
N_ITER = 3

# Trajectory error metric

def trajectory_error(sim):
    traj = np.array(sim.trajectory)
    waypoints = np.array(sim.waypoints)
    min_len = min(len(traj), len(waypoints))
    if min_len == 0:
        return float('inf')
    return np.mean(np.linalg.norm(traj[:min_len] - waypoints[:min_len], axis=1))


def run_simulation(lqr_gain, max_rpm):
    sim = QuadcopterSimulation()
    sim.max_rpm = max_rpm
    if hasattr(sim, 'lqr_gain'):
        sim.lqr_gain = lqr_gain
    sim.reset()
    steps = 0
    crashed = False
    log = []
    while steps < 2000:
        sim.step(0.02)
        steps += 1
        # Log detailed state for first 20 steps and at crash
        if steps < 20 or (hasattr(sim, 'crashed') and sim.crashed):
            log.append({
                'step': steps,
                'z': sim.state[2],
                'vz': sim.state[5],
                'thrust': getattr(sim, 'rotor_thrusts', None),
                'rotor_speeds': sim.rotor_speeds.copy(),
                'target': sim.waypoints[sim.wp_index] if hasattr(sim, 'wp_index') and hasattr(sim, 'waypoints') else None,
                'crashed': getattr(sim, 'crashed', False)
            })
        if hasattr(sim, 'crashed') and sim.crashed:
            crashed = True
            break
    err = trajectory_error(sim)
    # Print log if crashed
    if crashed:
        print("Detailed log for crash:")
        for entry in log:
            print(entry)
    return err, crashed


def main():
    best_params = None
    best_error = float('inf')
    for lqr_gain in PARAM_GRID['lqr_gain']:
        for max_rpm in PARAM_GRID['max_rpm']:
            errors = []
            crashes = 0
            for i in range(N_ITER):
                err, crashed = run_simulation(lqr_gain, max_rpm)
                errors.append(err)
                if crashed:
                    crashes += 1
            avg_err = np.mean(errors)
            print(f"lqr_gain={lqr_gain}, max_rpm={max_rpm} | avg_err={avg_err:.3f} | crashes={crashes}/{N_ITER}")
            if avg_err < best_error and crashes == 0:
                best_error = avg_err
                best_params = (lqr_gain, max_rpm)
    print("Best params:", best_params, "with avg error:", best_error)

if __name__ == "__main__":
    main()
