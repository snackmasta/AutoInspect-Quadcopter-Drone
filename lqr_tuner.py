import numpy as np
from quadcopter_sim.simulation import QuadcopterSimulation
from quadcopter_sim.main_trajectory import get_main_trajectory
from skopt import gp_minimize
from skopt.space import Real
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

SIM_TIME = 30.0  # seconds, increase to allow full trajectory
DT = 0.02  # Match simulation step size for efficiency
WAYPOINTS = get_main_trajectory()  # Use main trajectory for tuning

# Define search space for Q and R diagonal elements (6 for Q, 3 for R)
space = [
    Real(5, 100, name='Qx'),
    Real(5, 100, name='Qy'),
    Real(50, 500, name='Qz'),
    Real(1, 20, name='Qvx'),
    Real(1, 20, name='Qvy'),
    Real(10, 200, name='Qvz'),
    Real(0.01, 2, name='Rx'),
    Real(0.01, 2, name='Ry'),
    Real(0.05, 0.3, name='Rthrust'),  # narrowed range for Rthrust
]

def trajectory_error(sim):
    traj = np.array(sim.trajectory)
    waypoints = np.array(WAYPOINTS)
    velocities = np.diff(traj, axis=0) / DT
    osc_penalty = np.mean(np.linalg.norm(velocities, axis=1))
    min_len = min(len(traj), len(waypoints))
    if min_len == 0:
        return float('inf')
    return np.mean(np.linalg.norm(traj[:min_len] - waypoints[:min_len], axis=1)) + 0.5 * osc_penalty

def run_simulation(Q_diag, R_diag):
    Q = np.diag(Q_diag)
    R = np.diag(R_diag)
    sim = QuadcopterSimulation(Q=Q, R=R)
    sim.use_pid = False
    sim.reset()
    # Set initial state to first trajectory point after reset
    sim.state[:3] = np.array(WAYPOINTS[0])
    sim.state[3:6] = 0  # zero initial velocity
    # Overwrite waypoints with main trajectory
    sim.waypoints = [np.array(wp) for wp in WAYPOINTS]
    # Start at the first trajectory point
    sim.state[:3] = np.array(WAYPOINTS[0])
    steps = 0
    crashed = False
    while steps < int(SIM_TIME / DT):
        sim.step(DT)
        steps += 1
        if hasattr(sim, 'crashed') and sim.crashed:
            crashed = True
            break
    err = trajectory_error(sim)
    if crashed:
        err += 1e6
    return err, crashed

def objective(params):
    Q_diag = params[:6]
    R_diag = params[6:]
    score, crashed = run_simulation(Q_diag, R_diag)
    print(f"Tested: Q={Q_diag}, R={R_diag} | Score={score:.2f} | Crashed={crashed}")
    return score

def main():
    print("Starting Bayesian Optimization for LQR tuning on main trajectory...\n")
    res = gp_minimize(
        objective,
        space,
        n_calls=100,  # Start with 1 for quick test, increase for full tuning
        n_random_starts=1,
        random_state=42,
        verbose=True
    )
    best = res.x
    print(f"\nBest Q: {best[:6]}")
    print(f"Best R: {best[6:]}")
    print(f"Best score: {res.fun:.2f}")

if __name__ == "__main__":
    main()
