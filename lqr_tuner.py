import numpy as np
from quadcopter_sim.simulation import QuadcopterSimulation
from skopt import gp_minimize
from skopt.space import Real
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

SIM_TIME = 8.0  # seconds
DT = 0.02  # Match simulation step size for efficiency
TARGET = np.array([0.0, 0.0, 3.0])  # Hover at 3m

# Define search space for Q and R diagonal elements (6 for Q, 4 for R)
space = [
    Real(5, 100, name='Qx'),
    Real(5, 100, name='Qy'),
    Real(50, 500, name='Qz'),
    Real(1, 20, name='Qvx'),
    Real(1, 20, name='Qvy'),
    Real(10, 200, name='Qvz'),
    Real(0.01, 2, name='Rx'),
    Real(0.01, 2, name='Ry'),
    Real(0.01, 1, name='Rthrust'),
    Real(0.01, 2, name='Rz'),
]

def trajectory_error(sim):
    traj = np.array(sim.trajectory)
    waypoints = np.array(sim.waypoints)
    min_len = min(len(traj), len(waypoints))
    if min_len == 0:
        return float('inf')
    return np.mean(np.linalg.norm(traj[:min_len] - waypoints[:min_len], axis=1))

def run_simulation(Q_diag, R_diag):
    Q = np.diag(Q_diag)
    R = np.diag(R_diag)
    sim = QuadcopterSimulation()
    sim.use_pid = False
    sim.reset()
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
    print("Starting Bayesian Optimization for LQR tuning...\n")
    res = gp_minimize(
        objective,
        space,
        n_calls=100,  # Reasonable for efficiency
        n_random_starts=20,
        random_state=42,
        verbose=True
    )
    best = res.x
    print(f"\nBest Q: {best[:6]}")
    print(f"Best R: {best[6:]}")
    print(f"Best score: {res.fun:.2f}")

if __name__ == "__main__":
    main()
