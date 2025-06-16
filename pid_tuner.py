# Advanced PID tuning using Bayesian Optimization (scikit-optimize)
# Requires: pip install scikit-optimize
import numpy as np
from quadcopter_sim.simulation import QuadcopterSimulation
from skopt import gp_minimize
from skopt.space import Real
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

SIM_TIME = 8.0  # seconds
DT = 0.01
TARGET = np.array([0.0, 0.0, 3.0])  # Hover at 3m

# Define safer, narrower search space for each gain
space = [
    Real(0.2, 0.7, name='KP_POS'),
    Real(0.5, 2.0, name='KD_POS'),
    Real(1.0, 4.0, name='KP_ATT'),
    Real(0.2, 1.5, name='KD_ATT'),
    Real(0.2, 1.0, name='KP_YAW'),
    Real(0.1, 1.0, name='KD_YAW'),
]

def set_pid_gains(kp_pos, kd_pos, kp_att, kd_att, kp_yaw, kd_yaw):
    import quadcopter_sim.position_controller as pc
    pc.KP_POS = kp_pos
    pc.KD_POS = kd_pos
    pc.KP_ATT = kp_att
    pc.KD_ATT = kd_att
    pc.KP_YAW = kp_yaw
    pc.KD_YAW = kd_yaw

def run_simulation(kp_pos, kd_pos, kp_att, kd_att, kp_yaw, kd_yaw):
    set_pid_gains(kp_pos, kd_pos, kp_att, kd_att, kp_yaw, kd_yaw)
    sim = QuadcopterSimulation()
    sim.reset()
    sim.use_pid = True
    total_error = 0.0
    crashed = False
    for t in np.arange(0, SIM_TIME, DT):
        sim.step(DT)
        pos = sim.state[:3]
        error = np.linalg.norm(pos - TARGET)
        total_error += error * DT
        if hasattr(sim, 'crashed') and sim.crashed:
            crashed = True
            break
    # Penalize crashes very heavily
    if crashed:
        total_error += 1e6
    return total_error, crashed

def objective(params):
    score, crashed = run_simulation(*params)
    print(f"Tested: KP_POS={params[0]:.3f}, KD_POS={params[1]:.3f}, KP_ATT={params[2]:.3f}, KD_ATT={params[3]:.3f}, KP_YAW={params[4]:.3f}, KD_YAW={params[5]:.3f} | Score={score:.2f} | Crashed={crashed}")
    return score

def main():
    print("Starting Bayesian Optimization for PID tuning...\n")
    res = gp_minimize(
        objective,
        space,
        n_calls=80,      # More evaluations for better results
        n_random_starts=20,  # More random points
        random_state=42,
        verbose=True
    )
    best = res.x
    print(f"\nBest gains found:")
    print(f"KP_POS={best[0]:.3f}, KD_POS={best[1]:.3f}, KP_ATT={best[2]:.3f}, KD_ATT={best[3]:.3f}, KP_YAW={best[4]:.3f}, KD_YAW={best[5]:.3f}")
    print(f"Best score: {res.fun:.2f}")

if __name__ == "__main__":
    main()
