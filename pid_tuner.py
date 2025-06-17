import numpy as np
import itertools
import concurrent.futures
import random
from quadcopter_sim.simulation import QuadcopterSimulation
import sys
import subprocess
try:
    from skopt import Optimizer
except ImportError:
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'scikit-optimize'])
    from skopt import Optimizer

# --- User options ---
SEARCH_METHOD = 'random'  # 'grid' or 'random'
N_RANDOM_SAMPLES = 100    # Used if SEARCH_METHOD == 'random'
SCORING_METRIC = 'total'  # 'total', 'max', or 'rms'
N_WORKERS = 4             # Number of parallel workers

# Define ranges for PID gains (position and attitude)
KP_POS_RANGE = [0.1, 0.3, 0.5, 0.7, 1.0]
KD_POS_RANGE = [0.0, 1.0, 2.0, 3.0, 5.0]
KP_ATT_RANGE = [1.0, 2.0, 4.0, 6.0]
KD_ATT_RANGE = [0.0, 1.0, 2.0, 3.0]
KP_YAW_RANGE = [0.2, 0.5, 1.0]
KD_YAW_RANGE = [0.0, 0.5, 1.0]

# Define bounds for PID gains (for skopt)
BOUNDS = [
    (min(KP_POS_RANGE), max(KP_POS_RANGE)),
    (min(KD_POS_RANGE), max(KD_POS_RANGE)),
    (min(KP_ATT_RANGE), max(KP_ATT_RANGE)),
    (min(KD_ATT_RANGE), max(KD_ATT_RANGE)),
    (min(KP_YAW_RANGE), max(KP_YAW_RANGE)),
    (min(KD_YAW_RANGE), max(KD_YAW_RANGE)),
]
N_BAYES_ITER = 50  # Number of Bayesian optimization steps

SIM_TIME = 8.0  # seconds
DT = 0.01
TARGET = np.array([0.0, 0.0, 3.0])  # Hover at 3m


def set_pid_gains(kp_pos, kd_pos, kp_att, kd_att, kp_yaw, kd_yaw):
    import quadcopter_sim.position_controller as pc
    pc.KP_POS = kp_pos
    pc.KD_POS = kd_pos
    pc.KP_ATT = kp_att
    pc.KD_ATT = kd_att
    pc.KP_YAW = kp_yaw
    pc.KD_YAW = kd_yaw

def score_errors(errors, metric):
    if metric == 'total':
        return np.sum(errors)
    elif metric == 'max':
        return np.max(errors)
    elif metric == 'rms':
        return np.sqrt(np.mean(errors ** 2))
    else:
        raise ValueError('Unknown metric')

def run_simulation(kp_pos, kd_pos, kp_att, kd_att, kp_yaw, kd_yaw):
    set_pid_gains(kp_pos, kd_pos, kp_att, kd_att, kp_yaw, kd_yaw)
    sim = QuadcopterSimulation()
    sim.reset()
    sim.use_pid = True
    errors = []
    crashed = False
    for t in np.arange(0, SIM_TIME, DT):
        sim.step(DT)
        pos = sim.state[:3]
        error = np.linalg.norm(pos - TARGET)
        errors.append(error)
        if hasattr(sim, 'crashed') and sim.crashed:
            crashed = True
            break
    errors = np.array(errors)
    score = score_errors(errors, SCORING_METRIC)
    return score, crashed

# Helper to sample random gains
def sample_random_gains():
    return (
        random.choice(KP_POS_RANGE),
        random.choice(KD_POS_RANGE),
        random.choice(KP_ATT_RANGE),
        random.choice(KD_ATT_RANGE),
        random.choice(KP_YAW_RANGE),
        random.choice(KD_YAW_RANGE),
    )

def main():
    best_gains = None
    best_score = float('inf')
    results = []
    optimizer = Optimizer(BOUNDS, random_state=42, acq_func="EI")
    for i in range(N_BAYES_ITER):
        next_gains = optimizer.ask()
        # Round to nearest in-range value for each gain
        def snap(val, allowed):
            return min(allowed, key=lambda x: abs(x - val))
        gains = [
            snap(next_gains[0], KP_POS_RANGE),
            snap(next_gains[1], KD_POS_RANGE),
            snap(next_gains[2], KP_ATT_RANGE),
            snap(next_gains[3], KD_ATT_RANGE),
            snap(next_gains[4], KP_YAW_RANGE),
            snap(next_gains[5], KD_YAW_RANGE),
        ]
        score, crashed = run_simulation(*gains)
        print(f"[BO] Iter {i+1}/{N_BAYES_ITER}: Gains={gains} | Score={score:.2f} | Crashed={crashed}")
        # Penalize crashed runs
        y = score if not crashed else 1e6
        optimizer.tell(next_gains, y)
        if not crashed and score < best_score:
            best_score = score
            best_gains = gains
    if best_gains:
        print(f"\nBest gains: KP_POS={best_gains[0]}, KD_POS={best_gains[1]}, KP_ATT={best_gains[2]}, KD_ATT={best_gains[3]}, KP_YAW={best_gains[4]}, KD_YAW={best_gains[5]}")
        print(f"Best score: {best_score:.2f} (metric: {SCORING_METRIC})")
    else:
        print("No stable configuration found.")

def test_score():
    """Test and print the score for a specific set of PID gains."""
    # These should match the values you want to test
    kp_pos = 0.8
    kd_pos = 5.0
    kp_att = 6.0
    kd_att = 4.0
    kp_yaw = 1.2
    kd_yaw = 1.0
    score, crashed = run_simulation(kp_pos, kd_pos, kp_att, kd_att, kp_yaw, kd_yaw)
    print(f"Tested gains: KP_POS={kp_pos}, KD_POS={kd_pos}, KP_ATT={kp_att}, KD_ATT={kd_att}, KP_YAW={kp_yaw}, KD_YAW={kd_yaw}")
    print(f"Score: {score:.2f} | Crashed: {crashed}")

if __name__ == "__main__":
    mode = input("Type 'test' to test a specific PID score, or 'tune' to run optimization: ").strip().lower()
    if mode == 'test':
        test_score()
    else:
        main()
