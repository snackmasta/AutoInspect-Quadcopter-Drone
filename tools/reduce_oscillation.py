import sys
import os
# Add parent directory to path so we can import quadcopter_sim
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import itertools
from quadcopter_sim.simulation import QuadcopterSimulation

def set_pid_gains(kp_pos, kd_pos, kp_att, kd_att, kp_yaw, kd_yaw):
    import quadcopter_sim.controllers.position_controller as pc
    pc.KP_POS = kp_pos
    pc.KD_POS = kd_pos
    pc.KP_ATT = kp_att
    pc.KD_ATT = kd_att
    pc.KP_YAW = kp_yaw
    pc.KD_YAW = kd_yaw

def oscillation_score(errors):
    # Penalize overshoot and number of zero-crossings (oscillation)
    overshoot = np.max(np.abs(errors - np.mean(errors)))
    zero_crossings = np.sum(np.diff(np.sign(errors - np.mean(errors))) != 0)
    return overshoot + 0.5 * zero_crossings

def run_simulation(kp_pos, kd_pos, kp_att, kd_att, kp_yaw, kd_yaw):
    set_pid_gains(kp_pos, kd_pos, kp_att, kd_att, kp_yaw, kd_yaw)
    sim = QuadcopterSimulation()
    sim.reset()
    sim.use_pid = True
    errors = []
    crashed = False
    TARGET = np.array([0.0, 0.0, 3.0])
    SIM_TIME = 8.0
    DT = 0.01
    for t in np.arange(0, SIM_TIME, DT):
        sim.step(DT)
        pos = sim.state[:3]
        error = np.linalg.norm(pos - TARGET)
        errors.append(error)
        if hasattr(sim, 'crashed') and sim.crashed:
            crashed = True
            break
    errors = np.array(errors)
    score = oscillation_score(errors)
    return score, crashed

# Focus search on KD_POS, KD_ATT, KD_YAW (damping)
KP_POS = 0.8
KP_ATT = 6.0
KP_YAW = 1.2
KD_POS_RANGE = [2.0, 3.0, 4.0, 5.0, 6.0]
KD_ATT_RANGE = [2.0, 3.0, 4.0, 5.0, 6.0]
KD_YAW_RANGE = [0.5, 1.0, 1.5, 2.0]

best_score = float('inf')
best_gains = None
for kd_pos, kd_att, kd_yaw in itertools.product(KD_POS_RANGE, KD_ATT_RANGE, KD_YAW_RANGE):
    score, crashed = run_simulation(KP_POS, kd_pos, KP_ATT, kd_att, KP_YAW, kd_yaw)
    print(f"Tested: KD_POS={kd_pos}, KD_ATT={kd_att}, KD_YAW={kd_yaw} | Score={score:.2f} | Crashed={crashed}")
    if not crashed and score < best_score:
        best_score = score
        best_gains = (kd_pos, kd_att, kd_yaw)
if best_gains:
    print(f"\nBest damping gains: KD_POS={best_gains[0]}, KD_ATT={best_gains[1]}, KD_YAW={best_gains[2]}")
    print(f"Best oscillation score: {best_score:.2f}")
else:
    print("No stable configuration found.")
