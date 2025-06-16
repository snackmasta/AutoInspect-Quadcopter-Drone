import numpy as np
import itertools
from quadcopter_sim.simulation import QuadcopterSimulation

# Define ranges for PID gains (position and attitude)
KP_POS_RANGE = [0.1, 0.3, 0.5, 0.7, 1.0]
KD_POS_RANGE = [0.0, 1.0, 2.0, 3.0, 5.0]
KP_ATT_RANGE = [1.0, 2.0, 4.0, 6.0]
KD_ATT_RANGE = [0.0, 1.0, 2.0, 3.0]
KP_YAW_RANGE = [0.2, 0.5, 1.0]
KD_YAW_RANGE = [0.0, 0.5, 1.0]

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
    return total_error, crashed

def main():
    best_gains = None
    best_score = float('inf')
    for kp_pos, kd_pos, kp_att, kd_att, kp_yaw, kd_yaw in itertools.product(
        KP_POS_RANGE, KD_POS_RANGE, KP_ATT_RANGE, KD_ATT_RANGE, KP_YAW_RANGE, KD_YAW_RANGE):
        score, crashed = run_simulation(kp_pos, kd_pos, kp_att, kd_att, kp_yaw, kd_yaw)
        print(f"Tested: KP_POS={kp_pos}, KD_POS={kd_pos}, KP_ATT={kp_att}, KD_ATT={kd_att}, KP_YAW={kp_yaw}, KD_YAW={kd_yaw} | Score={score:.2f} | Crashed={crashed}")
        if not crashed and score < best_score:
            best_score = score
            best_gains = (kp_pos, kd_pos, kp_att, kd_att, kp_yaw, kd_yaw)
    if best_gains:
        print(f"\nBest gains: KP_POS={best_gains[0]}, KD_POS={best_gains[1]}, KP_ATT={best_gains[2]}, KD_ATT={best_gains[3]}, KP_YAW={best_gains[4]}, KD_YAW={best_gains[5]}")
        print(f"Best score: {best_score:.2f}")
    else:
        print("No stable configuration found.")

if __name__ == "__main__":
    main()
