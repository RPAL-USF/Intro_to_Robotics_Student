# q3_mujoco.py — STUDENT FILE (only implement the TODOs)
import os, sys, numpy as np
import mujoco.viewer
import random
import time
import csv

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if ROOT not in sys.path:
    sys.path.append(ROOT)

from Core.simulation import Simulation
from Core.utils import random_joint_angles
from Assignment_2.Q6.q6 import FK

MODEL_PATH = "Scene/scene.xml"
JOINT_COUNT = 6


def rrt_plan(model, data, joint_ids, q_start, q_goal, qmin, qmax,
             step=0.15, iters=6000, goal_tol=0.05, goal_bias=0.15):
    return None

# ---------- DRIVER ----------
def main():
    with Simulation(MODEL_PATH, show_viewer=True, realtime=True, seed=42) as sim:
        # --- Setup ---
        data_for_collision = mujoco.MjData(sim.model)
        joint_names = [f"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        joint_ids = [mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names]
        qmin = sim.model.jnt_range[joint_ids, 0]
        qmax = sim.model.jnt_range[joint_ids, 1]
        q_start = sim.q_slice(joint_ids)
        q_goal = np.clip(q_start + np.array([0.0, -1.6, -1.2, 0.2, 0.0, 0.0], float), qmin, qmax)
        sim.set_first_n_qpos(JOINT_COUNT, q_start)

        # --- Planning ---
        path, _, _ = rrt_plan(sim.model, data_for_collision, joint_ids,q_start, q_goal, qmin, qmax)

        # --- Stepping trough the path ---
        if path is not None:
            with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
                # Animate the planned path
                for i, q in enumerate(path):
                    if not viewer.is_running():
                        break
                    
                    sim.set_first_n_qpos(JOINT_COUNT, q)
                    viewer.sync()
                    time.sleep(0.05)
        else:
            with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
                while viewer.is_running():
                    viewer.sync()

if __name__ == "__main__":
    try:
        main()
    except NotImplementedError as e:
        print("[Not Implemented] Finish the student functions:", e)
        sys.exit(1)