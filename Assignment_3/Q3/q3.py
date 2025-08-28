# q3_mujoco.py — STUDENT FILE (only implement the TODOs)
import os, sys, numpy as np
import mujoco.viewer

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if ROOT not in sys.path:
    sys.path.append(ROOT)

from Core.simulation import Simulation
from Core.utils import random_joint_angles
from Assignment_2.Q6.q6 import FK

MODEL_PATH = "Scene/scene.xml"
JOINT_COUNT = 6

# ---------- DRIVER ----------
def main():
    rng = np.random.default_rng(42)
    sweep_qs = random_joint_angles(num_poses=1, num_joints=JOINT_COUNT, deg_range=60.0, rng=rng)

    with Simulation(MODEL_PATH, show_viewer=False, realtime=True, seed=0) as sim:
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            for q in sweep_qs:
                if not viewer.is_running(): 
                    break
                sim.set_first_n_qpos(JOINT_COUNT, q)
                coll = sim.has_collision()

                viewer.sync()

                input(f"--- Displaying pose. Press Enter to continue to the next pose ---")

if __name__ == "__main__":
    try:
        main()
    except NotImplementedError as e:
        print("[Not Implemented] Finish the student functions:", e)
        sys.exit(1)
