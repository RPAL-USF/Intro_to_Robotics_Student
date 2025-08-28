import mujoco.viewer
import os, sys, numpy as np

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
    DH_UR5 = np.array([
        [0.0, 0.1625,  0.0,       np.pi/2],
        [0.0, 0.0,    -0.425,     0.0],
        [0.0, 0.0,    -0.3922,    0.0],
        [0.0, 0.1333,  0.0,       np.pi/2],
        [0.0, 0.0997,  0.0,      -np.pi/2],
        [0.0, 0.0996,  0.0,       0.0],
    ], dtype=float)

    rng = np.random.default_rng(42)
    joint_sets = random_joint_angles(num_poses=10, num_joints=JOINT_COUNT, deg_range=60.0, rng=rng)

    rows = []
    with Simulation(MODEL_PATH, show_viewer=False, realtime=True, seed=0) as sim:
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            print("Comparing FK against MuJoCo site pose...")
            for q in joint_sets:
                if not viewer.is_running(): 
                    break
                sim.set_first_n_qpos(JOINT_COUNT, q)


                viewer.sync()

                input(f"--- Displaying pose. Press Enter to continue to the next pose. ---")
        
if __name__ == "__main__":
    try:
        main()
    except NotImplementedError as e:
        print("[Not Implemented] Finish the student functions:", e)
        sys.exit(1)
