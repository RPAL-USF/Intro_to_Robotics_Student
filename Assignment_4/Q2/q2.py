# q3_mujoco.py — STUDENT FILE (only implement the TODOs)
import os, sys, numpy as np
import mujoco.viewer
import random
import time
import csv
from scipy.interpolate import CubicSpline

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if ROOT not in sys.path:
    sys.path.append(ROOT)

from Core.simulation import Simulation
from Core.utils import random_joint_angles
from Assignment_2.Q6.q6 import FK

MODEL_PATH = "Scene/scene.xml"
JOINT_COUNT = 6

def smooth_path_cubic_spline(path, num_points=200):
    """
    Smooths a path using a 6-D interpolating cubic spline.
    
    Args:
        path (np.ndarray): The coarse path from RRT (shape: [N, 6]).
        num_points (int): The number of points for the final smoothed path.
        
    Returns:
        np.ndarray: The smoothed, dense path (shape: [num_points, 6]).
    """
    pass

# ---------- DRIVER ----------
def main():
    with Simulation(MODEL_PATH, show_viewer=True, realtime=True, seed=42) as sim:
        # --- Load and Process Path ---
        coarse_path = None
        
        print("Smoothing path with cubic spline...")
        smoothed_path = smooth_path_cubic_spline(coarse_path, num_points=200)

        # ---------- Stepping through the pass ----------
        print("Path is valid. Press Enter to step through the smoothed trajectory.")
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            for i, q in enumerate(smoothed_path):
                if not viewer.is_running():
                    break
                
                sim.set_first_n_qpos(JOINT_COUNT, q)
                viewer.sync()

                time.sleep(0.02) # A small delay for smooth animation
        

if __name__ == "__main__":
    try:
        main()
    except NotImplementedError as e:
        print("[Not Implemented] Finish the student functions:", e)
        sys.exit(1)