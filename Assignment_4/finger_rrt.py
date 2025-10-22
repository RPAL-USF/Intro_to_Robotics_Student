# finger_rrt.py

import os, sys, numpy as np
import mujoco.viewer as mjv
import mujoco
import random
import time
import math

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.append(ROOT)

from Core.simulation_hw4 import Simulation
from Core.utils import steer, is_valid_hand_config, sample_ik_biased, plan_path_segment

# --- CONSTANTS AND HARDCODED POSE ---
MODEL_PATH = os.path.join(ROOT, "Core", "Scene", "scene_HW4.xml")

ARM_JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
HAND_JOINT_NAMES = [
    "1", "0", "2", "3",    # Finger 1 (Index)
    "5", "4", "6", "7",    # Finger 2 (Middle)
    "9", "8", "10", "11",  # Finger 3 (Ring)
    "12", "13", "14", "15" # Thumb
]
HAND_JOINT_COUNT = len(HAND_JOINT_NAMES)

# HARDCODED STATIC POSE DATA (Unchanged)
Q_ARM_PALM_UP = np.array([
    1.9124011, -1.40915355, -2.22703827, -1.09140087, -1.57020288, -1.91239219
])


OBJECT_BODY_NAME = "obj1" 

# --- HAND RRT PLANNER (Updated to maintain fixed joints) ---

def rrt_plan_hand(sim, q_hand_start, 
                  q_hand_min, q_hand_max,
                  q_arm_static, obj_body_id, 
                  active_fingertip_body_ids, active_fingertip_site_ids, active_hand_dof_ids,
                  non_active_hand_joint_indices,
                  target_positions,
                  step=0.01, iters=100000, pos_tol=0.015):
    """
    RRT planning for the hand/finger joints with a task-space goal (target positions),
    only considering a subset of active joints/sites.
    """
    model = sim.model
    data_for_collision = mujoco.MjData(model)
    
    arm_joint_ids = sim.arm_joint_ids
    hand_joint_ids = sim.hand_joint_ids

    print(f"RRT Hand Planning: {HAND_JOINT_COUNT} total dimensions, {len(active_hand_dof_ids)} active DOFs, {iters} iterations.")
    print(f"Goal is {len(active_fingertip_site_ids)} fingertip sites reaching task-space goal within {pos_tol}m.")

    # Your assignment code goes here.
    #
    # You need to implement the RRT algorithm to find a path from 
    # q_hand_start to a configuration where the active fingertips 
    # (active_fingertip_site_ids) reach their corresponding 
    # target_positions within 'pos_tol'.
    #
    # Helper function to check goal (you can nest this inside):
    # def check_goal_achieved(q_hand_test, data_test) -> float:
    #   ... (computes max error for active sites) ...
    #
    # Remember to:
    # 1. Initialize the tree (e.g., nodes = [q_hand_start], parents = [-1]).
    # 2. Set up the collision data (e.g., static arm pose, object pose).
    # 3. Loop for 'iters' iterations:
    #    a. Sample a configuration 'qs'. Use sample_ik_biased().
    #       - You'll need to pick a q_base for this (e.g., random.choice(nodes)).
    #    b. Find the nearest neighbor 'qnear' in the tree to 'qs'.
    #    c. Steer from 'qnear' to 'qs' to get 'qnew' (use steer()).
    #    d. Clip 'qnew' to joint limits.
    #    e. CRITICAL: Enforce fixed joints: 
    #       qnew[non_active_hand_joint_indices] = qnear[non_active_hand_joint_indices]
    #    f. Check if 'qnew' is valid (use is_valid_hand_config()).
    #    g. If valid:
    #       i.   Add 'qnew' to the tree (nodes.append, parents.append).
    #       ii.  Check if 'qnew' achieves the goal (use your check_goal_achieved helper).
    #       iii. If goal achieved, reconstruct the path from 'parents' and return it.
    #
    # 4. If the loop finishes without finding a path, return None.


    print("RRT planner not implemented. Returning None.")
    return None

def smooth_path(path, sim, q_hand_min, q_hand_max,
                q_arm_static, obj_body_id, 
                active_fingertip_body_ids,
                non_active_hand_joint_indices):
    """
    Optional: Smooths and/or interpolates the raw RRT path.
    """
    if path is None:
        return None

    print("Path smoothing/interpolation not implemented. Returning raw path.")

    # Your assignment code could go here.
    #
    # To do this, you'll need a collision-checking setup, similar to the RRT.
    # model = sim.model
    # data_for_collision = mujoco.MjData(model)
    # arm_joint_ids = sim.arm_joint_ids
    # hand_joint_ids = sim.hand_joint_ids
    #
    # ... (set static object/arm pose in data_for_collision) ...
    #
    # You might want to implement:
    # 1.  Path Shortcutting:
    #     - Pick two random indices (i, j) in the path.
    #     - Check if a direct line between path[i] and path[j] is collision-free.
    #       (This requires checking multiple points along the line using is_valid_hand_config).
    #     - If it is, you can remove all nodes between i and j.
    #     - Repeat this N times.
    #
    # 2.  Path Interpolation (Densification):
    #     - The RRT path might have large steps.
    #     - Create a new, denser path.
    #     - For each segment (path[k], path[k+1]):
    #         - Generate N intermediate points using linear interpolation (e.g., np.linspace).
    #         - Check EACH intermediate point for collisions (is_valid_hand_config).
    #         - If any point is in collision, you might discard the segment or 
    #           just use the original, less-dense points.
    #
    # 3.  B-Spline Smoothing:
    #     - Fit a B-Spline to the path waypoints.
    #     - Sample points from the spline to create a new path.
    #     - Check all sampled points for collisions.
    #
    # Remember to handle non-active joints: interpolated/smoothed points
    # should still respect the fixed values for non_active_hand_joint_indices.
    #
    # For now, we just return the original path.

    return path

# --- DRIVER (Main function updated to choose fingers) ---

def main():
    # --- Initial Setup ---
    OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "outputs")
    os.makedirs(OUT_DIR, exist_ok=True)
    
    sim = Simulation(MODEL_PATH) 
    sim.load() 
    model = sim.model
    
    sim.ids_by_name(ARM_JOINT_NAMES, mujoco.mjtObj.mjOBJ_JOINT, 'arm')
    sim.actuators_for_joints('arm')
    
    full_hand_joint_ids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in HAND_JOINT_NAMES])
    sim.hand_joint_ids = full_hand_joint_ids 
    
    if -1 in full_hand_joint_ids:
        print("\nFATAL: One or more joints in HAND_JOINT_NAMES were not found in the model.")
        return
    
    sim.actuators_for_joints('hand')
    hand_act_ids = sim.hand_act_ids

    q_hand_min = model.jnt_range[full_hand_joint_ids, 0]
    q_hand_max = model.jnt_range[full_hand_joint_ids, 1]

    # --- Set Initial Arm and Object Pose ---
    print("Setting RRT starting state (Arm/Hand/Object placed)...")
    q_arm_static = Q_ARM_PALM_UP.copy()
    q_open_angles = np.zeros(HAND_JOINT_COUNT)
    q_open_angles[13] = 2.4 # Pre-set thumb joint
    
    sim.set_joint_positions(sim.arm_joint_ids, q_arm_static)
    sim.set_joint_positions(full_hand_joint_ids, q_open_angles)
    sim._set_ctrl_arm_qdes(q_arm_static) 
    
    mujoco.mj_forward(model, sim.data) 
    
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
    if site_id < 0: raise RuntimeError(f"Site not found: attachment_site")

    palm_surface_pos = sim.data.site_xpos[site_id].copy()
    object_start_pos = palm_surface_pos + np.array([0.0, 0.0, 0.08])
    
    obj_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, OBJECT_BODY_NAME)
    obj_jnt_adr = model.body_jntadr[obj_body_id]
    obj_qpos_adr = model.jnt_qposadr[obj_jnt_adr]

    sim.data.qpos[obj_qpos_adr : obj_qpos_adr + 3] = object_start_pos
    sim.data.qpos[obj_qpos_adr + 3 : obj_qpos_adr + 7] = [1, 0, 0, 0] 

    temp_data = mujoco.MjData(model)
    temp_data.qpos[:] = sim.data.qpos.copy() 
    
    for _ in range(150): 
        temp_data.ctrl[sim.arm_act_ids] = q_arm_static
        for i, act_id in enumerate(hand_act_ids):
            temp_data.ctrl[act_id] = q_open_angles[i]
        mujoco.mj_step(model, temp_data)
        
    sim.data.qpos[:] = temp_data.qpos[:]
    mujoco.mj_forward(model, sim.data)
    print("Initial state established (Arm/Hand/Object settled).")

    # --- SEQUENTIAL PLANNING ---
    all_paths = []
    
    # === SEGMENT 1: PLAN ALL 4 FINGERS (IN PARALLEL) ===
    print("\n--- Starting SEGMENT 1: Grasp Motion ---")
    current_q_start_1 = sim.q_slice(full_hand_joint_ids).copy() 

    # --- Plan for THUMB (Segment 1) ---
    print("\nStarting Plan 1: THUMB (Segment 1)")
    fingers_segment_1 = ['thumb']
    goals_segment_1 = { 'thumb': [-0.04, 0, 0] } # Grasp Goal
    d1 = plan_path_segment(sim, current_q_start_1, fingers_segment_1, goals_segment_1, 
                              q_arm_static, full_hand_joint_ids, q_hand_min, q_hand_max)
    path1 = None
    if d1:
        path1 = rrt_plan_hand(sim, d1["q_hand_start"], q_hand_min, q_hand_max,
                            q_arm_static, obj_body_id, 
                            d1["active_fingertip_body_ids"], d1["active_fingertip_site_ids"], d1["active_hand_dof_ids"],
                            d1["non_active_hand_joint_indices"],
                            d1["target_positions_for_active_sites"])

    # --- Plan for INDEX (Segment 1) ---
    print("\nStarting Plan 2: INDEX (Segment 1)")
    fingers_segment_2 = ['index'] 
    goals_segment_2 = { 'index': [0.0, -0.04, 0.0] } # Grasp Goal
    d2 = plan_path_segment(sim, current_q_start_1, fingers_segment_2, goals_segment_2, 
                              q_arm_static, full_hand_joint_ids, q_hand_min, q_hand_max)
    path2 = None
    if d2:
        path2 = rrt_plan_hand(sim, d2["q_hand_start"], q_hand_min, q_hand_max,
                            q_arm_static, obj_body_id, 
                            d2["active_fingertip_body_ids"], d2["active_fingertip_site_ids"], d2["active_hand_dof_ids"],
                            d2["non_active_hand_joint_indices"],
                            d2["target_positions_for_active_sites"])
    
    # --- Plan for MIDDLE (Segment 1) ---
    print("\nStarting Plan 3: MIDDLE (Segment 1)")
    fingers_segment_3 = ['middle'] 
    goals_segment_3 = { 'middle': [0.04, 0.0, 0.0] } # Grasp Goal
    d3 = plan_path_segment(sim, current_q_start_1, fingers_segment_3, goals_segment_3, 
                              q_arm_static, full_hand_joint_ids, q_hand_min, q_hand_max)
    path3 = None
    if d3:
        path3 = rrt_plan_hand(sim, d3["q_hand_start"], q_hand_min, q_hand_max,
                            q_arm_static, obj_body_id, 
                            d3["active_fingertip_body_ids"], d3["active_fingertip_site_ids"], d3["active_hand_dof_ids"],
                            d3["non_active_hand_joint_indices"],
                            d3["target_positions_for_active_sites"])
    
    # --- Plan for RING (Segment 1) ---
    print("\nStarting Plan 4: RING (Segment 1)")
    fingers_segment_4 = ['ring'] 
    goals_segment_4 = { 'ring': [0.0, 0.04, 0.0] } # Grasp Goal
    d4 = plan_path_segment(sim, current_q_start_1, fingers_segment_4, goals_segment_4, 
                              q_arm_static, full_hand_joint_ids, q_hand_min, q_hand_max)
    path4 = None
    if d4:
        path4 = rrt_plan_hand(sim, d4["q_hand_start"], q_hand_min, q_hand_max,
                            q_arm_static, obj_body_id, 
                            d4["active_fingertip_body_ids"], d4["active_fingertip_site_ids"], d4["active_hand_dof_ids"],
                            d4["non_active_hand_joint_indices"],
                            d4["target_positions_for_active_sites"])

    # --- MERGE Parallel Paths (Segment 1) ---
    path_merged_1 = None
    if path1 is None or path2 is None or path3 is None or path4 is None:
        print("Planning for one or more fingers (Segment 1) failed. Aborting.")
        return
    else:
        print("Merging all four finger paths (Segment 1)...")
        
        # Helper function for resampling
        def resample_path_with_padding(path, new_length):
            if len(path) == new_length: return path
            n_joints = path.shape[1]
            if len(path) > new_length:
                resampled = np.zeros((new_length, n_joints))
                xp = np.linspace(0, 1, len(path))
                x_new = np.linspace(0, 1, new_length)
                for i in range(n_joints):
                    resampled[:, i] = np.interp(x_new, xp, path[:, i])
                return resampled
            else:
                n_padding = new_length - len(path)
                initial_pose = path[0] 
                padding = np.tile(initial_pose, (n_padding, 1))
                return np.vstack([padding, path])

        max_len_1 = max(len(path1), len(path2), len(path3), len(path4))
        print(f"All paths (Seg 1) will be resampled to max length: {max_len_1}")
        
        path1_resampled = resample_path_with_padding(path1, max_len_1)
        path2_resampled = resample_path_with_padding(path2, max_len_1)
        path3_resampled = resample_path_with_padding(path3, max_len_1)
        path4_resampled = resample_path_with_padding(path4, max_len_1)

        q_start_1 = current_q_start_1
        delta1 = path1_resampled - q_start_1
        delta2 = path2_resampled - q_start_1
        delta3 = path3_resampled - q_start_1
        delta4 = path4_resampled - q_start_1
        
        path_merged_1 = q_start_1 + delta1 + delta2 + delta3 + delta4
        all_paths.append(path_merged_1) # Append the first full path
        
    # --- END OF SEGMENT 1 ---


    # === SEGMENT 2: PLAN ALL 4 FINGERS AGAIN (FROM NEW START) ===
    if path_merged_1 is not None:
        print("\n--- Starting SEGMENT 2: Release Motion ---")
        # The new start pose is the *last* pose of the first merged path
        current_q_start_2 = path_merged_1[-1].copy()

        # --- Plan for THUMB (Segment 2) ---
        print("\nStarting Plan 5: THUMB (Segment 2)")
        fingers_segment_5 = ['thumb']
        # Define new "release" goals (e.g., further out, or back to 0)
        goals_segment_5 = { 'thumb': [-0.02, -0.04, 0.0] } # Release Goal
        d5 = plan_path_segment(sim, current_q_start_2, fingers_segment_5, goals_segment_5, 
                                  q_arm_static, full_hand_joint_ids, q_hand_min, q_hand_max)
        path5 = None
        if d5:
            path5 = rrt_plan_hand(sim, d5["q_hand_start"], q_hand_min, q_hand_max,
                                q_arm_static, obj_body_id, 
                                d5["active_fingertip_body_ids"], d5["active_fingertip_site_ids"], d5["active_hand_dof_ids"],
                                d5["non_active_hand_joint_indices"],
                                d5["target_positions_for_active_sites"], touch=True)

        # --- Plan for INDEX (Segment 2) ---
        print("\nStarting Plan 6: INDEX (Segment 2)")
        fingers_segment_6 = ['index'] 
        goals_segment_6 = { 'index': [0.04, -0.02, 0.0] } # Release Goal
        d6 = plan_path_segment(sim, current_q_start_2, fingers_segment_6, goals_segment_6, 
                                  q_arm_static, full_hand_joint_ids, q_hand_min, q_hand_max)
        path6 = None
        if d6:
            path6 = rrt_plan_hand(sim, d6["q_hand_start"], q_hand_min, q_hand_max,
                                q_arm_static, obj_body_id, 
                                d6["active_fingertip_body_ids"], d6["active_fingertip_site_ids"], d6["active_hand_dof_ids"],
                                d6["non_active_hand_joint_indices"],
                                d6["target_positions_for_active_sites"], touch=True)
        
        # --- Plan for MIDDLE (Segment 2) ---
        print("\nStarting Plan 7: MIDDLE (Segment 2)")
        fingers_segment_7 = ['middle'] 
        goals_segment_7 = { 'middle': [0.02, 0.04, 0.0] } # Release Goal
        d7 = plan_path_segment(sim, current_q_start_2, fingers_segment_7, goals_segment_7, 
                                  q_arm_static, full_hand_joint_ids, q_hand_min, q_hand_max)
        path7 = None
        if d7:
            path7 = rrt_plan_hand(sim, d7["q_hand_start"], q_hand_min, q_hand_max,
                                q_arm_static, obj_body_id, 
                                d7["active_fingertip_body_ids"], d7["active_fingertip_site_ids"], d7["active_hand_dof_ids"],
                                d7["non_active_hand_joint_indices"],
                                d7["target_positions_for_active_sites"], touch=True)
        
        # --- Plan for RING (Segment 2) ---
        print("\nStarting Plan 8: RING (Segment 2)")
        fingers_segment_8 = ['ring'] 
        goals_segment_8 = { 'ring': [-0.04, 0.02, 0.0] } # Release Goal
        d8 = plan_path_segment(sim, current_q_start_2, fingers_segment_8, goals_segment_8, 
                                  q_arm_static, full_hand_joint_ids, q_hand_min, q_hand_max)
        path8 = None
        if d8:
            path8 = rrt_plan_hand(sim, d8["q_hand_start"], q_hand_min, q_hand_max,
                                q_arm_static, obj_body_id, 
                                d8["active_fingertip_body_ids"], d8["active_fingertip_site_ids"], d8["active_hand_dof_ids"],
                                d8["non_active_hand_joint_indices"],
                                d8["target_positions_for_active_sites"], touch=True)

        # --- MERGE Parallel Paths (Segment 2) ---
        if path5 is None or path6 is None or path7 is None or path8 is None:
            print("Planning for one or more fingers (Segment 2) failed. Executing Seg 1 only.")
        else:
            print("Merging all four finger paths (Segment 2)...")
            
            # We can reuse the helper function from Segment 1
            max_len_2 = max(len(path5), len(path6), len(path7), len(path8))
            print(f"All paths (Seg 2) will be resampled to max length: {max_len_2}")
            
            path5_resampled = resample_path_with_padding(path5, max_len_2)
            path6_resampled = resample_path_with_padding(path6, max_len_2)
            path7_resampled = resample_path_with_padding(path7, max_len_2)
            path8_resampled = resample_path_with_padding(path8, max_len_2)

            q_start_2 = current_q_start_2
            delta5 = path5_resampled - q_start_2
            delta6 = path6_resampled - q_start_2
            delta7 = path7_resampled - q_start_2
            delta8 = path8_resampled - q_start_2
            
            path_merged_2 = q_start_2 + delta5 + delta6 + delta7 + delta8
            # Append the second path, skipping the first frame to avoid a pause
            all_paths.append(path_merged_2[1:]) 
            
    # --- END OF SEGMENT 2 ---


    # --- Combine paths into a single trajectory ---
    if not all_paths:
        print("No paths were successfully planned.")
        return
        
    full_path = np.vstack(all_paths)

    # --- Execution ---

    with mjv.launch_passive(model, sim.data) as viewer:
        if full_path is not None and len(full_path) > 0:
            print(f"\nExecuting combined path with {len(full_path)} waypoints!")
            
            for q_hand_target in full_path:
                if not viewer.is_running():
                    break
                
                steps_per_waypoint = 10 
                for _ in range(steps_per_waypoint):
                    if not viewer.is_running():
                        break
                    
                    sim._set_ctrl_arm_qdes(q_arm_static)
                    for i, act_id in enumerate(hand_act_ids):
                        sim.data.ctrl[act_id] = q_hand_target[i]

                    mujoco.mj_step(model, sim.data)
                    viewer.sync()
            
            print("\nPath finished. Holding final pose.")
            final_q_hand = full_path[-1]
            while viewer.is_running():
                sim._set_ctrl_arm_qdes(q_arm_static)
                for i, act_id in enumerate(hand_act_ids):
                    sim.data.ctrl[act_id] = final_q_hand[i]
                mujoco.mj_step(model, sim.data)
                viewer.sync()
                
        else:
            print("RRT failed. Showing established initial configuration.")
            while viewer.is_running():
                sim._set_ctrl_arm_qdes(q_arm_static)
                mujoco.mj_step(model, sim.data)
                viewer.sync()

if __name__ == "__main__":
    main()
