import math
import numpy as np
import matplotlib.pyplot as plt
import csv
from typing import List, Tuple
import mujoco
import random

PROB = 0.1  # Probability of IK-biased sampling

HAND_JOINT_NAMES = [
    "1", "0", "2", "3",    # Finger 1 (Index)
    "5", "4", "6", "7",    # Finger 2 (Middle)
    "9", "8", "10", "11",  # Finger 3 (Ring)
    "12", "13", "14", "15" # Thumb
]
HAND_JOINT_COUNT = len(HAND_JOINT_NAMES)

# Mapping of a FINGER NAME to its site name, body name, and joint names.
FINGER_MAPPING = {
    'index': {
        'site_name': "fingertip1_site", 
        'body_name': "fingertip", 
        'joint_names': ["1", "0", "2", "3"] 
    },
    'middle': { 
        'site_name': "fingertip2_site", 
        'body_name': "fingertip_2", 
        'joint_names': ["5", "4", "6", "7"]
    },
    'ring': { 
        'site_name': "fingertip3_site", 
        'body_name': "fingertip_3", 
        'joint_names': ["9", "8", "10", "11"]
    },
    'thumb': {
        'site_name': "thumb_fingertip", 
        'body_name': "thumb_fingertip", 
        'joint_names': ["12", "13", "14", "15"]
    }
}
ALL_FINGER_NAMES = list(FINGER_MAPPING.keys())

FINGERTIP_BODY_NAMES = [m['body_name'] for m in FINGER_MAPPING.values()]
FINGERTIP_SITE_NAMES = [m['site_name'] for m in FINGER_MAPPING.values()]

PALM_BODY_NAME = "palm_lower"

OBJECT_BODY_NAME = "obj1" 

def d2r(deg: float) -> float:
    return deg * math.pi / 180.0

def rotz(alpha_rad: float) -> np.ndarray:
    c, s = math.cos(alpha_rad), math.sin(alpha_rad)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=float)

def plot_vectors(vectors, labels, title, filename):
    """
    vectors: list of 3D numpy arrays
    labels:  list of strings
    Plots arrows from origin to each vector, with equal aspect, saves to filename.
    """
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="3d")

    # Determine bounds for a nicer auto-scale
    all_pts = np.stack(vectors, axis=0) if len(vectors) > 0 else np.zeros((1,3))
    vmax = np.max(np.abs(all_pts)) if all_pts.size > 0 else 1.0
    lim = max(1.0, float(vmax) * 1.2)

    # Axes
    ax.quiver(0,0,0, 1,0,0, length=1.0, normalize=False, label='x', linewidth=1.0)
    ax.quiver(0,0,0, 0,1,0, length=1.0, normalize=False, label='y', linewidth=1.0)
    ax.quiver(0,0,0, 0,0,1, length=1.0, normalize=False, label='z', linewidth=1.0)

    # Vectors
    for v, lbl in zip(vectors, labels):
        ax.quiver(0,0,0, v[0], v[1], v[2], arrow_length_ratio=0.1, linewidth=2.0)
        ax.text(v[0], v[1], v[2], f' {lbl}', fontsize=10)

    ax.set_xlim([-lim, lim]); ax.set_ylim([-lim, lim]); ax.set_zlim([-lim, lim])
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title(title)
    ax.view_init(elev=20, azim=45)
    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close(fig)

def print_matrix(name, M):
    np.set_printoptions(precision=4, suppress=True)
    print(f"\n{name} =\n{M}")

def print_vector(name, v):
    np.set_printoptions(precision=4, suppress=True)
    print(f"\n{name} = {v}")

def random_joint_angles(num_poses: int, num_joints: int, deg_range: float, rng=None) -> np.ndarray:
    """Uniform degrees in [-deg_range, +deg_range], returned in radians."""
    rng = rng or np.random.default_rng()
    deg = rng.uniform(-deg_range, deg_range, (num_poses, num_joints))
    return np.deg2rad(deg)

def pose_error(T_des: np.ndarray, T_cur: np.ndarray) -> Tuple[float, float]:
    """(pos L2, orientation angle via trace)."""
    R_d, p_d = T_des[:3, :3], T_des[:3, 3]
    R_c, p_c = T_cur[:3, :3], T_cur[:3, 3]
    pos_err = float(np.linalg.norm(p_d - p_c))
    R_err = R_c.T @ R_d
    tr = float(np.trace(R_err))
    val = max(-1.0, min(1.0, (tr - 1.0) * 0.5))
    ori_err = float(math.acos(val))
    return pos_err, ori_err

def write_csv(path: str, header: List[str], rows: List[List[float]]) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

def evaluate_random_poses(FK_func, DH_base: np.ndarray, joint_angles: np.ndarray):
    """
    Return (positions, transforms) for each q in joint_angles.
    Overwrites theta column (col 0) per Craig DH.
    """
    T_list, p_list = [], []
    for q in joint_angles:
        DH = np.asarray(DH_base, dtype=float).copy()
        DH[:, 0] = q
        T = FK_func(DH)
        T_list.append(T)
        p_list.append(T[:3, 3])
    return p_list, T_list

# --- RRT HELPER FUNCTIONS (Modified) ---

def steer(q_from, q_to, step):
    d = q_to - q_from
    L = np.linalg.norm(d)
    return q_to.copy() if L < 1e-9 else (q_from + d * min(step / L, 1.0))

# --- NEW IK SAMPLER FUNCTION (Updated to use active joints/sites and q_base_hand) ---

def sample_ik_biased(sim, data_for_collision, q_hand_min, q_hand_max,
                     hand_joint_ids, arm_joint_ids, q_arm_static,
                     active_hand_dof_ids, non_active_hand_joint_indices,
                     fingertip_site_ids, target_positions,
                     q_base_hand, # Pass in the base Q (e.g., qnear) for fixed joints
                     ik_bias_prob=PROB, ik_step_size=0.1, ik_n_iter=20):
    """
    Samples a joint configuration, either uniformly or biased toward 
    a task-space goal using Inverse Kinematics. Non-active joints are fixed 
    to their position in q_base_hand.
    """
    
    q_current_hand = q_base_hand.copy()

    # 1. Start from a random, but valid, current hand joint position for IK
    # Randomize only the active joints' start positions, or use the base pos
    if random.random() < ik_bias_prob and len(active_hand_dof_ids) > 0:
        # --- IK-Biased Sampling ---
        
        # Initialize q_start_hand from q_base_hand (to keep non-active joints fixed)
        q_start_hand = q_base_hand.copy()
        
        # Randomize the active joints' starting position
        for i, dof_id in enumerate(active_hand_dof_ids):
            jid = sim.model.dof_jntid[dof_id]
            # Find the index of this joint in the full HAND_JOINT_NAMES list
            j_idx = np.where(hand_joint_ids == jid)[0][0] 
            
            j_min = q_hand_min[j_idx]
            j_max = q_hand_max[j_idx]
            
            # Sample position for this active joint
            q_start_hand[j_idx] = j_min + (j_max - j_min) * random.random()
        
        # 2. Set the collision data's qpos with the static arm pose and sampled hand pose
        for i, jid in enumerate(arm_joint_ids):
            qpos_addr = sim.model.jnt_qposadr[jid]
            data_for_collision.qpos[qpos_addr] = q_arm_static[i]
        for i, jid in enumerate(hand_joint_ids):
            qpos_addr = sim.model.jnt_qposadr[jid]
            data_for_collision.qpos[qpos_addr] = q_start_hand[i]
            
        # 3. Setup IK (Combined position control for multiple sites)
        ik_solver_n = len(fingertip_site_ids) * 3 
        ik_jac = np.zeros((ik_solver_n, sim.model.nv))
        ik_err = np.zeros(ik_solver_n)
        
        # 4. Perform IK iterations
        for _ in range(ik_n_iter):
            mujoco.mj_forward(sim.model, data_for_collision)
            
            # Populate Jacobian and error vector
            for i, site_id in enumerate(fingertip_site_ids):
                site_pos = data_for_collision.site_xpos[site_id]
                target_pos = target_positions[i]
                
                # Position error (desired - current)
                err = target_pos - site_pos
                ik_err[i*3 : i*3 + 3] = err
                
                # Position Jacobian
                mujoco.mj_jacSite(sim.model, data_for_collision, 
                                  ik_jac[i*3 : i*3 + 3, :], None, site_id)

            # Only consider the ACTIVE hand joint DOFs for the IK solution
            ik_jac_active = ik_jac[:, active_hand_dof_ids]
            
            # Stop if error is small enough
            if np.linalg.norm(ik_err) < 0.01:
                break
                
            # Compute a scaled step using the pseudo-inverse (DLS)
            try:
                reg = 1e-3 
                damped_j_inv = ik_jac_active.T @ np.linalg.inv(ik_jac_active @ ik_jac_active.T + reg * np.eye(ik_jac_active.shape[0]))
                dq = damped_j_inv @ ik_err
            except np.linalg.LinAlgError:
                dq = np.linalg.pinv(ik_jac_active) @ ik_err

            # Scale the step and apply to qpos
            dq_scaled = dq * ik_step_size 
            
            # Apply the step ONLY to the active DOFs
            for i, dof_id in enumerate(active_hand_dof_ids):
                data_for_collision.qpos[sim.model.dof_jntid[dof_id]] += dq_scaled[i]
                
            # Clamp joint positions to limits (for all joints)
            for i, jid in enumerate(hand_joint_ids):
                qpos_addr = sim.model.jnt_qposadr[jid]
                data_for_collision.qpos[qpos_addr] = np.clip(
                    data_for_collision.qpos[qpos_addr], q_hand_min[i], q_hand_max[i]
                )
                
        # Return the resulting joint configuration (the hand part only)
        q_new_hand = np.zeros(HAND_JOINT_COUNT)
        for i, jid in enumerate(hand_joint_ids):
            qpos_addr = sim.model.jnt_qposadr[jid]
            q_new_hand[i] = data_for_collision.qpos[qpos_addr]
            
        return q_new_hand

    else:
        # --- Uniform Sampling (Fallback) ---
        # Only randomize active joints
        q_uniform_hand = q_base_hand.copy()
        
        for i, dof_id in enumerate(active_hand_dof_ids):
            jid = sim.model.dof_jntid[dof_id]
            j_idx = np.where(hand_joint_ids == jid)[0][0] 
            
            j_min = q_hand_min[j_idx]
            j_max = q_hand_max[j_idx]
            q_uniform_hand[j_idx] = j_min + (j_max - j_min) * random.random()
            
        return q_uniform_hand

# --- COLLISION FILTERING (Unchanged) ---

def is_valid_hand_config(sim, model, data_for_collision, hand_joint_ids, q_hand, 
                         arm_joint_ids, q_arm_static, 
                         obj_body_id, fingertip_body_ids) -> bool:
    """
    Checks for collisions.
    ONLY allows fingertip-object contacts. All other contacts (palm-object, 
    arm-object, self-collisions, etc.) will return False.
    """
    
    # 1. Set the configuration (as before)
    for i, jid in enumerate(arm_joint_ids):
        qpos_addr = model.jnt_qposadr[jid]
        data_for_collision.qpos[qpos_addr] = q_arm_static[i]
        
    for i, jid in enumerate(hand_joint_ids):
        qpos_addr = model.jnt_qposadr[jid]
        data_for_collision.qpos[qpos_addr] = q_hand[i]
        
    # 2. Compute collisions
    mujoco.mj_forward(model, data_for_collision)
    mujoco.mj_collision(model, data_for_collision)

    # 3. Check contacts
    if data_for_collision.ncon == 0:
        return True # No contacts is always valid
    else:
        # Loop through all found contacts
        for i in range(data_for_collision.ncon):
            contact = data_for_collision.contact[i]
            body1_id = model.geom_bodyid[contact.geom1]
            body2_id = model.geom_bodyid[contact.geom2]
            
            # Is this contact between an active fingertip
            is_fingertip_contact = (body1_id in fingertip_body_ids) or (body2_id in fingertip_body_ids)

            if is_fingertip_contact:
                return False 
    return True

def plan_path_segment(sim, q_hand_start, fingers_to_move, goal_offsets, q_arm_static, full_hand_joint_ids, q_hand_min, q_hand_max):
    """
    Plans a single segment of the hand's motion.
    
    Args:
        sim: The simulation object.
        q_hand_start: The starting joint configuration for this segment.
        fingers_to_move: A list of finger names to be used in this plan.
        goal_offsets: A dictionary mapping finger names to their XYZ goal offsets from the object center.
        ... other necessary parameters ...
    
    Returns:
        The calculated path as a numpy array, or None if planning fails.
    """
    model = sim.model
    obj_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, OBJECT_BODY_NAME)
    
    print(f"\n--- Planning Segment for Fingers: {fingers_to_move} ---")

    # --- Setup specific to this segment ---
    active_site_names = [FINGER_MAPPING[f]['site_name'] for f in fingers_to_move]
    active_body_names = [FINGER_MAPPING[f]['body_name'] for f in fingers_to_move]
    active_joint_names = [j for f in fingers_to_move for j in FINGER_MAPPING[f]['joint_names']]
    
    active_fingertip_body_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in active_body_names]
    active_fingertip_site_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name) for name in active_site_names]

    active_hand_joint_ids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in active_joint_names])
    active_hand_joint_ids = active_hand_joint_ids[active_hand_joint_ids != -1]
    active_hand_dof_ids = np.array([model.jnt_dofadr[jid] for jid in active_hand_joint_ids])
    
    non_active_joint_names = [j for j in HAND_JOINT_NAMES if j not in active_joint_names]
    non_active_hand_joint_ids_all = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in non_active_joint_names]
    valid_non_active_joint_ids = [jid for jid in non_active_hand_joint_ids_all if jid != -1]
    
    non_active_hand_joint_indices = []
    for jid in valid_non_active_joint_ids:
        idx = np.flatnonzero(full_hand_joint_ids == jid)
        if len(idx) > 0:
            non_active_hand_joint_indices.append(idx[0])

    # --- Define Task-Space Goal for this segment ---
    obj_xpos = sim.data.xpos[obj_body_id].copy()
    obj_xmat = sim.data.xmat[obj_body_id].reshape(3, 3)
    
    target_positions_for_active_sites = []
    for f_name in fingers_to_move:
        offset = goal_offsets.get(f_name)
        if offset is None:
            print(f"Warning: No goal offset defined for finger '{f_name}' in this segment.")
            continue
        target_positions_for_active_sites.append(obj_xpos + obj_xmat @ np.array(offset))

    # If no valid goals were created, we cannot plan. Abort this segment.
    if not target_positions_for_active_sites:
        print("Error: No valid goal positions were generated for this segment. Skipping.")
        return None
    # -----------------------

    print(f"Start Pose: {q_hand_start}")
    print(f"Goal Positions:\n{np.array(target_positions_for_active_sites)}")

    d = {"q_hand_start" : q_hand_start,
         "active_fingertip_body_ids": active_fingertip_body_ids,
         "active_fingertip_site_ids": active_fingertip_site_ids,
         "active_hand_dof_ids": active_hand_dof_ids,
         "non_active_hand_joint_indices": non_active_hand_joint_indices,
         "target_positions_for_active_sites": target_positions_for_active_sites}

    return d