import math
import numpy as np
import matplotlib.pyplot as plt
import csv
from typing import List, Tuple

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