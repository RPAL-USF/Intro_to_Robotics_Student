import math
import numpy as np
import matplotlib.pyplot as plt

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