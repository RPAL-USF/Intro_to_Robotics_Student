import os, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if ROOT not in sys.path:
    sys.path.append(ROOT)
import numpy as np
import matplotlib.pyplot as plt
from Core.utils import print_matrix, plot_vectors, print_vector, d2r, rotz
from Assignment_1.Q1.q1 import matrix_multiply, matrix_transpose, matrix_inverse, vector_cross, vector_dot

# ------------------ Main computations ------------------


def part_a(A: np.ndarray, B: np.ndarray, p: np.ndarray):
    """
    (a) compute AB, Ap, ABp, ABp+p; draw vectors Ap, ABp, ABp+p
    """
    print("\n=== (a) Using provided A, B, p ===")
    AB = matrix_multiply(A, B)
    Ap = matrix_multiply(A, p)
    ABp = matrix_multiply(AB, p)
    ABp_plus_p = ABp + p

    print_matrix("A", A)
    print_matrix("B", B)
    print_matrix("AB", AB)
    print_vector("p", p)
    print_vector("Ap", Ap)
    print_vector("ABp", ABp)
    print_vector("ABp + p", ABp_plus_p)

    plot_vectors(
        [Ap, ABp, ABp_plus_p],
        ["Ap", "ABp", "ABp+p"],
        title="(a) Vectors from given A, B, p",
        filename="q2a.png"
    )

def part_b(A: np.ndarray, B: np.ndarray, p: np.ndarray):
    """
    (b) compute AB, Ap, ABp; draw Ap, ABp
    """
    print("\n=== (b) Using provided A, B, p ===")
    AB = matrix_multiply(A, B)
    Ap  = matrix_multiply(A, p)
    ABp = matrix_multiply(AB, p)

    print_matrix("AB", AB)
    print_vector("Ap", Ap)
    print_vector("ABp", ABp)

    plot_vectors(
        [Ap, ABp],
        ["Ap", "ABp"],
        title="(b) Vectors from given A, B, p",
        filename="q2b.png"
    )

def part_c(A: np.ndarray, B: np.ndarray, p: np.ndarray):
    """
    (c) compute AB, Ap, ABp; draw Ap, ABp
    """
    print("\n=== (c) Using provided A, B, p ===")
    AB = matrix_multiply(A, B)
    Ap  = matrix_multiply(A, p)
    ABp = matrix_multiply(AB, p)

    print_matrix("AB", AB)
    print_vector("Ap", Ap)
    print_vector("ABp", ABp)

    plot_vectors(
        [Ap, ABp],
        ["Ap", "ABp"],
        title="(c) Vectors from given A, B, p",
        filename="q2c.png"
    )

def part_d(A: np.ndarray, B: np.ndarray, p: np.ndarray):
    """
    (d) compute AB^{-1}, (AB)^T, A^{-1}p, A^T p, AB^{-1}p, (AB)^T p
        draw A^{-1}p, A^T p, AB^{-1}p, (AB)^T p
    """
    print("\n=== (d) Using provided A, B, p ===")
    AB = matrix_multiply(A, B)
    AB_inv = matrix_inverse(AB)
    AB_T   = matrix_transpose(AB)
    A_inv  = matrix_inverse(A)
    A_T    = matrix_transpose(A)

    Ainv_p  = matrix_multiply(A_inv, p)
    AT_p    = matrix_multiply(A_T,   p)
    ABinv_p = matrix_multiply(AB_inv, p)
    ABT_p   = matrix_multiply(AB_T,   p)

    print_matrix("AB^{-1}", AB_inv)
    print_matrix("(AB)^T", AB_T)
    print_vector("A^{-1} p", Ainv_p)
    print_vector("A^T p", AT_p)
    print_vector("AB^{-1} p", ABinv_p)
    print_vector("(AB)^T p", ABT_p)

    plot_vectors(
        [Ainv_p, AT_p, ABinv_p, ABT_p],
        ["A^{-1}p", "A^T p", "AB^{-1}p", "(AB)^T p"],
        title="(d) Transforms applied to p (given A, B)",
        filename="q2d.png"
    )

def part_e(A: np.ndarray, B: np.ndarray, p1: np.ndarray, p2: np.ndarray):
    """
    (e) compute (A p1) x (B p2) and its norm; draw (A p1) x (B p2)
        (If you want the same p in both, pass p1==p2 from main.)
    """
    print("\n=== (e) Using provided A, B, p1, p2 ===")
    Ap1 = matrix_multiply(A, p1)
    Bp2 = matrix_multiply(B, p2)
    c   = vector_cross(Ap1, Bp2)
    n   = np.linalg.norm(c)

    print_vector("A p1", Ap1)
    print_vector("B p2", Bp2)
    print_vector("(A p1) x (B p2)", c)
    print(f"\n||(A p1) x (B p2)|| = {n:.6f}")

    plot_vectors(
        [c],
        ["(A p1) x (B p2)"],
        title="(e) vector_cross product vector from given A,B,p1,p2",
        filename="q2e.png"
    )

# ------------------ main: define inputs & call parts ------------------

def main():
    # Define your matrices/vectors once here:
    A = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]], dtype=float)
    B = np.array([[0, -1, 0],
                  [1,  0, 0],
                  [0,  0, 1]], dtype=float)
    p1 = np.array([1., 2., 3.])
    p2 = np.array([4., 5., 6.])

    # Call all parts using the parameters defined above
    part_a(A, B, p1)
    part_b(A, B, p1)
    part_c(A, B, p1)
    part_d(A, B, p1)
    part_e(A, B, p1, p2)

    print("\nDone. Figures saved as q2a.png ... q2e.png in this folder.")

if __name__ == "__main__":
    main()
