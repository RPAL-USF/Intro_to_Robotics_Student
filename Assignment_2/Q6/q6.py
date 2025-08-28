import numpy as np

def FK(DH):
    """
    TODO: Compute forward kinematics by chaining DH transforms.

    Parameters
    ----------
    DH : list/array of rows [theta, d, a, alpha] for each link (standard DH)

    Returns
    -------
    A : (4,4) np.ndarray  # base-to-end-effector transform
    """
    pass

# Example tests
if __name__ == "__main__":
    DH = [
        [0, 0.4, 0.2, np.pi / 2],
        [np.pi / 3, 0, 0.5, 0],
        [np.pi / 4, 0, 0.3, -np.pi / 2]
    ]

    print("Forward Kinematics Matrix:\n", FK(DH))