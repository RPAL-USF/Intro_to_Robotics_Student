import numpy as np
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Q4.q4 import RotX, RotY, RotZ

def Euler(theta1, theta2, theta3):
    """
    TODO:  Apply rotations in euler order
    """
    pass

def RPY(theta1, theta2, theta3):
    """
    TODO: Apply rotations in roll-pitch-yaw order
    """
    pass


# Example tests
if __name__ == "__main__":
    theta1, theta2, theta3 = np.pi / 6, np.pi / 4, np.pi / 3  # 30, 45, 60 degrees
    print("Euler Angles (Z-Y-X):\n", Euler(theta1, theta2, theta3))
    print("Roll-Pitch-Yaw (X-Y-Z):\n", RPY(theta1, theta2, theta3))