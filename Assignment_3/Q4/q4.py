import numpy as np
from collections import deque

def wavefront_planner(grid, start, goal):
    """
    2D Wavefront Planner to find a path from start to goal.

    Parameters:
    - grid: 2D numpy array (0 for free space, 1 for obstacles)
    - start: tuple (x, y), starting point
    - goal: tuple (x, y), goal point

    Returns:
    - path: List of tuples representing the path from start to goal
    """
    pass

# Example tests
if __name__ == "__main__":
    grid = np.array([
        [0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [1, 1, 0, 0, 0],
        [0, 0, 0, 1, 0]
    ])
    start = (0, 0)
    goal = (4, 4)
    path = wavefront_planner(grid, start, goal)
    print("Path:", path)
