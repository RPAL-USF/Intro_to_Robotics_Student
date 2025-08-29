# Intro to Robotics

Welcome to the **Intro to Robotics** repository!  
This repository contains all the materials, assignments, and code examples needed for the course. The assignments will help you gain hands-on experience with robotics concepts such as forward kinematics, Jacobians, motion planning, and using simulation tools like **MuJoCo**.

---

## Table of Contents

1. [Setup](#setup)  
2. [Quick Start](#quick-start)  
3. [Repository Structure](#repository-structure)  
4. [Assignments Overview](#assignments-overview)  
5. [Submission Guidelines](#submission-guidelines)  
6. [Resources](#resources)  

---

## Setup

To get started, follow these instructions to install the required tools and dependencies.

### Install MuJoCo

We will use **MuJoCo** for robot simulation.  
Follow the instructions for your operating system.

---

### Clone the Repository

Use the option of "Use this template" in GitHub.

---

## Quick Start

To verify your installation and run your first MuJoCo simulation, you can launch a UR5 example model:

1. Ensure you have a UR5 model file (`scene.xml`).  
2. Run the following test script:

```python
import mujoco
import mujoco.viewer

# Load the model
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

# Launch the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
```

You should see a 3D simulation window with the UR5 robot loaded.

---

## Repository Structure

```
Intro_to_Robotics/
├── Core/
│   ├── Scene/Required scene files
|   ├── simulation.py
|   ├── utils.py
├── Assignment_1/
│   ├── Homework1.pdf
│   ├── Q1/q1.py
│   ├── Q2/q2.py
├── Assignment_2/
│   ├── Homework2.pdf
│   ├── Q1/q1.py
│   ├── Q2/q2.py
└── ...
```

Each assignment includes:
- **Python files** – Starter code for students to complete.

---

## Assignments Overview

### Assignment 1  
Topics: Matrix operations, transformations, and MuJoCo setup.
- Compute rotation matrices and vectors.
- Set up a UR5 robotic arm in MuJoCo.
- Write Python functions for matrix operations.

### Assignment 2  
Topics: Rotation matrices and forward kinematics.
- Define coordinate systems using DH parameters.
- Implement Euler and Roll-Pitch-Yaw rotations.
- Compute forward kinematics.

### Assignment 3  
Topics: Jacobians and collision detection.
- Derive the Jacobian matrix.
- Randomly generate joint angles and detect collisions in MuJoCo.
- Implement a wavefront planner for 2D motion planning.

### Assignment 4  
Topics: Motion planning and trajectory generation.
- Implement the RRT algorithm for UR5 motion planning.
- Generate 6D interpolating polynomials for trajectories.

### Assignment 5  
Under development

---

## Submission Guidelines

Please, utilize the provided template code and submit through Canvas as a zip folder. For example, the first assignment folder is Assignment\_1, zip that folder and submit through Canvas.

**Naming Conventions**:
- Do not change the folder structure or filenames.
- Include comments in your Python files for any changes you make.

---

## Resources

- [MuJoCo Documentation](https://mujoco.readthedocs.io/en/stable/)
- [MuJoCo GitHub](https://github.com/google-deepmind/mujoco)
- [Python Official Docs](https://docs.python.org/3/)
- [NumPy](https://numpy.org/doc/stable/)
- [Matplotlib](https://matplotlib.org/stable/index.html)
