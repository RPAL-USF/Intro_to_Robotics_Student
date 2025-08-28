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
6. [Common Issues & Fixes](#common-issues--fixes)  
7. [Resources](#resources)  

---

## Setup

To get started, follow these instructions to install the required tools and dependencies.

### Install MuJoCo

We will use **MuJoCo** for robot simulation.  
Follow the instructions for your operating system.

---

### **Ubuntu / Linux (20.04+)**

#### Install dependencies
```bash
sudo apt update
sudo apt install -y git wget unzip libosmesa6-dev libgl1-mesa-dev libglfw3
```

#### Download and install MuJoCo 3.x
```bash
wget https://github.com/google-deepmind/mujoco/releases/download/3.1.6/mujoco-3.1.6-linux-x86_64.tar.gz
mkdir -p $HOME/.mujoco
tar -xzf mujoco-3.1.6-linux-x86_64.tar.gz -C $HOME/.mujoco
```

#### Set environment variables (add to ~/.bashrc for persistence)
```bash
export MUJOCO_DIR=$HOME/.mujoco/mujoco-3.1.6
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_DIR/lib
export PATH=$PATH:$MUJOCO_DIR/bin
```

Test installation:

```bash
$MUJOCO_DIR/bin/simulate
```

---

### **Windows 10/11**

1. Download the latest **MuJoCo** release from:  
   [https://github.com/google-deepmind/mujoco/releases](https://github.com/google-deepmind/mujoco/releases)

2. Extract the `.zip` file to a location of your choice, for example:  
   `C:\mujoco\mujoco-3.1.6`

3. Add MuJoCo to your **PATH**:  
   - Press `Win + S` → Search for “Environment Variables” → Open  
   - Under *System Variables*, find **Path** → Edit → Add:  
     `C:\mujoco\mujoco-3.1.6\bin`

4. Test by opening **Command Prompt** and running:  
   ```bash
   simulate.exe
   ```

---

### **macOS (Intel or Apple Silicon)**

#### Install dependencies (Homebrew required)
```bash
brew install glfw
```

#### Download MuJoCo
```bash
curl -L -o mujoco-macos.tar.gz https://github.com/google-deepmind/mujoco/releases/download/3.1.6/mujoco-3.1.6-macos-universal2.tar.gz
mkdir -p $HOME/.mujoco
tar -xzf mujoco-macos.tar.gz -C $HOME/.mujoco
```

#### Set environment variables (add to ~/.zshrc for persistence)
```bash
export MUJOCO_DIR=$HOME/.mujoco/mujoco-3.1.6
export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:$MUJOCO_DIR/lib
export PATH=$PATH:$MUJOCO_DIR/bin
```

Test installation:

```bash
simulate
```

---

### Clone the Repository

```bash
git clone <repository_url>
cd Intro_to_Robotics
```

---

### Install Python Dependencies

```bash
pip install numpy matplotlib scipy mujoco
```

---

## Quick Start

To verify your installation and run your first MuJoCo simulation, you can launch a UR5 example model:

1. Ensure you have a UR5 model file (`ur5e_scene.xml`) in a `models/` folder.  
2. Run the following test script:

```python
import mujoco
import mujoco.viewer

# Load the model
model = mujoco.MjModel.from_xml_path("models/ur5e_scene.xml")
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
- **PDF file** – Detailed problem statements.
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
Topics: System design and reporting.
- Design a robotic system to solve a given task.
- Provide schematics, flowcharts, and discuss challenges.

---

## Submission Guidelines

- Create a directory for each assignment (e.g., `HW1`, `HW2`).
- Inside each directory, create subfolders for each question (`Q1`, `Q2`).
- Save handwritten or image-based answers as PDFs in their respective folders.
- Zip the assignment directory (e.g., `HW1.zip`) and upload it to Canvas.

**Naming Conventions**:
- Do not change the folder structure or filenames.
- Include comments in your Python files for any changes you make.

---

## Common Issues & Fixes

| Issue | Possible Fix |
|-------|--------------|
| **GLXBadContext** or blank simulation window on Linux | Install OpenGL/Mesa drivers: `sudo apt install mesa-utils` |
| **ModuleNotFoundError: 'mujoco'** | Make sure you installed MuJoCo in the same Python environment: `pip install mujoco` |
| **Simulation window not opening** | Check that `LD_LIBRARY_PATH` (Linux) or `DYLD_LIBRARY_PATH` (macOS) includes `$MUJOCO_DIR/lib` |
| **Permission denied on simulate.exe (Windows)** | Ensure you extracted MuJoCo to a folder you own, not `C:\Program Files` |
| **XML model not found** | Verify the file path in your script matches the actual model location |

---

## Resources

- [MuJoCo Documentation](https://mujoco.readthedocs.io/en/stable/)
- [MuJoCo GitHub](https://github.com/google-deepmind/mujoco)
- [Python Official Docs](https://docs.python.org/3/)
- [NumPy](https://numpy.org/doc/stable/)
- [Matplotlib](https://matplotlib.org/stable/index.html)
