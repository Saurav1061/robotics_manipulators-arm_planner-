Here are the final two files to complete your project deliverables. You can create these in the root of your `arm_planner_project/` directory.

### 1. `README.md`

Create a file named `README.md` and paste the following markdown block into it:

```markdown
# Arm Planner: Robotics Manipulator Motion Planning

A completely from-scratch Python motion planning library for a 6-DOF robotic manipulator (UR5). This project parses a URDF, computes Forward and Inverse Kinematics, plans smooth joint-space trajectories, performs self-collision checks, and animates the resulting motion. 

**No external robotics middleware (like MoveIt, ROS, or Drake) was used for the mathematical computations.**

## Features
* **URDF Parsing:** Extracts the kinematic chain, joint limits, and axes using `urdf-parser-py`.
* **Forward Kinematics (FK):** Implements $4 \times 4$ homogeneous transformation matrices using `numpy` and `scipy.spatial.transform`.
* **Inverse Kinematics (IK):** Uses a custom numerical solver based on the Damped Least-Squares (DLS) Jacobian pseudoinverse to elegantly handle singularities.
* **Trajectory Planning:** Generates smooth, zero-start/zero-end velocity motions using cubic polynomial interpolation.
* **Collision Detection:** Implements a bounding-sphere approximation algorithm to check for self-intersections across the trajectory.
* **Visualization:** Renders a 3D animation of the arm and end-effector trace using `matplotlib`.

## Setup & Installation

It is recommended to run this project inside a Conda environment.

```bash
# Create and activate the environment
conda create -n arm_planner python=3.10
conda activate arm_planner

# Install dependencies
pip install numpy scipy matplotlib pytest urdf-parser-py

```

*Note: Ensure the pre-compiled `ur5.urdf` file is placed in the `urdf/` directory.*

## Usage

Run the master Command Line Interface (CLI) to execute the full pipeline (IK -> Trajectory -> Collision Check -> Animation):

```bash
python -m arm_planner.cli

```

### Outputs

Upon successful execution, the script generates two files in the `output/` directory:

1. `planned_motion.gif`: A 3D animation of the robotic arm completing the trajectory.
2. `joint_trajectory.png`: A plot of the 6 joint angles over time showing the smooth cubic interpolation.

## Testing

Run the included Forward Kinematics verification tests using `pytest`:

```bash
pytest tests/test_fk.py

```

## Design Notes

* **Inverse Kinematics:** A purely analytical approach for the UR5 is possible, but a numerical DLS solver was chosen for its flexibility and robustness near singularities. A damping factor ($\lambda = 0.1$) prevents wild joint angle swings.
* **Collision Detection:** The "Fat Robot" problem was encountered where tightly packed wrist joints falsely registered as collisions. This was solved by tuning the bounding sphere radius to 8cm ($0.08m$) and enforcing a check that ignores semi-adjacent links (`i + 3`) in the kinematic chain, taking advantage of the UR5's built-in lateral joint offsets.
* **Architecture:** The package is strictly modular. The pipeline ensures a goal configuration is safely verified before a trajectory is planned, and the entire trajectory is swept for collisions before visualization is permitted.

```

---
