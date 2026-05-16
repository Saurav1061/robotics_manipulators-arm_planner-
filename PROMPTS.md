# LLM Prompting Documentation

The following prompts document the most critical interactions with the LLM used to architect, debug, and refine the `arm-planner` codebase from scratch.

### 1. Architectural Strategy & Tooling
> "I need to build a robotics arm-planner from scratch in Python without using heavy middleware like ROS, MoveIt, or Drake. It requires URDF parsing, Forward/Inverse kinematics, trajectory planning, and collision checking. What is the best standard Python stack to achieve this, and how should I structure the package directories?"

### 2. Mathematical Implementation: Inverse Kinematics
> "I need to write a numerical Inverse Kinematics solver using the Damped Least-Squares (DLS) Jacobian pseudoinverse method. How do I compute the 6xN Jacobian matrix using finite differences, and what does the iterative update loop look like using pure `numpy`?"

### 3. Motion Smoothness & Trajectory
> "How do I implement cubic polynomial interpolation to generate a smooth joint-space trajectory over 50 timesteps? The math needs to ensure that the starting velocity and ending velocity of the robot arm are exactly zero."

### 4. Resolving the "Fat Robot" Collision Problem
> "My bounding sphere collision detection works mathematically, but it's triggering false positives on the UR5 wrist joints even when the arm is completely stretched out because the joints are so close together. How do I adjust the logic to mathematically ignore adjacent and semi-adjacent links in the kinematic chain?"

### 5. Environment & Pytest Conflicts
> "When I run `pytest`, it fails with a `ModuleNotFoundError: No module named 'lark'`, and the traceback shows it's trying to load ROS 2 Humble `launch_testing` plugins from my system instead of running my isolated Conda environment. How do I force Pytest to ignore system-wide ROS plugins and only use my local packages?"

### 6. Resolving URDF Mathematical Offsets
> "My Forward Kinematics manual reference test is failing. My math outputs `[-0.1733, -0.6235, 0.5509]`, but the mathematical Denavit-Hartenberg (DH) reference expects `[0.3804, 0.2039, 0.3541]`. Why does the `ros-industrial` UR5 URDF file produce different Cartesian coordinates than the pure DH model, and how should I update my test?"
