import numpy as np
import matplotlib.pyplot as plt
import os
from urdf import RobotModel
from kinematics import Kinematics
from trajectory import TrajectoryPlanner
from visualize import Visualizer
from collision import CollisionDetector

def main():
    # Setup directories
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.normpath(os.path.join(current_dir, ".."))
    urdf_file = os.path.join(project_root, "urdf", "ur5.urdf")
    output_dir = os.path.join(project_root, "output")
    os.makedirs(output_dir, exist_ok=True)

    # 1. Initialize System Components
    print("\n--- Initializing Arm Planner ---")
    robot = RobotModel(urdf_file, base_link_name="base_link", end_effector_name="tool0")
    kin = Kinematics(robot)
    planner = TrajectoryPlanner(num_joints=6)
    detector = CollisionDetector(kin, sphere_radius=0.08)
    vis = Visualizer(kin)

    # 2. Define the Challenge: Start configuration to Target Pose
    start_q = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]  # The standard 'L' shape

    # Define a reachable, interesting target pose
    target_p = np.array([
        [-1.0, 0.0, 0.0, 0.1],  # End-effector will face negative X
        [ 0.0, 1.0, 0.0, 0.4],  # Positioned slightly left (positive Y)
        [ 0.0, 0.0,-1.0, 0.5],  # Positioned upward (Z)
        [ 0.0, 0.0, 0.0, 1.0]
    ])

    print("\n--- Step 1: Solving Inverse Kinematics ---")
    print(f"Goal position (meters): {target_p[:3, 3]}")
    goal_q = kin.inverse_kinematics(target_p, initial_guess=start_q)
    
    if goal_q is None:
        print("IK Failed to find a solution. Exiting.")
        return

    # 3. Pre-check target for safety
    print("\n--- Step 2: Checking Goal safety ---")
    is_colliding, _ = detector.check_state(goal_q)
    if is_colliding:
        print("CRASH! Goal configuration is self-colliding. Aborting.")
        return
    print("Goal is safe.")

    # 4. Generate Trajectory
    print("\n--- Step 3: Planning 30-step smooth trajectory ---")
    steps = 30
    traj = planner.generate_cubic_trajectory(start_q, goal_q, steps=steps)

    # 5. Full Trajectory Safety Verification
    print("\n--- Step 4: Verifying Full Trajectory Safety ---")
    detector.check_trajectory(traj)

    # 6. Generate Animation GIF
    gif_path = os.path.join(output_dir, "planned_motion.gif")
    print(f"\n--- Step 5: Generating Animation -> {gif_path} ---")
    vis.animate_trajectory(traj, save_path=gif_path)

    # 7. Generate Joint Plot deliverable
    plot_path = os.path.join(output_dir, "joint_trajectory.png")
    print(f"\n--- Step 6: Generating Plot -> {plot_path} ---")
    t_array = np.linspace(0, 5.0, steps)
    plt.figure(figsize=(10, 6))
    for i in range(6):
        plt.plot(t_array, traj[:, i], label=f'Joint {i+1}')
    plt.xlabel('Normalized Time (s)')
    plt.ylabel('Joint Angle (rad)')
    plt.title('Planned Joint Trajectory (Cubic Polynomial)')
    plt.legend()
    plt.grid(True)
    plt.savefig(plot_path)
    print("All tasks complete. Check output/ directory.")

if __name__ == "__main__":
    main()