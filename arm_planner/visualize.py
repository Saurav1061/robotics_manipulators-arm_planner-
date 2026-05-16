import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os

class Visualizer:
    def __init__(self, kinematics_solver):
        self.kin = kinematics_solver

    def animate_trajectory(self, trajectory, save_path="output/animation.gif"):
        """
        Animates a joint-space trajectory and saves it to a file.
        """
        # Ensure the output directory exists
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Set up the 3D plot limits (roughly sized for the UR5 reach)
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)
        ax.set_zlim(-0.2, 1.2)
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.set_zlabel('Z (meters)')
        ax.set_title('Robot Arm Trajectory')
        
        # Initialize the line representing the robot arm
        # 'o-' means draw dots at the joints and lines between them
        line, = ax.plot([], [], [], 'o-', lw=5, markersize=8, color='blue')
        
        # Initialize the line for the end-effector trace (the path it draws in the air)
        trace, = ax.plot([], [], [], '-', lw=2, color='orange')
        ee_xs, ee_ys, ee_zs = [], [], []
        
        def init():
            line.set_data([], [])
            line.set_3d_properties([])
            trace.set_data([], [])
            trace.set_3d_properties([])
            return line, trace
            
        def update(frame_idx):
            joint_angles = trajectory[frame_idx]
            transforms = self.kin.forward_kinematics(joint_angles)
            
            # Extract XYZ for all joints (the translational part of the matrix)
            xs = [T[0, 3] for T in transforms]
            ys = [T[1, 3] for T in transforms]
            zs = [T[2, 3] for T in transforms]
            
            # Update the arm line
            line.set_data(xs, ys)
            line.set_3d_properties(zs)
            
            # Update the end-effector trace
            ee_xs.append(xs[-1])
            ee_ys.append(ys[-1])
            ee_zs.append(zs[-1])
            trace.set_data(ee_xs, ee_ys)
            trace.set_3d_properties(ee_zs)
            
            return line, trace
            
        print(f"Generating animation with {len(trajectory)} frames...")
        anim = FuncAnimation(fig, update, frames=len(trajectory), 
                             init_func=init, blit=False, interval=100)
                             
        print(f"Saving to {save_path} (This might take a moment)...")
        anim.save(save_path, writer='pillow', fps=10)
        print("Animation saved successfully!")


