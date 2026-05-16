import numpy as np

class CollisionDetector:
    def __init__(self, kinematics_solver, sphere_radius=0.1):
        """
        Uses bounding spheres around each joint frame to check for self-collision.
        
        Args:
            kinematics_solver: An instance of your Kinematics class
            sphere_radius: The radius of the safety sphere in meters (0.1m = 10cm)
        """
        self.kin = kinematics_solver
        self.radius = sphere_radius

    def check_state(self, joint_angles):
        """
        Checks a single robot configuration for self-collisions.
        
        Returns:
            (is_collision, (link_i, link_j))
        """
        # Get the 3D poses of all joints from FK
        transforms = self.kin.forward_kinematics(joint_angles)
        
        # Extract just the XYZ Cartesian coordinates (the translation part of the matrix)
        positions = [T[:3, 3] for T in transforms]
        num_points = len(positions)
        
        # Check non-adjacent pairs (j starts at i + 2 to skip connected links)
        for i in range(num_points):
            for j in range(i + 3, num_points):
                dist = np.linalg.norm(positions[i] - positions[j])
                
                if dist < (2 * self.radius):
                    return True, (i, j)
                    
        return False, None

    def check_trajectory(self, trajectory):
        """
        Sweeps through an entire N-step trajectory checking for collisions.
        """
        for step_idx, joint_angles in enumerate(trajectory):
            is_collision, links = self.check_state(joint_angles)
            if is_collision:
                print(f"CRASH! Collision detected at timestep {step_idx} between joint {links[0]} and joint {links[1]}.")
                return True
                
        print("Success: Trajectory is 100% collision-free!")
        return False
    
