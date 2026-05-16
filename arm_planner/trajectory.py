import numpy as np

class TrajectoryPlanner:
    def __init__(self, num_joints):
        self.num_joints = num_joints

    def generate_cubic_trajectory(self, q_start, q_end, duration=5.0, steps=50):
        """
        Generates a smooth joint-space trajectory using cubic polynomial interpolation.
        
        Args:
            q_start: Starting joint angles (array-like, length N)
            q_end: Ending joint angles (array-like, length N)
            duration: Total time for the motion in seconds
            steps: Number of discrete timesteps to generate
            
        Returns:
            A numpy array of shape (steps, num_joints) containing the trajectory.
        """
        q_start = np.array(q_start, dtype=float)
        q_end = np.array(q_end, dtype=float)
        
        if len(q_start) != self.num_joints or len(q_end) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joints, got {len(q_start)} and {len(q_end)}")
            
        # Create an array of times from 0 to T
        t_array = np.linspace(0, duration, steps)
        
        # Initialize the output trajectory matrix
        trajectory = np.zeros((steps, self.num_joints))
        
        # Calculate the cubic coefficients for each joint
        # a0 = q_start
        # a1 = 0
        # a2 = 3 * (q_end - q_start) / T^2
        # a3 = -2 * (q_end - q_start) / T^3
        
        delta_q = q_end - q_start
        a0 = q_start
        a2 = 3.0 * delta_q / (duration ** 2)
        a3 = -2.0 * delta_q / (duration ** 3)
        
        # Compute the positions at each timestep
        for i, t in enumerate(t_array):
            # q(t) = a0 + a2*t^2 + a3*t^3
            trajectory[i, :] = a0 + a2 * (t ** 2) + a3 * (t ** 3)
            
        return trajectory

