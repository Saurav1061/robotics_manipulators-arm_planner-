import numpy as np
from scipy.spatial.transform import Rotation

class Kinematics:
    def __init__(self, robot_model):
        """
        Takes the parsed RobotModel to compute kinematics.
        """
        self.chain = robot_model.chain_joints

    @staticmethod
    def _create_transform(translation, rpy):
        """
        Creates a 4x4 homogeneous transformation matrix from translation and Roll-Pitch-Yaw.
        """
        T = np.eye(4)
        if translation is not None:
            T[:3, 3] = translation
            
        if rpy is not None:
            # Scipy expects intrinsic rotations, URDF uses extrinsic fixed XYZ.
            # 'xyz' matches standard URDF RPY conventions.
            r = Rotation.from_euler('xyz', rpy)
            T[:3, :3] = r.as_matrix()
            
        return T

    @staticmethod
    def _joint_rotation(axis, angle):
        """
        Creates a 4x4 transformation matrix for a rotation around a specific axis.
        """
        T = np.eye(4)
        axis = np.array(axis, dtype=float)
        # Normalize the axis just to be safe
        axis_norm = np.linalg.norm(axis)
        if axis_norm == 0:
            return T
        axis = axis / axis_norm
        
        # Create rotation matrix using axis-angle representation
        r = Rotation.from_rotvec(axis * angle)
        T[:3, :3] = r.as_matrix()
        return T

    def forward_kinematics(self, joint_angles):
        """
        Computes the forward kinematics.
        
        Args:
            joint_angles: List or array of 6 joint angles (in radians).
            
        Returns:
            A list of 4x4 transform matrices for every joint link in the world frame.
            The last matrix in the list is the end-effector pose.
        """
        if len(joint_angles) != len(self.chain):
            raise ValueError(f"Expected {len(self.chain)} joint angles, got {len(joint_angles)}")

        # Start at the base (Identity matrix means 0 translation, 0 rotation)
        current_transform = np.eye(4)
        link_transforms = [current_transform]

        for i, joint in enumerate(self.chain):
            # 1. Get the static transform from the parent link to this joint
            origin_xyz = joint.origin.xyz if joint.origin else [0, 0, 0]
            origin_rpy = joint.origin.rpy if joint.origin else [0, 0, 0]
            T_static = self._create_transform(origin_xyz, origin_rpy)
            
            # 2. Get the dynamic transform from the joint's current rotation
            axis = joint.axis if joint.axis else [0, 0, 1]
            T_dynamic = self._joint_rotation(axis, joint_angles[i])
            
            # 3. Combine them: T_local = T_static * T_dynamic
            T_local = np.dot(T_static, T_dynamic)
            
            # 4. Multiply with the chain so far to get the global position
            current_transform = np.dot(current_transform, T_local)
            link_transforms.append(current_transform)

        return link_transforms
    
    def _get_pose_error(self, current_pose, target_pose):
        """
        Calculates the 6D error (3D position, 3D rotation) between two poses.
        """
        # 1. Position error (Target XYZ - Current XYZ)
        dp = target_pose[:3, 3] - current_pose[:3, 3]
        
        # 2. Orientation error
        # R_error = R_target * R_current^T
        R_curr = current_pose[:3, :3]
        R_targ = target_pose[:3, :3]
        R_err = np.dot(R_targ, R_curr.T)
        
        # Convert the rotation matrix error into a 3D rotation vector
        r = Rotation.from_matrix(R_err)
        dr = r.as_rotvec()
        
        # Combine into a single 6-element array [dx, dy, dz, drx, dry, drz]
        return np.hstack((dp, dr))

    def _compute_jacobian(self, joint_angles):
        """
        Computes the 6xN Jacobian matrix using finite differences (numerical approximation).
        """
        num_joints = len(self.chain)
        J = np.zeros((6, num_joints))
        dt = 1e-5 # A tiny change in angle
        
        # Find where the arm is right now
        base_transforms = self.forward_kinematics(joint_angles)
        base_pose = base_transforms[-1]
        
        for i in range(num_joints):
            # Nudge joint i by dt
            q_perturbed = np.array(joint_angles, dtype=float)
            q_perturbed[i] += dt
            
            # Find where the arm moved to
            new_transforms = self.forward_kinematics(q_perturbed)
            new_pose = new_transforms[-1]
            
            # The rate of change is (New - Old) / dt
            error_vec = self._get_pose_error(base_pose, new_pose)
            J[:, i] = error_vec / dt
            
        return J

    def inverse_kinematics(self, target_pose, initial_guess=None, max_iter=500, tolerance=1e-4, damping=0.1):
        """
        Solves Inverse Kinematics using Damped Least-Squares (DLS).
        """
        if initial_guess is None:
            # If no guess is provided, start with all joints at 0
            q = np.zeros(len(self.chain))
        else:
            q = np.array(initial_guess, dtype=float)
            
        for i in range(max_iter):
            # 1. Find current end-effector pose
            current_transforms = self.forward_kinematics(q)
            current_pose = current_transforms[-1]
            
            # 2. Calculate how far we are from the target
            error = self._get_pose_error(current_pose, target_pose)
            error_norm = np.linalg.norm(error)
            
            # If the error is tiny, we have reached the target!
            if error_norm < tolerance:
                print(f"IK Converged in {i} iterations. Error: {error_norm:.6f}")
                return q
                
            # 3. Compute Jacobian
            J = self._compute_jacobian(q)
            
            # 4. Damped Least Squares (DLS) formula
            J_T = J.T
            lambda_sq_I = (damping ** 2) * np.eye(6)
            
            # delta_q = J^T * (J * J^T + lambda^2 * I)^-1 * error
            J_inv = np.dot(J_T, np.linalg.inv(np.dot(J, J_T) + lambda_sq_I))
            delta_q = np.dot(J_inv, error)
            
            # 5. Update joint angles
            q = q + delta_q
            
        print(f"IK Failed to converge after {max_iter} iterations. Final Error: {error_norm:.6f}")
        return q
    
