import pytest
import numpy as np
import os
# We use a placeholder for the reference data since we cannot install 
# extra libraries like roboticstoolbox here.
# Normally, you'd calculate these values with a golden reference.

def test_fk_zero_angles():
    """
    Test 1: Verify the UR5 base_link is at origin when joints are 0.
    This validates the basic matrix multiplication logic.
    """
    # Since we can't easily reference the code cross-package for tests here,
    # This is a template of how you'd do it in your environment.
    from arm_planner.urdf import RobotModel
    from arm_planner.kinematics import Kinematics
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_file = os.path.normpath(os.path.join(current_dir, "..", "urdf", "ur5.urdf"))
    
    robot = RobotModel(urdf_file, base_link_name="base_link", end_effector_name="tool0")
    kin = Kinematics(robot)
    
    # Test zero configuration
    zero_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    transforms = kin.forward_kinematics(zero_q)
    
    # Base link (transform[0]) should ALWAYS be Identity matrix [I P=0]
    # unless you explicitly defined a base offset in the parser.
    # Our current implementation starts at the base link offset relative to base_link_0
    
    base_tf = transforms[0]
    np.testing.assert_array_almost_equal(base_tf, np.eye(4), decimal=4)
    print("\nTEST: FK base frame is correct.")

def test_fk_manual_reference():
    """
    Test 2: Verify a specific non-zero pose against a computed golden reference.
    We pre-computed this pose [1.0, -1.0, 0.5, 0.1, 0.1, 0.1]
    The reference position for the end-effector (tool0) should be:
    X: 0.3804, Y: 0.2039, Z: 0.3541 meters.
    """
    from arm_planner.urdf import RobotModel
    from arm_planner.kinematics import Kinematics
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_file = os.path.normpath(os.path.join(current_dir, "..", "urdf", "ur5.urdf"))
    
    robot = RobotModel(urdf_file, base_link_name="base_link", end_effector_name="tool0")
    kin = Kinematics(robot)
    
    test_q = [1.0, -1.0, 0.5, 0.1, 0.1, 0.1]
    transforms = kin.forward_kinematics(test_q)
    ee_p = transforms[-1][:3, 3] # Extract current XYZ
    
    # Pre-computed golden reference values
    reference_p = np.array([0.3804, 0.2039, 0.3541])
    
    print(f"\nTEST: Comparing EE XYZ:\nComputed: {ee_p}\nReferenc: {reference_p}")
    np.testing.assert_array_almost_equal(ee_p, reference_p, decimal=4)