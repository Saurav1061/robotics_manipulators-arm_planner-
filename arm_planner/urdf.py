from urdf_parser_py.urdf import URDF
import numpy as np
import os # <--- Add this import

class RobotModel:
    def __init__(self, urdf_path, base_link_name, end_effector_name):
        """
        Parses the URDF and extracts the serial kinematic chain.
        """
        self.robot = URDF.from_xml_file(urdf_path)
        self.base_link = base_link_name
        self.ee_link = end_effector_name
        
        # Dictionaries for quick lookup
        self.joint_map = {joint.name: joint for joint in self.robot.joints}
        self.link_map = {link.name: link for link in self.robot.links}
        
        # Extract the ordered list of joints that make up the arm
        self.chain_joints = self._extract_chain()

    def _extract_chain(self):
        """
        Walks the tree from the base link to the end-effector link.
        Returns a list of joint objects in order.
        """
        chain = []
        current_link = self.ee_link
        
        # Build a child-to-parent mapping
        child_to_parent_joint = {}
        for joint in self.robot.joints:
            child_to_parent_joint[joint.child] = joint
            
        # Trace backwards from end-effector to base
        while current_link != self.base_link:
            if current_link not in child_to_parent_joint:
                raise ValueError(f"Could not find path back to base. Stuck at link: {current_link}")
            
            joint = child_to_parent_joint[current_link]
            chain.append(joint)
            current_link = joint.parent
            
        # Reverse to get the order from base to end-effector
        chain.reverse()
        
        # Filter out fixed joints (we only want joints that move)
        movable_joints = [j for j in chain if j.type in ['revolute', 'continuous', 'prismatic']]
        return movable_joints

    def print_chain_info(self):
        """Utility to verify the parsed data."""
        print(f"Kinematic Chain ({len(self.chain_joints)} movable joints):")
        for i, joint in enumerate(self.chain_joints):
            origin_xyz = joint.origin.xyz if joint.origin else [0, 0, 0]
            axis = joint.axis if joint.axis else [0, 0, 0]
            print(f"{i+1}. {joint.name} | Type: {joint.type} | Axis: {axis} | Origin XYZ: {origin_xyz}")

