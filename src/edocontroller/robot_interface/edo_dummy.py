#!/usr/bin/env python

import numpy as np

from geometry_msgs.msg import Point, Quaternion, Pose

from edocontroller.edo_abstract_class import EdoAbstractClass
from geometry_representation import GeometryParser, GeometryPose

GRIPPER_OPENING_MAX = 2

class EdoDummy(EdoAbstractClass):
    """Dummy controller that only remember the last pose or last joint_values it received"""
    def __init__(self):

        self.current_pose = None
        self.current_joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gripper_opening = GRIPPER_OPENING_MAX

    def get_current_joint_values(self):
        return self.current_joint_values

    def get_current_pose(self):
        return GeometryParser.geometry_pose_from_pose(self.current_pose)

    def set_path_constraint(self, quaternion_wanted):
        pass

    def go_to_joint(self, joint1, joint2, joint3, joint4, joint5, joint6):
        '''joints in radiant'''
        joint_goal = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        joint_goal[0] = joint1
        joint_goal[1] = joint2
        joint_goal[2] = joint3
        joint_goal[3] = joint4
        joint_goal[4] = joint5
        joint_goal[5] = joint6
        print(joint6)
        print(joint_goal[5]*100)

        self.current_joint_values = joint_goal
        self.current_pose = None


    def go_to_pose_goal(self, pose_goal_geometry):
        """pose_goal type should be a GeometryPose object """
        # The length of the gripper is not taken into account in moveit, so we need to correct the value

        pose_goal = GeometryParser.pose_from_geometry_pose(pose_goal_geometry)
        self.current_pose = pose_goal
        self.current_joint_values = None

    def move_without_changing_orientation(self, delta):
        start_pose = self.move_group.get_current_pose()
        start_pose = GeometryParser.geometry_pose_from_pose(start_pose)
        next_position = start_pose.position + delta
        next_pose = GeometryPose(next_position, start_pose.orientation)
        self.current_pose = next_pose
        self.current_joint_values = None

    def reset_position(self):
        pass

    def close_gripper(self):
        self.gripper_opening = 0