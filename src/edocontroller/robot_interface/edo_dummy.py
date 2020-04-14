#!/usr/bin/env python

import numpy as np

from geometry_msgs.msg import Pose

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

    def go_to_joint(self, joint1, joint2, joint3, joint4, joint5, joint6):
        '''joints in radiant'''
        joint_goal = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        joint_goal[0] = joint1
        joint_goal[1] = joint2
        joint_goal[2] = joint3
        joint_goal[3] = joint4
        joint_goal[4] = joint5
        joint_goal[5] = joint6

        self.current_joint_values = joint_goal
        self.current_pose = None

    def go_to_pose_goal(self, pose_goal_geometry):
        """pose_goal type should be a GeometryPose object """
        # The length of the gripper is not taken into account in moveit, so we need to correct the value

        pose_goal = pose_goal_geometry
        if isinstance(pose_goal_geometry, GeometryPose):
            pose_goal = GeometryParser.pose_from_geometry_pose(pose_goal_geometry)
        else:
            if not isinstance(pose_goal_geometry, Pose):
                raise ValueError("argument should be of type GeometryPose or Pose")
        self.current_pose = pose_goal
        self.current_joint_values = None

    def reset_position(self):
        self.current_pose = None
        self.current_joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def close_gripper(self):
        self.gripper_opening = 0

    def open_gripper(self):
        self.gripper_opening = GRIPPER_OPENING_MAX
