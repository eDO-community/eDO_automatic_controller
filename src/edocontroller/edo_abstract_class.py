#!/usr/bin/env python

from geometry_representation import GeometryPose, GeometryParser
from env_representation import MapperEnvMoveit, DIST_END_EFFECTOR_GRIPPER_CLOSED


class EdoAbstractClass:
    """This class is an abstract class of the edo robot, it allows to test some features without starting the robot
    using a dummy, while not changing any other class
    Here are the list of all functions that should be implemented :
    __init__
    get_current_joint_values(self)
    get_current_pose(self)
    go_to_joint(self, joint1, joint2, joint3, joint4, joint5, joint6)
    go_to_pose_goal(self, pose_goal_geometry)
    reset_position(self)
    close_gripper(self)
    """

    def get_current_joint_values(self):
        raise NotImplementedError("get_current_joint_values should be implemented")

    def get_current_pose(self):
        raise NotImplementedError("get_current_pose should be implemented")

    def go_to_joint(self, joint1, joint2, joint3, joint4, joint5, joint6):
        raise NotImplementedError("go_to_joint should be implemented")

    def _correct_pose_goal(self, pose_goal_geometry):
        """The length of the gripper is not taken into account in moveit, so we need to correct the value"""
        end_effector_correction_when_reset = np.quaternion(0, 0, 0, -DIST_END_EFFECTOR_GRIPPER_CLOSED)
        rotation_end_effector = pose_goal_geometry.orientation
        end_effector_correction = rotation_end_effector * end_effector_correction_when_reset / rotation_end_effector
        position_corrected = pose_goal_geometry.position + end_effector_correction

        pose_goal_geometry_corrected = GeometryPose(position_corrected, pose_goal_geometry.orientation)
        return pose_goal_geometry_corrected

    def go_to_pose_goal(self, pose_goal_geometry):
        """pose_goal type should be a GeometryPose object """
        raise NotImplementedError("go_to_pose_goal should be implemented")

    def reset_position(self):
        raise NotImplementedError("reset_position should be implemented")

    def close_gripper(self):
        raise NotImplementedError("close_gripper should be implemented")

    def open_gripper(self):
        raise NotImplementedError("open_gripper should be implemented")