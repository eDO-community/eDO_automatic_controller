#!/usr/bin/env python

from geometry_msgs.msg import Transform
from geometry_msgs.msg import Pose


class MessageTransformer:
    """This class is responsible for transforming message into another message"""
    def __init__(self):
        pass

    def pose_equals(cls, pose1, pose2):
        equal = (pose1.position.x == pose2.position.x and
                pose1.position.y == pose2.position.y and
                pose1.position.z == pose2.position.z and
                pose1.orientation.x == pose2.orientation.x and
                pose1.orientation.y == pose2.orientation.y and
                pose1.orientation.z == pose2.orientation.z and
                pose1.orientation.w == pose2.orientation.w)
        return equal

    pose_equals = classmethod(pose_equals)

    def transform_equals(cls, transform1, transform2):
        equal = (transform1.translation.x == transform2.translation.x and
                 transform1.translation.y == transform2.translation.y and
                 transform1.translation.z == transform2.translation.z and
                 transform1.rotation.x == transform2.rotation.x and
                 transform1.rotation.y == transform2.rotation.y and
                 transform1.rotation.z == transform2.rotation.z and
                 transform1.rotation.w == transform2.rotation.w)
        return equal

    transform_equals = classmethod(transform_equals)

    def pose_from_transform(cls, transform):
        pose = Pose()
        pose.position = transform.translation
        pose.orientation = transform.rotation
        return pose

    pose_from_transform = classmethod(pose_from_transform)

    def transform_from_pose(cls, pose):
        transform = Transform()
        transform.translation = pose.position
        transform.rotation = pose.orientation
        return transform

    transform_from_pose = classmethod(transform_from_pose)

