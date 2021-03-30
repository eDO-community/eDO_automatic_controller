#!/usr/bin/env python

import numpy as np
import quaternion

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point, Pose
from geometry_msgs.msg import Transform, Vector3

from .geometry_pose import GeometryPose
from .geometry_transform import GeometryTransform
from .geometry_transformer import GeometryTransformer
from .geometry_fiducial_transform import GeometryFiducialTransform


class GeometryParser:
    """this class is responsible for transforming message into geometric object and vice versa"""

    def __init__(self):
        pass

    def np_array_from_msg(self, point_msg):
        return np.array([point_msg.x, point_msg.y, point_msg.z])

    np_array_from_msg = classmethod(np_array_from_msg)

    def msg_from_np_array(self, point_array):
        point = Point(point_array[0], point_array[1], point_array[2])
        return point

    msg_from_np_array = classmethod(msg_from_np_array)

    def quat_from_msg(self, quaternion1):
        return np.quaternion(quaternion1.w, quaternion1.x, quaternion1.y, quaternion1.z)

    quat_from_msg = classmethod(quat_from_msg)

    def msg_from_quat(self, quaternion):
        quat = Quaternion()
        quat.x = quaternion.x
        quat.y = quaternion.y
        quat.z = quaternion.z
        quat.w = quaternion.w
        return quat

    msg_from_quat = classmethod(msg_from_quat)

    def geometry_transform_from_transform(self, transform):
        translation = GeometryParser.quat_from_msg(Quaternion(
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
            0
        ))
        rotation = GeometryParser.quat_from_msg(transform.rotation)
        return GeometryTransform(translation, rotation)

    geometry_transform_from_transform = classmethod(geometry_transform_from_transform)

    def transform_from_geometry_transform(self, geometry_transform):
        translation_array = GeometryTransformer.np_array_from_quat(geometry_transform.translation.quaternion())
        translation = GeometryParser.msg_from_np_array(translation_array)
        rotation = GeometryParser.msg_from_quat(geometry_transform.rotation)
        return Transform(translation, rotation)

    transform_from_geometry_transform = classmethod(transform_from_geometry_transform)

    def geometry_pose_from_pose(self, pose_msg):
        position_array = GeometryParser.np_array_from_msg(pose_msg.position)
        position = GeometryTransformer.quat_from_np_array(position_array)
        orientation = GeometryParser.quat_from_msg(pose_msg.orientation)
        return GeometryPose(position, orientation)

    geometry_pose_from_pose = classmethod(geometry_pose_from_pose)

    def pose_from_geometry_pose(self, pose_object):
        position_array = GeometryTransformer.np_array_from_quat(pose_object.position.quaternion())
        position = GeometryParser.msg_from_np_array(position_array)
        orientation = GeometryParser.msg_from_quat(pose_object.orientation)
        return Pose(position, orientation)

    pose_from_geometry_pose = classmethod(pose_from_geometry_pose)

    def geometry_fiducial_from_fiducial(self, fiducial_msg):
        geometry_transform = self.geometry_transform_from_transform(fiducial_msg.transform)
        return GeometryFiducialTransform(fiducial_msg.fiducial_id, geometry_transform)

    geometry_fiducial_from_fiducial = classmethod(geometry_fiducial_from_fiducial)
