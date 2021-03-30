#!/usr/bin/env python

from .geometry_representation import GeometryParser

class QuaternionExtended:

    def dot_between_quaternion(self, quat1, quat2):
        """takes two quaternions and returns the dot product of their x, y, z vector,
        w should be equal to 0"""
        quat1 = GeometryParser.array_normalized_from_quat(quat1)
        quat2 = GeometryParser.array_normalized_from_quat(quat2)

        return quat1.dot(quat2)

    dot_between_quaternion = classmethod(dot_between_quaternion)

    def apply_transform_to_unit_vector(self, rotation, point):
        new_point = rotation*point/rotation
        new_point = new_point.normalized()
        return new_point

    apply_transform_to_unit_vector = classmethod(apply_transform_to_unit_vector)
