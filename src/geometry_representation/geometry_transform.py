#!/usr/bin/env python

import numpy as np
import quaternion

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point, Pose
from geometry_msgs.msg import Transform, Vector3

from .vector_quaternion import VectorQuaternion

class GeometryTransform:
    """Representation of transform message using VectorQuaternion and np quaternion"""
    def __init__(self, translation, rotation):
        # a quaternion with 0 as w is the same as a vector, translation and rotation are quaternions
        # two VectorQuaternion should be given, and the one with position should have a 0 as w
        if isinstance(translation, quaternion.quaternion) or isinstance(translation, np.ndarray):
            self.translation = VectorQuaternion(translation)
        elif isinstance(translation, VectorQuaternion):
            self.translation = translation
        else:
            raise NotImplementedError("position of type : " + str(type(translation)) + " does not permit initialisation of class")

        if isinstance(rotation, quaternion.quaternion):
            self.rotation = rotation
        else:
            raise NotImplementedError("orientation of type : " + str(type(rotation)) + " does not permit initialisation of class")

    def __repr__(self):
        return "[Translation : np_" + repr(self.translation) + "\nRotation : np_" + repr(self.rotation) + "]"

    def __eq__(self, other):
        print(self.translation)
        translation = self.translation.quaternion()
        other_translation = other.translation.quaternion()
        if isinstance(other, GeometryTransform) and \
                (translation.x == other_translation.x) and \
                (translation.y == other_translation.y) and \
                (translation.z == other_translation.z) and \
                (translation.w == other_translation.w) and \
                (self.rotation.x == other.rotation.x) and \
                (self.rotation.y == other.rotation.y) and \
                (self.rotation.z == other.rotation.z) and \
                (self.rotation.w == other.rotation.w):
            return True
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)
