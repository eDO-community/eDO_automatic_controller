#!/usr/bin/env python

import numpy as np
import quaternion

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point, Pose
from geometry_msgs.msg import Transform, Vector3

from .vector_quaternion import VectorQuaternion
from .geometry_transform import GeometryTransform

class GeometryFiducialTransform:
    """Representation of fuducial transform message using VectorQuaternion and np quaternion"""
    def __init__(self, id, geometry_transform):
        # a quaternion with 0 as w is the same as a vector, translation and rotation are quaternions
        # two VectorQuaternion should be given, and the one with position should have a 0 as w
        self.fiducial_id = id

        if isinstance(geometry_transform, GeometryTransform):
            self.transform = geometry_transform
        else:
            raise NotImplementedError("transform of type : " + str(geometry_transform.__class__.__name__) + " does not permit initialisation of class")


    def __repr__(self):
        return "[id: " + str(self.fiducial_id) + "transform : " + str(self.transform) + "]"

