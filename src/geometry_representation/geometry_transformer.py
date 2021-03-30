#!/usr/bin/env python

import numpy as np
import quaternion


class GeometryTransformer():
    """This class is responsible for transforming geometric object into another geometric object"""
    def quat_from_np_array(self, array):
        return np.quaternion(0, array[0], array[1], array[2])

    quat_from_np_array = classmethod(quat_from_np_array)

    def np_array_from_quat(self, quaternion):
        return np.array([quaternion.x, quaternion.y, quaternion.z])

    np_array_from_quat = classmethod(np_array_from_quat)

    def array_normalized_from_quat(self, quaternion):
        quaternion_normalized = quaternion.normalized()
        array = np.array([quaternion_normalized.x, quaternion_normalized.y, quaternion_normalized.z])
        return array

    array_normalized_from_quat = classmethod(array_normalized_from_quat)