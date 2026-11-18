"""Small pose helpers shared by the ROS nodes (no ROS dependencies).

Keeping them here means the index mapping of the 6x6 pose covariance can be
unit-tested without ROS.
"""

import math

import numpy as np

#: Indices of the 3x3 ``[x, y, yaw]`` block inside a row-major 6x6 pose
#: covariance (``nav_msgs/Odometry``: x, y, z, roll, pitch, yaw).
POSE_COVARIANCE_INDICES = (0, 1, 5, 6, 7, 11, 30, 31, 35)


def pose_covariance(covariance):
    """3x3 ``[x, y, yaw]`` block of a row-major 6x6 pose covariance."""
    values = [float(covariance[index]) for index in POSE_COVARIANCE_INDICES]
    return np.asarray(values, dtype=float).reshape(3, 3)


def quaternion_to_yaw(quaternion):
    """Yaw of a ``geometry_msgs/Quaternion``-like object (duck-typed)."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )
