"""Minimal planar quaternion <-> yaw helpers (avoids a tf_transformations dependency)."""

import math


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
