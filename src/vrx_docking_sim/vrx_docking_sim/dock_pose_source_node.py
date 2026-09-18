#!/usr/bin/env python3
# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
"""Simulates the dock's onboard GPS+IMU: converts its Gazebo ground-truth
odometry (local ENU, from docking_sim_spawn.launch.py's odometry bridge)
into lat/lon + roll/pitch/yaw, the same representation a real GPS+SBG would
produce on the physical dock. Feeds dock_udp_broadcaster_node.

Lat/lon uses an equirectangular approximation around the world's
spherical_coordinates origin (see sydney_regatta.sdf) -- accurate enough at
the few-hundred-metre scale of this simulator, not meant for geodesy.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

# WGS84 equatorial radius, sufficient for a local flat-earth approximation.
EARTH_RADIUS_M = 6378137.0


def euler_from_quaternion(x, y, z, w):
    """Standard roll/pitch/yaw decomposition (radians), aerospace convention."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class DockPoseSourceNode(Node):

    def __init__(self):
        super().__init__('dock_pose_source')

        self.declare_parameter('origin_lat_deg', -33.724223)
        self.declare_parameter('origin_lon_deg', 150.679736)
        self.declare_parameter('odom_topic', '/vrx_docking_sim/dock/odometry')
        self.declare_parameter('geo_pose_topic', '/vrx_docking_sim/dock/geo_pose')

        self._origin_lat = self.get_parameter('origin_lat_deg').value
        self._origin_lon = self.get_parameter('origin_lon_deg').value
        self._lon_scale = math.cos(math.radians(self._origin_lat))

        self._pub = self.create_publisher(
            PoseStamped, self.get_parameter('geo_pose_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 10)

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        lat = self._origin_lat + math.degrees(y / EARTH_RADIUS_M)
        lon = self._origin_lon + math.degrees(x / (EARTH_RADIUS_M * self._lon_scale))

        out = PoseStamped()
        out.header = msg.header
        # Same field reuse as the original guerledocking/guerleboat wire
        # format: position.x/y carry lat/lon (not metres), orientation.x/y/z
        # carry roll/pitch/yaw (not a real quaternion) -- kept for fidelity
        # with the ported protocol, see CORRESPONDANCE_DOCKING.md.
        out.pose.position.x = lat
        out.pose.position.y = lon
        out.pose.orientation.x = roll
        out.pose.orientation.y = pitch
        out.pose.orientation.z = yaw
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DockPoseSourceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
