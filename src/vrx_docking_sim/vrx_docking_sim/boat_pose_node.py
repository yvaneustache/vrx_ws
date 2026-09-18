#!/usr/bin/env python3
# Portage de docking/catkin_ws/src/guerleboat/scripts/node_boat.py (ROS1 Melodic -> ROS2 Jazzy)
"""Converts the boat's own pose and the dock's UDP-received pose (raw
lat/lon, see dock_udp_receiver_node) into the same local frame -- exactly
node_boat.py's job in the original guerleboat, which is the only place in
that codebase where both the boat's and the dock's positions get expressed
in one shared coordinate system.

Livrable 3: the boat's own pose source is Gazebo ground truth
(/vrx_docking_sim/wamv/odometry, already in the local ENU frame -- no
lat/lon conversion needed for it here). It will be replaced by MAVROS's
local_position/odom once ArduPilot SITL is wired in (livrable 4); this
node's *output* topics/contract stay the same either way, matching how
node_boat.py itself is decoupled from where the boat's raw fix comes from.

The dock's lat/lon is converted back to local (x, y) with the exact inverse
of dock_pose_source_node's equirectangular projection, around the same
sydney_regatta origin -- both nodes must agree on that origin for the two
poses to land in the same frame.

Also broadcasts dynamic "world" -> "wamv" and "world" -> "dock" TF
transforms matching the boat's and dock's live poses (discovered useful
while validating livrable 7: with Fixed Frame set to "wamv" or "dock" in
RViz2 instead of "world", that vehicle sits at its own frame's origin at
all times, so a camera view centred on that origin visually follows it --
RViz2 has no separate "center on model" feature, this is the standard way
to get one).

Both transforms are re-broadcast on a fast fixed-rate timer (tf_rate_hz),
stamped with the *current* sim time, rather than only from their source
callback with that message's own stamp. The dock's pose only updates at
the UDP link's 5 Hz (faithful to the original guerledocking wire rate --
see dock_udp_broadcaster_node), while marker nodes stamp their output with
"now" at a much higher rate; a marker timestamped even slightly ahead of
the latest available TF sample is silently dropped by RViz2's tf2 message
filter until a newer sample lands, which at 5 Hz is visible as flicker
(discovered live: "les fleches apparaissent puis disparaissent" with
Fixed Frame "dock", and the same for "wamv" under clock/DDS jitter between
separate node processes). Re-publishing the last known pose at a high,
uniform rate keeps the TF buffer's newest sample close enough to "now"
that this race stops happening, independent of how often the underlying
pose source itself actually updates.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from vrx_apf_dock.geometry_utils import quaternion_from_yaw

EARTH_RADIUS_M = 6378137.0


class BoatPoseNode(Node):

    def __init__(self):
        super().__init__('boat_pose')

        self.declare_parameter('origin_lat_deg', -33.724223)
        self.declare_parameter('origin_lon_deg', 150.679736)
        self.declare_parameter('wamv_odom_topic', '/vrx_docking_sim/wamv/odometry')
        self.declare_parameter('dock_udp_pose_topic', '/vrx_docking_sim/dock/udp_pose')
        self.declare_parameter('wamv_pose_topic', '/vrx_docking_sim/wamv/pose')
        self.declare_parameter('dock_pose_topic', '/vrx_docking_sim/dock/pose')
        self.declare_parameter('tf_rate_hz', 20.0)

        self._origin_lat = self.get_parameter('origin_lat_deg').value
        self._origin_lon = self.get_parameter('origin_lon_deg').value
        self._lon_scale = math.cos(math.radians(self._origin_lat))

        self._wamv_pub = self.create_publisher(
            PoseStamped, self.get_parameter('wamv_pose_topic').value, 10)
        self._dock_pub = self.create_publisher(
            PoseStamped, self.get_parameter('dock_pose_topic').value, 10)

        self.create_subscription(
            Odometry, self.get_parameter('wamv_odom_topic').value, self._wamv_odom_cb, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('dock_udp_pose_topic').value, self._dock_udp_cb, 10)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._wamv_transform = None
        self._dock_transform = None

        tf_period = 1.0 / self.get_parameter('tf_rate_hz').value
        self.create_timer(tf_period, self._broadcast_transforms)

    def _wamv_odom_cb(self, msg: Odometry):
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose.pose
        self._wamv_pub.publish(out)

        tf_msg = TransformStamped()
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'wamv'
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self._wamv_transform = tf_msg

    def _dock_udp_cb(self, msg: PoseStamped):
        # Inverse of dock_pose_source_node's local -> lat/lon projection.
        lat = msg.pose.position.x
        lon = msg.pose.position.y
        yaw = msg.pose.orientation.z

        y = math.radians(lat - self._origin_lat) * EARTH_RADIUS_M
        x = math.radians(lon - self._origin_lon) * EARTH_RADIUS_M * self._lon_scale

        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = 'world'
        out.pose.position.x = x
        out.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self._dock_pub.publish(out)

        tf_msg = TransformStamped()
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'dock'
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = out.pose.orientation
        self._dock_transform = tf_msg

    def _broadcast_transforms(self):
        now = self.get_clock().now().to_msg()
        for tf_msg in (self._wamv_transform, self._dock_transform):
            if tf_msg is None:
                continue
            tf_msg.header.stamp = now
            self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoatPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
