#!/usr/bin/env python3
# Portage de docking/catkin_ws/src/guerledocking/scripts/node_data_broadcaster.py (ROS1 Melodic -> ROS2 Jazzy)
"""Broadcasts the dock's pose over UDP in the exact wire format used by the
real guerledocking/guerleboat pair: "$lat,lon;roll,pitch,yaw", ASCII,
fire-and-forget, at 5 Hz -- so this node (fed by the simulated
dock_pose_source_node here) is a drop-in stand-in for the real dock's
node_data_broadcaster.py when guerleboat/node_udp_converter.py (ported here
as dock_udp_receiver_node) is the listener. IP/port are ROS 2 parameters
instead of the original's hardcoded "10.0.11.100"/5005 (both still default
to loopback:5005 for this simulator).
"""

import socket

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class DockUdpBroadcasterNode(Node):

    def __init__(self):
        super().__init__('dock_udp_broadcaster')

        self.declare_parameter('udp_ip', '127.0.0.1')
        self.declare_parameter('udp_port', 5005)
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('geo_pose_topic', '/vrx_docking_sim/dock/geo_pose')
        self.declare_parameter('string_data_topic', '/vrx_docking_sim/dock/string_data')

        self._udp_addr = (
            self.get_parameter('udp_ip').value, self.get_parameter('udp_port').value)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._latest_pose = None
        self._string_pub = self.create_publisher(
            String, self.get_parameter('string_data_topic').value, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('geo_pose_topic').value, self._pose_cb, 10)

        rate_hz = self.get_parameter('rate_hz').value
        self.create_timer(1.0 / rate_hz, self._broadcast)

    def _pose_cb(self, msg: PoseStamped):
        self._latest_pose = msg

    def _broadcast(self):
        if self._latest_pose is None:
            return
        p = self._latest_pose.pose
        data = '${},{};{},{},{}'.format(
            p.position.x, p.position.y,
            p.orientation.x, p.orientation.y, p.orientation.z)
        self._socket.sendto(data.encode('utf-8'), self._udp_addr)
        self._string_pub.publish(String(data=data))


def main(args=None):
    rclpy.init(args=args)
    node = DockUdpBroadcasterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
