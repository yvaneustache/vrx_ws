#!/usr/bin/env python3
# Portage de docking/catkin_ws/src/guerleboat/scripts/node_udp_converter.py (ROS1 Melodic -> ROS2 Jazzy)
"""Listens for the dock's UDP broadcast ("$lat,lon;roll,pitch,yaw", see
dock_udp_broadcaster_node) and republishes it on ROS -- the USV-side half
of the guerledocking/guerleboat UDP link, ported from
guerleboat/scripts/node_udp_converter.py. Publishes the pose exactly as
received (lat/lon, not yet converted to a local frame): that conversion is
guerleboat/node_boat.py's job, ported here as boat_pose_node (livrable 3),
which needs the boat's own GPS origin to convert both boat and dock into
the same local frame.
"""

import socket
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class DockUdpReceiverNode(Node):

    def __init__(self):
        super().__init__('dock_udp_receiver')

        self.declare_parameter('udp_ip', '0.0.0.0')
        self.declare_parameter('udp_port', 5005)
        self.declare_parameter('udp_pose_topic', '/vrx_docking_sim/dock/udp_pose')

        self._pub = self.create_publisher(
            PoseStamped, self.get_parameter('udp_pose_topic').value, 10)

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.settimeout(1.0)
        self._socket.bind((
            self.get_parameter('udp_ip').value, self.get_parameter('udp_port').value))

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def _recv_loop(self):
        while not self._stop.is_set():
            try:
                data, _addr = self._socket.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            self._handle_packet(data)

    def _handle_packet(self, data: bytes):
        text = data.decode('utf-8', errors='replace')
        if not text.startswith('$'):
            self.get_logger().warning('##### ERROR, WRONG DATA FORMAT #####: %r' % text)
            return
        try:
            gps, angles = text[1:].split(';')
            lat, lon = (float(v) for v in gps.split(','))
            roll, pitch, yaw = (float(v) for v in angles.split(','))
        except ValueError:
            self.get_logger().warning('##### ERROR, WRONG DATA FORMAT #####: %r' % text)
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'dock_gps'
        # Same lat/lon-in-position, rpy-in-orientation convention as the
        # sender (dock_udp_broadcaster_node) -- see its docstring.
        msg.pose.position.x = lat
        msg.pose.position.y = lon
        msg.pose.orientation.x = roll
        msg.pose.orientation.y = pitch
        msg.pose.orientation.z = yaw
        self._pub.publish(msg)

    def destroy_node(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DockUdpReceiverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
