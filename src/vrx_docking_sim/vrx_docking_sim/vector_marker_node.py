#!/usr/bin/env python3
# Portage de docking/catkin_ws/src/guerleboat/scripts/node_visualization.py (ROS1 Melodic -> ROS2 Jazzy)
"""Draws the current and desired heading vectors as arrows -- new copy for
vrx_docking_sim, distinct from vrx_apf_dock/vector_marker_node.py (which
stays untouched and keeps serving Mode A, see CORRESPONDANCE_DOCKING.md).

Two arrows, published both to the Gazebo GUI (gz-transport, via
vrx_apf_dock.gz_marker_utils, reused unchanged by import) and to RViz2
(visualization_msgs/MarkerArray, via ros_marker_utils, new in this
package) -- see PROPOSITION_SIMULATEUR_COMPLET.md §3.3:
  - "current" (red): the WAM-V's actual heading/speed, from Gazebo ground
    truth odometry (/vrx_docking_sim/wamv/odometry) -- deliberately not the
    EKF estimate apf_controller_node uses internally: this is a display
    tool, not part of the control loop, and ground truth is simpler and
    arguably more informative here than a filtered estimate.
  - "desired" (green): the APF setpoint (/vrx_docking_sim/wamv/setpoint_apf,
    linear.x=speed, angular.z=absolute heading -- same convention as
    apf_controller_node's output, see that node's docstring).
"""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node as RosNode
from visualization_msgs.msg import MarkerArray

from vrx_apf_dock import gz_marker_utils as gzm
from vrx_apf_dock.geometry_utils import yaw_from_quaternion
from vrx_docking_sim import ros_marker_utils as rosm

import gz.transport13 as gz_transport

NS = 'vrx_docking_sim'
FRAME_ID = 'world'

MIN_SHAFT_LENGTH = 1.0
LENGTH_PER_MPS = 6.0


def _shaft_length(speed):
    return MIN_SHAFT_LENGTH + LENGTH_PER_MPS * abs(speed)


class VectorMarkerNode(RosNode):

    def __init__(self):
        super().__init__('vector_marker_node')

        self._wamv_xy = None
        self._wamv_yaw = None
        self._current_v = 0.0
        self._heading_cmd = 0.0
        self._speed_cmd = 0.0

        self.create_subscription(
            Odometry, '/vrx_docking_sim/wamv/odometry', self._odom_cb, 10)
        self.create_subscription(
            Twist, '/vrx_docking_sim/wamv/setpoint_apf', self._setpoint_cb, 10)

        self._gz_node = gz_transport.Node()
        self._ros_marker_pub = self.create_publisher(
            MarkerArray, '/vrx_docking_sim/markers', 10)

    def _odom_cb(self, msg: Odometry):
        self._wamv_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self._wamv_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._current_v = msg.twist.twist.linear.x
        self._publish_markers()

    def _setpoint_cb(self, msg: Twist):
        self._speed_cmd = msg.linear.x
        self._heading_cmd = msg.angular.z

    def _publish_markers(self):
        if self._wamv_xy is None:
            return

        current_len = _shaft_length(self._current_v)
        desired_len = _shaft_length(self._speed_cmd)

        gz_markers = (
            gzm.make_arrow_markers(NS, 1, self._wamv_xy, self._wamv_yaw,
                                    current_len, (0.9, 0.1, 0.1))
            + gzm.make_arrow_markers(NS, 3, self._wamv_xy, self._heading_cmd,
                                      desired_len, (0.1, 0.9, 0.1)))
        gzm.publish_markers(self._gz_node, gz_markers)

        stamp = self.get_clock().now().to_msg()
        ros_markers = MarkerArray(markers=[
            rosm.make_arrow_marker(FRAME_ID, NS, 1, self._wamv_xy, self._wamv_yaw,
                                    current_len, (0.9, 0.1, 0.1), stamp),
            rosm.make_arrow_marker(FRAME_ID, NS, 2, self._wamv_xy, self._heading_cmd,
                                    desired_len, (0.1, 0.9, 0.1), stamp),
        ])
        self._ros_marker_pub.publish(ros_markers)


def main(args=None):
    rclpy.init(args=args)
    node = VectorMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
