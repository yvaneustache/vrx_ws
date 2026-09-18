"""Draws the current and desired heading vectors as arrows in the Gazebo GUI.

Two arrows are kept updated in place (fixed ns/id, action=ADD_MODIFY):
  - "current" (red): the WAM-V's actual heading, from its own odometry.
  - "desired" (green): the APF setpoint heading (thetabar).
Both start at the WAM-V's current position. See gz_marker_utils for how the
arrows are actually drawn (gz-transport, /marker_array service).
"""

import rclpy
from rclpy.node import Node as RosNode

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from vrx_apf_dock.geometry_utils import yaw_from_quaternion
from vrx_apf_dock import gz_marker_utils as gzm

import gz.transport13 as gz_transport

NS = 'vrx_apf_dock'

MIN_SHAFT_LENGTH = 1.0
LENGTH_PER_MPS = 6.0  # shaft length at v=VMAX (apf_lib.py) matches the old fixed 6.0


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

        self.create_subscription(Odometry, '/vrx_apf_dock/wamv/odom', self._odom_cb, 10)
        self.create_subscription(Twist, '/vrx_apf_dock/wamv/setpoint', self._setpoint_cb, 10)

        self._gz_node = gz_transport.Node()

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
        # Triggered directly from the odom callback (not a separate timer) so
        # the markers use the freshest pose available instead of lagging by
        # up to one extra timer period behind the WAM-V's actual position.
        if self._wamv_xy is None:
            return
        markers = (
            gzm.make_arrow_markers(
                NS, 1, self._wamv_xy, self._wamv_yaw,
                _shaft_length(self._current_v), (0.9, 0.1, 0.1))
            + gzm.make_arrow_markers(
                NS, 3, self._wamv_xy, self._heading_cmd,
                _shaft_length(self._speed_cmd), (0.1, 0.9, 0.1)))
        gzm.publish_markers(self._gz_node, markers)


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
