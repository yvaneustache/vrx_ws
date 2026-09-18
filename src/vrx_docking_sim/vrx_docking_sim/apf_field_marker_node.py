#!/usr/bin/env python3
# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
"""Draws the APF docking law's vector field around the dock -- new copy for
vrx_docking_sim, distinct from vrx_apf_dock/apf_field_marker_node.py (which
stays untouched, Mode A), same idea otherwise: sample
apf_lib.ApfDockController.compute() on a grid centred on the dock's live
pose (/vrx_docking_sim/dock/pose, the UDP-derived pose, so the field
follows the dock even if it drifts/is driven by the joystick), a fresh
controller instance per grid point (hysteresis latches must not leak
between independent samples), published both to the Gazebo GUI
(gz-transport, gz_marker_utils, reused unchanged by import) and to RViz2
(visualization_msgs/MarkerArray, ros_marker_utils) -- see
PROPOSITION_SIMULATEUR_COMPLET.md §3.3.

Gains mirror this workspace's actual APF controller
(/docking_apf_controller/get_parameters, vrx_docking_sim's node name --
NOT vrx_apf_dock's apf_controller_node, a different node), polled on a
timer so the field redraws to match whatever's tuned live via
`ros2 param set`/rqt_reconfigure. Falls back to apf_lib's DEFAULT_GAINS
(and keeps retrying) if that node isn't up yet.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node as RosNode
from visualization_msgs.msg import MarkerArray

from vrx_apf_dock import gz_marker_utils as gzm
from vrx_apf_dock.apf_lib import DEFAULT_GAINS, ApfDockController
from vrx_apf_dock.geometry_utils import yaw_from_quaternion
from vrx_docking_sim import ros_marker_utils as rosm

import gz.transport13 as gz_transport

NS = 'vrx_docking_sim_field'
FRAME_ID = 'world'

FAST_COLOR = (1.0, 0.9, 0.1)   # yellow

GAIN_NAMES = list(DEFAULT_GAINS.keys())


class ApfFieldMarkerNode(RosNode):

    def __init__(self):
        super().__init__('apf_field_marker_node')

        self.declare_parameter('grid_size', 50.0)
        self.declare_parameter('grid_step', 3.0)
        self.declare_parameter('update_period', 1.0)

        self._dock_xy = None
        self._dock_yaw = None
        self._gains = dict(DEFAULT_GAINS)

        self.create_subscription(
            PoseStamped, '/vrx_docking_sim/dock/pose', self._dock_pose_cb, 10)

        self._gz_node = gz_transport.Node()
        self._ros_marker_pub = self.create_publisher(
            MarkerArray, '/vrx_docking_sim/markers', 10)

        self._gains_client = self.create_client(
            GetParameters, '/docking_apf_controller/get_parameters')

        period = self.get_parameter('update_period').value
        self.create_timer(period, self._on_timer)

    def _dock_pose_cb(self, msg: PoseStamped):
        self._dock_xy = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation
        self._dock_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_timer(self):
        self._refresh_gains()
        self._publish_field()

    def _refresh_gains(self):
        if not self._gains_client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = GAIN_NAMES
        self._gains_client.call_async(request).add_done_callback(self._on_gains_response)

    def _on_gains_response(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 -- log and keep last-known gains
            self.get_logger().warning(f'failed to fetch APF gains: {exc}', throttle_duration_sec=5.0)
            return
        for name, value in zip(GAIN_NAMES, response.values):
            self._gains[name] = value.double_value

    def _publish_field(self):
        if self._dock_xy is None:
            return

        size = self.get_parameter('grid_size').value
        step = self.get_parameter('grid_step').value
        half = size / 2.0
        coords = [half - i * step for i in range(int(size / step) + 1)]

        dock_x, dock_y = self._dock_xy

        stamp = self.get_clock().now().to_msg()
        gz_markers = []
        ros_markers = []
        marker_id = 1
        for dx in coords:
            for dy in coords:
                if math.hypot(dx, dy) < 1e-6:
                    continue
                x, y = dock_x + dx, dock_y + dy
                controller = ApfDockController(gains=dict(self._gains))
                _v_cmd, theta_cmd = controller.compute((x, y), self._dock_xy, self._dock_yaw)
                gz_markers += gzm.make_arrow_markers(
                    NS, marker_id, (x, y), theta_cmd, 1.0, FAST_COLOR,
                    shaft_radius=0.03, head_length=0.15, head_radius=0.08, z_offset=0.5)
                ros_markers.append(rosm.make_arrow_marker(
                    FRAME_ID, NS, marker_id, (x, y), theta_cmd, 1.0, FAST_COLOR, stamp,
                    shaft_diameter=0.06, head_diameter=0.16, head_length=0.15, z_offset=0.5))
                marker_id += 2

        gzm.publish_markers(self._gz_node, gz_markers)
        self._ros_marker_pub.publish(MarkerArray(markers=ros_markers))


def main(args=None):
    rclpy.init(args=args)
    node = ApfFieldMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
