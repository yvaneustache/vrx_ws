"""Draws the APF docking law's vector field around the dock in the Gazebo GUI.

Same idea as scripts/plot_apf_field.py (matplotlib quiver, offline) but live,
in-place around the dock's actual (possibly drifting) pose in the running
simulation: samples ApfDockController.compute() on a grid centred on the
dock, using a FRESH controller instance per grid point (same reasoning as
the matplotlib script -- a stateful run's hysteresis latches shouldn't leak
between independent field samples), and draws each as a small arrow via
gz_marker_utils (/marker_array service, GUI-only -- no-op in headless mode).

Gains mirror apf_controller_node's own live-tunable parameters: this node
polls /apf_controller_node/get_parameters on a timer rather than keeping its
own separate copy, so the field redraws to match whatever you're dialling in
via rqt_reconfigure on the real controller. Falls back to apf_lib's
DEFAULT_GAINS (and keeps retrying) if that node isn't up yet.
"""

import math

import rclpy
from rclpy.node import Node as RosNode
from rcl_interfaces.srv import GetParameters

from geometry_msgs.msg import PoseStamped

from vrx_apf_dock.geometry_utils import yaw_from_quaternion
from vrx_apf_dock.apf_lib import ApfDockController, DEFAULT_GAINS
from vrx_apf_dock import gz_marker_utils as gzm

import gz.transport13 as gz_transport

NS = 'vrx_apf_dock_field'

SLOW_COLOR = (0.1, 0.1, 0.9)   # blue
FAST_COLOR = (1.0, 0.9, 0.1)   # yellow

GAIN_NAMES = list(DEFAULT_GAINS.keys())


def _speed_color(speed, vmax):
    t = 0.0 if vmax <= 0.0 else max(0.0, min(1.0, abs(speed) / vmax))
    return tuple(lo + t * (hi - lo) for lo, hi in zip(SLOW_COLOR, FAST_COLOR))


class ApfFieldMarkerNode(RosNode):

    def __init__(self):
        super().__init__('apf_field_marker_node')

        self.declare_parameter('grid_size', 50.0)
        self.declare_parameter('grid_step', 3.0)
        self.declare_parameter('update_period', 1.0)
        self.declare_parameter('max_shaft_length', 1.0)

        self._dock_xy = None
        self._dock_yaw = None
        self._gains = dict(DEFAULT_GAINS)

        self.create_subscription(PoseStamped, '/vrx_apf_dock/dock/pose', self._dock_pose_cb, 10)
        self._gz_node = gz_transport.Node()

        self._gains_client = self.create_client(
            GetParameters, '/apf_controller_node/get_parameters')

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
            # apf_controller_node not up (e.g. teleop-only launch, or still
            # starting) -- keep drawing with the last-known/default gains
            # instead of blocking the field on it.
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
        max_shaft = self.get_parameter('max_shaft_length').value
        half = size / 2.0
        coords = [half - i * step for i in range(int(size / step) + 1)]
        # (descending order is arbitrary -- just needs to cover [-half, half])

        dock_x, dock_y = self._dock_xy
        vmax = self._gains['vmax']

        markers = []
        marker_id = 1
        for dx in coords:
            for dy in coords:
                if math.hypot(dx, dy) < 1e-6:
                    continue  # skip the dock's own position (singular)
                x, y = dock_x + dx, dock_y + dy
                controller = ApfDockController(gains=dict(self._gains))
                v_cmd, theta_cmd = controller.compute((x, y), self._dock_xy, self._dock_yaw)
                shaft_length = min(max_shaft, 0.1 + max_shaft * (
                    0.0 if vmax <= 0.0 else abs(v_cmd) / vmax))
                markers += gzm.make_arrow_markers(
                    NS, marker_id, (x, y), theta_cmd, 1.0, #shaft_length,
                    FAST_COLOR, #_speed_color(v_cmd, vmax),
                    shaft_radius=0.03, head_length=0.15, head_radius=0.08,
                    z_offset=0.5)
                marker_id += 2

        gzm.publish_markers(self._gz_node, markers)


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
