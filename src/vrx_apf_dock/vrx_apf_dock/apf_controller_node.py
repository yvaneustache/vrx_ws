"""Runs the artificial potential field docking law at a fixed rate.

Publishes a geometry_msgs/Twist repurposed as a speed/heading setpoint:
  linear.x  = vbar   (desired forward speed, m/s)
  angular.z = thetabar (desired ABSOLUTE heading, rad -- not a turn rate)
This avoids introducing a custom message type for a two-field setpoint.

Published on /vrx_apf_dock/wamv/setpoint_apf, NOT the final
/vrx_apf_dock/wamv/setpoint that thrust_mixer_node consumes -- this node
always computes the docking law regardless of who's actually driving, and
vrx_apf_dock_teleop's joystick_setpoint_node is the single arbitrator that
forwards either this or a manual joystick setpoint onto the final topic
(toggled with the LT button). Keeps this node fully unaware of teleop.

Also listens on /vrx_apf_dock/apf/reset (std_msgs/Empty) to reset the APF
controller's internal state (reached/switch_value/active latches) on
demand -- e.g. joystick_setpoint_node fires this each time docking mode is
re-engaged, so a second docking attempt in the same run doesn't inherit
"reached" from a previous one. Generic trigger, no teleop-specific meaning
here.

The APF law's gains (c11/c12/c21/c22/vmax/margin/transition_dist/
reached_radius/safety_radius/safety_gain, see apf_lib.DEFAULT_GAINS) are
declared as ROS parameters, live-tunable with `ros2 param set` or
rqt_reconfigure -- same mechanism as thrust_mixer_node's PID gains.
"""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

from vrx_apf_dock.geometry_utils import yaw_from_quaternion
from vrx_apf_dock.apf_lib import ApfDockController, DEFAULT_GAINS

CONTROL_PERIOD = 0.05  # s, matches dt in the reference controller.py


class ApfControllerNode(Node):

    def __init__(self):
        super().__init__('apf_controller_node')

        self._wamv_xy = None
        self._wamv_yaw = None
        self._dock_xy = None
        self._dock_yaw = None

        self._apf = ApfDockController()
        self.declare_parameters('', list(DEFAULT_GAINS.items()))
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.create_subscription(Odometry, '/vrx_apf_dock/wamv/odom', self._wamv_odom_cb, 10)
        self.create_subscription(PoseStamped, '/vrx_apf_dock/dock/pose', self._dock_pose_cb, 10)
        self.create_subscription(Empty, '/vrx_apf_dock/apf/reset', self._reset_cb, 10)

        self._setpoint_pub = self.create_publisher(Twist, '/vrx_apf_dock/wamv/setpoint_apf', 10)
        self.create_timer(CONTROL_PERIOD, self._on_timer)

    def _on_set_parameters(self, params):
        for param in params:
            if param.name in self._apf.gains:
                self._apf.gains[param.name] = param.value
        return SetParametersResult(successful=True)

    def _reset_cb(self, msg: Empty):
        self._apf.reset()
        self.get_logger().info('APF controller state reset')

    def _wamv_odom_cb(self, msg: Odometry):
        self._wamv_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self._wamv_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _dock_pose_cb(self, msg: PoseStamped):
        self._dock_xy = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation
        self._dock_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_timer(self):
        if self._wamv_xy is None or self._dock_xy is None:
            return
        v_cmd, theta_cmd = self._apf.compute(self._wamv_xy, self._dock_xy, self._dock_yaw)
        if self._apf.reached:
            # compute() returns a literal (0, 0) once docked; 0 rad is an
            # absolute heading (east), not "stop turning", so holding it
            # would make the boat swing to face east instead of settling.
            # Hold the current heading instead so heading_error stays ~0
            # and the thrust_mixer's PID actually converges to zero thrust.
            theta_cmd = self._wamv_yaw
        setpoint = Twist()
        setpoint.linear.x = float(v_cmd)
        setpoint.angular.z = float(theta_cmd)
        self._setpoint_pub.publish(setpoint)

        r = math.hypot(self._wamv_xy[0] - self._dock_xy[0], self._wamv_xy[1] - self._dock_xy[1])
        self.get_logger().info(
            f'wamv pos=({self._wamv_xy[0]:.2f}, {self._wamv_xy[1]:.2f}) '
            f'yaw={self._wamv_yaw:.2f} | '
            f'dock pos=({self._dock_xy[0]:.2f}, {self._dock_xy[1]:.2f}) '
            f'yaw={self._dock_yaw:.2f} | r={r:.2f} | '
            f'setpoint v={v_cmd:.2f} theta={theta_cmd:.2f} reached={self._apf.reached}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ApfControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
