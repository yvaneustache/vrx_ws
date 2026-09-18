"""Converts a (speed, absolute heading) setpoint into left/right thrust commands.

WAM-V default layout ('H') has two fixed aft thrusters, no azimuth: heading
is controlled purely by differential thrust (right thruster forward torque
is +z / turns left, left thruster forward torque is -z / turns right, for
their symmetric mount positions y=+-1.027).
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from vrx_apf_dock.geometry_utils import yaw_from_quaternion, wrap_to_pi

# Defaults only -- live values are ROS parameters declared in __init__,
# tunable at runtime with `ros2 param set` or rqt_reconfigure.
DEFAULT_GAINS = {
    'kp_heading': 1200.0,
    'ki_heading': 110.0,
    'kd_heading': 20.0,
    'heading_integral_limit': 0.8,
    'kp_speed': 100.0,
    'ki_speed': 20.0,
    'kd_speed': 5.0,
    'speed_integral_limit': 0.8,
    'max_thrust': 10000.0,  # 10000.0 max thrust wam-v initial, 100 wam-v mini
}


class ThrustMixerNode(Node):

    def __init__(self):
        super().__init__('thrust_mixer_node')

        self._current_yaw = None
        self._current_v = 0.0
        self._speed_cmd = 0.0
        self._heading_cmd = 0.0
        self._speed_error_integral = 0.0
        self._heading_error_integral = 0.0
        self._prev_heading_error = 0.0
        self._prev_speed_error = 0.0
        self._last_time = None

        self._gains = dict(DEFAULT_GAINS)
        self.declare_parameters('', list(DEFAULT_GAINS.items()))
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.create_subscription(Odometry, '/vrx_apf_dock/wamv/odom', self._odom_cb, 10)
        self.create_subscription(Twist, '/vrx_apf_dock/wamv/setpoint', self._setpoint_cb, 10)

        self._left_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self._right_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

    def _on_set_parameters(self, params):
        for param in params:
            if param.name in self._gains:
                self._gains[param.name] = param.value
        return SetParametersResult(successful=True)

    def _setpoint_cb(self, msg: Twist):
        self._speed_cmd = msg.linear.x
        self._heading_cmd = msg.angular.z

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self._current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._current_v = msg.twist.twist.linear.x

        stamp = msg.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9
        dt = 0.0 if self._last_time is None else max(0.0, min(1.0, now - self._last_time))
        self._last_time = now

        self._mix_and_publish(dt)

    def _mix_and_publish(self, dt):
        if self._current_yaw is None:
            return

        gains = self._gains

        if self._speed_cmd == 0.0:
            # Hard-stop request (e.g. docked, see ApfDockController.reached):
            # cut propulsion outright instead of running the PIDs against
            # _current_v, which comes from a leaky IMU-acceleration
            # integration that stays noisy/biased at low speed (the WAM-V's
            # IMU orientation is known to be imperfect, see classBoat.py) --
            # letting KP_SPEED chase that noise keeps the thrusters spinning
            # to "correct" a speed that may not really be there.
            self._speed_error_integral = 0.0
            self._heading_error_integral = 0.0
            self._left_pub.publish(Float64(data=0.0))
            self._right_pub.publish(Float64(data=0.0))
            return

        heading_error = wrap_to_pi(self._heading_cmd - self._current_yaw)
        heading_error_rate = 0.0 if dt <= 0.0 else (
            wrap_to_pi(heading_error - self._prev_heading_error) / dt)
        self._prev_heading_error = heading_error
        heading_integral_limit = gains['heading_integral_limit']
        self._heading_error_integral = max(-heading_integral_limit, min(
            heading_integral_limit, self._heading_error_integral + heading_error * dt))
        diff_thrust = (gains['kp_heading'] * heading_error
                        + gains['ki_heading'] * self._heading_error_integral
                        + gains['kd_heading'] * heading_error_rate)

        speed_error = self._speed_cmd - self._current_v
        speed_error_rate = 0.0 if dt <= 0.0 else (speed_error - self._prev_speed_error) / dt
        self._prev_speed_error = speed_error
        speed_integral_limit = gains['speed_integral_limit']
        self._speed_error_integral = max(-speed_integral_limit, min(
            speed_integral_limit, self._speed_error_integral + speed_error * dt))
        total_thrust = (gains['kp_speed'] * speed_error
                         + gains['ki_speed'] * self._speed_error_integral
                         + gains['kd_speed'] * speed_error_rate)

        max_thrust = gains['max_thrust']
        left = max(-max_thrust, min(max_thrust, total_thrust / 2.0 - diff_thrust / 2.0))
        right = max(-max_thrust, min(max_thrust, total_thrust / 2.0 + diff_thrust / 2.0))

        self._left_pub.publish(Float64(data=left))
        self._right_pub.publish(Float64(data=right))

        self.get_logger().info(
            f'yaw={self._current_yaw:.2f} heading_cmd={self._heading_cmd:.2f} '
            f'heading_error={heading_error:.2f} (rate={heading_error_rate:.2f}, '
            f'integral={self._heading_error_integral:.2f}) diff_thrust={diff_thrust:.1f} | '
            f'v={self._current_v:.2f} speed_cmd={self._speed_cmd:.2f} '
            f'(rate={speed_error_rate:.2f}) total_thrust={total_thrust:.1f} | '
            f'left={left:.1f} right={right:.1f}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ThrustMixerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
