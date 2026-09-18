"""Joystick teleop for tuning thrust_mixer_node's PID gains directly, with a
one-button handoff to the APF docking controller.

Sole arbitrator on /vrx_apf_dock/wamv/setpoint (the topic thrust_mixer_node
actually consumes), forwarding one of two sources depending on the LT
toggle:
  - manual: this node's own stick/button-derived setpoint (see mapping
    below) -- the default on startup.
  - docking: apf_controller_node's output, received here on
    /vrx_apf_dock/wamv/setpoint_apf (that node always runs and always
    computes the docking law, but publishes to setpoint_apf instead of the
    final topic precisely so it can be arbitrated here rather than fighting
    this node over the same topic).
Run via apf_dock_teleop.launch.py, which starts apf_controller_node
unremapped (-> setpoint_apf) alongside this node.

Mapping (matches the axis/button convention already used for direct-thrust
teleop in vrx_gz/config/wamv.yaml):
  - Left stick vertical (axis 1): speed setpoint, scaled by ~max_speed.
  - Right stick horizontal (axis 2): heading rate (virtual rudder), scaled
    by ~max_heading_rate and integrated over time into an absolute heading
    setpoint -- thrust_mixer_node expects an absolute heading, not a rate.
  - Deadman button 4 (L1): must be held to command anything *in manual
    mode*; releasing it hard-stops (speed_cmd=0), matching
    thrust_mixer_node's own reached/idle convention, and freezes (does not
    reset) the heading setpoint so re-engaging the deadman doesn't snap-turn
    the boat toward a stale target. Not required in docking mode -- LT is
    the dedicated engage/disengage control there, same as a real autopilot
    button rather than a held deadman.
  - RB/R1 (button 5), held: overrides the stick and fixes speed to the
    `vconf` parameter -- a precise, repeatable step input for PID tuning
    instead of eyeballing a stick position.
  - RT/R2 (button 7), edge-triggered (once per press, not held): shifts the
    absolute heading setpoint by +`phiconf` degrees -- a precise step input
    for tuning the heading PID's step response.
  - LT/L2 (button 6), edge-triggered: toggles docking mode on/off. Pressing
    it again hands control straight back to manual stick/heading control
    (deadman-gated, as above) -- no re-arming step needed.
  - A (button 1), held: redirects the SAME two sticks (left vertical =
    thrust, right horizontal = differential/turn) to drive the dock
    (wamv2)'s thrusters directly instead of the WAM-V's -- open-loop, no
    PID, since the dock has no odometry/IMU speed estimate (state_estimator_
    node only produces one for the WAM-V; the dock only gets a raw pose).
    Releasing A publishes zero thrust so the dock goes back to drifting
    free under wind/current (its normal state -- see apf_lib docs) instead
    of latching whatever thrust was last commanded, since gz-sim's thruster
    plugin holds the last command with no timeout of its own.
  Both `vconf` (m/s) and `phiconf` (deg) are ROS parameters, adjustable live
  via `ros2 param set` or rqt_reconfigure, same as thrust_mixer_node's gains.
"""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float64

from vrx_apf_dock.geometry_utils import yaw_from_quaternion, wrap_to_pi

LEFT_VERT_AXIS = 1
RIGHT_HORIZ_AXIS = 2
DOCK_DRIVE_BUTTON = 1  # A
DEADMAN_BUTTON = 4
SPEED_PRESET_BUTTON = 5  # RB / R1
DOCKING_TOGGLE_BUTTON = 6  # LT / L2
HEADING_STEP_BUTTON = 7  # RT / R2

PUBLISH_RATE_HZ = 20.0

DEFAULT_STEP_PARAMS = {
    'vconf': 1.0,     # m/s, speed commanded while SPEED_PRESET_BUTTON is held
    'phiconf': 15.0,  # deg, heading step applied per HEADING_STEP_BUTTON press
}


class JoystickSetpointNode(Node):

    def __init__(self):
        super().__init__('joystick_setpoint_node')

        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('max_heading_rate', 1.0)
        self._max_speed = self.get_parameter('max_speed').value
        self._max_heading_rate = self.get_parameter('max_heading_rate').value

        self.declare_parameter('dock_max_thrust', 300.0)
        self.declare_parameter('dock_max_diff_thrust', 150.0)
        self._dock_max_thrust = self.get_parameter('dock_max_thrust').value
        self._dock_max_diff_thrust = self.get_parameter('dock_max_diff_thrust').value

        self._step_params = dict(DEFAULT_STEP_PARAMS)
        self.declare_parameters('', list(DEFAULT_STEP_PARAMS.items()))
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._speed_axis = 0.0
        self._heading_rate_axis = 0.0
        self._dock_drive_held = False
        self._deadman_held = False
        self._speed_preset_held = False
        self._heading_step_prev_held = False
        self._docking_active = False
        self._docking_toggle_prev_held = False
        self._apf_setpoint = None
        self._current_yaw = None
        # Absolute heading setpoint, seeded from the WAM-V's own yaw on the
        # first odometry fix so the boat doesn't lurch to face an arbitrary
        # heading (e.g. 0/east) the moment the deadman is first pressed.
        self._heading_cmd = None

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(Odometry, '/vrx_apf_dock/wamv/odom', self._odom_cb, 10)
        self.create_subscription(
            Twist, '/vrx_apf_dock/wamv/setpoint_apf', self._apf_setpoint_cb, 10)
        self._setpoint_pub = self.create_publisher(Twist, '/vrx_apf_dock/wamv/setpoint', 10)
        self._apf_reset_pub = self.create_publisher(Empty, '/vrx_apf_dock/apf/reset', 10)
        self._dock_left_pub = self.create_publisher(Float64, '/wamv2/thrusters/left/thrust', 10)
        self._dock_right_pub = self.create_publisher(Float64, '/wamv2/thrusters/right/thrust', 10)

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_setpoint)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_dock_thrust)

    def _on_set_parameters(self, params):
        for param in params:
            if param.name in self._step_params:
                self._step_params[param.name] = param.value
        return SetParametersResult(successful=True)

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self._current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        if self._heading_cmd is None:
            self._heading_cmd = self._current_yaw

    def _apf_setpoint_cb(self, msg: Twist):
        self._apf_setpoint = msg

    def _joy_cb(self, msg: Joy):
        self._speed_axis = msg.axes[LEFT_VERT_AXIS]
        self._heading_rate_axis = msg.axes[RIGHT_HORIZ_AXIS]
        self._dock_drive_held = bool(msg.buttons[DOCK_DRIVE_BUTTON])
        self._deadman_held = bool(msg.buttons[DEADMAN_BUTTON])
        self._speed_preset_held = bool(msg.buttons[SPEED_PRESET_BUTTON])

        docking_toggle_held = bool(msg.buttons[DOCKING_TOGGLE_BUTTON])
        if docking_toggle_held and not self._docking_toggle_prev_held:
            self._docking_active = not self._docking_active
            self.get_logger().info(
                f'docking {"engaged" if self._docking_active else "disengaged (manual)"}')
            if self._docking_active:
                # Re-engaging: reset apf_controller_node's reached/switch_value/
                # active latches so a repeated docking attempt in the same run
                # starts fresh instead of inheriting "reached" from a previous
                # one, and drop the last-seen setpoint so _publish_setpoint
                # hard-stops for the one tick until a post-reset value arrives
                # instead of acting on a stale (possibly already-"reached")
                # setpoint from before.
                self._apf_setpoint = None
                self._apf_reset_pub.publish(Empty())
            elif self._current_yaw is not None:
                # Handing back to manual: re-seed the virtual-rudder heading
                # from the boat's actual current yaw -- it was frozen while
                # docking had control, so resuming from it as-is could snap
                # the heading setpoint back to a stale target.
                self._heading_cmd = self._current_yaw
        self._docking_toggle_prev_held = docking_toggle_held

        heading_step_held = bool(msg.buttons[HEADING_STEP_BUTTON])
        if (heading_step_held and not self._heading_step_prev_held
                and not self._docking_active and self._deadman_held
                and self._heading_cmd is not None):
            self._heading_cmd = wrap_to_pi(
                self._heading_cmd + math.radians(self._step_params['phiconf']))
        self._heading_step_prev_held = heading_step_held

    def _publish_setpoint(self):
        msg = Twist()

        if self._docking_active:
            if self._apf_setpoint is None:
                msg.linear.x = 0.0
                msg.angular.z = self._heading_cmd if self._heading_cmd is not None else 0.0
            else:
                msg = self._apf_setpoint
            self._setpoint_pub.publish(msg)
            return

        if not self._deadman_held or self._heading_cmd is None:
            msg.linear.x = 0.0
            msg.angular.z = self._heading_cmd if self._heading_cmd is not None else 0.0
            self._setpoint_pub.publish(msg)
            return

        dt = 1.0 / PUBLISH_RATE_HZ
        self._heading_cmd = wrap_to_pi(
            self._heading_cmd + self._heading_rate_axis * self._max_heading_rate * dt)

        msg.linear.x = (self._step_params['vconf'] if self._speed_preset_held
                         else self._speed_axis * self._max_speed)
        msg.angular.z = self._heading_cmd
        self._setpoint_pub.publish(msg)

    def _publish_dock_thrust(self):
        if not self._dock_drive_held:
            # Zero, not "don't publish": gz-sim's thruster plugin holds the
            # last command indefinitely, so releasing A must actively
            # command a stop or the dock would keep whatever thrust it had
            # instead of going back to drifting free.
            self._dock_left_pub.publish(Float64(data=0.0))
            self._dock_right_pub.publish(Float64(data=0.0))
            return

        total_thrust = self._speed_axis * self._dock_max_thrust
        diff_thrust = self._heading_rate_axis * self._dock_max_diff_thrust
        self._dock_left_pub.publish(Float64(data=total_thrust / 2.0 - diff_thrust / 2.0))
        self._dock_right_pub.publish(Float64(data=total_thrust / 2.0 + diff_thrust / 2.0))


def main(args=None):
    rclpy.init(args=args)
    node = JoystickSetpointNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
