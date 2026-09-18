#!/usr/bin/env python3
# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
"""Emulates a real ArduPilot RC transmitter, rather than the ROS-side
manual/auto arbitration used by vrx_apf_dock_teleop/joystick_setpoint_node.py
(see that file for the older, superseded-for-this-mode approach). Two
distinct kinds of control, matching how a real transmitter works:

  - Sticks: sent CONTINUOUSLY to MAVROS via /mavros/manual_control/send
    (mavros_msgs/ManualControl), like RC channels always being transmitted.
    ArduPilot itself decides whether to act on them (mode MANUAL) or ignore
    them (AUTO: waypoint mission; GUIDED: apf_controller_node/
    mavros_setpoint_bridge_node drive the boat instead). This node never
    arbitrates manual-vs-auto -- that decision lives entirely in ArduPilot's
    current mode, not here.
  - 4 dedicated buttons, each a single direct MAVROS call, edge-triggered
    (once per press, not held): arm/disarm toggle, and one button per mode
    (MANUAL / AUTO / GUIDED) -- not a toggle, each button always requests
    that exact mode, exactly like flipping a 3-position switch on a real
    transmitter.
  - A 5th button drives the dock's thrusters directly in open loop (same
    idea as joystick_setpoint_node.py's DOCK_DRIVE_BUTTON/A), independent of
    the USV's ArduPilot mode.

MANUAL_CONTROL field mapping/scaling: confirmed against ArduPilot Rover's
own handler (GCS_MAVLink_Rover.cpp, handle_manual_control_axes) rather than
assumed from the generic MAVLink spec -- Rover reads y = steering and
z = throttle specifically (x and r are ignored for Rover), each in
-1000..1000, converted internally via manual_override() into a simulated
RC PWM override on channel_steer/channel_throttle. Verified live: an SITL
run with y/z left at 0 (an earlier version of this node used x/r instead)
produced zero motion in MANUAL despite a held stick, confirming this
mapping was not just theoretical.

Steering sign: y is negated (-heading_axis) -- verified live that the
un-negated mapping steered the USV opposite to the stick (right stick ->
boat turned left). Not chased further upstream (RC1_REVERSED, the
joy_node axis convention, or ArduPilot's own channel polarity could each
explain it) since the fix is the same either way: this is the one place
translating stick intent into the MANUAL_CONTROL wire value, so the
correction belongs here. The exact effect of the manual_override()
offset/scaler pair at the two ends of stick travel is still unconfirmed
on real hardware (see PROPOSITION_SIMULATEUR_COMPLET.md §3.2 note on the
joystick mapping being provisional).

Button indices are placeholders (0-4) -- to be corrected against the actual
controller's real `ros2 topic echo /joy` output once physical hardware is
available, same caveat as vrx_apf_dock_teleop's existing mapping.
"""

import rclpy
from mavros_msgs.msg import ManualControl, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

LEFT_VERT_AXIS = 1
RIGHT_HORIZ_AXIS = 2

ARM_TOGGLE_BUTTON = 0
MANUAL_MODE_BUTTON = 1
AUTO_MODE_BUTTON = 2
GUIDED_MODE_BUTTON = 3
DOCK_DRIVE_BUTTON = 4

PUBLISH_RATE_HZ = 20.0
STICK_TO_MANUAL_CONTROL_SCALE = 1000.0  # MAVLink MANUAL_CONTROL x/y/z/r range


class JoystickRcNode(Node):

    def __init__(self):
        super().__init__('joystick_rc')

        self.declare_parameter('dock_max_thrust', 300.0)
        self.declare_parameter('dock_max_diff_thrust', 150.0)
        self._dock_max_thrust = self.get_parameter('dock_max_thrust').value
        self._dock_max_diff_thrust = self.get_parameter('dock_max_diff_thrust').value

        self._speed_axis = 0.0
        self._heading_axis = 0.0
        self._dock_drive_held = False
        self._armed = False

        self._prev_buttons = {
            ARM_TOGGLE_BUTTON: False,
            MANUAL_MODE_BUTTON: False,
            AUTO_MODE_BUTTON: False,
            GUIDED_MODE_BUTTON: False,
        }

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)

        self._manual_control_pub = self.create_publisher(
            ManualControl, '/mavros/manual_control/send', 10)
        self._dock_left_pub = self.create_publisher(Float64, '/wamv2/thrusters/left/thrust', 10)
        self._dock_right_pub = self.create_publisher(Float64, '/wamv2/thrusters/right/thrust', 10)

        self._arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_manual_control)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_dock_thrust)

    def _state_cb(self, msg):
        self._armed = msg.armed

    def _edge(self, button_index, held):
        was_held = self._prev_buttons[button_index]
        self._prev_buttons[button_index] = held
        return held and not was_held

    def _joy_cb(self, msg: Joy):
        self._speed_axis = msg.axes[LEFT_VERT_AXIS]
        self._heading_axis = msg.axes[RIGHT_HORIZ_AXIS]
        self._dock_drive_held = bool(msg.buttons[DOCK_DRIVE_BUTTON])

        if self._edge(ARM_TOGGLE_BUTTON, bool(msg.buttons[ARM_TOGGLE_BUTTON])):
            self._call_arming(not self._armed)
        if self._edge(MANUAL_MODE_BUTTON, bool(msg.buttons[MANUAL_MODE_BUTTON])):
            self._call_set_mode('MANUAL')
        if self._edge(AUTO_MODE_BUTTON, bool(msg.buttons[AUTO_MODE_BUTTON])):
            self._call_set_mode('AUTO')
        if self._edge(GUIDED_MODE_BUTTON, bool(msg.buttons[GUIDED_MODE_BUTTON])):
            self._call_set_mode('GUIDED')

    def _call_arming(self, value):
        if not self._arming_client.service_is_ready():
            self.get_logger().warning('arming service not ready, ignoring button')
            return
        req = CommandBool.Request()
        req.value = value
        self.get_logger().info(f'requesting {"arm" if value else "disarm"}')
        self._arming_client.call_async(req)

    def _call_set_mode(self, mode):
        if not self._set_mode_client.service_is_ready():
            self.get_logger().warning('set_mode service not ready, ignoring button')
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.get_logger().info(f'requesting mode {mode}')
        self._set_mode_client.call_async(req)

    def _publish_manual_control(self):
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x = 0.0
        # Rover-specific mapping (GCS_MAVLink_Rover::handle_manual_control_axes):
        # y = steering, z = throttle -- x/r are ignored for Rover. Negated:
        # verified live that the boat otherwise steered opposite the stick.
        msg.y = -self._heading_axis * STICK_TO_MANUAL_CONTROL_SCALE
        msg.z = self._speed_axis * STICK_TO_MANUAL_CONTROL_SCALE
        msg.r = 0.0
        msg.buttons = 0
        self._manual_control_pub.publish(msg)

    def _publish_dock_thrust(self):
        if not self._dock_drive_held:
            # Zero, not "don't publish": gz-sim's thruster plugin holds the
            # last command indefinitely (see joystick_setpoint_node.py).
            self._dock_left_pub.publish(Float64(data=0.0))
            self._dock_right_pub.publish(Float64(data=0.0))
            return

        total_thrust = self._speed_axis * self._dock_max_thrust
        diff_thrust = self._heading_axis * self._dock_max_diff_thrust
        self._dock_left_pub.publish(Float64(data=total_thrust / 2.0 - diff_thrust / 2.0))
        self._dock_right_pub.publish(Float64(data=total_thrust / 2.0 + diff_thrust / 2.0))


def main(args=None):
    rclpy.init(args=args)
    node = JoystickRcNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
