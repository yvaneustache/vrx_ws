#!/usr/bin/env python3
# Portage de docking/catkin_ws/src/guerleboat/scripts/node_control.py (ROS1 Melodic -> ROS2 Jazzy)
"""Turns the APF docking law's (speed, absolute heading) setpoint into a
MAVROS GUIDED velocity command -- the ArduPilot-SITL-drives-the-motors
equivalent of guerleboat/node_control.py's final step (publish a Twist,
gated on /mavros/state armed+guided, that MAVROS's setpoint_velocity plugin
turns into a MAVLink velocity setpoint). See PROPOSITION_SIMULATEUR_COMPLET.md
§6 for why this exists as a separate node.

The heading part reuses vrx_apf_dock/thrust_mixer_node.py's PID *structure*
(P + I-with-clamp + D on wrap_to_pi(heading_error), same pattern) -- but
NOT its gain values: thrust_mixer_node's PID outputs a differential thrust
in Newtons (WAM-V-mass-and-inertia-specific), this one outputs a yaw RATE
in rad/s for MAVROS/ArduPilot's GUIDED velocity setpoint, a completely
different physical domain. New gains, same structure; both still
ROS-parameter-tunable at runtime.

Livrable 4 scope: the boat's pose still comes from
/vrx_docking_sim/wamv/pose (Gazebo ground truth, same as livrable 3) rather
than MAVROS's own /mavros/local_position/odom. Switching to the latter is
a deliberately separate, later step: ArduPilot's EKF-local frame origin
(set from wherever the vehicle first gets a good position estimate, close
to its spawn/arm point) is not guaranteed to line up with the fixed
sydney_regatta-origin frame boat_pose_node/dock_pose_source_node use for
the dock's UDP-derived position -- reconciling the two needs an explicit
offset, out of scope for "does GUIDED docking work at all".

Publishes to /mavros/setpoint_velocity/cmd_vel_unstamped continuously
(zero Twist whenever not armed+GUIDED, matching guerleboat's own
defensive "always publish something, zero when not in control" pattern --
avoids a stale nonzero setpoint lingering in MAVROS if GUIDED re-engages).

Also treats /vrx_docking_sim/wamv/docking_reached (apf_controller_node)
the same way as "not armed+GUIDED": a hard zero Twist and a reset PID
state, rather than trusting the heading PID to merely converge near zero
on its own -- its integral term does not reset just because the input
setpoint's heading error is already ~0, so without this a residual
command could still reach the motors after docking (see
apf_controller_node's docstring for the live-discovered symptom).

That zero Twist alone turned out not to be enough: verified live that
ArduPilot Rover in GUIDED, sent a steady (velocity=0, yaw_rate=0) target,
kept the boat circling in a tight, non-decaying loop indefinitely instead
of coming to rest -- GUIDED's velocity controller treats "zero" as a
target to converge/hold around, not a hard motor cutoff, and skid-steer
Rover apparently does not settle there cleanly. So on the rising edge of
`docking_reached` (while GUIDED), this node also calls MAVROS's
/mavros/set_mode once to switch to HOLD -- Rover's actual "stop and hold,
ignore all navigation/velocity targets" mode, which is what guarantees
the motors themselves go to zero, rather than trying to coax GUIDED into
behaving like a stop.
"""

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool

from vrx_apf_dock.geometry_utils import wrap_to_pi, yaw_from_quaternion

CONTROL_PERIOD = 0.05  # s

DEFAULT_GAINS = {
    # Was 1.5: reduced live ("tu prends la main sur les parametres pour
    # resoudre l'oscillation du docking") while investigating GUIDED
    # oscillation. Root cause turned out to be an invalid test (approaching
    # the dock from its repulsive/back-branch side, see apf_lib.py's
    # ApfDockController.compute() docstring) rather than this gain, but
    # 0.5 was verified live to still converge cleanly (r: 37m->1.4m, no
    # oscillation, theta_cmd steady +-2 deg) at 1 m/s from the correct
    # approach side, so it's kept rather than reverted.
    'kp_heading': 0.5,   # rad/s per rad of heading error
    'ki_heading': 0.05,
    # kd_heading was 0.3, on the raw (unfiltered) heading-error derivative
    # (heading_error - self._prev_heading_error) / dt -- discovered live
    # ("quand je passe en docking, ca oscille") to be the same class of
    # problem the ArduPilot-side ATC_STR_RAT_D tuning ran into: an
    # unfiltered derivative term amplifies estimation noise and drives
    # sustained thruster saturation (measured live: 31% of samples pinned
    # at +-600N in GUIDED, vs 0% in AUTO once ATC_STR_RAT_D was tamed).
    # Zeroed for the same reason ArduPilot's own guidance gives for Rover's
    # steering-rate D term: "can normally be left at zero" -- see
    # https://ardupilot.org/rover/docs/rover-tuning-steering-rate.html
    'kd_heading': 0.0,
    'heading_integral_limit': 0.5,
    # rad/s clamp. Was 1.0 (a placeholder -- "not tuned against real
    # ArduPilot limits yet"): the boat's actual maximum yaw rate, measured
    # live (ManualControl at full deflection, yaw rate read from Gazebo
    # ground truth), is ~42-54 deg/s (~0.73-0.94 rad/s) regardless of
    # forward speed -- matches ATC_STR_RAT_MAX's calibration (see
    # vrx_docking_sim/config/rover_tuning.parm). Asking for more than that
    # just saturates without achieving a faster turn.
    'max_yaw_rate': 0.785,
}


class MavrosSetpointBridgeNode(Node):

    def __init__(self):
        super().__init__('mavros_setpoint_bridge')

        self._current_yaw = None
        self._speed_cmd = 0.0
        self._heading_cmd = 0.0
        self._armed = False
        self._mode = ''
        self._reached = False
        self._heading_error_integral = 0.0
        self._prev_heading_error = 0.0

        self._gains = dict(DEFAULT_GAINS)
        self.declare_parameters('', list(DEFAULT_GAINS.items()))
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.create_subscription(
            Twist, '/vrx_docking_sim/wamv/setpoint_apf', self._setpoint_cb, 10)
        self.create_subscription(
            PoseStamped, '/vrx_docking_sim/wamv/pose', self._pose_cb, 10)
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.create_subscription(
            Bool, '/vrx_docking_sim/wamv/docking_reached', self._reached_cb, 10)

        self._cmd_vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self._set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.create_timer(CONTROL_PERIOD, self._on_timer)

    def _on_set_parameters(self, params):
        for param in params:
            if param.name in self._gains:
                self._gains[param.name] = param.value
        return SetParametersResult(successful=True)

    def _setpoint_cb(self, msg: Twist):
        self._speed_cmd = msg.linear.x
        self._heading_cmd = msg.angular.z

    def _pose_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        self._current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _state_cb(self, msg: State):
        self._armed = msg.armed
        self._mode = msg.mode

    def _reached_cb(self, msg: Bool):
        was_reached = self._reached
        self._reached = msg.data
        if self._reached and not was_reached and self._guided_active():
            self._call_hold()

    def _call_hold(self):
        if not self._set_mode_client.service_is_ready():
            self.get_logger().warning('set_mode service not ready, cannot switch to HOLD')
            return
        req = SetMode.Request()
        req.custom_mode = 'HOLD'
        self.get_logger().info('docking reached: requesting mode HOLD')
        self._set_mode_client.call_async(req)

    def _guided_active(self):
        return self._armed and self._mode == 'GUIDED'

    def _on_timer(self):
        cmd = Twist()

        if not self._guided_active() or self._current_yaw is None or self._reached:
            self._heading_error_integral = 0.0
            self._prev_heading_error = 0.0
            self._cmd_vel_pub.publish(cmd)
            return

        gains = self._gains
        dt = CONTROL_PERIOD

        heading_error = wrap_to_pi(self._heading_cmd - self._current_yaw)
        heading_error_rate = wrap_to_pi(heading_error - self._prev_heading_error) / dt
        self._prev_heading_error = heading_error

        limit = gains['heading_integral_limit']
        self._heading_error_integral = max(-limit, min(
            limit, self._heading_error_integral + heading_error * dt))

        yaw_rate = (gains['kp_heading'] * heading_error
                    + gains['ki_heading'] * self._heading_error_integral
                    + gains['kd_heading'] * heading_error_rate)
        max_rate = gains['max_yaw_rate']
        yaw_rate = max(-max_rate, min(max_rate, yaw_rate))

        cmd.linear.x = float(self._speed_cmd)
        cmd.angular.z = float(yaw_rate)
        self._cmd_vel_pub.publish(cmd)

        self.get_logger().info(
            f'GUIDED yaw={self._current_yaw:.2f} heading_cmd={self._heading_cmd:.2f} '
            f'heading_error={heading_error:.2f} yaw_rate={yaw_rate:.2f} '
            f'v_cmd={self._speed_cmd:.2f}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosSetpointBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
