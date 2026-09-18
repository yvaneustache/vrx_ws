#!/usr/bin/env python3
# Portage de docking/catkin_ws/src/guerleboat/scripts/node_control.py (ROS1 Melodic -> ROS2 Jazzy)
"""Runs the APF docking law (vrx_apf_dock.apf_lib.ApfDockController, reused
by import -- see CORRESPONDANCE_DOCKING.md) on a Kalman-filtered estimate
of the boat's own pose, exactly node_control.py's role in the original
guerleboat: fuse the boat's position/heading (here: a 5-state
constant-velocity/turn-rate EKF over /vrx_docking_sim/wamv/pose, see
_PoseEKF below) before feeding it to the docking law, while the dock's pose
(/vrx_docking_sim/dock/pose) is used as-received -- it is the target, not
something to smooth against a boat motion model.

Deliberate difference from the original 4-state linear KF in
node_control.py: that state included an unused/buggy "pitch" slot (see
CORRESPONDANCE_DOCKING.md investigation notes) and relied on the
controller's own output as the prediction's control input -- which only
makes sense once that output is actually driving the boat (mavros_setpoint_
bridge_node, livrable 4). Livrable 3 has no motor link yet, so this EKF is
self-contained instead: velocity and yaw-rate are part of the estimated
state itself (constant-velocity/turn-rate model), derived purely from
consecutive pose measurements. Same role (Kalman-filtered ego pose feeding
the potential-field law), cleaner and stage-independent.

Publishes a geometry_msgs/Twist repurposed as a speed/heading setpoint,
same convention as vrx_apf_dock/apf_controller_node.py:
  linear.x  = vbar    (desired forward speed, m/s)
  angular.z = thetabar (desired ABSOLUTE heading, rad -- not a turn rate)
mavros_setpoint_bridge_node (livrable 4) is the one that turns this into a
yaw-rate command for MAVROS -- see PROPOSITION_SIMULATEUR_COMPLET.md §6.

Also publishes /vrx_docking_sim/wamv/docking_reached (std_msgs/Bool),
mirroring apf_lib's own `reached` latch: v_cmd is already forced to 0.0 by
apf_lib once reached (see apf_lib.compute), but mavros_setpoint_bridge_node
still runs its heading PID on the (already-near-zero) setpoint, and that
PID's integral term does not reset on its own just because the error is
small -- so a residual command could still reach the motors. This topic
lets the bridge short-circuit to a hard zero Twist and reset its PID state
once docking is reached, instead of relying on the loop merely converging
close to zero (discovered live: "quand reached=true, tout doit etre
stoppe, la commande et les moteurs").

Also resets the APF `reached` latch (same as /vrx_docking_sim/apf/reset)
on any transition into MANUAL or AUTO mode, read from /mavros/state --
otherwise the latch (permanent by design, see apf_lib.ApfDockController's
docstring) stays set from a previous docking run, and the very next time
GUIDED re-engages the boat starts already "reached" with a stale target,
immediately re-triggering mavros_setpoint_bridge_node's HOLD-on-reach
switch instead of actually running the docking law (requested live:
"reached doit etre mis a false quand on passe en manuel ou auto").

Also resets on the armed -> disarmed edge (same /mavros/state topic),
for the same reason: disarming and re-arming without an intervening
MANUAL/AUTO mode change (e.g. staying in GUIDED the whole time) would
otherwise leave the stale latch in place too (requested live: "...ou le
desarmage doit modifie reached en false").
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, Empty

RESET_ON_MODES = ('MANUAL', 'AUTO')

from vrx_apf_dock.apf_lib import DEFAULT_GAINS, ApfDockController
from vrx_apf_dock.geometry_utils import wrap_to_pi, yaw_from_quaternion

CONTROL_PERIOD = 0.05  # s

# Process noise (per second, scaled by dt**2 in predict): position/heading
# are allowed to drift with the estimated velocity/yaw-rate, which are
# themselves assumed near-constant between measurements (weakly, small Q).
Q_DIAG = np.array([0.05, 0.05, 0.02, 0.5, 0.3])
R_DIAG = np.array([0.3, 0.3, 0.05])  # measurement noise: x, y, yaw


class _PoseEKF:
    """5-state [x, y, yaw, v, yaw_rate] constant-velocity/turn-rate EKF.

    Predicts with a unicycle model linearized (Jacobian recomputed) around
    the current estimate every cycle -- the same "recompute the transition
    matrix from the current heading each loop" idea as the original
    node_control.py, just applied to a self-contained state instead of an
    externally supplied control input.
    """

    def __init__(self):
        self.x = np.zeros(5)
        self.P = np.diag([1.0, 1.0, 0.5, 1.0, 1.0])
        self._initialized = False

    def initialize(self, x, y, yaw):
        self.x = np.array([x, y, yaw, 0.0, 0.0])
        self._initialized = True

    def predict(self, dt):
        x, y, yaw, v, yaw_rate = self.x
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        self.x = np.array([
            x + v * cos_y * dt,
            y + v * sin_y * dt,
            wrap_to_pi(yaw + yaw_rate * dt),
            v,
            yaw_rate,
        ])

        F = np.eye(5)
        F[0, 2] = -v * sin_y * dt
        F[0, 3] = cos_y * dt
        F[1, 2] = v * cos_y * dt
        F[1, 3] = sin_y * dt
        F[2, 4] = dt

        self.P = F @ self.P @ F.T + np.diag(Q_DIAG * dt)

    def correct(self, x_meas, y_meas, yaw_meas):
        H = np.zeros((3, 5))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0

        innovation = np.array([
            x_meas - self.x[0],
            y_meas - self.x[1],
            wrap_to_pi(yaw_meas - self.x[2]),
        ])
        S = H @ self.P @ H.T + np.diag(R_DIAG)
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation
        self.x[2] = wrap_to_pi(self.x[2])
        self.P = (np.eye(5) - K @ H) @ self.P

    def update(self, x_meas, y_meas, yaw_meas, dt):
        if not self._initialized:
            self.initialize(x_meas, y_meas, yaw_meas)
            return
        self.predict(dt)
        self.correct(x_meas, y_meas, yaw_meas)

    @property
    def xy(self):
        return self.x[0], self.x[1]

    @property
    def yaw(self):
        return self.x[2]


class ApfControllerNode(Node):

    def __init__(self):
        super().__init__('docking_apf_controller')

        self._wamv_xy = None
        self._wamv_yaw = None
        self._dock_xy = None
        self._dock_yaw = None
        self._last_update_time = None
        self._mode = ''
        self._armed = False

        # Node-level override, not touched in vrx_apf_dock.apf_lib.DEFAULT_GAINS
        # (Mode A keeps its own untouched default): 0.8 m let the hull-to-hull
        # contact happen during final approach (confirmed live -- boat and dock
        # co-rotating at r~1.2-1.3 m, a sustained collision, not a clean stop).
        # 2.5 m stops the boat before contact; still to be fine-tuned against
        # the actual WAM-V hull dimensions (length ~4.9*scale, beam ~2.5*scale).
        gains = dict(DEFAULT_GAINS)
        gains['reached_radius'] = 2.5
        # vmax as a float (1.0), not apf_lib's shared int default (1) --
        # declare_parameters() locks a ROS parameter's type to whatever
        # Python type its initial value has, so leaving this as an int
        # rejects any later `ros2 param set vmax <float>` with a type
        # mismatch error. Value itself unchanged (still 1.0 m/s) here --
        # only the type is fixed, so runtime speed tuning (e.g. testing at
        # 1 knot = 0.5144 m/s) is possible without touching apf_lib.py.
        gains['vmax'] = 1.0

        self._ekf = _PoseEKF()
        self._apf = ApfDockController(gains=gains)
        self.declare_parameters('', list(gains.items()))
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.create_subscription(
            PoseStamped, '/vrx_docking_sim/wamv/pose', self._wamv_pose_cb, 10)
        self.create_subscription(
            PoseStamped, '/vrx_docking_sim/dock/pose', self._dock_pose_cb, 10)
        self.create_subscription(Empty, '/vrx_docking_sim/apf/reset', self._reset_cb, 10)
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)

        self._setpoint_pub = self.create_publisher(
            Twist, '/vrx_docking_sim/wamv/setpoint_apf', 10)
        self._reached_pub = self.create_publisher(
            Bool, '/vrx_docking_sim/wamv/docking_reached', 10)
        self.create_timer(CONTROL_PERIOD, self._on_timer)

    def _on_set_parameters(self, params):
        for param in params:
            if param.name in self._apf.gains:
                self._apf.gains[param.name] = param.value
        return SetParametersResult(successful=True)

    def _reset_cb(self, msg: Empty):
        self._apf.reset()
        self.get_logger().info('APF controller state reset')

    def _state_cb(self, msg: State):
        previous_mode = self._mode
        previous_armed = self._armed
        self._mode = msg.mode
        self._armed = msg.armed
        if msg.mode in RESET_ON_MODES and previous_mode != msg.mode:
            self._apf.reset()
            self.get_logger().info(
                f'APF controller state reset (mode -> {msg.mode})')
        elif previous_armed and not msg.armed:
            self._apf.reset()
            self.get_logger().info('APF controller state reset (disarmed)')

    def _wamv_pose_cb(self, msg: PoseStamped):
        now = self.get_clock().now()
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        dt = CONTROL_PERIOD
        if self._last_update_time is not None:
            dt = max((now - self._last_update_time).nanoseconds * 1e-9, 1e-3)
        self._last_update_time = now

        self._ekf.update(x, y, yaw, dt)
        self._wamv_xy = self._ekf.xy
        self._wamv_yaw = self._ekf.yaw

    def _dock_pose_cb(self, msg: PoseStamped):
        self._dock_xy = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation
        self._dock_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_timer(self):
        if self._wamv_xy is None or self._dock_xy is None:
            return

        if self._mode == 'GUIDED':
            v_cmd, theta_cmd = self._apf.compute(self._wamv_xy, self._dock_xy, self._dock_yaw)
            if self._apf.reached:
                # See vrx_apf_dock/apf_controller_node.py: hold current
                # heading, a literal 0 rad would swing the boat to face east.
                theta_cmd = self._wamv_yaw
        else:
            # Don't re-evaluate the docking law outside GUIDED: apf.compute()
            # re-latches `reached` on its very next call if the boat is still
            # within reached_radius of the dock, which undoes _state_cb's
            # reset() before it has any visible effect -- discovered live:
            # switching to MANUAL while docked (r=1.42, well inside
            # reached_radius=2.5) left `reached` stuck at True, because the
            # 20 Hz timer kept calling compute() regardless of mode. Freezing
            # the setpoint/reached values here instead until GUIDED
            # re-engages is harmless: mavros_setpoint_bridge_node already
            # ignores the setpoint outside GUIDED.
            v_cmd, theta_cmd = 0.0, self._wamv_yaw
        setpoint = Twist()
        setpoint.linear.x = float(v_cmd)
        setpoint.angular.z = float(theta_cmd)
        self._setpoint_pub.publish(setpoint)
        self._reached_pub.publish(Bool(data=bool(self._apf.reached)))

        r = math.hypot(self._wamv_xy[0] - self._dock_xy[0], self._wamv_xy[1] - self._dock_xy[1])
        self.get_logger().info(
            f'wamv (EKF) pos=({self._wamv_xy[0]:.2f}, {self._wamv_xy[1]:.2f}) '
            f'yaw={self._wamv_yaw:.2f} v_est={self._ekf.x[3]:.2f} | '
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
