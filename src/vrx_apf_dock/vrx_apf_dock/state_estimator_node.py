"""Publishes usable state for the APF docking controller from raw VRX/Gazebo topics.

- WAM-V and dock world poses come from VRX's OdometryPublisher plugin
  (ground_truth_enabled), bridged as /vrx_apf_dock/{wamv,dock}/raw_odom
  (nav_msgs/Odometry). Two other candidates were dead ends: the default
  /model/<name>/pose bridge only carries static sensor-to-link offsets, and
  the generic Pose_V -> TFMessage bridge on dynamic_pose/info drops entity
  names, making it impossible to tell which pose belongs to which robot.
- Forward speed is estimated by leaky integration of the IMU's body-frame
  x acceleration (dv/dt = a - leak*v) since there is no direct speed sensor.
  The leak term bounds accelerometer-bias drift; it trades long-term
  accuracy for stability, which is fine since v only feeds a closed-loop
  thrust regulator downstream.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

SPEED_LEAK = 0.2  # 1/s


class StateEstimatorNode(Node):

    def __init__(self):
        super().__init__('state_estimator_node')

        self._wamv_v = 0.0
        self._last_imu_stamp = None

        self.create_subscription(
            Odometry, '/vrx_apf_dock/wamv/raw_odom', self._wamv_odom_cb, 10)
        self.create_subscription(
            Odometry, '/vrx_apf_dock/dock/raw_odom', self._dock_odom_cb, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self._wamv_imu_cb, 10)

        self._wamv_odom_pub = self.create_publisher(Odometry, '/vrx_apf_dock/wamv/odom', 10)
        self._dock_pose_pub = self.create_publisher(PoseStamped, '/vrx_apf_dock/dock/pose', 10)

    def _wamv_imu_cb(self, msg: Imu):
        stamp = msg.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9
        if self._last_imu_stamp is not None:
            dt = now - self._last_imu_stamp
            if 0.0 < dt < 1.0:
                a_x = msg.linear_acceleration.x
                self._wamv_v += (a_x - SPEED_LEAK * self._wamv_v) * dt
        self._last_imu_stamp = now

    def _dock_odom_cb(self, msg: Odometry):
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose.pose
        self._dock_pose_pub.publish(out)

    def _wamv_odom_cb(self, msg: Odometry):
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = msg.child_frame_id
        odom.pose.pose = msg.pose.pose
        odom.twist.twist.linear.x = self._wamv_v
        self._wamv_odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
