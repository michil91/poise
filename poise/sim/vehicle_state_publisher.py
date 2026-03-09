#!/usr/bin/env python3
"""
Vehicle State Publisher — Phase 3.

Publishes geometry_msgs/TwistStamped on /sim/vehicle_state at 50 Hz.
Reads linear_velocity_mps and turn_rate_radps from the same parameter names
as the IMU publisher.  Both nodes are configured from the imu_publisher section
of sim_config.yaml via YAML anchors so the values are defined once and shared.

Topic: /sim/vehicle_state  (geometry_msgs/TwistStamped)
  twist.linear.x  — forward velocity (m/s)
  twist.angular.z — yaw rate         (rad/s)

Autoware remapping: /sim/vehicle_state → /localization/kinematic_state
(kinematic_state carries a twist field that POISE checkers can subscribe to
 without modification when running against a live Autoware stack)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from poise.qos import SENSOR_QOS


class VehicleStatePublisher(Node):
    """Publishes vehicle linear and angular velocity for standstill detection."""

    def __init__(self):
        super().__init__('vehicle_state_publisher')

        # ── Parameters (shared values with imu_publisher via YAML anchors) ──
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('linear_velocity_mps', 0.0)
        self.declare_parameter('turn_rate_radps', 0.0)

        self._rate      = self.get_parameter('publish_rate_hz').value
        self._velocity  = self.get_parameter('linear_velocity_mps').value
        self._turn_rate = self.get_parameter('turn_rate_radps').value

        # ── Publisher ────────────────────────────────────────────────────────
        self._pub = self.create_publisher(TwistStamped, '/sim/vehicle_state', SENSOR_QOS)
        self._timer = self.create_timer(1.0 / self._rate, self._publish_cb)

        self.get_logger().info(
            f'VehicleStatePublisher started | rate={self._rate} Hz | '
            f'velocity={self._velocity} m/s | turn_rate={self._turn_rate} rad/s'
        )

    def _publish_cb(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = self._velocity
        msg.twist.angular.z = self._turn_rate
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
