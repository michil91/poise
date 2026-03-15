#!/usr/bin/env python3
"""
Wheel Odometry Simulator Node — publishes nav_msgs/Odometry on /sim/odometry at 50 Hz.

Models the same constant-velocity straight-line motion as the vehicle state publisher.
linear_velocity_mps and turn_rate_radps default to 0.0, matching imu_publisher defaults.
They are NOT duplicated in sim_config.yaml under odometry_publisher.ros__parameters;
override them per-scenario via scenario yaml files when non-zero velocity is needed.

Fault modes
-----------
slip:    Multiplies reported linear.x velocity by slip_factor from slip_time_s onward.
         Simulates wheel slip causing the odometer to undercount distance travelled.
         A warning is logged once when slip becomes active.
dropout: Suppresses topic publication for dropout_duration_s seconds from dropout_time_s.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from poise.qos import SENSOR_QOS


class OdometryPublisher(Node):
    """Simulates a wheel odometer with configurable fault injection."""

    def __init__(self):
        super().__init__('odometry_publisher')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('publish_rate_hz', 50.0)

        # Shared motion model.  Intentionally absent from sim_config.yaml under
        # odometry_publisher.ros__parameters — defaults match imu_publisher (0.0).
        # Set per-scenario under odometry_publisher.ros__parameters when needed.
        self.declare_parameter('linear_velocity_mps', 0.0)
        self.declare_parameter('turn_rate_radps', 0.0)

        self.declare_parameter('fault_mode', 'none')

        # slip
        self.declare_parameter('slip_factor', 0.95)
        self.declare_parameter('slip_time_s', 10.0)

        # dropout
        self.declare_parameter('dropout_time_s', 15.0)
        self.declare_parameter('dropout_duration_s', 20.0)

        # ── Read parameters ──────────────────────────────────────────────────
        self.rate_hz      = self.get_parameter('publish_rate_hz').value
        self.velocity     = self.get_parameter('linear_velocity_mps').value
        self.turn_rate    = self.get_parameter('turn_rate_radps').value
        self.fault_mode   = self.get_parameter('fault_mode').value

        self.slip_factor  = self.get_parameter('slip_factor').value
        self.slip_time    = self.get_parameter('slip_time_s').value

        self.dropout_time = self.get_parameter('dropout_time_s').value
        self.dropout_dur  = self.get_parameter('dropout_duration_s').value

        # ── State ────────────────────────────────────────────────────────────
        self._elapsed_s   = 0.0
        self._slip_warned = False

        # Accumulated pose — straight northward motion (east=0, north=distance)
        self._pos_north = 0.0

        # ── Publisher ────────────────────────────────────────────────────────
        self._pub = self.create_publisher(Odometry, '/sim/odometry', SENSOR_QOS)

        period = 1.0 / self.rate_hz
        self._timer = self.create_timer(period, self._publish_cb)

        self.get_logger().info(
            f'OdometryPublisher started | rate={self.rate_hz} Hz | '
            f'velocity={self.velocity} m/s | fault_mode={self.fault_mode}'
        )

    # ── main callback ─────────────────────────────────────────────────────────

    def _publish_cb(self):
        dt = 1.0 / self.rate_hz
        self._elapsed_s += dt

        # ── Dropout: suppress publication ─────────────────────────────────
        if self.fault_mode == 'dropout':
            t = self._elapsed_s
            if self.dropout_time <= t <= self.dropout_time + self.dropout_dur:
                self.get_logger().debug('Odometry dropout active — suppressing message')
                return

        # ── Reported velocity ─────────────────────────────────────────────
        reported_vel = self.velocity

        if self.fault_mode == 'slip' and self._elapsed_s >= self.slip_time:
            reported_vel *= self.slip_factor
            if not self._slip_warned:
                self._slip_warned = True
                self.get_logger().warn(
                    f'[FAULT INJECTION] Odometry slip active at t={self._elapsed_s:.1f}s — '
                    f'reporting velocity={reported_vel:.3f} m/s '
                    f'(slip_factor={self.slip_factor}, true={self.velocity:.3f} m/s)'
                )

        # Accumulate position — straight northward motion (heading = 0)
        self._pos_north += reported_vel * dt

        # ── Build message ─────────────────────────────────────────────────
        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'

        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = self._pos_north
        msg.pose.pose.position.z = 0.0
        # Identity quaternion — vehicle always faces north
        msg.pose.pose.orientation.w = 1.0

        msg.twist.twist.linear.x  = reported_vel
        msg.twist.twist.angular.z = self.turn_rate

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
