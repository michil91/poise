#!/usr/bin/env python3
"""
Extrinsic Consistency Validator — Phase 3.

Subscribes to /sim/imu (sensor_msgs/Imu) and /sim/vehicle_state
(geometry_msgs/TwistStamped).  Detects systematic IMU mount rotation by
comparing the mean Z-axis acceleration against the expected gravity vector
when the vehicle is stationary.

Publishes poise/IntegrityStatus on /poise/integrity_status using INTEGRITY_QOS.

Why mean over a rolling window (not per-sample)
------------------------------------------------
Individual IMU samples contain random noise (typically 0.02 m/s² 1σ).
A per-sample check would produce false positives from noise alone.
A systematic mount shift produces a *constant bias* — it shifts the mean,
not the variance.  Averaging over `imu_sample_window` (default 100) samples
reduces Gaussian noise by √100 = 10×, making the check sensitive to
systematic offsets of order 0.05–0.1 m/s² while ignoring random noise.

Tunnel / standstill interaction
---------------------------------
The check only accumulates samples while the vehicle is stationary
(velocity < imu_stationary_velocity_threshold).  When the vehicle moves:
  - Accumulation pauses.
  - The last determined status is held and re-published at 1 Hz.
When the vehicle transitions from moving to stationary:
  - The sample window is reset so fresh samples are used for evaluation.
  - Status reverts to EXTRINSIC_CHECK_PENDING until min_stationary_samples
    are collected.
This prevents motion-induced accelerations from contaminating the gravity check
and resets the baseline each time the vehicle parks.

Extrinsic faults are always non-recoverable
--------------------------------------------
A systematic gravity deviation indicates a physical change in the IMU's
mounting angle — e.g. loose bracket, chassis deformation, or improper
re-installation after maintenance.  This condition does not self-correct.
The aggregator will require an operator /poise/reset after the fault is
confirmed and the mount physically inspected.

Note: the check assumes level terrain.  A sustained slope will produce an
apparent gravity deviation.  In a production deployment, terrain gradient
from a map or barometer should be compensated before this check runs.

Fault codes
-----------
IMU_EXTRINSIC_WARN      Non-recoverable  Mean gravity deviation > warn_tolerance
IMU_EXTRINSIC_CRITICAL  Non-recoverable  Mean gravity deviation > critical_tolerance
EXTRINSIC_CHECK_PENDING (STATUS_OK)      Awaiting stationary samples or vehicle moving
"""

import collections
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped

from poise.msg import IntegrityStatus
from poise.qos import SENSOR_QOS, INTEGRITY_QOS


class ExtrinsicValidator(Node):
    """Detects systematic IMU mount rotation via stationary gravity check."""

    def __init__(self):
        super().__init__('extrinsic_validator')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('imu_expected_gravity_z',          -9.81)
        self.declare_parameter('imu_gravity_warn_tolerance',        0.5)
        self.declare_parameter('imu_gravity_critical_tolerance',    1.5)
        self.declare_parameter('imu_stationary_velocity_threshold', 0.5)
        self.declare_parameter('imu_min_stationary_samples',       50)
        self.declare_parameter('imu_sample_window',               100)

        self._expected_gravity = self.get_parameter('imu_expected_gravity_z').value
        self._warn_tol         = self.get_parameter('imu_gravity_warn_tolerance').value
        self._crit_tol         = self.get_parameter('imu_gravity_critical_tolerance').value
        self._vel_threshold    = self.get_parameter('imu_stationary_velocity_threshold').value
        self._min_samples      = self.get_parameter('imu_min_stationary_samples').value
        self._window_size      = self.get_parameter('imu_sample_window').value

        # ── State ────────────────────────────────────────────────────────────
        # Vehicle motion state
        self._is_stationary  = True   # conservative initial assumption
        self._prev_stationary = True  # for transition detection

        # Rolling window of Z-axis acceleration samples (stationary only)
        self._sample_window: collections.deque = collections.deque(
            maxlen=self._window_size
        )

        # Determined status — set once min_samples are collected
        self._has_determination      = False
        self._determined_status      = IntegrityStatus.STATUS_OK
        self._determined_fault_code  = ''
        self._last_mean_z            = 0.0
        self._last_deviation         = 0.0

        # ── Publisher / Subscribers ──────────────────────────────────────────
        self._status_pub = self.create_publisher(
            IntegrityStatus, '/poise/integrity_status', INTEGRITY_QOS
        )

        self.create_subscription(Imu, '/sim/imu', self._imu_cb, SENSOR_QOS)
        self.create_subscription(
            TwistStamped, '/sim/vehicle_state', self._vehicle_state_cb, SENSOR_QOS
        )

        # 1 Hz timer — publish PENDING or hold determined status
        self.create_timer(1.0, self._periodic_cb)

        self.get_logger().info(
            f'ExtrinsicValidator started | '
            f'expected_gravity_z={self._expected_gravity} m/s² | '
            f'warn_tol={self._warn_tol} m/s² | '
            f'crit_tol={self._crit_tol} m/s² | '
            f'min_samples={self._min_samples} | '
            f'window={self._window_size}'
        )

    # ── Vehicle state callback ─────────────────────────────────────────────────

    def _vehicle_state_cb(self, msg: TwistStamped):
        currently_stationary = abs(msg.twist.linear.x) < self._vel_threshold

        # Transition: moving → stationary — reset sample window for fresh evaluation
        if not self._is_stationary and currently_stationary:
            self._sample_window.clear()
            # Only clear determination if no fault was found; faults are non-recoverable
            if self._determined_status == IntegrityStatus.STATUS_OK:
                self._has_determination = False
            self.get_logger().info(
                'ExtrinsicValidator: vehicle now stationary — sample window reset'
            )

        self._is_stationary = currently_stationary

    # ── IMU callback (100 Hz — only accumulates while stationary) ─────────────

    def _imu_cb(self, msg: Imu):
        if not self._is_stationary:
            return

        self._sample_window.append(msg.linear_acceleration.z)

        if len(self._sample_window) < self._min_samples:
            return

        # Compute mean over window
        mean_z    = sum(self._sample_window) / len(self._sample_window)
        deviation = abs(mean_z - self._expected_gravity)

        self._last_mean_z    = mean_z
        self._last_deviation = deviation
        self._has_determination = True

        if deviation > self._crit_tol:
            self._determined_status     = IntegrityStatus.STATUS_CRITICAL
            self._determined_fault_code = 'IMU_EXTRINSIC_CRITICAL'
        elif deviation > self._warn_tol:
            self._determined_status     = IntegrityStatus.STATUS_WARN
            self._determined_fault_code = 'IMU_EXTRINSIC_WARN'
        else:
            self._determined_status     = IntegrityStatus.STATUS_OK
            self._determined_fault_code = ''

    # ── Periodic callback (1 Hz) ──────────────────────────────────────────────

    def _periodic_cb(self):
        stamp = self.get_clock().now().to_msg()
        n     = len(self._sample_window)

        if not self._has_determination:
            # No determination made yet — explain why
            if not self._is_stationary:
                desc = 'Extrinsic check pending — vehicle is moving'
            else:
                desc = f'Awaiting stationary samples: {n}/{self._min_samples}'

            self._publish_status(
                check_name='imu_extrinsic',
                status=IntegrityStatus.STATUS_OK,
                recoverable=False,
                fault_code='EXTRINSIC_CHECK_PENDING',
                description=desc,
                measured=float(n),
                threshold=float(self._min_samples),
                units='samples', stamp=stamp,
            )
            return

        # Determination is available — publish it at 1 Hz
        if self._determined_status == IntegrityStatus.STATUS_CRITICAL:
            self._publish_status(
                check_name='imu_extrinsic',
                status=IntegrityStatus.STATUS_CRITICAL,
                recoverable=False,
                fault_code='IMU_EXTRINSIC_CRITICAL',
                description=(
                    f'Mean Z-axis gravity deviation {self._last_deviation:.3f} m/s² '
                    f'exceeds critical tolerance {self._crit_tol:.1f} m/s² '
                    f'(mean={self._last_mean_z:.3f} m/s², '
                    f'expected={self._expected_gravity:.2f} m/s²)'
                ),
                measured=self._last_deviation,
                threshold=self._crit_tol,
                units='m/s²', stamp=stamp,
            )
        elif self._determined_status == IntegrityStatus.STATUS_WARN:
            self._publish_status(
                check_name='imu_extrinsic',
                status=IntegrityStatus.STATUS_WARN,
                recoverable=False,
                fault_code='IMU_EXTRINSIC_WARN',
                description=(
                    f'Mean Z-axis gravity deviation {self._last_deviation:.3f} m/s² '
                    f'exceeds warn tolerance {self._warn_tol:.1f} m/s² '
                    f'(mean={self._last_mean_z:.3f} m/s², '
                    f'expected={self._expected_gravity:.2f} m/s²)'
                ),
                measured=self._last_deviation,
                threshold=self._warn_tol,
                units='m/s²', stamp=stamp,
            )
        else:
            self._publish_status(
                check_name='imu_extrinsic',
                status=IntegrityStatus.STATUS_OK,
                recoverable=False,
                fault_code='',
                description=(
                    f'IMU extrinsic check nominal: '
                    f'mean Z={self._last_mean_z:.3f} m/s², '
                    f'deviation={self._last_deviation:.3f} m/s²'
                ),
                measured=self._last_deviation,
                threshold=self._warn_tol,
                units='m/s²', stamp=stamp,
            )

    # ── Status publisher ──────────────────────────────────────────────────────

    def _publish_status(self, *, check_name, status, recoverable, fault_code,
                        description, measured, threshold, units, stamp):
        msg = IntegrityStatus()
        msg.header.stamp        = stamp
        msg.header.frame_id     = 'extrinsic_validator'
        msg.check_name          = check_name
        msg.status              = status
        msg.recoverable         = recoverable
        msg.fault_code          = fault_code
        msg.description         = description
        msg.measured_value      = float(measured)
        msg.threshold_exceeded  = float(threshold)
        msg.units               = units
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExtrinsicValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
