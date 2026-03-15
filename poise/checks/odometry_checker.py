#!/usr/bin/env python3
"""
Odometry Cross-Check Node.

Subscribes to /sim/odometry (nav_msgs/Odometry) and /sim/gnss (sensor_msgs/NavSatFix).
Publishes IntegrityStatus on /poise/integrity_status.

Detection method
----------------
At each GNSS update, compares GNSS-reported displacement since the window anchor
against odometry-integrated displacement over the same interval.  Odometry velocity
(twist.twist.linear.x) is integrated between GNSS updates using the same sliding
window approach as gnss_imu_checker but with a longer window.

The integration window is 180 s by default — longer than the IMU dead-reckoning
window (60 s) because odometry error grows linearly O(T) rather than O(T^1.5) for
IMU, making odometry a more reliable reference over longer horizons.  With a 180 s
window and a 1.5 m warn threshold the minimum detectable drift rate is ~8 mm/s,
covering realistic urban GNSS multipath scenarios.

Fault codes
-----------
ODOM_GNSS_DIVERGENCE_WARN     (recoverable=False)
    Absolute displacement delta between GNSS and odometry exceeds warn_threshold_m.

ODOM_GNSS_DIVERGENCE_CRITICAL (recoverable=False)
    Absolute displacement delta exceeds critical_threshold_m.

ODOM_DROPOUT                  (recoverable=True)
    Odometry topic has been silent for longer than dropout_timeout_s.
    Clears automatically when odometry resumes.

ODOM_SLIP_SUSPECTED           (recoverable=False)
    Odometry systematically understates GNSS displacement by more than 10% over
    the integration window (slip_ratio < 0.90).  Requires at least 5 m of GNSS
    displacement to avoid false alarms near standstill.
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from poise.msg import IntegrityStatus
from poise.qos import SENSOR_QOS, INTEGRITY_QOS


# WGS-84 semi-major axis (metres)
_R_EARTH = 6_378_137.0

# Minimum GNSS displacement before the slip check is evaluated.
# Suppresses false positives when the vehicle is effectively stationary.
_SLIP_MIN_GNSS_DISP_M = 5.0

# Slip detection threshold: flag when odom understates GNSS by more than 10 %
_SLIP_RATIO_THRESHOLD = 0.90


def _geodetic_to_enu(lat_ref_deg, lon_ref_deg,
                     lat_deg, lon_deg, alt_m=0.0, alt_ref_m=0.0):
    """Flat-Earth ENU conversion — valid for baselines < ~10 km."""
    dlat = math.radians(lat_deg - lat_ref_deg)
    dlon = math.radians(lon_deg - lon_ref_deg)
    lat_ref = math.radians(lat_ref_deg)
    north = dlat * _R_EARTH
    east  = dlon * _R_EARTH * math.cos(lat_ref)
    up    = alt_m - alt_ref_m
    return east, north, up


class OdometryChecker(Node):
    """Cross-checks GNSS displacement against wheel odometry integration."""

    def __init__(self):
        super().__init__('odometry_checker')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('warn_threshold_m',     1.5)
        self.declare_parameter('critical_threshold_m', 3.0)
        self.declare_parameter('dropout_timeout_s',    2.0)
        self.declare_parameter('integration_window_s', 180.0)

        self.warn_thr        = self.get_parameter('warn_threshold_m').value
        self.crit_thr        = self.get_parameter('critical_threshold_m').value
        self.dropout_timeout = self.get_parameter('dropout_timeout_s').value
        self.window_s        = self.get_parameter('integration_window_s').value

        # ── GNSS reference and sliding-window anchor ─────────────────────────
        self._ref_lat  = None
        self._ref_lon  = None
        self._ref_alt  = 0.0

        self._anchor_east  = 0.0
        self._anchor_north = 0.0
        self._anchor_time: float | None = None   # monotonic clock

        # ── Odometry integration state ────────────────────────────────────────
        self._odom_disp          = 0.0   # signed sum of (velocity × dt) since anchor
        self._last_odom_stamp_ns: int | None = None
        self._odom_samples       = 0     # messages integrated since anchor

        # ── Dropout tracking ──────────────────────────────────────────────────
        self._last_odom_time: float | None = None   # monotonic
        self._dropout_active = False

        # ── Per-check fault state ─────────────────────────────────────────────
        self._divergence_fault_active = False
        self._slip_fault_active       = False

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self._status_pub = self.create_publisher(
            IntegrityStatus, '/poise/integrity_status', INTEGRITY_QOS
        )

        self.create_subscription(Odometry,  '/sim/odometry', self._odom_cb,  SENSOR_QOS)
        self.create_subscription(NavSatFix, '/sim/gnss',     self._gnss_cb,  SENSOR_QOS)

        # 1 Hz: dropout detection + STATUS_OK heartbeat
        self._periodic_timer = self.create_timer(1.0, self._periodic_cb)

        self.get_logger().info(
            f'OdometryChecker started | warn={self.warn_thr} m | '
            f'critical={self.crit_thr} m | window={self.window_s} s | '
            f'dropout_timeout={self.dropout_timeout} s'
        )

    # ── Odometry callback ─────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        now = time.monotonic()

        # Clear dropout when odometry resumes — reset dt baseline to avoid
        # integrating a spuriously large step across the silent gap.
        if self._dropout_active:
            self._dropout_active = False
            self._last_odom_stamp_ns = None
            self._publish_status(
                check_name='odom_dropout',
                status=IntegrityStatus.STATUS_OK,
                recoverable=True,
                fault_code='',
                description='Odometry resumed',
                measured=0.0, threshold=self.dropout_timeout,
                units='s', stamp=msg.header.stamp,
            )

        self._last_odom_time = now

        # Cannot integrate before first GNSS fix establishes the anchor
        if self._anchor_time is None:
            return

        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if self._last_odom_stamp_ns is None:
            self._last_odom_stamp_ns = stamp_ns
            return

        dt = (stamp_ns - self._last_odom_stamp_ns) * 1e-9
        self._last_odom_stamp_ns = stamp_ns

        if dt <= 0.0 or dt > 1.0:
            return

        # Forward velocity in the vehicle body frame (x = forward for ground vehicle)
        self._odom_disp  += msg.twist.twist.linear.x * dt
        self._odom_samples += 1

    # ── GNSS callback ─────────────────────────────────────────────────────────

    def _gnss_cb(self, msg: NavSatFix):
        now_mono  = time.monotonic()
        now_stamp = msg.header.stamp

        # ── First fix: set reference origin ───────────────────────────────
        if self._ref_lat is None:
            self._ref_lat = msg.latitude
            self._ref_lon = msg.longitude
            self._ref_alt = msg.altitude
            self.get_logger().info(
                f'Reference origin set: lat={self._ref_lat:.6f}, '
                f'lon={self._ref_lon:.6f}'
            )
            self._realign_anchor(0.0, 0.0, now_mono)
            return

        # ── Convert fix to ENU ─────────────────────────────────────────────
        east, north, _ = _geodetic_to_enu(
            self._ref_lat, self._ref_lon,
            msg.latitude, msg.longitude,
            msg.altitude, self._ref_alt,
        )

        if self._anchor_time is None:
            self._realign_anchor(east, north, now_mono)
            return

        elapsed = now_mono - self._anchor_time

        # ── Run cross-check at this fix ────────────────────────────────────
        if self._odom_samples > 0:
            self._run_checks(east, north, elapsed, now_stamp)

        # ── Periodic window realignment ────────────────────────────────────
        if elapsed >= self.window_s:
            self.get_logger().info(
                f'Odometry window realignment after {elapsed:.1f}s'
            )
            self._realign_anchor(east, north, now_mono)

    # ── Check logic ───────────────────────────────────────────────────────────

    def _run_checks(self, east: float, north: float,
                    elapsed: float, stamp) -> None:
        # Displacement magnitudes from the window anchor
        delta_east  = east  - self._anchor_east
        delta_north = north - self._anchor_north
        gnss_disp   = math.sqrt(delta_east ** 2 + delta_north ** 2)
        odom_disp   = abs(self._odom_disp)

        # Absolute divergence between GNSS and odometry displacement
        delta_m = abs(gnss_disp - odom_disp)

        self.get_logger().debug(
            f'gnss_disp={gnss_disp:.3f}m  odom_disp={odom_disp:.3f}m  '
            f'delta={delta_m:.3f}m  age={elapsed:.1f}s'
        )

        # ── Divergence check ─────────────────────────────────────────────
        if delta_m > self.crit_thr:
            self._divergence_fault_active = True
            self._publish_status(
                check_name='odom_gnss_cross_check',
                status=IntegrityStatus.STATUS_CRITICAL,
                recoverable=False,
                fault_code='ODOM_GNSS_DIVERGENCE_CRITICAL',
                description=(
                    f'Odometry/GNSS divergence {delta_m:.3f} m exceeds '
                    f'critical threshold {self.crit_thr:.3f} m '
                    f'(window age {elapsed:.1f}s)'
                ),
                measured=delta_m, threshold=self.crit_thr,
                units='m', stamp=stamp,
            )
        elif delta_m > self.warn_thr:
            self._divergence_fault_active = True
            self._publish_status(
                check_name='odom_gnss_cross_check',
                status=IntegrityStatus.STATUS_WARN,
                recoverable=False,
                fault_code='ODOM_GNSS_DIVERGENCE_WARN',
                description=(
                    f'Odometry/GNSS divergence {delta_m:.3f} m exceeds '
                    f'warn threshold {self.warn_thr:.3f} m '
                    f'(window age {elapsed:.1f}s)'
                ),
                measured=delta_m, threshold=self.warn_thr,
                units='m', stamp=stamp,
            )
        # Non-recoverable — divergence fault is NOT cleared once set.

        # ── Slip check ────────────────────────────────────────────────────
        # Only evaluate when there is meaningful GNSS displacement (avoids
        # divide-by-near-zero false alarms at standstill or startup).
        if gnss_disp > _SLIP_MIN_GNSS_DISP_M and not self._slip_fault_active:
            slip_ratio = odom_disp / gnss_disp
            if slip_ratio < _SLIP_RATIO_THRESHOLD:
                self._slip_fault_active = True
                understatement_pct = (1.0 - slip_ratio) * 100.0
                self._publish_status(
                    check_name='odom_slip_check',
                    status=IntegrityStatus.STATUS_WARN,
                    recoverable=False,
                    fault_code='ODOM_SLIP_SUSPECTED',
                    description=(
                        f'Odometry understates GNSS displacement by '
                        f'{understatement_pct:.1f}% — wheel slip suspected '
                        f'(odom={odom_disp:.2f}m, gnss={gnss_disp:.2f}m, '
                        f'window age {elapsed:.1f}s)'
                    ),
                    measured=slip_ratio,
                    threshold=_SLIP_RATIO_THRESHOLD,
                    units='ratio', stamp=stamp,
                )
                self.get_logger().warn(
                    f'ODOM_SLIP_SUSPECTED: ratio={slip_ratio:.3f} < '
                    f'{_SLIP_RATIO_THRESHOLD} '
                    f'(odom={odom_disp:.2f}m vs gnss={gnss_disp:.2f}m)'
                )

    # ── Anchor realignment ────────────────────────────────────────────────────

    def _realign_anchor(self, east: float, north: float,
                        mono_time: float) -> None:
        self._anchor_east    = east
        self._anchor_north   = north
        self._anchor_time    = mono_time
        self._odom_disp      = 0.0
        self._odom_samples   = 0
        self._last_odom_stamp_ns = None   # fresh dt baseline after realign

    # ── 1 Hz periodic callback ────────────────────────────────────────────────

    def _periodic_cb(self) -> None:
        """Dropout detection and STATUS_OK heartbeat."""
        stamp = self.get_clock().now().to_msg()

        # Dropout detection — only once odometry has been seen at least once
        if self._last_odom_time is not None and not self._dropout_active:
            since = time.monotonic() - self._last_odom_time
            if since > self.dropout_timeout:
                self._dropout_active = True
                self._publish_status(
                    check_name='odom_dropout',
                    status=IntegrityStatus.STATUS_WARN,
                    recoverable=True,
                    fault_code='ODOM_DROPOUT',
                    description=(
                        f'No odometry received for {since:.1f}s '
                        f'(timeout={self.dropout_timeout}s)'
                    ),
                    measured=since, threshold=self.dropout_timeout,
                    units='s', stamp=stamp,
                )
                self.get_logger().warn(
                    f'ODOM_DROPOUT: {since:.1f}s without odometry'
                )

        # Heartbeat STATUS_OK when all checks are nominal
        no_faults = (
            not self._dropout_active
            and not self._divergence_fault_active
            and not self._slip_fault_active
            and self._ref_lat is not None
        )
        if no_faults:
            self._publish_status(
                check_name='odometry_checker',
                status=IntegrityStatus.STATUS_OK,
                recoverable=True,
                fault_code='',
                description='All odometry checks nominal',
                measured=0.0, threshold=0.0,
                units='', stamp=stamp,
            )

    # ── Status publisher ──────────────────────────────────────────────────────

    def _publish_status(self, *, check_name, status, recoverable, fault_code,
                        description, measured, threshold, units, stamp) -> None:
        msg = IntegrityStatus()
        msg.header.stamp       = stamp
        msg.header.frame_id    = 'odometry_checker'
        msg.check_name         = check_name
        msg.status             = status
        msg.recoverable        = recoverable
        msg.fault_code         = fault_code
        msg.description        = description
        msg.measured_value     = float(measured)
        msg.threshold_exceeded = float(threshold)
        msg.units              = units
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
