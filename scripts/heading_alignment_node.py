#!/usr/bin/env python3
"""
Heading Alignment Node
----------------------
Start-of-mission heading calibration for GPS navigation.

Why: the Xsens yaw (NorthReference, magnetometer) is disturbed on the robot (field bags
2026-11-23: up to 60 deg wrong after spins), so it is never used here. The EKFs take yaw from
GLIM, whose odom frame has an arbitrary heading (the robot's at GLIM start). This node measures
the GLIM-to-ENU rotation once, by driving straight and comparing the GPS course over ground
with GLIM's heading, and gives it to navsat_transform_node.

Flow (Enter = yes everywhere, n + Enter = no):
  1. checks that GLIM, /odometry/global and navsat_transform_node are up and prints the GPS state,
  2. Enter -> drives STRAIGHT FORWARD calib_distance_m at calib_drive_speed_mps on /cmd_vel,
     whatever the GPS quality (no waiting for RTK),
  3. fits a line through the /gnss fixes of the drive (true ENU course) and compares it with
     GLIM's yaw over the same window -> GLIM-to-ENU correction. Hard failures only when no
     heading can be computed (too few fixes, too short a GPS track, robot not moving forward);
     a poor fit (residual, GPS/GLIM distance mismatch, poor fix quality) is shown as a warning,
  4. Enter -> applies it through navsat's ~/datum service (wait_for_datum: true).
Run it again to redo the calibration (e.g. after a poor result), then start the mission.

What navsat_transform_node does with a manual datum (navsat_transform.cpp, ros2 branch):
  - the datum lat/lon is paired with an IDENTITY world pose, i.e. it is the lat/lon of the
    map-frame ORIGIN, not of the robot;
  - the datum yaw is the ENU heading of the map +x axis, and navsat still adds
    magnetic_declination_radians + yaw_offset (+ UTM meridian convergence) to it.
So the node computes the map origin and map-axis heading from the robot's current GPS position,
its corrected true heading and its /odometry/global pose, and subtracts navsat's own
yaw_offset/magnetic_declination (read from its parameters) before calling ~/datum.

Exit code: 0 on success (datum applied), 1 on failure/abort.

Operator input (confirm_interface):
  stdin    Enter / n + Enter in the terminal (read from /dev/tty, so it also works under ros2 launch)
  service  ros2 service call /heading_alignment_node/operator_confirm std_srvs/srv/Trigger
           ros2 service call /heading_alignment_node/operator_abort   std_srvs/srv/Trigger

Abort at any time (stops the robot): Ctrl-C, or
  ros2 service call /heading_alignment_node/abort std_srvs/srv/Trigger

SAFETY: the Bunker driver has no /cmd_vel watchdog. This node's zero-velocity publishing on
every exit path is the only stop mechanism; keep the e-stop / RC in hand. The base must be in
CAN control mode (RC switch), otherwise it ignores /cmd_vel and the drive fails (no motion).
"""

import math
import select
import signal
import sys
import threading
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from robot_localization.srv import SetDatum, SetUTMZone

import tf2_ros


EARTH_RADIUS_M = 6378137.0


# ──────────────────────────────────────────────────────────────────────────────
# Pure math helpers (no ROS, unit-testable)
# ──────────────────────────────────────────────────────────────────────────────

def wrap(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def circular_mean(angles) -> float:
    angles = np.asarray(angles, dtype=float)
    return math.atan2(float(np.mean(np.sin(angles))), float(np.mean(np.cos(angles))))


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def enu_yaw_to_compass_deg(yaw_enu: float) -> float:
    """ENU yaw (0 = east, CCW positive) -> compass bearing (0 = north, CW positive), degrees.

    This is the convention conversion most likely to hide a sign/offset bug (the
    old yaw_offset: 1.57 is exactly this 90 deg). Used for operator-facing logs only:
    every computation in this node stays in ENU.
    """
    return math.degrees(wrap(math.pi / 2.0 - yaw_enu)) % 360.0


def latlon_to_enu(lat: float, lon: float, lat0: float, lon0: float):
    """Local tangent-plane (equirectangular) east/north of (lat, lon) around (lat0, lon0), meters.

    Accurate to well under a centimeter over the few hundred meters this node deals with.
    """
    east = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    north = math.radians(lat - lat0) * EARTH_RADIUS_M
    return east, north


def enu_to_latlon(east: float, north: float, lat0: float, lon0: float):
    """Inverse of latlon_to_enu."""
    lat = lat0 + math.degrees(north / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


def fit_line_bearing(points):
    """Least-squares (total least squares) line through 2D points.

    Returns (yaw_enu, rms_perpendicular_residual_m, extent_m). The line direction is
    oriented along the travel order (first -> last point).
    """
    p = np.asarray(points, dtype=float)
    c = p.mean(axis=0)
    d = p - c
    w, v = np.linalg.eigh(d.T @ d)
    direction = v[:, np.argmax(w)]
    if np.dot(direction, p[-1] - p[0]) < 0.0:
        direction = -direction
    perp = d @ np.array([-direction[1], direction[0]])
    along = d @ direction
    residual = float(math.sqrt(np.mean(perp ** 2)))
    extent = float(along.max() - along.min())
    return math.atan2(direction[1], direction[0]), residual, extent


def parse_gga_quality(sentence: str):
    """GGA fix quality (0 invalid, 1 single, 2 DGPS, 4 RTK fixed, 5 RTK float), or None."""
    s = sentence.strip()
    if not s.startswith('$') or len(s) < 7 or s[3:6] != 'GGA':
        return None
    fields = s.split('*')[0].split(',')
    if len(fields) < 7 or not fields[6]:
        return None
    try:
        return int(fields[6])
    except ValueError:
        return None


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────

class AlignmentFailed(Exception):
    pass


class HeadingAlignmentNode(Node):

    def __init__(self):
        super().__init__('heading_alignment_node')

        # Interfaces
        self.declare_parameter('confirm_interface', 'stdin')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('glim_odom_topic', '/glim_ros/odometry/local')
        self.declare_parameter('nmea_topic', '/nmea')
        self.declare_parameter('gnss_topic', '/gnss')
        self.declare_parameter('global_odom_topic', '/odometry/global')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('navsat_node', '/navsat_transform_node')
        self.declare_parameter('setutm_service', '/setUTMZone')
        self.declare_parameter('datum_service', '/datum')
        self.declare_parameter('utm_zone', '32N')

        # Startup / timeouts
        self.declare_parameter('startup_topic_timeout_s', 15.0)
        self.declare_parameter('glim_staleness_timeout_s', 1.0)
        self.declare_parameter('gga_max_age_s', 1.0)
        self.declare_parameter('confirm_timeout_s', 600.0)
        self.declare_parameter('service_call_retries', 3)
        self.declare_parameter('service_timeout_s', 5.0)

        # GPS: fixes used for the fit (GGA quality: 0 invalid, 1 single, 2 DGPS, 4 RTK fixed, 5 RTK float)
        self.declare_parameter('fix_min_quality', 1)

        # Calibration drive (open loop, no obstacle avoidance)
        self.declare_parameter('calib_drive_speed_mps', 0.3)
        self.declare_parameter('calib_distance_m', 10.0)
        self.declare_parameter('calib_max_duration_s', 60.0)
        self.declare_parameter('calib_max_yawrate_dps', 8.0)
        self.declare_parameter('calib_settle_s', 2.0)
        self.declare_parameter('cmd_vel_publish_rate_hz', 10.0)
        self.declare_parameter('calib_no_motion_timeout_s', 5.0)

        # Hard gates (no heading without these)
        self.declare_parameter('calib_min_fix_count', 3)
        self.declare_parameter('calib_min_gps_extent_m', 2.0)
        self.declare_parameter('calib_direction_sanity_deg', 90.0)
        # Soft gates (warnings shown before applying)
        self.declare_parameter('calib_warn_fix_count', 10)
        self.declare_parameter('calib_warn_fit_residual_m', 0.5)
        self.declare_parameter('calib_warn_distance_mismatch_ratio', 0.3)

        # Apply
        self.declare_parameter('apply_verify_tolerance_m', 3.0)

        gp = self.get_parameter
        self.p = {name: gp(name).value for name in self._parameters}

        if self.p['confirm_interface'] not in ('stdin', 'service'):
            raise RuntimeError(
                f"Parameter 'confirm_interface' must be 'stdin' or 'service', "
                f"got '{self.p['confirm_interface']}'.")

        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        # Shared state written by callbacks, read by the worker thread
        self._glim = deque(maxlen=30000)      # (t, x, y, yaw)
        self._gga = deque(maxlen=5000)        # (t, quality)
        self._fixes = deque(maxlen=5000)      # (t, lat, lon, alt, quality, frame_id)
        self._global_odom = None              # (t, x, y, yaw)
        self._imu_yaw = None                  # (t, yaw)

        self.abort_event = threading.Event()
        self._confirm_event = threading.Event()
        self._confirm_answer = None
        self._awaiting_confirm = False
        self._drive_active = False
        self.exit_code = 1

        # Subscriptions
        self.create_subscription(Odometry, self.p['glim_odom_topic'], self._glim_cb,
                                 qos_profile_sensor_data, callback_group=self._cb)
        self.create_subscription(Sentence, self.p['nmea_topic'], self._nmea_cb,
                                 qos_profile_sensor_data, callback_group=self._cb)
        self.create_subscription(NavSatFix, self.p['gnss_topic'], self._gnss_cb,
                                 qos_profile_sensor_data, callback_group=self._cb)
        self.create_subscription(Odometry, self.p['global_odom_topic'], self._global_odom_cb,
                                 10, callback_group=self._cb)
        # Xsens heading: diagnostic comparison only, never used for the result
        self.create_subscription(Imu, self.p['imu_topic'], self._imu_cb,
                                 qos_profile_sensor_data, callback_group=self._cb)

        # Publishers
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._cmd_pub = self.create_publisher(Twist, self.p['cmd_vel_topic'], 10)
        self._ready_pub = self.create_publisher(Bool, '/heading_alignment/ready', latched)
        self._status_pub = self.create_publisher(String, '/heading_alignment/status', latched)
        self._pending_pub = self.create_publisher(String, '~/pending_confirmation', latched)

        # Services
        self.create_service(Trigger, '~/abort', self._abort_srv, callback_group=self._cb)
        self.create_service(Trigger, '~/operator_confirm', self._confirm_srv,
                            callback_group=self._cb)
        self.create_service(Trigger, '~/operator_abort', self._reject_srv,
                            callback_group=self._cb)

        # Clients
        self._utm_client = self.create_client(SetUTMZone, self.p['setutm_service'],
                                              callback_group=self._cb)
        self._datum_client = self.create_client(SetDatum, self.p['datum_service'],
                                                callback_group=self._cb)
        self._navsat_params_client = self.create_client(
            GetParameters, self.p['navsat_node'].rstrip('/') + '/get_parameters',
            callback_group=self._cb)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._ready_pub.publish(Bool(data=False))
        self._log_config()

    # ──────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────

    def _glim_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        with self._lock:
            self._glim.append((time.monotonic(), p.x, p.y, yaw_from_quat(msg.pose.pose.orientation)))

    def _nmea_cb(self, msg: Sentence):
        q = parse_gga_quality(msg.sentence)
        if q is not None:
            with self._lock:
                self._gga.append((time.monotonic(), q))

    def _gnss_cb(self, msg: NavSatFix):
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        now = time.monotonic()
        with self._lock:
            # /gnss and /nmea come from the same Xsens packet: tag the fix with the latest GGA quality
            if self._gga and now - self._gga[-1][0] <= self.p['gga_max_age_s']:
                quality = self._gga[-1][1]
            elif not self._gga:
                quality = 1  # no NMEA at all: trust the NavSatFix status below (single fix)
            else:
                quality = 0
            # Only an explicit NO_FIX overrides GGA (Jazzy's default status is STATUS_UNKNOWN)
            if msg.status.status == NavSatStatus.STATUS_NO_FIX:
                quality = 0
            self._fixes.append((now, msg.latitude, msg.longitude, msg.altitude, quality,
                                msg.header.frame_id))

    def _global_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._global_odom = (time.monotonic(), p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))

    def _imu_cb(self, msg: Imu):
        self._imu_yaw = (time.monotonic(), yaw_from_quat(msg.orientation))

    def _abort_srv(self, request, response):
        self.get_logger().error('ABORT requested via service.')
        self.abort_event.set()
        self._confirm_event.set()
        response.success = True
        response.message = 'Abort requested.'
        return response

    def _confirm_srv(self, request, response):
        return self._answer_srv(True, response)

    def _reject_srv(self, request, response):
        return self._answer_srv(False, response)

    def _answer_srv(self, answer: bool, response):
        if not self._awaiting_confirm:
            response.success = False
            response.message = 'No confirmation is pending.'
            return response
        self._confirm_answer = answer
        self._confirm_event.set()
        response.success = True
        response.message = 'Confirmed.' if answer else 'Rejected.'
        return response

    # ──────────────────────────────────────────────────────────────────────
    # Small helpers
    # ──────────────────────────────────────────────────────────────────────

    def _log_config(self):
        self.get_logger().info(
            f"confirm_interface={self.p['confirm_interface']}  dry_run={self.p['dry_run']}  "
            f"speed={self.p['calib_drive_speed_mps']} m/s  "
            f"distance={self.p['calib_distance_m']} m, fixes with GGA quality >= {self.p['fix_min_quality']}")
        if self.p['dry_run']:
            self.get_logger().warn('DRY RUN: /cmd_vel will NOT be published.')

    def _set_state(self, state: str, detail: str = ''):
        text = state if not detail else f'{state}: {detail}'
        self._status_pub.publish(String(data=text))
        self.get_logger().info(f'[{state}] {detail}' if detail else f'[{state}]')

    def _check_abort(self):
        if self.abort_event.is_set():
            raise AlignmentFailed('aborted by operator/signal')

    def _sleep(self, seconds: float):
        """Sleep that wakes up immediately on abort."""
        if self.abort_event.wait(seconds):
            raise AlignmentFailed('aborted by operator/signal')

    def _glim_latest(self):
        with self._lock:
            return self._glim[-1] if self._glim else None

    def _glim_window(self, t0: float, t1: float):
        with self._lock:
            return [s for s in self._glim if t0 <= s[0] <= t1]

    def _gga_window(self, t0: float):
        with self._lock:
            return [s for s in self._gga if s[0] >= t0]

    def _fix_window(self, t0: float, t1: float, qualities):
        with self._lock:
            return [f for f in self._fixes if t0 <= f[0] <= t1 and f[4] in qualities]

    def _publish_cmd(self, vx: float):
        if self.p['dry_run']:
            return
        msg = Twist()
        msg.linear.x = float(vx)
        self._cmd_pub.publish(msg)

    def stop_robot(self, duration_s: float = None):
        """Publish zero velocity for duration_s (default calib_settle_s). Never raises on abort."""
        if duration_s is None:
            duration_s = self.p['calib_settle_s']
        period = 1.0 / self.p['cmd_vel_publish_rate_hz']
        end = time.monotonic() + duration_s
        while True:
            try:
                self._publish_cmd(0.0)
            except Exception as e:  # context already shut down: nothing more we can do
                self.get_logger().error(f'Could not publish zero velocity: {e}')
                return
            if time.monotonic() >= end:
                return
            time.sleep(period)

    def _call_service(self, client, request, name: str):
        timeout = self.p['service_timeout_s']
        retries = int(self.p['service_call_retries'])
        for attempt in range(1, retries + 1):
            self._check_abort()
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().warn(f'{name}: service not available (attempt {attempt}/{retries}).')
                continue
            future = client.call_async(request)
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if done.wait(timeout) and future.result() is not None:
                return future.result()
            self.get_logger().warn(f'{name}: no response (attempt {attempt}/{retries}).')
        raise AlignmentFailed(f'{name} failed after {retries} attempts')

    # ──────────────────────────────────────────────────────────────────────
    # Operator confirmation
    # ──────────────────────────────────────────────────────────────────────

    def confirm(self, question: str) -> bool:
        """Block until the operator confirms (True) or rejects (False). Timeout -> False."""
        timeout = self.p['confirm_timeout_s']
        self._pending_pub.publish(String(data=question))
        bar = '=' * 60
        try:
            if self.p['confirm_interface'] == 'stdin':
                return self._confirm_stdin(question, timeout, bar)
            return self._confirm_service(question, timeout, bar)
        finally:
            self._pending_pub.publish(String(data=''))

    def _confirm_stdin(self, question: str, timeout: float, bar: str) -> bool:
        # ros2 launch connects the child's stdin to a pipe, not to the terminal; the
        # controlling terminal is still reachable through /dev/tty.
        try:
            tty = open('/dev/tty', 'r')
        except OSError:
            if not sys.stdin.isatty():
                raise AlignmentFailed(
                    "confirm_interface=stdin but no terminal is available; "
                    "use confirm_interface:=service")
            tty = sys.stdin
        self.get_logger().warn(f'\n{bar}\n{question}\n\nPress Enter to continue, n + Enter to stop  '
                               f'(timeout {timeout:.0f} s)\n{bar}')
        end = time.monotonic() + timeout
        try:
            while time.monotonic() < end:
                self._check_abort()
                ready, _, _ = select.select([tty], [], [], 0.2)
                if not ready:
                    continue
                answer = tty.readline().strip().lower()
                if answer in ('', 'y', 'yes'):
                    return True
                if answer in ('n', 'no'):
                    return False
                self.get_logger().warn("Press Enter to continue or type 'n'.")
        finally:
            if tty is not sys.stdin:
                tty.close()
        self.get_logger().error('Confirmation timed out.')
        return False

    def _confirm_service(self, question: str, timeout: float, bar: str) -> bool:
        node = self.get_fully_qualified_name()
        self._confirm_answer = None
        self._confirm_event.clear()
        self._awaiting_confirm = True
        self.get_logger().warn(
            f'\n{bar}\n{question}\n\n'
            f'  confirm: ros2 service call {node}/operator_confirm std_srvs/srv/Trigger\n'
            f'  reject : ros2 service call {node}/operator_abort std_srvs/srv/Trigger\n'
            f'(timeout {timeout:.0f} s)\n{bar}')
        try:
            if not self._confirm_event.wait(timeout):
                self.get_logger().error('Confirmation timed out.')
                return False
            self._check_abort()
            return bool(self._confirm_answer)
        finally:
            self._awaiting_confirm = False

    # ──────────────────────────────────────────────────────────────────────
    # State machine (runs in its own thread; callbacks run in the executor)
    # ──────────────────────────────────────────────────────────────────────

    def run(self):
        try:
            self._state_init()
            self._state_start_confirm()
            fit, t_stop = self._state_calibration_drive()
            self._state_correction_confirm(fit)
            self._state_apply(fit, t_stop)
            self._ready()
            self.exit_code = 0
        except AlignmentFailed as e:
            self._fail(str(e))
        except Exception as e:  # anything unexpected: stop first, then report
            self.stop_robot()
            self.get_logger().error(f'Unexpected error: {e!r}')
            self._fail(f'unexpected error: {e!r}')

    def _qualities(self):
        return range(int(self.p['fix_min_quality']), 10)

    # INIT ────────────────────────────────────────────────────────────────

    def _state_init(self):
        self._set_state('INIT', 'checking GLIM, /odometry/global and navsat_transform_node')
        timeout = self.p['startup_topic_timeout_s']
        end = time.monotonic() + timeout
        missing = []
        while time.monotonic() < end:
            self._check_abort()
            now = time.monotonic()
            missing = []
            glim = self._glim_latest()
            if glim is None or now - glim[0] > self.p['glim_staleness_timeout_s']:
                missing.append(self.p['glim_odom_topic'])
            if self._global_odom is None or now - self._global_odom[0] > 2.0:
                missing.append(self.p['global_odom_topic'])
            if not missing:
                break
            self._sleep(0.5)
        if missing:
            raise AlignmentFailed(f'no recent data within {timeout:.0f} s on: {", ".join(missing)}')

        for client, name in ((self._utm_client, self.p['setutm_service']),
                             (self._datum_client, self.p['datum_service'])):
            if not client.wait_for_service(timeout_sec=self.p['service_timeout_s']):
                raise AlignmentFailed(f'service {name} not available (is navsat_transform_node up?)')

        self._navsat_offsets = self._read_navsat_offsets()
        self._set_state('INIT', 'all inputs alive')

    def _read_navsat_offsets(self):
        names = ['yaw_offset', 'magnetic_declination_radians', 'wait_for_datum']
        req = GetParameters.Request(names=names)
        res = self._call_service(self._navsat_params_client, req, 'navsat get_parameters')
        if len(res.values) != len(names):
            raise AlignmentFailed('could not read navsat_transform_node parameters')
        values = {}
        for name, v in zip(names, res.values):
            if v.type == ParameterType.PARAMETER_BOOL:
                values[name] = v.bool_value
            elif v.type == ParameterType.PARAMETER_DOUBLE:
                values[name] = v.double_value
            elif v.type == ParameterType.PARAMETER_INTEGER:
                values[name] = float(v.integer_value)
            else:
                raise AlignmentFailed(f'navsat_transform_node parameter {name} is not set')
        if not values['wait_for_datum']:
            self.get_logger().warn(
                'navsat_transform_node has wait_for_datum: false. The datum will still be applied, '
                'but until then the map->utm transform comes from the odometry heading.')
        self.get_logger().info(
            f"navsat yaw_offset={values['yaw_offset']:.4f} rad, "
            f"magnetic_declination={values['magnetic_declination_radians']:.4f} rad "
            f"(both are added by navsat to the datum yaw; compensated here)")
        return values

    # START_CONFIRM ───────────────────────────────────────────────────────

    def _gps_status_text(self) -> str:
        now = time.monotonic()
        names = {0: 'invalid', 1: 'single', 2: 'DGPS', 4: 'RTK fixed', 5: 'RTK float'}
        gga = self._gga_window(now - 2.0)
        fixes = self._fix_window(now - 2.0, now, range(0, 10))
        if not fixes:
            return f'GPS: NO DATA on {self.p["gnss_topic"]} -- the drive will not give a heading.'
        q = gga[-1][1] if gga else None
        qtxt = f'quality {q} ({names.get(q, "?")})' if q is not None else 'quality unknown (no GGA)'
        usable = q is not None and q >= self.p['fix_min_quality']
        return (f'GPS: {qtxt}' + ('' if usable else
                ' -- NO usable fix now: if it stays like this the fit will fail'))

    def _state_start_confirm(self):
        self._set_state('START_CONFIRM')
        n_pub = self.count_publishers(self.p['cmd_vel_topic'])
        if n_pub > 1:
            self.get_logger().warn(
                f'{n_pub - 1} other publisher(s) on {self.p["cmd_vel_topic"]}: '
                f'make sure nothing else is commanding the robot (no mission running).')
        question = (
            f'{self._gps_status_text()}\n\n'
            f'CALIBRATION DRIVE: the robot will drive STRAIGHT FORWARD '
            f'{self.p["calib_distance_m"]:.0f} m at {self.p["calib_drive_speed_mps"]:.2f} m/s, '
            f'with NO obstacle avoidance.\n'
            f'Check that the area ahead is clear and flat, the base is in CAN mode and the RC / '
            f'e-stop is in hand.'
            + ('\n(DRY RUN: no /cmd_vel will be published.)' if self.p['dry_run'] else ''))
        if not self.confirm(question):
            raise AlignmentFailed('operator stopped before the calibration drive')

    # CALIBRATION_DRIVE ───────────────────────────────────────────────────

    def _state_calibration_drive(self):
        self._set_state('CALIBRATION_DRIVE', f'driving {self.p["calib_distance_m"]:.1f} m at '
                                             f'{self.p["calib_drive_speed_mps"]:.2f} m/s')
        speed = self.p['calib_drive_speed_mps']
        period = 1.0 / self.p['cmd_vel_publish_rate_hz']
        max_yawrate = math.radians(self.p['calib_max_yawrate_dps'])
        start = self._glim_latest()
        if start is None:
            raise AlignmentFailed('GLIM odometry lost before the drive')
        t_start = time.monotonic()
        stop_reason = ''
        last_log = t_start
        # a longer calib_distance_m extends the cap: 1.5x the nominal drive time
        time_cap = max(self.p['calib_max_duration_s'], 1.5 * self.p['calib_distance_m'] / max(speed, 0.05))

        self._drive_active = True
        try:
            while True:
                self._check_abort()
                now = time.monotonic()
                glim = self._glim_latest()
                if glim is None or now - glim[0] > self.p['glim_staleness_timeout_s']:
                    raise AlignmentFailed('GLIM odometry went stale during the drive')

                yawrate = self._glim_yawrate(now)
                if yawrate is not None and abs(yawrate) > max_yawrate:
                    raise AlignmentFailed(
                        f'yaw rate {math.degrees(yawrate):.1f} deg/s exceeds '
                        f'{self.p["calib_max_yawrate_dps"]:.1f} deg/s (robot is not driving straight)')

                distance = math.hypot(glim[1] - start[1], glim[2] - start[2])
                if (now - t_start > self.p['calib_no_motion_timeout_s'] and
                        distance < 0.1 * speed * self.p['calib_no_motion_timeout_s']):
                    raise AlignmentFailed(
                        f'the robot does not move ({distance:.2f} m in {now - t_start:.0f} s): '
                        f'base not in CAN mode (RC switch) or another node commanding it?')
                if distance >= self.p['calib_distance_m']:
                    stop_reason = f'distance reached ({distance:.1f} m)'
                    break
                if now - t_start >= time_cap:
                    stop_reason = f'time cap ({time_cap:.0f} s) at {distance:.1f} m'
                    break
                if now - last_log >= 5.0:
                    last_log = now
                    n = len(self._fix_window(t_start, now, self._qualities()))
                    self.get_logger().info(f'{distance:.1f} / {self.p["calib_distance_m"]:.1f} m, '
                                           f'{n} usable GPS fixes so far')

                self._publish_cmd(speed)
                time.sleep(period)
        finally:
            # Hard requirement: the base has no watchdog. Always stop, whatever happened above.
            t_stop = time.monotonic()
            self.stop_robot()
            self._drive_active = False

        self.get_logger().info(f'Drive stopped: {stop_reason}')
        return self._state_compute(t_start, t_stop), t_stop

    def _glim_yawrate(self, now: float, window_s: float = 0.5):
        samples = self._glim_window(now - window_s, now)
        if len(samples) < 2 or samples[-1][0] - samples[0][0] < 0.2:
            return None
        yaws = np.unwrap([s[3] for s in samples])
        return float((yaws[-1] - yaws[0]) / (samples[-1][0] - samples[0][0]))

    # COMPUTE_CORRECTION ──────────────────────────────────────────────────

    def _evaluate_fit(self, t0: float, t1: float) -> dict:
        """Hard gates (failures: no heading) and soft gates (warnings)."""
        failures, warnings = [], []
        fixes = self._fix_window(t0, t1, self._qualities())
        result = {'ok': False, 'failures': failures, 'warnings': warnings, 'n_fixes': len(fixes)}

        if len(fixes) < self.p['calib_min_fix_count']:
            failures.append(f'{len(fixes)} usable GPS fixes during the drive '
                            f'(< {self.p["calib_min_fix_count"]}): no GPS, no heading')
            return result

        lat0, lon0 = fixes[0][1], fixes[0][2]
        pts = [latlon_to_enu(f[1], f[2], lat0, lon0) for f in fixes]
        gps_yaw, residual, gps_extent = fit_line_bearing(pts)

        glim = self._glim_window(fixes[0][0], fixes[-1][0])
        if len(glim) < 2:
            failures.append('no GLIM samples over the fix window')
            return result
        glim_yaw = circular_mean([s[3] for s in glim])
        glim_dist = math.hypot(glim[-1][1] - glim[0][1], glim[-1][2] - glim[0][2])
        glim_travel = math.atan2(glim[-1][2] - glim[0][2], glim[-1][1] - glim[0][1])
        quality_counts = {}
        for f in fixes:
            quality_counts[f[4]] = quality_counts.get(f[4], 0) + 1

        result.update(gps_yaw=gps_yaw, glim_yaw=glim_yaw, residual=residual,
                      gps_extent=gps_extent, glim_dist=glim_dist, qualities=quality_counts,
                      correction=wrap(gps_yaw - glim_yaw))

        if gps_extent < self.p['calib_min_gps_extent_m']:
            failures.append(f'GPS track only {gps_extent:.1f} m long '
                            f'(< {self.p["calib_min_gps_extent_m"]:.1f} m): no direction to measure')
        # Commanded direction is +x (forward): GLIM must see the robot moving along its own heading.
        # A reversed motion is a fault, never an ambiguity to resolve by flipping the sign.
        direction_err = math.degrees(abs(wrap(glim_travel - glim_yaw)))
        if glim_dist > 0.5 and direction_err > self.p['calib_direction_sanity_deg']:
            failures.append(f'robot moved {direction_err:.0f} deg away from its forward axis '
                            f'(commanded forward)')

        if len(fixes) < self.p['calib_warn_fix_count']:
            warnings.append(f'only {len(fixes)} GPS fixes')
        if residual > self.p['calib_warn_fit_residual_m']:
            warnings.append(f'GPS track not straight: fit residual {residual:.2f} m '
                            f'(> {self.p["calib_warn_fit_residual_m"]:.2f} m)')
        if glim_dist > 0.5:
            mismatch = abs(gps_extent - glim_dist) / glim_dist
            if mismatch > self.p['calib_warn_distance_mismatch_ratio']:
                warnings.append(f'GPS distance {gps_extent:.1f} m vs GLIM {glim_dist:.1f} m '
                                f'({100 * mismatch:.0f}% mismatch)')
        if not any(q in (4, 5) for q in quality_counts):
            warnings.append('no RTK fix during the drive (single/DGPS only): expect a few degrees '
                            'of error')

        result['ok'] = not failures
        return result

    def _state_compute(self, t_start, t_stop):
        self._set_state('COMPUTE_CORRECTION')
        fit = self._evaluate_fit(t_start, t_stop)
        if not fit['ok']:
            raise AlignmentFailed('no heading: ' + '; '.join(fit['failures']))
        self.get_logger().info(
            f"Fit: {fit['n_fixes']} fixes (quality counts {fit['qualities']}), residual "
            f"{fit['residual']:.3f} m, GPS {fit['gps_extent']:.2f} m / GLIM {fit['glim_dist']:.2f} m, "
            f"GPS course {math.degrees(fit['gps_yaw']):+.1f} deg ENU, "
            f"GLIM yaw {math.degrees(fit['glim_yaw']):+.1f} deg, "
            f"correction {math.degrees(fit['correction']):+.2f} deg")
        return fit

    # CORRECTION_CONFIRM ──────────────────────────────────────────────────

    def _state_correction_confirm(self, fit):
        self._set_state('CORRECTION_CONFIRM')
        glim = self._glim_latest()
        new_yaw = wrap(glim[3] + fit['correction'])
        lines = [f'NEW heading (GPS-calibrated GLIM): {math.degrees(new_yaw):+7.1f} deg ENU '
                 f'= {enu_yaw_to_compass_deg(new_yaw):5.1f} deg compass',
                 f'GLIM-to-ENU correction: {math.degrees(fit["correction"]):+.1f} deg']
        # Xsens heading, for comparison only (it is disturbed on the robot: never used)
        if self._imu_yaw is not None and time.monotonic() - self._imu_yaw[0] < 1.0:
            off = self._navsat_offsets
            xs = wrap(self._imu_yaw[1] + off['yaw_offset'] + off['magnetic_declination_radians'])
            lines.append(f'(Xsens + yaw_offset says {math.degrees(xs):+.1f} deg ENU, '
                         f'{math.degrees(wrap(new_yaw - xs)):+.1f} deg away: comparison only)')
        if fit['warnings']:
            lines.append('WARNINGS:')
            lines += [f'  - {w}' for w in fit['warnings']]
            if any(w.startswith(('GPS track not straight', 'GPS distance')) for w in fit['warnings']):
                lines.append('  The GPS track does not match the straight drive GLIM measured: the heading may be '
                             'off by tens of degrees.\n  Better: n, then run again with a longer drive, e.g. '
                             'heading_align -p calib_distance_m:=20.0')
        else:
            lines.append('Fit quality: OK')
        summary = '\n'.join(lines)
        self.get_logger().warn('\n' + summary)
        if not self.confirm(summary + '\n\nApply this heading?'):
            raise AlignmentFailed('operator discarded the heading correction')

    # APPLY_CORRECTION ────────────────────────────────────────────────────

    def _state_apply(self, fit, t_stop):
        self._set_state('APPLY_CORRECTION')
        now = time.monotonic()
        glim = self._glim_latest()
        odom = self._global_odom
        if glim is None or now - glim[0] > self.p['glim_staleness_timeout_s']:
            raise AlignmentFailed('GLIM odometry stale at apply time')
        if odom is None or now - odom[0] > 2.0:
            raise AlignmentFailed(f'{self.p["global_odom_topic"]} stale at apply time')

        # Robot GPS position: mean of the stationary fixes after the stop
        fixes = self._fix_window(t_stop, now, self._qualities())
        if not fixes:
            fixes = self._fix_window(t_stop - 5.0, now, self._qualities())[-1:]
        if not fixes:
            raise AlignmentFailed('no usable GPS fix at the end of the drive')
        lat0, lon0 = fixes[0][1], fixes[0][2]
        enu = np.array([latlon_to_enu(f[1], f[2], lat0, lon0) for f in fixes]).mean(axis=0)
        alt = float(np.mean([f[3] for f in fixes]))

        # Robot true heading (base_link, ENU) and its map-frame pose at the same instant
        true_yaw = wrap(glim[3] + fit['correction'])
        _, map_x, map_y, map_yaw = odom
        map_heading = wrap(true_yaw - map_yaw)        # ENU heading of the map +x axis

        # Antenna -> base_link, the same TF navsat uses for later fixes
        ant_x, ant_y = self._antenna_offset(fixes[-1][5])
        c, s = math.cos(true_yaw), math.sin(true_yaw)
        base_e = enu[0] - (c * ant_x - s * ant_y)
        base_n = enu[1] - (s * ant_x + c * ant_y)

        # navsat pairs the datum with an identity map pose: the datum is the map ORIGIN
        c, s = math.cos(map_heading), math.sin(map_heading)
        origin_e = base_e - (c * map_x - s * map_y)
        origin_n = base_n - (s * map_x + c * map_y)
        origin_lat, origin_lon = enu_to_latlon(origin_e, origin_n, lat0, lon0)

        # navsat adds yaw_offset + declination (+ meridian convergence, which it handles itself)
        off = self._navsat_offsets
        datum_yaw = wrap(map_heading - off['yaw_offset'] - off['magnetic_declination_radians'])

        self.get_logger().info(
            f'Robot at map ({map_x:.2f}, {map_y:.2f}, {math.degrees(map_yaw):+.1f} deg), '
            f'true heading {math.degrees(true_yaw):+.1f} deg ENU; map +x axis heading '
            f'{math.degrees(map_heading):+.1f} deg ENU; map origin {origin_lat:.8f}, {origin_lon:.8f}; '
            f'datum yaw sent {math.degrees(datum_yaw):+.2f} deg')

        # Back-to-back, in this order: /setUTMZone re-enables navsat's auto-latch from /gnss,
        # /datum right after re-asserts manual mode with our values (see plan).
        self._call_service(self._utm_client, SetUTMZone.Request(utm_zone=self.p['utm_zone']),
                           self.p['setutm_service'])
        req = SetDatum.Request()
        req.geo_pose.position.latitude = origin_lat
        req.geo_pose.position.longitude = origin_lon
        req.geo_pose.position.altitude = alt
        req.geo_pose.orientation.z = math.sin(datum_yaw / 2.0)
        req.geo_pose.orientation.w = math.cos(datum_yaw / 2.0)
        self._call_service(self._datum_client, req, self.p['datum_service'])
        self._verify_apply(map_x, map_y)

    def _antenna_offset(self, gnss_frame: str):
        base = self.p['base_link_frame']
        if not gnss_frame or gnss_frame == base:
            return 0.0, 0.0
        try:
            tf = self._tf_buffer.lookup_transform(base, gnss_frame, Time())
            t = tf.transform.translation
            return t.x, t.y
        except Exception as e:
            self.get_logger().warn(f'No TF {base} -> {gnss_frame} ({e}); assuming zero antenna offset.')
            return 0.0, 0.0

    def _verify_apply(self, map_x: float, map_y: float):
        """Warn-only check: after the datum, /odometry/gps should land near the robot's map pose."""
        got = threading.Event()
        holder = {}

        def cb(msg):
            holder['p'] = msg.pose.pose.position
            got.set()

        # Give navsat a couple of timer ticks to latch the new transform first
        self._sleep(1.5)
        sub = self.create_subscription(Odometry, '/odometry/gps', cb, 10, callback_group=self._cb)
        try:
            if not got.wait(5.0):
                self.get_logger().warn('No /odometry/gps after the datum: could not verify the result.')
                return
            p = holder['p']
            err = math.hypot(p.x - map_x, p.y - map_y)
            msg = (f'/odometry/gps ({p.x:.2f}, {p.y:.2f}) vs robot map pose ({map_x:.2f}, {map_y:.2f}): '
                   f'{err:.2f} m')
            if err > self.p['apply_verify_tolerance_m']:
                self.get_logger().warn(msg + ' -- larger than expected, check RViz before navigating.')
            else:
                self.get_logger().info(msg)
        finally:
            self.destroy_subscription(sub)

    # READY / FAILED ──────────────────────────────────────────────────────

    def _ready(self):
        self._ready_pub.publish(Bool(data=True))
        self._set_state('READY', 'heading alignment applied')
        bar = '#' * 60
        self.get_logger().info('\n'.join(['', bar, bar, '', '   HEADING ALIGNMENT COMPLETE', '',
                                          '   You can now start the GPS mission (send_gps).', '', bar, bar]))

    def _fail(self, reason: str):
        if self._drive_active:
            self.stop_robot()
        self._ready_pub.publish(Bool(data=False))
        self._set_state('FAILED', reason)
        bar = '#' * 60
        self.get_logger().error('\n'.join(['', bar, bar, '', '   HEADING ALIGNMENT FAILED', '',
                                           f'   {reason}', '', '   Do NOT start the GPS mission: run heading_align again.',
                                           '', bar, bar]))


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main():
    # Own signal handling: rclpy's default handler shuts the context down immediately,
    # which would make the final zero-velocity publishes impossible.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = HeadingAlignmentNode()

    def on_signal(signum, frame):
        node.get_logger().error(f'Signal {signum} received: stopping.')
        node.abort_event.set()
        node._confirm_event.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=node.run, daemon=True)
    worker.start()

    try:
        while worker.is_alive():
            executor.spin_once(timeout_sec=0.1)
        # Let the latched status/ready messages go out before exiting
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.1)
    finally:
        if node._drive_active:
            node.stop_robot()
        exit_code = node.exit_code
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
