#!/usr/bin/env python3
"""
GPS Waypoint Follower Node
--------------------------
Reads a YAML mission file containing named GPS coordinates and a waypoint sequence,
converts them to map-frame poses via robot_localization's /fromLL service,
and sends them to Nav2's FollowGPSWaypoints action server.

Modes:
  --mode auto    Send all waypoints at once (default)
  --mode step    Send one waypoint at a time, waiting for Enter between each

Usage:
  ros2 run <package> gps_waypoint_follower.py --mission mission.yaml --mode auto
  ros2 run <package> gps_waypoint_follower.py --mission mission.yaml --mode step

Mission YAML format:
  points:
    home:
      lat: 43.7234
      lon: 10.4012
    gate:
      lat: 43.7240
      lon: 10.4018
    field_a:
      lat: 43.7245
      lon: 10.4025

  mission:
    - home
    - gate
    - field_a
    - home
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

import yaml
from geographic_msgs.msg import GeoPose
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Quaternion
from nav2_msgs.action import FollowGPSWaypoints


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────

class GpsWaypointFollower(Node):

    def __init__(self):
        super().__init__('gps_waypoint_follower')

        # Declare parameters
        self.declare_parameter('mission_file', '')
        self.declare_parameter('mode', 'auto')

        mission_file = self.get_parameter('mission_file').get_parameter_value().string_value
        mode         = self.get_parameter('mode').get_parameter_value().string_value

        # Validate
        if not mission_file:
            raise RuntimeError(
                "Parameter 'mission_file' is required but was not set. "
                "Pass it via CLI:  --ros-args -p mission_file:=/path/to/mission.yaml"
            )
        if mode not in ('auto', 'step'):
            raise RuntimeError(f"Parameter 'mode' must be 'auto' or 'step', got '{mode}'.")

        self._mode = mode
        self._mission_file = mission_file

        # Action client for Nav2 GPS waypoint follower
        self._action_client = ActionClient(
            self,
            FollowGPSWaypoints,
            'follow_gps_waypoints'
        )

        self.get_logger().info(f'Mode       : {self._mode}')
        self.get_logger().info(f'Mission    : {self._mission_file}')

        # Load and resolve the mission
        self._waypoints = self._load_mission(mission_file)
        self.get_logger().info(
            f'Loaded {len(self._waypoints)} waypoint(s): '
            + ', '.join(wp["label"] for wp in self._waypoints)
        )

        # Wait for the action server
        self.get_logger().info('Waiting for follow_gps_waypoints action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready.')

        # Kick off navigation after a short delay so the node is fully spun up
        self.create_timer(1.0, self._start)
        self._started = False

    # ──────────────────────────────────────────────────────────────────────
    # Mission loading
    # ──────────────────────────────────────────────────────────────────────

    def _load_mission(self, path: str) -> list:
        """Parse the YAML file and return an ordered list of waypoint dicts."""
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        # Validate top-level keys
        if 'points' not in data:
            raise ValueError("Mission YAML must contain a 'points' section.")
        if 'mission' not in data:
            raise ValueError("Mission YAML must contain a 'mission' section.")

        named_points = data['points']
        sequence     = data['mission']

        waypoints = []
        for label in sequence:
            if label not in named_points:
                raise ValueError(
                    f"Waypoint '{label}' listed in mission but not defined in points."
                )
            pt = named_points[label]
            if 'lat' not in pt or 'lon' not in pt:
                raise ValueError(
                    f"Point '{label}' must have 'lat' and 'lon' fields."
                )
            waypoints.append({
                'label': label,
                'lat':   float(pt['lat']),
                'lon':   float(pt['lon']),
                'alt':   float(pt.get('alt', 0.0)),
            })

        return waypoints

    # ──────────────────────────────────────────────────────────────────────
    # GeoPose helpers
    # ──────────────────────────────────────────────────────────────────────

    def _make_geopose(self, wp: dict) -> GeoPose:
        msg = GeoPose()
        msg.position.latitude  = wp['lat']
        msg.position.longitude = wp['lon']
        msg.position.altitude  = wp['alt']
        msg.orientation.w = 1.0  # identity quaternion, RotateToGoal handles final heading
        return msg

    # ──────────────────────────────────────────────────────────────────────
    # Navigation logic
    # ──────────────────────────────────────────────────────────────────────

    def _start(self):
        """Called once by the timer after node startup."""
        if self._started:
            return
        self._started = True

        if self._mode == 'auto':
            self._run_auto()
        else:
            self._run_step()

    def _run_auto(self):
        """Send all waypoints in a single action call."""
        self.get_logger().info('AUTO mode: sending all waypoints at once.')

        self._last_feedback_index = None

        goal = FollowGPSWaypoints.Goal()
        goal.number_of_loops = 0
        goal.gps_poses = [self._make_geopose(wp) for wp in self._waypoints]

        self.get_logger().info(
            'Sending ' + str(len(goal.gps_poses)) + ' waypoint(s)...'
        )
        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _run_step(self):
        """Send one waypoint at a time, waiting for user input between each."""
        self.get_logger().info('STEP mode: press Enter to advance to next waypoint.')
        self._step_index = 0
        self._reached_count = 0
        self._failed_count = 0
        self._send_next_step()

    def _send_next_step(self):
        if self._step_index >= len(self._waypoints):
            self.get_logger().info('All waypoints completed.')
            rclpy.shutdown()
            return

        wp = self._waypoints[self._step_index]
        self.get_logger().info(
            f'[{self._step_index + 1}/{len(self._waypoints)}] '
            f'Navigating to "{wp["label"]}"  '
            f'(lat={wp["lat"]:.6f}, lon={wp["lon"]:.6f})'
        )

        goal = FollowGPSWaypoints.Goal()
        goal.number_of_loops = 0
        goal.gps_poses = [self._make_geopose(wp)]

        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._step_goal_response_callback)

    def _wait_for_enter_then_next(self):
        """Block the calling thread waiting for Enter, then send the next wp."""
        if self._step_index < len(self._waypoints):
            input(
                f'\n  Press Enter to go to next waypoint '
                f'({self._step_index + 1}/{len(self._waypoints)})...\n'
            )
        self._send_next_step()

    # ──────────────────────────────────────────────────────────────────────
    # Mission summary
    # ──────────────────────────────────────────────────────────────────────

    def _log_mission_summary(self, reached: int, failed: int, total: int):
        """Log a large, hard-to-miss banner summarizing the whole mission."""
        bar   = '#' * 60
        title = 'MISSION COMPLETE' if failed == 0 else 'MISSION FINISHED WITH FAILURES'

        banner = '\n'.join([
            '',
            bar,
            bar,
            '',
            f'   {title}'.upper(),
            '',
            f'   WAYPOINTS REACHED : {reached}/{total}',
            f'   WAYPOINTS FAILED  : {failed}/{total}',
            '',
            bar,
            bar,
        ])

        if failed == 0:
            self.get_logger().info(banner)
        else:
            self.get_logger().warn(banner)

    # ──────────────────────────────────────────────────────────────────────
    # Action callbacks — AUTO mode
    # ──────────────────────────────────────────────────────────────────────

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server.')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted, navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        total  = len(self._waypoints)

        missed_by_index = {m.index: m for m in result.missed_waypoints}

        for i, wp in enumerate(self._waypoints):
            if i in missed_by_index:
                self.get_logger().warn(
                    f'Waypoint {i + 1}/{total} "{wp["label"]}" FAILED '
                    f'(error_code={missed_by_index[i].error_code}).'
                )
            else:
                self.get_logger().info(
                    f'Waypoint {i + 1}/{total} "{wp["label"]}" reached.'
                )

        failed  = len(missed_by_index)
        reached = total - failed
        self._log_mission_summary(reached, failed, total)
        rclpy.shutdown()

    # ──────────────────────────────────────────────────────────────────────
    # Action callbacks — STEP mode
    # ──────────────────────────────────────────────────────────────────────

    def _step_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            rclpy.shutdown()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._step_result_callback)

    def _step_result_callback(self, future):
        status = future.result().status
        wp     = self._waypoints[self._step_index]

        if status == 4:
            self._reached_count += 1
            self.get_logger().info(
                f'Waypoint {self._step_index + 1}/{len(self._waypoints)} "{wp["label"]}" reached.'
            )
        else:
            self._failed_count += 1
            self.get_logger().warn(
                f'Waypoint {self._step_index + 1}/{len(self._waypoints)} "{wp["label"]}" FAILED '
                f'(status={status}).'
            )

        self._step_index += 1

        if self._step_index >= len(self._waypoints):
            self._log_mission_summary(
                self._reached_count, self._failed_count, len(self._waypoints)
            )
            rclpy.shutdown()
            return

        # Block on Enter in a separate thread so the ROS executor keeps spinning
        import threading
        t = threading.Thread(target=self._wait_for_enter_then_next, daemon=True)
        t.start()

    # ──────────────────────────────────────────────────────────────────────
    # Feedback
    # ──────────────────────────────────────────────────────────────────────

    def _feedback_callback(self, feedback_msg):
        # Only relevant in AUTO mode: STEP mode sends one waypoint per goal,
        # so its feedback is always index 0 and is redundant with the
        # "[n/total] Navigating to ..." message already logged per step.
        if self._mode != 'auto':
            return

        idx = feedback_msg.feedback.current_waypoint
        if idx == self._last_feedback_index or idx >= len(self._waypoints):
            return
        self._last_feedback_index = idx

        wp = self._waypoints[idx]
        self.get_logger().info(
            f'Waypoint {idx + 1}/{len(self._waypoints)}: navigating to "{wp["label"]}" '
            f'(lat={wp["lat"]:.6f}, lon={wp["lon"]:.6f})'
        )


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main():

    rclpy.init()
    node = GpsWaypointFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()