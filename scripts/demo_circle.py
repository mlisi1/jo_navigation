#!/usr/bin/env python3
"""
Demo circle
-----------
Sends a circle of waypoints to Nav2's FollowWaypoints action (local navigation, `odom` frame).

The robot's pose at start is the circle's centre: the first waypoint is straight ahead on the
circumference, the others follow evenly spaced (counter-clockwise by default), each one oriented
along the tangent so the robot drives the circle instead of turning at every waypoint.

Parameters:
  radius_m    (double, 5.0)   circle radius
  points      (int, 12)       waypoints in total, evenly spaced on the circumference
  clockwise   (bool, false)   direction of travel
  close_loop  (bool, false)   append the first waypoint again at the end (points + 1 goals)
  frame_id    (string, odom)  Nav2's global frame (local navigation)
  base_frame  (string, base_link)

Usage (alias `demo_circle <radius> <points>` in jo-zotac):
  ros2 run jo_navigation demo_circle.py --ros-args -p radius_m:=5.0 -p points:=15
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DemoCircle(Node):

    def __init__(self):
        super().__init__('demo_circle')

        self.declare_parameter('radius_m', 5.0)
        self.declare_parameter('points', 12)
        self.declare_parameter('clockwise', False)
        self.declare_parameter('close_loop', False)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self._radius     = self.get_parameter('radius_m').value
        self._points     = self.get_parameter('points').value
        self._clockwise  = self.get_parameter('clockwise').value
        self._close_loop = self.get_parameter('close_loop').value
        self._frame      = self.get_parameter('frame_id').value
        self._base_frame = self.get_parameter('base_frame').value

        if self._radius <= 0.0:
            raise RuntimeError(f"'radius_m' must be > 0, got {self._radius}.")
        if self._points < 3:
            raise RuntimeError(f"'points' must be >= 3, got {self._points}.")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.get_logger().info(
            f'Mission    : circle r={self._radius:.2f} m, {self._points} waypoints, '
            f'{"clockwise" if self._clockwise else "counter-clockwise"}'
            f'{", closed loop" if self._close_loop else ""} ({self._frame})'
        )

        self.get_logger().info('Waiting for follow_waypoints action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready.')

        self._started = False
        self._waypoints = []
        self._last_feedback_index = None
        self.create_timer(1.0, self._start)

    # ──────────────────────────────────────────────────────────────────────
    # Circle
    # ──────────────────────────────────────────────────────────────────────

    def _robot_pose(self):
        """(x, y, yaw) of base_frame in frame_id, or None while TF is not available yet."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._frame, self._base_frame, Time(), timeout=Duration(seconds=0.5))
        except Exception as e:  # noqa: BLE001 - any TF error: retry at the next tick
            self.get_logger().warn(
                f'Waiting for TF {self._frame} -> {self._base_frame}: {e}',
                throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quaternion(tf.transform.rotation)

    def _build_circle(self, cx: float, cy: float, yaw0: float) -> list:
        sign = -1.0 if self._clockwise else 1.0
        poses = []
        for k in range(self._points):
            a = yaw0 + sign * 2.0 * math.pi * k / self._points   # angle from the centre
            heading = a + sign * math.pi / 2.0                    # tangent, direction of travel
            p = PoseStamped()
            p.header.frame_id = self._frame
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = cx + self._radius * math.cos(a)
            p.pose.position.y = cy + self._radius * math.sin(a)
            p.pose.orientation.z = math.sin(heading / 2.0)
            p.pose.orientation.w = math.cos(heading / 2.0)
            poses.append(p)
        if self._close_loop:
            poses.append(poses[0])
        return poses

    # ──────────────────────────────────────────────────────────────────────
    # Navigation
    # ──────────────────────────────────────────────────────────────────────

    def _start(self):
        if self._started:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        self._started = True

        cx, cy, yaw0 = pose
        self.get_logger().info(
            f'Centre     : x={cx:.2f} y={cy:.2f} yaw={math.degrees(yaw0):.1f} deg (robot pose)')

        self._waypoints = self._build_circle(cx, cy, yaw0)
        for i, p in enumerate(self._waypoints):
            self.get_logger().info(
                f'  wp {i + 1:2d}: x={p.pose.position.x:7.2f} y={p.pose.position.y:7.2f}')

        goal = FollowWaypoints.Goal()
        goal.number_of_loops = 0
        goal.poses = self._waypoints
        self.get_logger().info(f'Sending {len(goal.poses)} waypoint(s)...')
        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server.')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted, navigating...')
        goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        idx = feedback_msg.feedback.current_waypoint
        if idx == self._last_feedback_index or idx >= len(self._waypoints):
            return
        self._last_feedback_index = idx
        p = self._waypoints[idx].pose.position
        self.get_logger().info(
            f'Waypoint {idx + 1}/{len(self._waypoints)}: navigating to (x={p.x:.2f}, y={p.y:.2f})')

    def _result_callback(self, future):
        result = future.result().result
        total = len(self._waypoints)
        missed = {m.index: m for m in result.missed_waypoints}
        for i in sorted(missed):
            self.get_logger().warn(
                f'Waypoint {i + 1}/{total} FAILED (error_code={missed[i].error_code}).')
        failed = len(missed)
        self._log_summary(total - failed, failed, total)
        rclpy.shutdown()

    def _log_summary(self, reached: int, failed: int, total: int):
        bar = '#' * 60
        title = 'MISSION COMPLETE' if failed == 0 else 'MISSION FINISHED WITH FAILURES'
        banner = '\n'.join([
            '', bar, bar, '',
            f'   {title}',
            '',
            f'   WAYPOINTS REACHED : {reached}/{total}',
            f'   WAYPOINTS FAILED  : {failed}/{total}',
            '', bar, bar,
        ])
        if failed == 0:
            self.get_logger().info(banner)
        else:
            self.get_logger().warn(banner)


def main():
    rclpy.init()
    try:
        node = DemoCircle()
    except KeyboardInterrupt:   # Ctrl-C while waiting for the action server
        if rclpy.ok():
            rclpy.shutdown()
        return
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
