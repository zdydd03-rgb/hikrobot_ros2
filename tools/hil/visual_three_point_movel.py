#!/usr/bin/env python3
"""Move Tool 2 TCP above three vision points using the ROS 2 MoveL action.

The embedded base-frame points belong to vision request
09403b2d-178c-4c68-8d19-f8c2c28356ce and were transformed with the
capture-time base-to-flange pose and hkhand_eye_matrix89.json.

No camera-frame point is transformed again while the robot is moving.
"""

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import List

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from hikrobot_msgs.action import MoveL
from rclpy.action import ActionClient
from rclpy.node import Node


BASE_FRAME = "base_link"
TCP_POSE_TOPIC = "/hikrobot_driver/tcp_pose"
MOVE_L_ACTION = "/hikrobot_driver/move_l"
REQUEST_ID = "09403b2d-178c-4c68-8d19-f8c2c28356ce"


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


# Fixed seam points in base_link, metres. Do not reuse these values if the
# workpiece, camera mount, calibration, or capture-time robot pose changes.
SEAM_POINTS = {
    "NEAR_START": Point3(0.990064192284, 0.145845052304, 0.192985235772),
    "MIDDLE": Point3(0.990755040322, 0.055711070856, 0.193309255767),
    "NEAR_END": Point3(0.991325720847, -0.046329860990, 0.193252441221),
}


def distance(a: Point3, b: Point3) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def interpolate(start: Point3, target: Point3, max_step: float) -> List[Point3]:
    length = distance(start, target)
    if length <= 1.0e-9:
        return []
    count = max(1, math.ceil(length / max_step))
    return [
        Point3(
            start.x + (target.x - start.x) * index / count,
            start.y + (target.y - start.y) * index / count,
            start.z + (target.z - start.z) * index / count,
        )
        for index in range(1, count + 1)
    ]


def normalized_quaternion(q: Quaternion) -> Quaternion:
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("current TCP quaternion contains NaN or Inf")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise ValueError("current TCP quaternion has zero length")
    result = Quaternion()
    result.x = q.x / norm
    result.y = q.y / norm
    result.z = q.z / norm
    result.w = q.w / norm
    return result


class ThreePointMoveL(Node):
    def __init__(self) -> None:
        super().__init__("visual_three_point_movel")
        self.latest_pose = None
        self.pose_sequence = 0
        self.current_goal_handle = None
        self.create_subscription(PoseStamped, TCP_POSE_TOPIC, self._pose_callback, 10)
        self.action_client = ActionClient(self, MoveL, MOVE_L_ACTION)
        self.last_feedback_time = 0.0

    def _pose_callback(self, message: PoseStamped) -> None:
        self.latest_pose = message
        self.pose_sequence += 1

    def wait_for_tcp_pose(self, timeout_sec: float, after_sequence: int = -1) -> PoseStamped:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_pose is not None and self.pose_sequence > after_sequence:
                return self.latest_pose
        raise RuntimeError(f"no fresh {TCP_POSE_TOPIC} message within {timeout_sec:.1f} s")

    def _feedback(self, feedback_message) -> None:
        now = time.monotonic()
        if now - self.last_feedback_time < 1.0:
            return
        self.last_feedback_time = now
        feedback = feedback_message.feedback
        self.get_logger().info(
            "feedback path_id=%d position_error=%.6f m orientation_error=%.6f rad"
            % (
                feedback.path_id,
                feedback.position_error_m,
                feedback.orientation_error_rad,
            )
        )

    def cancel_current_goal(self) -> None:
        if self.current_goal_handle is None:
            return
        try:
            future = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        except Exception as error:  # best-effort cancellation during shutdown
            self.get_logger().error(f"cancel request failed: {error}")

    def move_to(
        self,
        label: str,
        target: Point3,
        orientation: Quaternion,
        speed_percent: int,
        timeout_sec: int,
        pose_verify_tolerance_m: float,
    ) -> Point3:
        goal = MoveL.Goal()
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.header.frame_id = BASE_FRAME
        goal.target_pose.pose.position.x = target.x
        goal.target_pose.pose.position.y = target.y
        goal.target_pose.pose.position.z = target.z
        goal.target_pose.pose.orientation = orientation
        goal.speed_percent = speed_percent
        goal.blend_radius = 0
        goal.timeout.sec = timeout_sec
        goal.timeout.nanosec = 0

        self.get_logger().info(
            f"send {label}: x={target.x:.9f} y={target.y:.9f} z={target.z:.9f}"
        )
        send_future = self.action_client.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done():
            raise RuntimeError(f"{label}: timed out while sending goal")

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"{label}: goal rejected")
        self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec + 10.0)
        if not result_future.done():
            self.cancel_current_goal()
            raise RuntimeError(f"{label}: client wait timed out; cancel requested")

        wrapped_result = result_future.result()
        self.current_goal_handle = None
        if wrapped_result is None:
            raise RuntimeError(f"{label}: action returned no result")
        result = wrapped_result.result
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED or result.result_code != 0:
            raise RuntimeError(
                f"{label}: failed status={wrapped_result.status} result_code={result.result_code} "
                f"sdk_error={result.sdk_error_code} message={result.message!r}"
            )

        previous_sequence = self.pose_sequence
        actual_message = self.wait_for_tcp_pose(3.0, after_sequence=previous_sequence)
        if actual_message.header.frame_id != BASE_FRAME:
            raise RuntimeError(
                f"{label}: TCP pose frame is {actual_message.header.frame_id!r}, "
                f"expected {BASE_FRAME!r}"
            )
        actual = Point3(
            actual_message.pose.position.x,
            actual_message.pose.position.y,
            actual_message.pose.position.z,
        )
        error = distance(actual, target)
        if error > pose_verify_tolerance_m:
            raise RuntimeError(
                f"{label}: post-motion TCP error {error:.6f} m exceeds "
                f"{pose_verify_tolerance_m:.6f} m"
            )
        self.get_logger().info(
            f"PASS {label}: path_id={result.path_id} post_error={error * 1000.0:.3f} mm"
        )
        return actual


def point_above(point: Point3, clearance: float) -> Point3:
    return Point3(point.x, point.y, point.z + clearance)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--execute",
        action="store_true",
        help="actually send MoveL goals; without this flag the script only prints the plan",
    )
    parser.add_argument("--clearance-m", type=float, default=0.050)
    parser.add_argument("--travel-clearance-m", type=float, default=0.100)
    parser.add_argument("--max-step-m", type=float, default=0.040)
    parser.add_argument("--speed-percent", type=int, default=1)
    parser.add_argument("--goal-timeout-sec", type=int, default=60)
    parser.add_argument("--verify-tolerance-m", type=float, default=0.005)
    return parser.parse_args()


def validate_arguments(args: argparse.Namespace) -> None:
    if not 0.0 < args.clearance_m:
        raise ValueError("--clearance-m must be positive")
    if args.travel_clearance_m < args.clearance_m:
        raise ValueError("--travel-clearance-m must be >= --clearance-m")
    if not 0.0 < args.max_step_m <= 0.040:
        raise ValueError("--max-step-m must be in (0, 0.040]")
    if not 1 <= args.speed_percent <= 100:
        raise ValueError("--speed-percent must be in [1, 100]")
    if args.goal_timeout_sec <= 0:
        raise ValueError("--goal-timeout-sec must be positive")
    if args.verify_tolerance_m <= 0.0:
        raise ValueError("--verify-tolerance-m must be positive")


def main() -> int:
    args = parse_arguments()
    try:
        validate_arguments(args)
    except ValueError as error:
        print(f"argument error: {error}", file=sys.stderr)
        return 2

    rclpy.init()
    node = ThreePointMoveL()
    try:
        if not node.action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"action server {MOVE_L_ACTION} is unavailable")

        current_message = node.wait_for_tcp_pose(5.0)
        if current_message.header.frame_id != BASE_FRAME:
            raise RuntimeError(
                f"current TCP frame is {current_message.header.frame_id!r}, expected {BASE_FRAME!r}"
            )
        current = Point3(
            current_message.pose.position.x,
            current_message.pose.position.y,
            current_message.pose.position.z,
        )
        orientation = normalized_quaternion(current_message.pose.orientation)

        start_target = point_above(SEAM_POINTS["NEAR_START"], args.clearance_m)
        middle_target = point_above(SEAM_POINTS["MIDDLE"], args.clearance_m)
        end_target = point_above(SEAM_POINTS["NEAR_END"], args.clearance_m)
        travel_z = max(
            current.z,
            SEAM_POINTS["NEAR_START"].z + args.travel_clearance_m,
            SEAM_POINTS["MIDDLE"].z + args.travel_clearance_m,
            SEAM_POINTS["NEAR_END"].z + args.travel_clearance_m,
        )

        logical_targets = [
            ("LIFT_TO_TRAVEL_HEIGHT", Point3(current.x, current.y, travel_z)),
            (
                "TRAVEL_ABOVE_NEAR_START",
                Point3(start_target.x, start_target.y, travel_z),
            ),
            ("NEAR_START", start_target),
            ("MIDDLE", middle_target),
            ("NEAR_END", end_target),
        ]

        segments = []
        cursor = current
        for leg_name, logical_target in logical_targets:
            leg_segments = interpolate(cursor, logical_target, args.max_step_m)
            for index, segment in enumerate(leg_segments, start=1):
                segments.append((f"{leg_name}_{index}/{len(leg_segments)}", segment))
            cursor = logical_target

        print(f"Vision request: {REQUEST_ID}")
        print(f"Current Tool2 TCP: {current}")
        print(f"Fixed orientation xyzw: {orientation.x:.9f}, {orientation.y:.9f}, "
              f"{orientation.z:.9f}, {orientation.w:.9f}")
        print(f"Travel height: {travel_z:.9f} m")
        print(f"Seam clearance: {args.clearance_m * 1000.0:.1f} mm")
        print(f"Generated MoveL segments: {len(segments)}")
        for index, (label, target) in enumerate(segments, start=1):
            print(
                f"  {index:02d} {label:32s} "
                f"x={target.x:.9f} y={target.y:.9f} z={target.z:.9f}"
            )

        if not args.execute:
            print("DRY RUN ONLY: add --execute to enable motion")
            return 0

        print("\nWARNING: no collision model is used.")
        print("The camera/workpiece/calibration must be unchanged since the captured request.")
        confirmation = input("Type EXECUTE THREE POINTS to start: ").strip()
        if confirmation != "EXECUTE THREE POINTS":
            print("Motion canceled by operator.")
            return 1

        actual = current
        for index, (label, target) in enumerate(segments, start=1):
            planned_step = distance(actual, target)
            if planned_step > 0.050:
                raise RuntimeError(
                    f"segment {index} actual-to-target step {planned_step:.6f} m exceeds 0.050 m"
                )
            actual = node.move_to(
                f"{index:02d}/{len(segments)} {label}",
                target,
                orientation,
                args.speed_percent,
                args.goal_timeout_sec,
                args.verify_tolerance_m,
            )

        print("PASS: Tool2 TCP reached NEAR_START, MIDDLE, and NEAR_END at configured clearance")
        print(f"Final Tool2 TCP: x={actual.x:.9f} y={actual.y:.9f} z={actual.z:.9f}")
        return 0
    except KeyboardInterrupt:
        node.cancel_current_goal()
        print("Interrupted: cancel requested; verify robot and Driver state before continuing.", file=sys.stderr)
        return 130
    except Exception as error:
        node.cancel_current_goal()
        print(f"STOP: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
