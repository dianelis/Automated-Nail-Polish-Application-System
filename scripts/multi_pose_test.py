"""Run the arm through a sequence of test poses using inverse kinematics.

Examples:
    python -m scripts.multi_pose_test
    python -m scripts.multi_pose_test --live
    python -m scripts.multi_pose_test --delay 2.0 --live
"""

from __future__ import annotations

import argparse
import time
from dataclasses import replace

from nailbot.config import CONFIG
from nailbot.control import RobotController
from nailbot.kinematics import KinematicsError, Pose, inverse_kinematics


DEFAULT_POSES = [
    ("x10-y0-z115", Pose(x_mm=10.0, y_mm=0.0, z_mm=115.0, tool_angle_deg=10.0)),
    ("x30-y20-z115", Pose(x_mm=30.0, y_mm=20.0, z_mm=115.0, tool_angle_deg=10.0)),
    ("x30-y0-z115", Pose(x_mm=30.0, y_mm=0.0, z_mm=115.0, tool_angle_deg=10.0)),
    ("x40-y35-z110", Pose(x_mm=40.0, y_mm=35.0, z_mm=110.0, tool_angle_deg=10.0)),
    ("x55-y55-z105", Pose(x_mm=20.0, y_mm=55.0, z_mm=105.0, tool_angle_deg=10.0)),
    ("retract-up", Pose(x_mm=0.0, y_mm=0.0, z_mm=250.0, tool_angle_deg=180.0)),
]


def build_hardware(dry_run: bool):
    return replace(CONFIG.hardware, dry_run=dry_run)


def run_pose_sequence(dry_run: bool, elbow_up: bool, delay_s: float) -> None:
    hardware = build_hardware(dry_run)

    with RobotController(hardware=hardware, paint=CONFIG.paint) as robot:
        for name, pose in DEFAULT_POSES:
            print(f"\nMoving to pose '{name}': {pose}")
            try:
                angles = inverse_kinematics(pose, CONFIG.robot, elbow_up=elbow_up)
            except KinematicsError as exc:
                print(f"Skipping '{name}': {exc}")
                continue

            print(f"Calculated joint angles: {angles}")
            robot.move_servos(angles.as_servo_dict(), delay_s=1.0)
            time.sleep(delay_s)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a fixed sequence of IK test poses.")
    parser.add_argument(
        "--delay",
        type=float,
        default=0.2,
        help="Extra delay in seconds to wait after each pose.",
    )
    parser.add_argument(
        "--elbow-down",
        action="store_true",
        help="Use the elbow-down IK branch instead of the default elbow-up branch.",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Actually drive the PCA9685. Without this flag the script stays in dry-run mode.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_pose_sequence(
        dry_run=not args.live,
        elbow_up=not args.elbow_down,
        delay_s=args.delay,
    )


if __name__ == "__main__":
    main()
