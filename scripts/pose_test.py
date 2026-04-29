"""Interactive pose test script for the nail-painting robot.

This script uses the current inverse kinematics model to move the arm to a
desired cartesian pose. It is intended for bench testing and calibration.

Examples:
    python -m scripts.pose_test --x 120 --y 0 --z 80 --tool-angle -90
    python -m scripts.pose_test --x 120 --y 0 --z 80 --tool-angle -90 --live
    python -m scripts.pose_test --interactive --live
"""

from __future__ import annotations

import argparse
from dataclasses import replace

from nailbot.config import CONFIG
from nailbot.control import RobotController
from nailbot.kinematics import KinematicsError, Pose, inverse_kinematics


def build_hardware(dry_run: bool):
    return replace(CONFIG.hardware, dry_run=dry_run)


def move_to_pose(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    tool_angle_deg: float,
    dry_run: bool,
    elbow_up: bool,
) -> None:
    pose = Pose(x_mm=x_mm, y_mm=y_mm, z_mm=z_mm, tool_angle_deg=tool_angle_deg)
    angles = inverse_kinematics(pose, CONFIG.robot, elbow_up=elbow_up)

    print(f"Target pose: {pose}")
    print(f"Calculated joint angles: {angles}")

    with RobotController(hardware=build_hardware(dry_run), paint=CONFIG.paint) as robot:
        robot.move_servos(angles.as_servo_dict(), delay_s=1.0)


def prompt_pose() -> tuple[float, float, float, float]:
    x_mm = float(input("x (mm): ").strip())
    y_mm = float(input("y (mm): ").strip())
    z_mm = float(input("z (mm): ").strip())
    tool_angle_deg = float(input("tool angle (deg, -90 points down): ").strip())
    return x_mm, y_mm, z_mm, tool_angle_deg


def run_interactive(dry_run: bool, elbow_up: bool) -> None:
    print("Interactive pose test. Press Enter on x to quit.")
    while True:
        raw_x = input("x (mm): ").strip()
        if not raw_x:
            print("Exiting pose test.")
            return

        try:
            x_mm = float(raw_x)
            y_mm = float(input("y (mm): ").strip())
            z_mm = float(input("z (mm): ").strip())
            tool_angle_deg = float(input("tool angle (deg, -90 points down): ").strip())
            move_to_pose(x_mm, y_mm, z_mm, tool_angle_deg, dry_run=dry_run, elbow_up=elbow_up)
        except ValueError:
            print("Please enter numeric values.")
        except KinematicsError as exc:
            print(f"IK failed: {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move the arm to a cartesian pose using inverse kinematics.")
    parser.add_argument("--x", type=float, help="Target x position in mm.")
    parser.add_argument("--y", type=float, help="Target y position in mm.")
    parser.add_argument("--z", type=float, help="Target z position in mm.")
    parser.add_argument(
        "--tool-angle",
        type=float,
        default=-90.0,
        help="Tool angle in degrees. Default keeps the syringe pointing down.",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Prompt for poses repeatedly instead of using command line values.",
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
    dry_run = not args.live
    elbow_up = not args.elbow_down

    if args.interactive:
        run_interactive(dry_run=dry_run, elbow_up=elbow_up)
        return

    if args.x is None or args.y is None or args.z is None:
        x_mm, y_mm, z_mm, tool_angle_deg = prompt_pose()
    else:
        x_mm, y_mm, z_mm, tool_angle_deg = args.x, args.y, args.z, args.tool_angle

    try:
        move_to_pose(
            x_mm=x_mm,
            y_mm=y_mm,
            z_mm=z_mm,
            tool_angle_deg=tool_angle_deg,
            dry_run=dry_run,
            elbow_up=elbow_up,
        )
    except KinematicsError as exc:
        raise SystemExit(f"IK failed: {exc}") from exc


if __name__ == "__main__":
    main()
