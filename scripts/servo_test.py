"""Simple bring-up script for PCA9685 servo testing on a Raspberry Pi.

Examples:
    python -m scripts.servo_test --list
    python -m scripts.servo_test --servo base --angle 110 --live
    python -m scripts.servo_test --servo shoulder --sweep 70 120 --live
    python -m scripts.servo_test --angles 90 100 110 90 --live
    python -m scripts.servo_test --pose 120 0 80 --tool-angle -90 --live
    python -m scripts.servo_test --home --live
"""

from __future__ import annotations

import argparse
from dataclasses import replace

from nailbot.config import CONFIG
from nailbot.control import RobotController
from nailbot.kinematics import KinematicsError, Pose, inverse_kinematics


def build_live_hardware(dry_run: bool):
    return replace(CONFIG.hardware, dry_run=dry_run)


def list_servos() -> None:
    print("Configured servos:")
    for name, servo in CONFIG.hardware.servos.items():
        print(
            f"  {name}: channel={servo.channel}, "
            f"range=({servo.min_angle}, {servo.max_angle}), home={servo.home_angle}, invert={servo.invert}"
        )


def move_single_servo(name: str, angle: float, dry_run: bool) -> None:
    hardware = build_live_hardware(dry_run)
    with RobotController(hardware=hardware, paint=CONFIG.paint) as robot:
        print(f"Moving servo '{name}' to {angle:.1f} degrees")
        robot.move_servos({name: angle}, delay_s=0.5)


def sweep_servo(name: str, start: float, end: float, dry_run: bool) -> None:
    hardware = build_live_hardware(dry_run)
    midpoint = (start + end) / 2.0
    with RobotController(hardware=hardware, paint=CONFIG.paint) as robot:
        print(f"Sweeping servo '{name}' from {start:.1f} -> {midpoint:.1f} -> {end:.1f}")
        robot.move_servos({name: start}, delay_s=0.8)
        robot.move_servos({name: midpoint}, delay_s=0.8)
        robot.move_servos({name: end}, delay_s=0.8)


def move_home(dry_run: bool) -> None:
    hardware = build_live_hardware(dry_run)
    with RobotController(hardware=hardware, paint=CONFIG.paint) as robot:
        print("Moving robot to configured home angles")
        robot.home()


def move_raw_angles(
    base: float,
    shoulder: float,
    elbow: float,
    wrist: float,
    dry_run: bool,
) -> None:
    hardware = build_live_hardware(dry_run)
    angles = {
        "base": base,
        "shoulder": shoulder,
        "elbow": elbow,
        "wrist": wrist,
    }
    print(f"Moving to raw servo angles: {angles}")
    with RobotController(hardware=hardware, paint=CONFIG.paint) as robot:
        robot.move_servos(angles, delay_s=1.0)


def move_to_pose(x_mm: float, y_mm: float, z_mm: float, tool_angle_deg: float, dry_run: bool) -> None:
    hardware = build_live_hardware(dry_run)
    pose = Pose(x_mm=x_mm, y_mm=y_mm, z_mm=z_mm, tool_angle_deg=tool_angle_deg)
    angles = inverse_kinematics(pose, CONFIG.robot)
    print(f"Target pose: {pose}")
    print(f"Calculated angles: {angles}")
    with RobotController(hardware=hardware, paint=CONFIG.paint) as robot:
        robot.move_servos(angles.as_servo_dict(), delay_s=1.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Servo bring-up and motion test for the nail-painting robot")
    parser.add_argument("--list", action="store_true", help="List configured servo names and channels.")
    parser.add_argument("--servo", choices=CONFIG.hardware.servos.keys(), help="Move one named servo.")
    parser.add_argument("--angle", type=float, help="Logical servo angle for --servo.")
    parser.add_argument(
        "--sweep",
        nargs=2,
        type=float,
        metavar=("START", "END"),
        help="Sweep a servo through two endpoint angles.",
    )
    parser.add_argument(
        "--angles",
        nargs=4,
        type=float,
        metavar=("BASE", "SHOULDER", "ELBOW", "WRIST"),
        help="Move directly to raw servo angles without inverse kinematics.",
    )
    parser.add_argument(
        "--pose",
        nargs=3,
        type=float,
        metavar=("X_MM", "Y_MM", "Z_MM"),
        help="Move the arm to a cartesian test pose using inverse kinematics.",
    )
    parser.add_argument(
        "--tool-angle",
        type=float,
        default=-90.0,
        help="Tool angle in degrees for --pose. Default keeps the syringe pointing down.",
    )
    parser.add_argument("--home", action="store_true", help="Move all servos to their configured home angles.")
    parser.add_argument(
        "--live",
        action="store_true",
        help="Actually drive the PCA9685. Without this flag the script stays in dry-run mode.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    dry_run = not args.live

    try:
        if args.list:
            list_servos()
        elif args.home:
            move_home(dry_run=dry_run)
        elif args.angles:
            base, shoulder, elbow, wrist = args.angles
            move_raw_angles(base, shoulder, elbow, wrist, dry_run=dry_run)
        elif args.pose:
            x_mm, y_mm, z_mm = args.pose
            move_to_pose(x_mm, y_mm, z_mm, args.tool_angle, dry_run=dry_run)
        elif args.servo and args.sweep:
            start, end = args.sweep
            sweep_servo(args.servo, start, end, dry_run=dry_run)
        elif args.servo and args.angle is not None:
            move_single_servo(args.servo, args.angle, dry_run=dry_run)
        else:
            raise SystemExit(
                "Choose one action: --list, --home, --angles B S E W, --pose X Y Z, "
                "--servo NAME --angle A, or --servo NAME --sweep START END"
            )
    except KinematicsError as exc:
        raise SystemExit(f"IK failed: {exc}") from exc


if __name__ == "__main__":
    main()
