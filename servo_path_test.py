"""Run a preset continuous single-servo motion sequence to trace a path.

This script moves one servo at a time through a fixed series of angles.
It does not prompt for input and is meant to let the arm trace a continuous path
from repeated per-servo motions.

Example:
    python servo_path_test.py --live
"""

from __future__ import annotations

import argparse
from dataclasses import replace

from config import CONFIG
from control import RobotController


def build_live_hardware(dry_run: bool):
    return replace(CONFIG.hardware, dry_run=dry_run)


def run_servo_path(dry_run: bool) -> None:
    sequence = [
        ("base", 110.0),
        ("shoulder", 65.0),
        ("elbow", 80.0),
        ("wrist", 70.0),
        ("base", 90.0),
        ("shoulder", 80.0),
        ("elbow", 100.0),
        ("wrist", 110.0),
        ("base", 130.0),
        ("shoulder", 55.0),
        ("elbow", 120.0),
        ("wrist", 90.0),
        ("base", 100.0),
        ("shoulder", 70.0),
        ("elbow", 95.0),
    ]

    hardware = build_live_hardware(dry_run)
    with RobotController(hardware=hardware, paint=CONFIG.paint) as robot:
        print("Starting preset servo path sequence...")
        robot.home()

        for step_index, (servo_name, angle) in enumerate(sequence, start=1):
            print(f"Step {step_index}/{len(sequence)}: moving {servo_name} to {angle:.1f} degrees")
            robot.move_servos({servo_name: angle}, delay_s=0.7)

        print("Servo path sequence complete. Returning home.")
        robot.home()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a fixed servo motion sequence without interactive input."
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
    run_servo_path(dry_run=dry_run)


if __name__ == "__main__":
    main()
