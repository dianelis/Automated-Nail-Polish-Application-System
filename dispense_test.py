"""Simple dispenser test script to dispense nail polish with a tiny angle adjustment.

This script moves the dispenser servo to the feed angle plus a tiny offset,
holds briefly, then returns to idle. It's designed for testing small dispensing
adjustments without full interactive control.

Example:
    python dispense_test.py --live
    python dispense_test.py --live --offset 2.0
"""

from __future__ import annotations

import argparse
import time
from dataclasses import replace

from config import CONFIG
from control import RobotController


def build_live_hardware(dry_run: bool):
    return replace(CONFIG.hardware, dry_run=dry_run)


def test_dispense(dry_run: bool, offset_deg: float) -> None:
    hardware = build_live_hardware(dry_run)
    feed_angle = CONFIG.paint.paint_feed_angle + offset_deg
    idle_angle = CONFIG.paint.paint_idle_angle

    with RobotController(hardware=hardware, paint=CONFIG.paint) as robot:
        print(f"Moving dispenser to idle angle: {idle_angle:.1f} degrees")
        robot.move_servos({"dispenser": idle_angle}, delay_s=0.5)

        print(f"Moving dispenser to feed angle: {feed_angle:.1f} degrees (offset: {offset_deg:.1f})")
        robot.move_servos({"dispenser": feed_angle}, delay_s=CONFIG.paint.dispense_pulse_s)

        print(f"Returning dispenser to idle angle: {idle_angle:.1f} degrees")
        robot.move_servos({"dispenser": idle_angle}, delay_s=0.5)

        print("Dispense test complete.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test dispenser with tiny angle adjustment.")
    parser.add_argument(
        "--offset",
        type=float,
        default=5.0,
        help="Tiny angle offset to add to the feed angle (default: 5.0 degrees).",
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
    test_dispense(dry_run=dry_run, offset_deg=args.offset)


if __name__ == "__main__":
    main()