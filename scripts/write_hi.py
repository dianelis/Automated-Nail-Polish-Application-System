"""Write 'Hi' on the work surface using the current inverse kinematics model.

This is a presentation-friendly demo path that traces block letters with simple
line segments. The script lifts between strokes and retracts upward at the end.

Examples:
    python -m scripts.write_hi
    python -m scripts.write_hi --live
    python -m scripts.write_hi --origin-x 20 --origin-y 10 --surface-z 100 --live
"""

from __future__ import annotations

import argparse
import time
from dataclasses import replace

from nailbot.config import CONFIG
from nailbot.control import RobotController
from nailbot.kinematics import KinematicsError, Pose, inverse_kinematics


def build_hardware(dry_run: bool):
    return replace(CONFIG.hardware, dry_run=dry_run)


def scale_point(origin_x: float, origin_y: float, width: float, height: float, px: float, py: float) -> tuple[float, float]:
    return origin_x + width * px, origin_y + height * py


def build_hi_strokes(origin_x: float, origin_y: float, width: float, height: float) -> list[list[tuple[float, float]]]:
    """Create simple block-letter strokes for 'Hi' in normalized 2D coordinates."""

    return [
        [
            scale_point(origin_x, origin_y, width, height, 0.00, 0.00),
            scale_point(origin_x, origin_y, width, height, 0.00, 1.00),
        ],
        [
            scale_point(origin_x, origin_y, width, height, 0.35, 0.00),
            scale_point(origin_x, origin_y, width, height, 0.35, 1.00),
        ],
        [
            scale_point(origin_x, origin_y, width, height, 0.00, 0.50),
            scale_point(origin_x, origin_y, width, height, 0.35, 0.50),
        ],
        [
            scale_point(origin_x, origin_y, width, height, 0.60, 0.00),
            scale_point(origin_x, origin_y, width, height, 0.60, 1.00),
        ],
        [
            scale_point(origin_x, origin_y, width, height, 0.78, 0.00),
            scale_point(origin_x, origin_y, width, height, 0.78, 0.70),
        ],
        [
            scale_point(origin_x, origin_y, width, height, 0.78, 0.85),
            scale_point(origin_x, origin_y, width, height, 0.78, 1.00),
        ],
    ]


def interpolate_segment(start: tuple[float, float], end: tuple[float, float], step_mm: float) -> list[tuple[float, float]]:
    x0, y0 = start
    x1, y1 = end
    dx = x1 - x0
    dy = y1 - y0
    distance = max((dx * dx + dy * dy) ** 0.5, 1e-6)
    steps = max(2, int(distance / step_mm) + 1)
    return [
        (x0 + dx * i / (steps - 1), y0 + dy * i / (steps - 1))
        for i in range(steps)
    ]


def move_pose(robot: RobotController, pose: Pose, elbow_up: bool, move_delay_s: float) -> None:
    angles = inverse_kinematics(pose, CONFIG.robot, elbow_up=elbow_up)
    print(f"Pose: {pose}")
    print(f"Angles: {angles}")
    robot.move_servos(angles.as_servo_dict(), delay_s=move_delay_s)


def write_hi(
    origin_x: float,
    origin_y: float,
    surface_z: float,
    hover_z: float,
    retract_z: float,
    width: float,
    height: float,
    step_mm: float,
    tool_angle_deg: float,
    dry_run: bool,
    elbow_up: bool,
    move_delay_s: float,
) -> None:
    strokes = build_hi_strokes(origin_x, origin_y, width, height)
    hardware = build_hardware(dry_run)

    with RobotController(hardware=hardware, paint=CONFIG.paint) as robot:
        for index, stroke in enumerate(strokes, start=1):
            print(f"\nStroke {index}/{len(strokes)}")
            start_x, start_y = stroke[0]
            move_pose(
                robot,
                Pose(start_x, start_y, hover_z, tool_angle_deg),
                elbow_up=elbow_up,
                move_delay_s=move_delay_s,
            )
            move_pose(
                robot,
                Pose(start_x, start_y, surface_z, tool_angle_deg),
                elbow_up=elbow_up,
                move_delay_s=move_delay_s,
            )

            for segment_start, segment_end in zip(stroke, stroke[1:]):
                for x_mm, y_mm in interpolate_segment(segment_start, segment_end, step_mm=step_mm)[1:]:
                    move_pose(
                        robot,
                        Pose(x_mm, y_mm, surface_z, tool_angle_deg),
                        elbow_up=elbow_up,
                        move_delay_s=move_delay_s,
                    )

            end_x, end_y = stroke[-1]
            move_pose(
                robot,
                Pose(end_x, end_y, hover_z, tool_angle_deg),
                elbow_up=elbow_up,
                move_delay_s=move_delay_s,
            )
            time.sleep(0.1)

        move_pose(
            robot,
            Pose(origin_x, origin_y + 10.0, retract_z, tool_angle_deg),
            elbow_up=elbow_up,
            move_delay_s=move_delay_s,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Write "Hi" using IK-controlled robot motion.')
    parser.add_argument("--origin-x", type=float, default=20.0, help="Letter origin x position in mm.")
    parser.add_argument("--origin-y", type=float, default=10.0, help="Letter origin y position in mm.")
    parser.add_argument("--surface-z", type=float, default=100.0, help="Writing height in mm.")
    parser.add_argument("--hover-z", type=float, default=115.0, help="Pen-up hover height in mm.")
    parser.add_argument("--retract-z", type=float, default=180.0, help="Final retract height in mm.")
    parser.add_argument("--width", type=float, default=80.0, help="Overall word width in mm.")
    parser.add_argument("--height", type=float, default=40.0, help="Letter height in mm.")
    parser.add_argument("--step-mm", type=float, default=6.0, help="Interpolation spacing along each stroke.")
    parser.add_argument("--tool-angle", type=float, default=10.0, help="Tool angle in degrees.")
    parser.add_argument("--move-delay", type=float, default=0.15, help="Delay after each IK move.")
    parser.add_argument("--elbow-down", action="store_true", help="Use elbow-down IK branch.")
    parser.add_argument("--live", action="store_true", help="Actually drive the arm instead of dry-run.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        write_hi(
            origin_x=args.origin_x,
            origin_y=args.origin_y,
            surface_z=args.surface_z,
            hover_z=args.hover_z,
            retract_z=args.retract_z,
            width=args.width,
            height=args.height,
            step_mm=args.step_mm,
            tool_angle_deg=args.tool_angle,
            dry_run=not args.live,
            elbow_up=not args.elbow_down,
            move_delay_s=args.move_delay,
        )
    except KinematicsError as exc:
        raise SystemExit(f"IK failed while writing path: {exc}") from exc


if __name__ == "__main__":
    main()
