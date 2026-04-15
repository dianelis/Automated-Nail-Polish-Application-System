"""Inverse kinematics for the nail-painting arm.

The current arm is modeled as:
- Motor 1: base yaw.
- Motor 2: shoulder pitch.
- Motor 3: elbow pitch.
- Motor 4: wrist/tool angle.
- Motor 5: dispenser cam, handled by control.py.

This model is intentionally simple and calibration-friendly. Measure the real
link lengths and servo zero directions, then update config.py.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, degrees, hypot, radians, sin, sqrt

from config import RobotGeometry


@dataclass(frozen=True)
class Pose:
    x_mm: float
    y_mm: float
    z_mm: float
    tool_angle_deg: float = -90.0


@dataclass(frozen=True)
class JointAngles:
    base: float
    shoulder: float
    elbow: float
    wrist: float

    def as_servo_dict(self) -> dict[str, float]:
        return {
            "base": self.base,
            "shoulder": self.shoulder,
            "elbow": self.elbow,
            "wrist": self.wrist,
        }


class KinematicsError(ValueError):
    """Raised when a target pose is outside the reachable workspace."""


def _clamp_cos(value: float) -> float:
    return max(-1.0, min(1.0, value))


def inverse_kinematics(
    pose: Pose,
    geometry: RobotGeometry,
    elbow_up: bool = True,
) -> JointAngles:
    """Convert a paint-tip pose into logical servo angles.

    The shoulder/elbow calculation solves a 2-link planar arm after removing
    the tool offset in the requested tool direction. Returned angles are logical
    0-180 degree servo commands and still need per-servo calibration in control.py.
    """

    base_deg = degrees(atan2(pose.y_mm, pose.x_mm)) + 90.0
    radial_mm = hypot(pose.x_mm, pose.y_mm)

    tool_rad = radians(pose.tool_angle_deg)
    wrist_r_mm = radial_mm - geometry.tool_offset_mm * cos(tool_rad)
    wrist_z_mm = pose.z_mm - geometry.base_height_mm - geometry.tool_offset_mm * sin(tool_rad)

    link1 = geometry.link1_mm
    link2 = geometry.link2_mm
    reach = hypot(wrist_r_mm, wrist_z_mm)

    if reach > link1 + link2 or reach < abs(link1 - link2):
        raise KinematicsError(
            f"Target {pose} is unreachable. Wrist reach is {reach:.1f} mm "
            f"for links {link1:.1f} mm and {link2:.1f} mm."
        )

    elbow_inner = acos(_clamp_cos((link1**2 + link2**2 - reach**2) / (2.0 * link1 * link2)))
    elbow_joint = 180.0 - degrees(elbow_inner)
    if not elbow_up:
        elbow_joint = -elbow_joint

    shoulder_offset = acos(_clamp_cos((link1**2 + reach**2 - link2**2) / (2.0 * link1 * reach)))
    shoulder_line = degrees(atan2(wrist_z_mm, wrist_r_mm))
    shoulder_joint = shoulder_line + shoulder_offset if elbow_up else shoulder_line - shoulder_offset

    wrist_joint = pose.tool_angle_deg - shoulder_joint - elbow_joint

    return JointAngles(
        base=base_deg,
        shoulder=90.0 - shoulder_joint,
        elbow=90.0 + elbow_joint,
        wrist=90.0 + wrist_joint,
    )


def forward_kinematics_preview(angles: JointAngles, geometry: RobotGeometry) -> Pose:
    """Approximate FK helper for debugging calibration, not closed-loop control."""

    base = radians(angles.base - 90.0)
    shoulder = radians(90.0 - angles.shoulder)
    elbow = radians(angles.elbow - 90.0)
    wrist = radians(angles.wrist - 90.0)
    tool = shoulder + elbow + wrist

    radial = (
        geometry.link1_mm * cos(shoulder)
        + geometry.link2_mm * cos(shoulder + elbow)
        + geometry.tool_offset_mm * cos(tool)
    )
    z = (
        geometry.base_height_mm
        + geometry.link1_mm * sin(shoulder)
        + geometry.link2_mm * sin(shoulder + elbow)
        + geometry.tool_offset_mm * sin(tool)
    )

    return Pose(
        x_mm=radial * cos(base),
        y_mm=radial * sin(base),
        z_mm=z,
        tool_angle_deg=degrees(tool),
    )
