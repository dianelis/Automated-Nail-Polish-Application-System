"""Configuration for the automated nail polish robot.

All geometry values are placeholders until the physical arm is measured.
Use millimeters for distances and degrees for servo angles.
"""

from dataclasses import dataclass, field
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent


@dataclass(frozen=True)
class ServoCalibration:
    channel: int
    min_angle: float = 0.0
    max_angle: float = 180.0
    home_angle: float = 90.0
    pulse_min_us: int = 500
    pulse_max_us: int = 2500
    invert: bool = False

    def clamp(self, angle: float) -> float:
        return max(self.min_angle, min(self.max_angle, angle))

    def to_servo_angle(self, logical_angle: float) -> float:
        clamped = self.clamp(logical_angle)
        if self.invert:
            return self.max_angle - (clamped - self.min_angle)
        return clamped


@dataclass(frozen=True)
class RobotGeometry:
    base_height_mm: float = 89.0
    link1_mm: float = 110.0
    link2_mm: float = 110.0
    tool_offset_mm: float = 100.0
    work_surface_z_mm: float = 0.0
    safe_z_mm: float = 35.0


@dataclass(frozen=True)
class WorkspaceCalibration:
    """Maps image pixels to the nail plane.

    x_mm_per_px and y_mm_per_px must be calibrated with a ruler or printed grid.
    origin is the image pixel that maps to workspace (0, 0).
    """

    origin_px: tuple[float, float] = (256.0, 256.0)
    x_mm_per_px: float = 0.08
    y_mm_per_px: float = 0.08
    z_paint_mm: float = 3.0


@dataclass(frozen=True)
class VisionConfig:
    model_path: Path = PROJECT_ROOT / "models" / "nail_unet.pt"
    image_size: tuple[int, int] = (512, 512)
    mask_threshold: float = 0.5
    camera_index: int = 0


@dataclass(frozen=True)
class PaintConfig:
    raster_spacing_px: int = 8
    min_component_area_px: int = 500
    move_delay_s: float = 0.03
    paint_feed_angle: float = 115.0
    paint_idle_angle: float = 90.0
    dispense_pulse_s: float = 0.04
    max_path_points: int = 250


@dataclass(frozen=True)
class HardwareConfig:
    i2c_address: int = 0x40
    mpr121_address: int = 0x5A
    pwm_frequency_hz: int = 50
    dry_run: bool = True
    servos: dict[str, ServoCalibration] = field(
        default_factory=lambda: {
            "base": ServoCalibration(channel=0, min_angle=10, max_angle=170, home_angle=90),
            "shoulder": ServoCalibration(channel=2, min_angle=15, max_angle=165, home_angle=90),
            "elbow": ServoCalibration(channel=4, min_angle=15, max_angle=165, home_angle=90, invert=True),
            "wrist": ServoCalibration(channel=5, min_angle=20, max_angle=160, home_angle=90),
            "dispenser": ServoCalibration(channel=6, min_angle=70, max_angle=130, home_angle=90, invert=True),
        }
    )


@dataclass(frozen=True)
class AppConfig:
    robot: RobotGeometry = RobotGeometry()
    workspace: WorkspaceCalibration = WorkspaceCalibration()
    vision: VisionConfig = VisionConfig()
    paint: PaintConfig = PaintConfig()
    hardware: HardwareConfig = HardwareConfig()


CONFIG = AppConfig()
