"""Servo control through a PCA9685 board.

The controller defaults to dry-run mode. Set HardwareConfig.dry_run=False only
after the arm is secured, the servo directions are calibrated, and power rails
share ground with the Raspberry Pi.
"""

from __future__ import annotations

import time
from contextlib import AbstractContextManager

from nailbot.config import HardwareConfig, PaintConfig, ServoCalibration


class RobotController(AbstractContextManager):
    def __init__(self, hardware: HardwareConfig, paint: PaintConfig):
        self.hardware = hardware
        self.paint = paint
        self._kit = None

        if not hardware.dry_run:
            try:
                from adafruit_servokit import ServoKit
            except ImportError as exc:
                raise RuntimeError(
                    "Install adafruit-circuitpython-servokit on the Raspberry Pi "
                    "to control the PCA9685."
                ) from exc

            self._kit = ServoKit(channels=16, address=hardware.i2c_address)
            for servo in hardware.servos.values():
                channel = self._kit.servo[servo.channel]
                channel.set_pulse_width_range(servo.pulse_min_us, servo.pulse_max_us)

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.home()

    def home(self) -> None:
        self.move_servos({name: servo.home_angle for name, servo in self.hardware.servos.items()})

    def move_servos(self, angles: dict[str, float], delay_s: float | None = None) -> None:
        for name, logical_angle in angles.items():
            if name not in self.hardware.servos:
                raise KeyError(f"Unknown servo '{name}'. Expected one of {list(self.hardware.servos)}.")

            calibration = self.hardware.servos[name]
            servo_angle = calibration.to_servo_angle(logical_angle)
            self._write_servo(calibration, servo_angle)

        time.sleep(self.paint.move_delay_s if delay_s is None else delay_s)

    def move_pose_angles(self, base: float, shoulder: float, elbow: float, wrist: float) -> None:
        self.move_servos(
            {
                "base": base,
                "shoulder": shoulder,
                "elbow": elbow,
                "wrist": wrist,
            }
        )

    def dispense(self, enabled: bool = True) -> None:
        angle = self.paint.paint_feed_angle if enabled else self.paint.paint_idle_angle
        self.move_servos({"dispenser": angle}, delay_s=self.paint.dispense_pulse_s)
        if enabled:
            self.move_servos({"dispenser": self.paint.paint_idle_angle}, delay_s=self.paint.dispense_pulse_s)

    def _write_servo(self, calibration: ServoCalibration, angle: float) -> None:
        if self.hardware.dry_run:
            print(f"[dry-run] channel {calibration.channel}: {angle:.1f} deg")
            return

        assert self._kit is not None
        self._kit.servo[calibration.channel].angle = angle
