"""Capacitive sensor helpers for future Z-height calibration."""

from __future__ import annotations

from config import HardwareConfig


class CapacitiveSensor:
    """Read an MPR121 electrode used as a proximity/contact cue."""

    def __init__(self, hardware: HardwareConfig):
        self.hardware = hardware
        self._mpr121 = None

        if hardware.dry_run:
            return

        try:
            import adafruit_mpr121
            import board
            import busio
        except ImportError as exc:
            raise RuntimeError(
                "Install adafruit-circuitpython-mpr121 on the Raspberry Pi "
                "to read the capacitive sensor."
            ) from exc

        i2c = busio.I2C(board.SCL, board.SDA)
        self._mpr121 = adafruit_mpr121.MPR121(i2c, address=hardware.mpr121_address)

    def touched(self, electrode: int = 0) -> bool:
        if self.hardware.dry_run:
            print(f"[dry-run] MPR121 electrode {electrode}: not touched")
            return False

        assert self._mpr121 is not None
        return bool(self._mpr121[electrode].value)

    def filtered_data(self, electrode: int = 0) -> int:
        if self.hardware.dry_run:
            print(f"[dry-run] MPR121 electrode {electrode}: filtered data unavailable")
            return 0

        assert self._mpr121 is not None
        return int(self._mpr121.filtered_data(electrode))

    def baseline_data(self, electrode: int = 0) -> int:
        if self.hardware.dry_run:
            print(f"[dry-run] MPR121 electrode {electrode}: baseline data unavailable")
            return 0

        assert self._mpr121 is not None
        return int(self._mpr121.baseline_data(electrode))
