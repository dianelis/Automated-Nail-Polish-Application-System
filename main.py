"""Run one nail-painting cycle from camera image to servo commands."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from config import CONFIG
from control import RobotController
from kinematics import KinematicsError, inverse_kinematics
from path_planning import clean_mask, path_to_workspace, raster_path
from vision import Camera, NailSegmenter


def run_once(image_path: Path | None = None, dry_run: bool = True) -> None:
    config = CONFIG
    hardware = config.hardware
    if dry_run != hardware.dry_run:
        hardware = type(hardware)(
            i2c_address=hardware.i2c_address,
            mpr121_address=hardware.mpr121_address,
            pwm_frequency_hz=hardware.pwm_frequency_hz,
            dry_run=dry_run,
            servos=hardware.servos,
        )

    image = _load_or_capture_image(image_path)
    segmenter = NailSegmenter(
        model_path=config.vision.model_path,
        image_size=config.vision.image_size,
        threshold=config.vision.mask_threshold,
        allow_fallback=True,
    )
    raw_mask = segmenter.predict_mask(image)
    mask = clean_mask(raw_mask, min_area_px=config.paint.min_component_area_px)
    path = raster_path(
        mask,
        spacing_px=config.paint.raster_spacing_px,
        max_points=config.paint.max_path_points,
    )
    poses = path_to_workspace(path, config.workspace)

    print(f"Generated {len(path)} paint points.")
    if not poses:
        print("No nail mask found. Check lighting, camera focus, or the segmentation model.")
        return

    with RobotController(hardware=hardware, paint=config.paint) as robot:
        robot.home()
        for pose in poses:
            try:
                angles = inverse_kinematics(pose, config.robot)
            except KinematicsError as exc:
                print(f"Skipping unreachable point: {exc}")
                continue

            robot.move_servos(angles.as_servo_dict())
            robot.dispense(enabled=True)


def _load_or_capture_image(image_path: Path | None):
    if image_path is not None:
        image = cv2.imread(str(image_path))
        if image is None:
            raise FileNotFoundError(f"Could not read image: {image_path}")
        return image

    return Camera(CONFIG.vision.camera_index).capture()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Automated nail polish robot controller")
    parser.add_argument("--image", type=Path, help="Use a saved image instead of the Pi camera.")
    parser.add_argument("--live", action="store_true", help="Drive real servos instead of dry-run printing.")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run_once(image_path=args.image, dry_run=not args.live)
