"""Mask-to-stroke planning for painting inside the nail segmentation."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from nailbot.config import WorkspaceCalibration
from nailbot.kinematics import Pose


@dataclass(frozen=True)
class PathPoint:
    x_px: int
    y_px: int
    paint: bool = True


def clean_mask(mask: np.ndarray, min_area_px: int) -> np.ndarray:
    """Return the largest clean binary nail mask."""

    binary = (mask > 0).astype(np.uint8) * 255
    kernel = np.ones((5, 5), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if count <= 1:
        return np.zeros_like(binary)

    areas = stats[1:, cv2.CC_STAT_AREA]
    largest_index = int(np.argmax(areas)) + 1
    if stats[largest_index, cv2.CC_STAT_AREA] < min_area_px:
        return np.zeros_like(binary)

    return np.where(labels == largest_index, 255, 0).astype(np.uint8)


def raster_path(mask: np.ndarray, spacing_px: int = 8, max_points: int | None = None) -> list[PathPoint]:
    """Generate a boustrophedon raster path inside a binary mask."""

    binary = mask > 0
    ys, xs = np.where(binary)
    if len(xs) == 0:
        return []

    y_min, y_max = int(ys.min()), int(ys.max())
    path: list[PathPoint] = []
    left_to_right = True

    for y in range(y_min, y_max + 1, spacing_px):
        row_xs = np.where(binary[y])[0]
        if len(row_xs) == 0:
            continue

        segments = _row_segments(row_xs)
        if not left_to_right:
            segments.reverse()

        for start_x, end_x in segments:
            x_values = range(start_x, end_x + 1, spacing_px)
            if not left_to_right:
                x_values = range(end_x, start_x - 1, -spacing_px)
            path.extend(PathPoint(int(x), int(y), paint=True) for x in x_values)

        left_to_right = not left_to_right

    if max_points and len(path) > max_points:
        indexes = np.linspace(0, len(path) - 1, max_points).astype(int)
        path = [path[i] for i in indexes]

    return path


def _row_segments(row_xs: np.ndarray) -> list[tuple[int, int]]:
    breaks = np.where(np.diff(row_xs) > 1)[0]
    starts = np.insert(row_xs[breaks + 1], 0, row_xs[0])
    ends = np.append(row_xs[breaks], row_xs[-1])
    return [(int(start), int(end)) for start, end in zip(starts, ends)]


def pixel_to_workspace(point: PathPoint, calibration: WorkspaceCalibration) -> Pose:
    origin_x, origin_y = calibration.origin_px
    x_mm = (point.x_px - origin_x) * calibration.x_mm_per_px
    y_mm = (origin_y - point.y_px) * calibration.y_mm_per_px
    return Pose(x_mm=x_mm, y_mm=y_mm, z_mm=calibration.z_paint_mm)


def path_to_workspace(path: list[PathPoint], calibration: WorkspaceCalibration) -> list[Pose]:
    return [pixel_to_workspace(point, calibration) for point in path]
