"""Command-line entry point for one nail-painting cycle."""

from __future__ import annotations

import argparse
from pathlib import Path

from nailbot.pipeline import run_once


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Automated nail polish robot controller")
    parser.add_argument("--image", type=Path, help="Use a saved image instead of the Pi camera.")
    parser.add_argument("--live", action="store_true", help="Drive real servos instead of dry-run printing.")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run_once(image_path=args.image, dry_run=not args.live)
