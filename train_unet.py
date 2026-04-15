"""Minimal training/export skeleton for the nail U-Net.

Use the Kaggle notebook for experiments, then keep this script as the repo
version that saves a Raspberry-Pi-loadable state_dict at models/nail_unet.pt.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch

from vision import UNet


def export_empty_model(output_path: Path) -> None:
    """Create a model file for pipeline testing before real training."""

    output_path.parent.mkdir(parents=True, exist_ok=True)
    model = UNet()
    torch.save(model.state_dict(), output_path)
    print(f"Saved untrained model to {output_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train/export the nail segmentation U-Net")
    parser.add_argument("--export-empty", action="store_true", help="Save an untrained model for integration tests.")
    parser.add_argument("--output", type=Path, default=Path("models/nail_unet.pt"))
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.export_empty:
        export_empty_model(args.output)
    else:
        raise SystemExit(
            "Training loop is intentionally left for dataset-specific setup. "
            "Start from the Kaggle notebook, then save model.state_dict() here."
        )
