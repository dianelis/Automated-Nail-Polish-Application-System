"""Camera capture and nail segmentation.

The UNet blocks mirror the Colab notebook structure so a trained state_dict can
be loaded on the Raspberry Pi. If no model is available, HSV thresholding can be
used for early mechanical testing with a high-contrast nail/background setup.
"""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np


class TwoConvLayers:
    pass


try:
    import torch
    import torch.nn as nn
except ImportError:
    torch = None
    nn = None


if nn is not None:

    class TwoConvLayers(nn.Module):
        def __init__(self, in_channels: int, out_channels: int):
            super().__init__()
            self.model = nn.Sequential(
                nn.Conv2d(in_channels, out_channels, kernel_size=3, padding=1, bias=False),
                nn.ReLU(inplace=True),
                nn.BatchNorm2d(out_channels),
                nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1, bias=False),
                nn.ReLU(inplace=True),
                nn.BatchNorm2d(out_channels),
            )

        def forward(self, x):
            return self.model(x)


    class Encoder(nn.Module):
        def __init__(self, in_channels: int, out_channels: int):
            super().__init__()
            self.block = TwoConvLayers(in_channels, out_channels)
            self.max_pool = nn.MaxPool2d(kernel_size=2, stride=2)

        def forward(self, x):
            skip = self.block(x)
            return self.max_pool(skip), skip


    class Decoder(nn.Module):
        def __init__(self, in_channels: int, out_channels: int):
            super().__init__()
            self.transpose = nn.ConvTranspose2d(in_channels, out_channels, kernel_size=2, stride=2)
            self.block = TwoConvLayers(in_channels, out_channels)

        def forward(self, x, skip):
            x = self.transpose(x)
            x = torch.cat([x, skip], dim=1)
            return self.block(x)


    class UNet(nn.Module):
        def __init__(self, in_channels: int = 3, num_classes: int = 1):
            super().__init__()
            self.enc_block1 = Encoder(in_channels, 32)
            self.enc_block2 = Encoder(32, 64)
            self.enc_block3 = Encoder(64, 128)
            self.enc_block4 = Encoder(128, 256)
            self.bottleneck = TwoConvLayers(256, 512)
            self.dec_block1 = Decoder(512, 256)
            self.dec_block2 = Decoder(256, 128)
            self.dec_block3 = Decoder(128, 64)
            self.dec_block4 = Decoder(64, 32)
            self.output = nn.Conv2d(32, num_classes, kernel_size=1)

        def forward(self, x):
            x, skip1 = self.enc_block1(x)
            x, skip2 = self.enc_block2(x)
            x, skip3 = self.enc_block3(x)
            x, skip4 = self.enc_block4(x)
            x = self.bottleneck(x)
            x = self.dec_block1(x, skip4)
            x = self.dec_block2(x, skip3)
            x = self.dec_block3(x, skip2)
            x = self.dec_block4(x, skip1)
            return self.output(x)


class Camera:
    def __init__(self, camera_index: int = 0):
        self.camera_index = camera_index

    def capture(self) -> np.ndarray:
        cap = cv2.VideoCapture(self.camera_index)
        try:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError(f"Could not capture image from camera index {self.camera_index}.")
            return frame
        finally:
            cap.release()


class NailSegmenter:
    def __init__(
        self,
        model_path: Path,
        image_size: tuple[int, int] = (256, 256),
        threshold: float = 0.5,
        device: str | None = None,
        allow_fallback: bool = True,
    ):
        self.model_path = Path(model_path)
        self.image_size = image_size
        self.threshold = threshold
        self.allow_fallback = allow_fallback
        self.device = device
        self.model = None

        if self.model_path.exists():
            self._load_model()
        elif not allow_fallback:
            raise FileNotFoundError(f"Segmentation model not found: {self.model_path}")

    def predict_mask(self, image_bgr: np.ndarray) -> np.ndarray:
        if self.model is None:
            return self._fallback_mask(image_bgr)
        return self._predict_unet(image_bgr)

    def _load_model(self) -> None:
        if torch is None:
            raise RuntimeError("PyTorch is required to load the nail segmentation model.")

        self.device = self.device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.model = UNet().to(self.device)
        checkpoint = torch.load(self.model_path, map_location=self.device)
        if isinstance(checkpoint, dict):
            state_dict = checkpoint.get("model_state_dict", checkpoint)
            if "threshold" in checkpoint:
                self.threshold = float(checkpoint["threshold"])
            if "image_size" in checkpoint:
                self.image_size = tuple(checkpoint["image_size"])
        else:
            state_dict = checkpoint
            self._load_sidecar_metadata()
        self.model.load_state_dict(state_dict)
        self.model.eval()

    def _load_sidecar_metadata(self) -> None:
        metadata_path = self.model_path.with_name(f"{self.model_path.stem}_metadata.json")
        if not metadata_path.exists():
            return

        with metadata_path.open() as metadata_file:
            metadata = json.load(metadata_file)
        if "threshold" in metadata:
            self.threshold = float(metadata["threshold"])
        if "image_size" in metadata:
            self.image_size = tuple(metadata["image_size"])

    def _predict_unet(self, image_bgr: np.ndarray) -> np.ndarray:
        assert torch is not None
        original_h, original_w = image_bgr.shape[:2]
        resized = cv2.resize(image_bgr, self.image_size)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        rgb = (rgb - np.array([0.485, 0.456, 0.406])) / np.array([0.229, 0.224, 0.225])
        tensor = torch.from_numpy(rgb.astype(np.float32)).permute(2, 0, 1).unsqueeze(0).to(self.device)

        with torch.no_grad():
            logits = self.model(tensor)
            probs = torch.sigmoid(logits).squeeze().cpu().numpy()

        mask = (probs >= self.threshold).astype(np.uint8) * 255
        return cv2.resize(mask, (original_w, original_h), interpolation=cv2.INTER_NEAREST)

    def _fallback_mask(self, image_bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        mask = np.where((saturation > 25) & (value > 50), 255, 0).astype(np.uint8)
        return mask
