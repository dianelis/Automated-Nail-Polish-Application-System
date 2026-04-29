# Automated Nail Polish Application System

Software architecture for a Raspberry Pi controlled robot arm that segments a fingernail, converts the nail mask into a paint-safe raster path, solves inverse kinematics, and drives servos plus a syringe-style dispenser.

The system is organized around the live automation loop:

```text
camera image
  -> nail segmentation
  -> mask cleanup
  -> raster path planning
  -> pixel-to-workspace calibration
  -> inverse kinematics
  -> servo + dispenser commands
```

The current implementation is intentionally safety-first. It defaults to dry-run mode, prints servo commands unless `--live` is passed, and keeps calibration values centralized.

## Repository Layout

```text
.
├── nailbot/                       # Importable robot software package
│   ├── config.py                  # Robot geometry, servo calibration, workspace, paint, vision config
│   ├── pipeline.py                # End-to-end capture -> segment -> plan -> actuate cycle
│   ├── vision.py                  # Camera wrapper, PyTorch U-Net, HSV fallback segmentation
│   ├── path_planning.py           # Mask cleanup, raster stroke planning, pixel -> workspace conversion
│   ├── kinematics.py              # Pose model, inverse kinematics, FK preview helper
│   ├── control.py                 # PCA9685 / ServoKit robot controller with dry-run behavior
│   └── sensors.py                 # MPR121 capacitive sensor helper for future Z calibration
│
├── scripts/                       # Command-line entry points
│   ├── run_cycle.py               # One full nail-painting cycle
│   ├── servo_test.py              # Individual servo bring-up and raw angle tests
│   ├── pose_test.py               # Interactive or one-shot cartesian pose test
│   ├── multi_pose_test.py         # Fixed IK pose sequence
│   ├── servo_path_test.py         # Preset servo-only motion path
│   ├── dispense_test.py           # Dispenser cam pulse test
│   └── write_hi.py                # Presentation demo path that writes "Hi"
│
├── training/                      # Lightweight model export helpers
│   └── export_unet.py             # Exports a Pi-loadable U-Net state_dict for integration testing
│
├── notebooks/                     # GPU notebook workflows
│   ├── train_nail_unet_colab.ipynb
│   └── train_nail_unet_kaggle.ipynb
│
├── models/                        # Drop trained model weights here
│   └── .gitkeep                   # Keeps directory tracked; .pt/.pth model files are ignored
│
├── requirements.txt
└── README.md
```

## Architecture Overview

### 1. Configuration Layer

[nailbot/config.py](nailbot/config.py) is the single source of truth for robot constants.

It contains:

- servo channels, angle limits, home angles, inversion flags, and pulse widths
- arm geometry in millimeters
- camera/model settings
- workspace pixel-to-millimeter calibration
- paint path and dispenser settings
- dry-run hardware mode

Update this file before live hardware tests. Most other modules accept config objects instead of hardcoding physical values.

### 2. Perception Layer

[nailbot/vision.py](nailbot/vision.py) handles:

- camera capture through OpenCV
- loading a PyTorch U-Net model from `models/nail_unet.pt`
- running inference on an image
- falling back to an HSV threshold mask if no trained model exists

The fallback mask is only for bench testing with high-contrast conditions. For real nails, use a trained model.

Expected model path:

```text
models/nail_unet.pt
```

### 3. Geometry and Planning Layer

[nailbot/path_planning.py](nailbot/path_planning.py) converts a segmentation mask into robot-usable geometry:

1. Threshold and clean the mask.
2. Keep the largest connected component.
3. Generate a boustrophedon raster path inside the nail boundary.
4. Convert image pixels into workspace millimeters.

The raster path is deliberately conservative: it stays inside the binary nail region and limits point count through `PaintConfig.max_path_points`.

### 4. Kinematics Layer

[nailbot/kinematics.py](nailbot/kinematics.py) converts cartesian paint-tip poses into logical servo angles.

It models:

- base yaw
- shoulder pitch
- elbow pitch
- wrist/tool angle

The dispenser cam is handled separately by the controller. If a target point is outside the arm workspace, `KinematicsError` is raised and the pipeline skips that point.

### 5. Control Layer

[nailbot/control.py](nailbot/control.py) owns hardware output.

In dry-run mode, it prints:

```text
[dry-run] channel 0: 90.0 deg
```

With `--live`, it uses `adafruit_servokit.ServoKit` to drive the PCA9685 board. The controller also returns the robot home when leaving its context manager.

### 6. Sensor Layer

[nailbot/sensors.py](nailbot/sensors.py) wraps the MPR121 capacitive sensor for future height/contact calibration.

Current planned use:

- record baseline and filtered readings
- detect proximity/contact near the nail surface
- support Z-height calibration before dispensing

### 7. Pipeline Layer

[nailbot/pipeline.py](nailbot/pipeline.py) is the full system loop:

```python
image = capture_or_load_image()
raw_mask = segmenter.predict_mask(image)
mask = clean_mask(raw_mask)
path_px = raster_path(mask)
poses_mm = path_to_workspace(path_px)

for pose in poses_mm:
    angles = inverse_kinematics(pose)
    robot.move_servos(angles)
    robot.dispense()
```

The command-line wrapper is [scripts/run_cycle.py](scripts/run_cycle.py).

## Install

On the Raspberry Pi:

```bash
sudo raspi-config
# Interface Options -> I2C -> Enable

python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

If `opencv-python` is too heavy for the Pi, use the system package instead:

```bash
sudo apt install python3-opencv
```

## Running the System

Run commands from the repository root.

### Full Pipeline, Dry Run

Use a saved image and print the generated servo commands:

```bash
python -m scripts.run_cycle --image test_nail.jpg
```

### Full Pipeline, Live Hardware

Only use this after calibration and dry-run verification:

```bash
python -m scripts.run_cycle --live
```

With no `--image`, the system captures from the configured camera index.

## Hardware Bring-Up

Start with the robot powered off or lifted away from hands and surfaces. Confirm each piece independently before running the full cycle.

### Check I2C

```bash
sudo apt install -y i2c-tools
i2cdetect -y 1
```

The PCA9685 board usually appears at `0x40`.

### List Configured Servos

```bash
python -m scripts.servo_test --list
```

### Dry-Run a Servo Command

```bash
python -m scripts.servo_test --servo base --angle 110
```

### Move One Servo Live

```bash
python -m scripts.servo_test --servo base --angle 110 --live
```

### Sweep One Servo

```bash
python -m scripts.servo_test --servo shoulder --sweep 70 110 --live
```

### Home the Robot

```bash
python -m scripts.servo_test --home --live
```

### Test Inverse Kinematics

```bash
python -m scripts.servo_test --pose 120 0 80 --tool-angle -90 --live
```

Or use the dedicated pose tester:

```bash
python -m scripts.pose_test --interactive --live
python -m scripts.pose_test --x 120 --y 0 --z 80 --tool-angle -90 --live
```

### Run a Fixed Pose Sequence

```bash
python -m scripts.multi_pose_test --live
```

### Test the Dispenser Cam

```bash
python -m scripts.dispense_test --live
python -m scripts.dispense_test --live --offset 2.0
```

### Presentation Demo Path

Trace a simple "Hi" path using inverse kinematics:

```bash
python -m scripts.write_hi --live
```

## Training and Model Export

The runtime code loads a PyTorch U-Net state dict from:

```text
models/nail_unet.pt
```

The notebooks in [notebooks/](notebooks/) train and validate that model:

- `train_nail_unet_colab.ipynb` is configured for Google Colab and Google Drive.
- `train_nail_unet_kaggle.ipynb` is configured for Kaggle datasets.

Both expect a split dataset:

```text
NailSegmentationDatasetV2/
  train/images/
  train/masks/
  val/images/
  val/masks/
  test/images/
  test/masks/
```

After training, export a plain state dict and copy it into the repo:

```text
models/nail_unet.pt
```

For pipeline testing before a real model is trained, generate an untrained placeholder:

```bash
python -m training.export_unet --export-empty --output models/nail_unet.pt
```

That only tests loading and integration. It will not produce useful segmentation.

## Calibration Checklist

Before using live hardware, update [nailbot/config.py](nailbot/config.py):

- `RobotGeometry.link1_mm`
- `RobotGeometry.link2_mm`
- `RobotGeometry.tool_offset_mm`
- each servo channel
- each servo min/max angle
- each servo home angle
- each servo `invert` flag
- `WorkspaceCalibration.origin_px`
- `WorkspaceCalibration.x_mm_per_px`
- `WorkspaceCalibration.y_mm_per_px`
- `WorkspaceCalibration.z_paint_mm`
- `PaintConfig.raster_spacing_px`
- `PaintConfig.paint_feed_angle`
- `PaintConfig.paint_idle_angle`
- `VisionConfig.mask_threshold`

Recommended calibration order:

1. Confirm I2C and PCA9685 detection.
2. Move each servo alone in dry-run, then live.
3. Find safe home angles.
4. Measure arm link lengths and tool offset.
5. Test IK poses without the syringe loaded.
6. Calibrate camera pixels to workspace millimeters using a printed grid.
7. Verify segmentation masks on real lighting/background conditions.
8. Run the full pipeline in dry-run with a saved image.
9. Run live motion with dispenser disabled or empty.
10. Add dispensing only after path and motion are stable.

## Safety Notes

- Use a separate high-current servo power rail.
- Do not power MG996R servos directly from the Raspberry Pi.
- Connect Pi ground, PCA9685 ground, and servo power ground together.
- Keep the arm away from hands during early live tests.
- Use `--live` only after printed dry-run commands look reasonable.
- Detach the syringe load during servo direction and range calibration.
- Start with small movements and conservative angle limits.

Minimum wiring concept:

```text
Battery/BEC/buck -> servo power rail
servo rail V+    -> PCA9685 V+
Pi 5V            -> PCA9685 logic VCC
common GND       -> Pi, PCA9685, servo rail
Pi SDA/SCL       -> PCA9685 SDA/SCL
Pi SDA/SCL       -> MPR121 SDA/SCL
```

## Development Notes

Run module commands from the repository root so Python can resolve the `nailbot` package:

```bash
python -m scripts.run_cycle --image test_nail.jpg
python -m scripts.servo_test --list
python -m training.export_unet --export-empty
```

The package split keeps reusable robot logic out of command-line scripts:

- add control logic to `nailbot/`
- add one-off hardware tests to `scripts/`
- add model training/export utilities to `training/`
- keep GPU experiments in `notebooks/`

## Current Prototype Status

Implemented:

- dry-run safe robot controller
- PCA9685 servo output path
- camera capture wrapper
- PyTorch U-Net runtime loader
- HSV fallback segmentation
- mask cleanup
- raster path generation
- pixel-to-workspace conversion
- inverse kinematics
- hardware bring-up scripts
- Colab/Kaggle training notebooks

Still needs physical-system calibration:

- measured arm geometry
- verified servo directions and angle limits
- camera-to-workspace calibration
- trained `models/nail_unet.pt`
- robust real-world lighting tests
- MPR121 Z-height/contact threshold characterization
- closed-loop flow/thickness feedback integration
