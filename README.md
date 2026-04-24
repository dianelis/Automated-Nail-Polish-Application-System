# Automated Nail Polish Application System

This repository is a first software scaffold for a Raspberry Pi robot arm that applies nail polish.

The planned loop is:

```python
image = capture()
mask = get_mask(image)
path = raster_path(mask)

for point in path:
    angles = inverse_kinematics(point)
    move_servos(angles)
    dispense(flow_rate)
```

## What The Robot Does

The current mechanical plan uses five MG996R servos:

| Motor | Purpose |
| --- | --- |
| Motor 1 | Rotates the whole arm/base link |
| Motor 2 | Moves link 1 |
| Motor 3 | Moves link 2 |
| Motor 4 | Rotates the syringe/end effector |
| Motor 5 | Drives the cam that pushes polish through the syringe |

The Raspberry Pi captures an image, segments the nail, converts the nail mask into raster paint strokes, solves inverse kinematics for each point, then sends servo PWM commands through the PCA9685 board.

## Files

| File | Purpose |
| --- | --- |
| `main.py` | Runs one capture, segmentation, path planning, IK, and control cycle |
| `vision.py` | Camera capture plus U-Net inference with an HSV fallback mask for early testing |
| `path_planning.py` | Cleans masks and creates raster stroke paths |
| `kinematics.py` | First-pass inverse kinematics for base, shoulder, elbow, and wrist |
| `control.py` | PCA9685/ServoKit robot control with dry-run mode |
| `servo_test.py` | Simple Raspberry Pi bring-up script for individual servo and pose testing |
| `sensors.py` | MPR121 capacitive sensor wrapper for future Z calibration |
| `config.py` | Servo channels, arm geometry, workspace calibration, and paint settings |
| `train_unet.py` | Minimal model export skeleton for saving a PyTorch U-Net state dict |

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

## Run Safely First

Dry-run mode prints servo commands without moving hardware:

```bash
python main.py --image test_nail.jpg
```

To drive the real robot after calibration:

```bash
python main.py --live
```

Keep the arm powered off or lifted away from a hand until the printed dry-run angles look reasonable.

## Servo Bring-Up On Raspberry Pi

First, confirm the Pi can see the PCA9685:

```bash
sudo apt install -y i2c-tools
i2cdetect -y 1
```

You should usually see the servo driver at `0x40`.

List the configured servo names:

```bash
python servo_test.py --list
```

Move one servo in dry-run mode first:

```bash
python servo_test.py --servo base --angle 110
```

Then move the real servo:

```bash
python servo_test.py --servo base --angle 110 --live
```

Sweep one servo gently:

```bash
python servo_test.py --servo shoulder --sweep 70 110 --live
```

Move all servos to their home angles:

```bash
python servo_test.py --home --live
```

Move the arm to a simple test pose using inverse kinematics:

```bash
python servo_test.py --pose 120 0 80 --tool-angle -90 --live
```

For first hardware tests, detach the syringe load, keep the motion range small, and test one servo at a time.

## Vision Model

The uploaded Kaggle notebook defines a PyTorch U-Net for binary nail segmentation. `vision.py` mirrors that model structure so one can train on your laptop, save the weights, copy them to the Raspberry Pi, and run inference. This script referrenced https://www.kaggle.com/code/wadzim/nails-segmentation-from-scratch-unet#Model-Architecture

Expected model path:

```text
models/nail_unet.pt
```

After training in the notebook or a future training script, save:

```python
torch.save(model.state_dict(), "models/nail_unet.pt")
```

Until that file exists, the code uses a simple HSV fallback mask. That fallback is only for bench testing and will not be reliable on real hands with polish-like colors or changing light.

## Calibration Needed

Before live painting, update `config.py` with measured values:

- `RobotGeometry.link1_mm`
- `RobotGeometry.link2_mm`
- `RobotGeometry.tool_offset_mm`
- each servo channel and angle limit
- each servo `invert` flag
- `WorkspaceCalibration.origin_px`
- `WorkspaceCalibration.x_mm_per_px`
- `WorkspaceCalibration.y_mm_per_px`

The capacitive sensor wrapper is in `sensors.py`. A good next experiment is to log `filtered_data()` and `baseline_data()` while lowering the syringe toward a test nail so you can decide whether the sensor should detect first contact, proximity, or a fixed threshold above baseline.

## Electronics Notes

Use a separate high-current servo power rail. The Raspberry Pi should not power the MG996R servos directly.

Minimum wiring concept:

- Battery/buck or BEC powers the servo rail.
- PCA9685 V+ connects to servo power.
- PCA9685 logic VCC connects to Pi 5V.
- Pi GND, PCA9685 GND, and servo power GND must be common.
- Pi I2C SDA/SCL connects to PCA9685 SDA/SCL.
- MPR121 connects over I2C too, usually at a different address.

## Next Steps

1. Measure the physical arm and update `config.py`.
2. Test each servo alone with dry-run off and no syringe attached.
3. Calibrate camera pixels to nail-plane millimeters using a printed grid.
4. Train/export the U-Net model from the Kaggle notebook.
5. Add capacitive Z calibration once the MPR121 readings are characterized.
