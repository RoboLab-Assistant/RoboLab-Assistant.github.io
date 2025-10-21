# Camera Calibration Guide

This guide explains how to calibrate your Intel RealSense D435 camera using your AprilTag bundle for improved pose estimation accuracy.

## Why Calibrate?

Factory camera intrinsics from Intel may not be perfectly accurate due to:
- Manufacturing tolerances
- Lens variations
- Environmental factors
- Specific mounting configurations

Calibrating with your **actual AprilTag bundle** provides:
- **Accurate intrinsics** specific to your camera unit
- **Better pose estimation** for your exact tag sizes and geometry
- **Lower reprojection errors** (typically < 0.5 pixels)

## Prerequisites

1. **Build the package** with the calibrator:
   ```bash
   cd ~/your_ros2_workspace
   colcon build --packages-select cv_module
   source install/setup.bash
   ```

2. **Print your AprilTag bundle** (if not already done):
   - Use the `tag16h5` family
   - Print tags with **exactly 14mm** edge size (black square)
   - Arrange in the 3×3 grid as defined in `config/tags_16h5.yaml`
   - Ensure accurate spacing (4mm between tag edges = 18mm center-to-center)
   - Mount on a **rigid, flat surface** (cardboard, acrylic, etc.)

3. **Verify bundle configuration**:
   - Check that `config/tags_16h5.yaml` matches your physical bundle
   - Confirm tag IDs: 0-7 in the correct positions
   - Confirm tag size: 14mm

## Calibration Process

### Step 1: Launch the Calibrator

```bash
ros2 launch cv_module calibrate_camera.launch.py
```

**Optional parameters:**
```bash
ros2 launch cv_module calibrate_camera.launch.py \
  tags_yaml:=/path/to/your/tags_16h5.yaml \
  output_yaml:=my_camera_calibration.yaml \
  num_images:=30 \
  tag_size:=0.014
```

You should see:
- Camera feed window titled "Camera Calibration"
- Green boxes around detected tags
- Status showing "Images: 0/25 | Tags: X"

### Step 2: Collect Calibration Images

**Important tips for good calibration:**

1. **Show the entire bundle** - all 8 tags should be visible
   - At least 4 tags must be visible per image (minimum)
   - More tags = better accuracy

2. **Vary the position** - move the bundle to different locations:
   - Left, right, top, bottom of frame
   - Center and corners
   - Cover the full field of view

3. **Vary the angle** - tilt and rotate the bundle:
   - Facing camera straight-on
   - Tilted 20-30° in different directions
   - Rotated around the camera axis

4. **Vary the distance**:
   - Close (~10cm from camera)
   - Medium (~20cm)
   - Far (~40cm)
   - Cover your typical working range

5. **Press SPACE** when you see:
   - ✓ At least 4 tags detected
   - ✓ Bundle clearly visible
   - ✓ Different pose from previous captures
   - You'll see "CAPTURED!" flash on screen

6. **Aim for 25-30 images** with good variety

### Step 3: Complete Calibration

When you have enough images (minimum 10, recommended 25-30):

1. Press **'q'** to quit and start calibration
2. Wait for processing (typically 2-10 seconds)
3. Check the terminal output for results:

```
======================================================================
Starting calibration...
Using 25 images
✓ Calibration successful! RMS reprojection error: 0.3521 pixels
Mean reprojection error: 0.3489 pixels

Camera Matrix (K):
  fx = 1385.42 pixels
  fy = 1384.98 pixels
  cx = 965.23 pixels
  cy = 551.78 pixels

Distortion Coefficients:
  k1 = 0.053421
  k2 = -0.148932
  p1 = 0.000154
  p2 = -0.000087
  k3 = 0.089765

✓ Calibration saved to: camera_intrinsics_calibrated.yaml
======================================================================
```

### Step 4: Evaluate Results

**Good calibration indicators:**
- ✓ RMS error < 0.5 pixels (excellent)
- ✓ RMS error < 1.0 pixels (good)
- ⚠ RMS error > 1.5 pixels (collect more/better images)

**If results are poor:**
- Press **'r'** to reset and start over
- Ensure bundle is printed accurately (measure with calipers!)
- Use better lighting (even, no shadows)
- Keep bundle flat and rigid
- Add more images with better variety

## Using Your Calibration

### Option 1: Replace Default Intrinsics

```bash
# Backup original
cp config/camera_intrinsics.yaml config/camera_intrinsics_factory.yaml

# Use new calibration
cp camera_intrinsics_calibrated.yaml config/camera_intrinsics.yaml

# Rebuild to update installed files
colcon build --packages-select cv_module
source install/setup.bash
```

### Option 2: Use Override Parameter

Launch with your calibration file:

```bash
ros2 launch cv_module full_system_apriltag3.launch.py \
  override_intrinsics_from_yaml:=true \
  intrinsics_yaml:=/full/path/to/camera_intrinsics_calibrated.yaml
```

### Option 3: Test Before Committing

Compare detection accuracy:

**Test 1: With factory intrinsics**
```bash
ros2 launch cv_module full_system_apriltag3.launch.py \
  override_intrinsics_from_yaml:=false
# Observe tag pose stability and accuracy in RViz
```

**Test 2: With new calibration**
```bash
ros2 launch cv_module full_system_apriltag3.launch.py \
  override_intrinsics_from_yaml:=true \
  intrinsics_yaml:=/path/to/camera_intrinsics_calibrated.yaml
# Compare stability and accuracy
```

**What to look for:**
- Smoother, more stable TF frames
- Better alignment when measuring known distances
- Lower jitter in bundle position
- More consistent tag detection

## Troubleshooting

### "Not enough points: X (need at least 16)"
- Show more tags (at least 4 tags = 16 corner points)
- Move bundle closer to camera
- Check lighting - tags must be clearly visible

### "Calibration failed!"
- Ensure at least 10 images collected
- Check that bundle YAML path is correct
- Verify tag sizes in YAML match physical tags

### High reprojection error (> 1.5 pixels)
- **Check tag printing accuracy** - use calipers to measure 14.0mm exactly
- **Verify bundle geometry** - measure center-to-center distances (should be 18mm)
- **Improve lighting** - reduce shadows and glare
- **Use more images** - aim for 30+ with better variety
- **Keep bundle flat** - any bending/warping will hurt accuracy

### Tags not detected
- Increase lighting
- Move bundle closer
- Ensure tags are not obscured
- Check that tags are from `tag16h5` family
- Clean camera lens

### CV window doesn't appear
- Check that `show_preview:=true` in launch file
- Ensure X11 forwarding if using SSH
- Run calibrator node directly:
  ```bash
  ros2 run cv_module camera_calibrator \
    --ros-param-file config/calibration_params.yaml
  ```

## Advanced Options

### Collect More Data for Specific Ranges

For close-range work (10-20cm):
```bash
# Collect extra images at your working distance
# Press SPACE multiple times at different angles
```

### Use Different Tag Sizes

If you have different tag sizes:
```bash
ros2 launch cv_module calibrate_camera.launch.py \
  tag_size:=0.020  # 20mm tags
```

### Batch Processing

For offline calibration from recorded images:
1. Record rosbag: `ros2 bag record /camera/color/image_raw`
2. Play back: `ros2 bag play your_bag.db3`
3. Run calibrator and capture images

## Theory

The calibrator uses the **pinhole camera model** with **Brown-Conrady distortion**:

**Intrinsic parameters (4 DOF):**
- `fx`, `fy`: Focal lengths in pixels
- `cx`, `cy`: Principal point (image center)

**Distortion parameters (5 DOF):**
- `k1`, `k2`, `k3`: Radial distortion coefficients
- `p1`, `p2`: Tangential distortion coefficients

By detecting AprilTag corners (2D image points) and matching them to known 3D positions in the bundle, we solve for these 9 parameters using nonlinear optimization to minimize reprojection error.

## Validation

After calibration, validate accuracy:

1. **Measure a known distance**:
   - Place bundle at exactly 200mm from camera
   - Check reported Z position in RViz
   - Should be within ±2mm

2. **Check tag spacing**:
   - View two adjacent tags in RViz
   - Measure distance between their centers
   - Should be 18mm ± 0.5mm

3. **Angular accuracy**:
   - Place bundle parallel to camera
   - Rotation should show ~0° for roll/pitch/yaw
   - Rotate 90° → should show ~90°

## References

- OpenCV Camera Calibration: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- AprilTag Bundle Calibration: https://github.com/ZXW2600/apriltag_bundle_calibrate
- ROS camera_info format: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html

---

**Questions or issues?** Check the terminal output for detailed error messages and consult the troubleshooting section above.

