# Camera Calibration - Quick Start

Get your camera calibrated in 5 minutes!

## 1. Build
```bash
colcon build --packages-select cv_module
source install/setup.bash
```

## 2. Launch
```bash
ros2 launch cv_module calibrate_camera.launch.py
```

## 3. Collect Images
- A window will open showing your camera feed
- Show your AprilTag bundle to the camera (at least 4 tags visible)
- **Move the bundle** to different positions, angles, and distances
- Press **SPACE** when you see a good detection (it will flash "CAPTURED!")
- Repeat until you have 25 images (more is better!)

**Tips:**
- ✓ Cover all areas of the image (corners, edges, center)
- ✓ Tilt the bundle at different angles
- ✓ Try close, medium, and far distances
- ✓ Keep the bundle flat and rigid

## 4. Calibrate
- Press **'q'** when you have enough images (minimum 10, recommended 25+)
- Wait a few seconds for processing
- Check terminal for results - look for **"RMS reprojection error"**
  - < 0.5 pixels = excellent!
  - < 1.0 pixels = good
  - \> 1.5 pixels = try again with better images

## 5. Use Your Calibration

The calibration is saved to `camera_intrinsics_calibrated.yaml`

**Option A: Test it first (recommended)**
```bash
ros2 launch cv_module full_system_apriltag3.launch.py \
  override_intrinsics_from_yaml:=true \
  intrinsics_yaml:=/full/path/to/camera_intrinsics_calibrated.yaml
```

**Option B: Replace default**
```bash
cp camera_intrinsics_calibrated.yaml config/camera_intrinsics.yaml
colcon build --packages-select cv_module
source install/setup.bash
```

## Troubleshooting

**Not enough tags?**
- Move bundle closer
- Improve lighting
- Check tags are tag16h5 family

**High error?**
- Verify tags are **exactly 14mm** (measure with calipers)
- Check bundle spacing is accurate (18mm center-to-center)
- Keep bundle flat - any bending hurts accuracy
- Collect more images with better variety

**Need help?** See `CALIBRATION_GUIDE.md` for detailed instructions.

---

**Keyboard Controls:**
- `SPACE` - Capture current frame
- `q` - Quit and calibrate
- `r` - Reset (start over)

