# cv_module

Production-ready ROS 2 (Humble) package providing AprilTag detection on an Intel RealSense **D435** colour stream (RGB-only, 1920×1080@30), **annotated detections image** for rqt, robust **TF** for individual tags **and a single bundle centroid frame**, and RViz visualisation including a **camera frustum** and bundle label.

## System packages

```bash
sudo apt update
sudo apt install -y   ros-humble-realsense2-camera   ros-humble-apriltag-detector   ros-humble-apriltag-draw   ros-humble-image-transport-plugins   ros-humble-rqt-image-view
```

## Python dependencies

```bash
# Install AprilTag 3 Python bindings and OpenCV
pip3 install --user -r requirements.txt
```

> Notes:> • **AprilTag 3** provides better detection performance and accuracy than `apriltag_ros`.> • `apriltag_draw` renders annotated images from detections without custom overlay code.> • RealSense publishes the *factory* RGB intrinsics in `camera_info`; you can optionally override via `config/camera_intrinsics.yaml` (see launch arg).

## Build

```bash
colcon build --packages-select cv_module
source install/setup.bash
```

## Launch (full stack with AprilTag 3)

```bash
ros2 launch cv_module full_system_apriltag3.launch.py   enable_rviz:=true enable_rqt:=true enable_frustum:=true   use_compressed_image_transport:=false   bundle_label:="DLS Receptacle"   lid_tag_size_mm:=14.0   override_intrinsics_from_yaml:=false   intrinsics_yaml:=share/cv_module/config/camera_intrinsics.yaml   tags_yaml:=share/cv_module/config/tags_16h5.yaml
```

## Launch (full stack with original apriltag_ros)

```bash
ros2 launch cv_module full_system.launch.py   enable_rviz:=true enable_rqt:=true enable_frustum:=true   use_compressed_image_transport:=false   bundle_label:="DLS Receptacle"   lid_tag_size_mm:=14.0   override_intrinsics_from_yaml:=false   intrinsics_yaml:=share/cv_module/config/camera_intrinsics.yaml   tags_yaml:=share/cv_module/config/tags_16h5.yaml   apriltag_settings_yaml:=share/cv_module/config/apriltag_settings.yaml
```

## Launch (detector only)

```bash
ros2 launch cv_module detector_only.launch.py   use_compressed_image_transport:=false   override_intrinsics_from_yaml:=false
```

## Launch (offline regression)

Publish images from a folder with a matching `CameraInfo`:

```bash
ros2 launch cv_module offline_test.launch.py   image_folder:=/path/to/images   camera_info_yaml:=/path/to/camera_intrinsics.yaml   apriltag_settings_yaml:=share/cv_module/config/apriltag_settings.yaml   tags_yaml:=share/cv_module/config/tags_16h5.yaml
```

## Camera Calibration

Calibrate your camera using the AprilTag bundle for improved accuracy:

```bash
ros2 launch cv_module calibrate_camera.launch.py
```

Follow the on-screen instructions:
1. Show the AprilTag bundle to the camera
2. Press SPACE to capture images from different angles (collect 25+)
3. Press Q to calibrate
4. Result saved to `camera_intrinsics_calibrated.yaml`

See **`CALIBRATION_QUICKSTART.md`** for a 5-minute guide or **`CALIBRATION_GUIDE.md`** for detailed instructions.

## Parameters & behaviour

- **Image transport**: rqt/RViz can subscribe to `/compressed` while the detector uses `raw`. Toggle with `use_compressed_image_transport`. Installing `image_transport_plugins` enables compressed topics.
- **Intrinsics override**: set `override_intrinsics_from_yaml:=true` to use `camera_info_override` (publishes `camera_info` with values in `config/camera_intrinsics.yaml` and timestamps matching incoming images).
- **Bundle TF**: `bundle_tf_filter` takes per-tag TFs from `apriltag_ros` and **publishes one TF** `dls_bundle_centroid` using the best visible bundle tag (highest `decision_margin`). No stale TF is kept.
- **PoseStamped convenience**: `bundle_tf_filter` republishes `/cv_module/tag_<id>/pose` and `/cv_module/dls_bundle_centroid/pose` for easy consumption.
- **Performance**: tune `config/apriltag_settings.yaml` (e.g., `detector.decimate`) to reach 30 FPS at 1080p on your CPU. Consider pinning CPU affinity for the apriltag node.

## Frames and topics

- Camera frame: `d435_color_optical_frame`.- Apriltag poses: TF on `/tf` + `apriltag_ros/detections`.- Annotated image: `/apriltag/annotated` (from `apriltag_draw`).- Bundle centroid TF: `/tf` frame `dls_bundle_centroid` (published by `bundle_tf_filter`).

## Tag layout (16h5)

- Cuvette: ID=10, **14 mm**.- DLS *bundle*: IDs **0–7** in a 3×3 grid with the centre missing; **14 mm** per tag, **4 mm** edge spacing. Bundle origin at grid centre.- Lid: ID=8, configurable (`lid_tag_size_mm`).

See `config/tags_16h5.yaml` for exact sizes and transforms.

