# Visual SLAM ROS Package

Phase 4: Visual SLAM with RTAB-Map and Intel RealSense D455

## Overview

This package provides:
1. **RTAB-Map integration** for RGB-D SLAM
2. **Feature detection node** for learning ORB features
3. **Launch files** for easy startup

## Package Structure

```
visual_slam_ros/
├── launch/
│   └── rtabmap_d455.launch.py    # Main RTAB-Map launcher
├── visual_slam_ros/
│   └── feature_detector_node.py  # Educational feature detection
├── config/                        # Parameter files (future)
└── README.md                      # This file
```

## Prerequisites

```bash
# Install dependencies
sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-rtabmap-viz ros-jazzy-realsense2-camera
```

## Building

```bash
cd ~/Desktop/Dev/SLAM_10-25/ros2_ws
colcon build --packages-select visual_slam_ros
source install/setup.bash
```

## Usage

### 1. Start RealSense Camera

```bash
ros2 launch realsense2_camera rs_launch.py \
    enable_depth:=true \
    enable_color:=true \
    align_depth.enable:=true
```

### 2. Start RTAB-Map SLAM

```bash
ros2 launch visual_slam_ros rtabmap_d455.launch.py delete_db_on_start:=true
```

### 3. Run Feature Detector (Optional - Educational)

In a separate terminal:
```bash
ros2 run visual_slam_ros feature_detector
```

View the output:
```bash
ros2 run rqt_image_view rqt_image_view /feature_detector_node/feature_visualization
```

## Topics

### Subscribed:
- `/camera/camera/color/image_raw` - RGB image from D455
- `/camera/camera/aligned_depth_to_color/image_raw` - Depth aligned to RGB
- `/camera/camera/color/camera_info` - Camera calibration

### Published:
- `/rtabmap/map` - Occupancy grid map
- `/rtabmap/mapData` - Full SLAM graph
- `/rtabmap/odom` - Visual odometry
- `/feature_detector_node/feature_visualization` - Feature detection viz

## Parameters

### RTAB-Map Key Parameters:
- `Kp/MaxFeatures`: 400 - Maximum ORB features per frame
- `Kp/DetectorStrategy`: 6 (ORB) - Fast binary descriptor
- `Vis/MinInliers`: 15 - Minimum matches for valid pose
- `Mem/STMSize`: 30 - Short-term memory size (keyframes)

### Feature Detector Parameters:
- `num_features`: 400 - Features to detect
- `min_feature_distance`: 7 pixels - Spacing between features
- `show_visualization`: true - Enable viz output

## Learning Exercises

### Exercise 1: Feature Detection
1. Run the feature detector node
2. Point camera at different surfaces:
   - Textured wall (many features)
   - White wall (few features)
   - Corner/edge (strong features)
3. Observe feature count changes

### Exercise 2: SLAM Testing
1. Start RTAB-Map
2. Move camera slowly around room
3. Watch for:
   - Green matches (good tracking)
   - Loop closure detection
   - 3D point cloud building

### Exercise 3: Database Analysis
```bash
# After SLAM session, view database
rtabmap-databaseViewer ~/.ros/rtabmap_d455.db
```

## Troubleshooting

### Camera not detected
```bash
lsusb | grep Intel  # Should show RealSense
```

### Low feature count
- Improve lighting
- Add textured objects to scene
- Check camera is in focus

### SLAM drift
- Move slower
- Ensure good feature distribution
- Look for loop closure (return to start)

## Connection to Phase 3

This visual SLAM uses the same concepts as your EKF-SLAM:
- **Features** = Landmarks
- **Feature matching** = Data association
- **Visual odometry** = Motion model
- **Loop closure** = Re-observation correction

But scales to 1000s of features using graph optimization instead of O(n²) covariance!

## Next Steps

1. Test SLAM in your room
2. Record datasets (easy/medium/hard scenarios)
3. Compare to Phase 2 Hector SLAM
4. Plan RFID sensor fusion (Phase 5)
