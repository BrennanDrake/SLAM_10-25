# hector_slam_custom

Classical 2D SLAM (Hector SLAM style) for ROS 2 Jazzy. Implements Gauss–Newton scan matching over a multi‑resolution occupancy grid with native TF2.

## Contents
- **Core nodes**
  - `src/hector_slam_processor.cpp`: Orchestrates SLAM. Subscribes to `sensor_msgs/LaserScan`, runs scan matching and map updates, publishes map, pose, and TF.
  - `src/scan_matcher.cpp`: Gauss–Newton optimizer. Builds Jacobian/Hessian, computes pose updates and match score.
  - `src/map_manager.cpp`: Multi‑resolution occupancy grid, log‑odds updates, Bresenham ray tracing.
- **Testing utilities**
  - `src/static_scan_publisher.cpp`: Deterministic 360° laser in a rectangular room for repeatable tests.
  - `launch/static_test.launch.py`: Starts static scan → SLAM → TF → monitor pipeline.
  - `launch/test_slam.launch.py`: Runs SLAM with the Phase 1 fake scan publisher (moving robot).
  - `scripts/monitor_slam.py`: Prints map coverage, pose, and scan rate.
  - `scripts/test_slam.sh`: Convenience build + launch script.

## Build
```bash
# Terminal 1
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select hector_slam_custom --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## Quick Tests

### Static environment (recommended)
Deterministic room for convergence‑friendly testing.
```bash
ros2 launch hector_slam_custom static_test.launch.py
```
Expected topics: `/scan`, `/map`, `/slam_pose`, `/tf`, `/tf_static`

### Dynamic fake motion (Phase 1 publisher)
Robot moves while publishing scans; good for end‑to‑end stress tests.
```bash
ros2 launch hector_slam_custom test_slam.launch.py use_rviz:=false
```

### Monitor
```bash
ros2 run hector_slam_custom monitor_slam.py
```
Shows map coverage breakdown and scan throughput every 2s.

## Parameters (selection)
Parameters are set on `hector_slam_custom_node` (see `static_test.launch.py`):
- **Frames**: `map_frame`, `base_frame`, `odom_frame`, `laser_frame`
- **Publishing**: `map_publish_rate` (Hz), `pose_publish_rate` (Hz)
- **Scan matching**: `scan_match_threshold`, `max_scan_match_iterations`
- **Map**: `map_resolution` (m/cell), `map_size` (m), `map_update_distance_threshold` (m), `map_update_angle_threshold` (rad)

## Architecture
1. Laser scan arrives → `HectorSlamProcessor::scanCallback()`
2. First scan seeds the map via `updateMap()` (ensures optimizer has gradient info)
3. Subsequent scans run Gauss–Newton in `ScanMatcher` → corrected pose
4. `MapManager` applies log‑odds ray updates (Bresenham) at 3 resolutions
5. Node publishes:
   - `nav_msgs/OccupancyGrid` on `map`
   - `geometry_msgs/PoseWithCovarianceStamped` on `slam_pose`
   - TF: `map → base_link`

## TF2 on ROS 2 Jazzy (important)
Jazzy ships TF2 headers in a double directory layout. We use native TF2 types and include paths correctly.

- Includes:
  - `#include "tf2_ros/tf2_ros/buffer.h"`
  - `#include "tf2_ros/tf2_ros/transform_listener.h"`
  - `#include "tf2_ros/tf2_ros/transform_broadcaster.h"`
  - `#include "tf2/time.h"`
- CMake:
  - `find_package(tf2 REQUIRED)`
  - `find_package(tf2_ros REQUIRED)`
  - Add `/opt/ros/jazzy/include` to include dirs or compile definitions as done in this package
- Usage:
  - Buffer: `tf2_ros::Buffer(clock, tf2::Duration)`
  - Listener: `tf2_ros::TransformListener(buffer, node)`
  - Broadcaster: `tf2_ros::TransformBroadcaster(node)`

## Troubleshooting
- **Linker error `tf2::fromMsg(...)`**: Ensure `tf2` and `tf2_geometry_msgs` are in `find_package()` and `ament_target_dependencies()`.
- **`bad_weak_ptr` in constructor**: Do not call `shared_from_this()` in constructors. We pass `this` where needed and avoid `shared_from_this()` until the object is fully constructed.
- **Scan matching doesn’t converge initially**: Normal when map is mostly unknown. We seed the map with the first scan and throttle warnings. Use the static test to validate.
- **Jazzy TF2 includes fail**: Verify double‑directory include paths and that `/opt/ros/jazzy/include` is on the include path.

## Phase 2 Status
- Phase 2.1 (Scan Matching): Complete
- Phase 2.2 (Map Management): Complete
- Phase 2.3 (Integration): Complete
- Phase 2.4 (Testing): Complete (static/dynamic tests, monitor)

## License
See `package.xml` for license and metadata.
