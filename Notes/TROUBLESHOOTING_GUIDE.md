# SLAM Project Troubleshooting Guide

## Common Issues and Solutions

### 1. Build Errors

#### TF2 Include Path Errors
**Error Message**:
```
fatal error: tf2_ros/transform_listener.h: No such file or directory
```

**Root Cause**: ROS 2 Jazzy has a nested directory structure for TF2 headers.

**Solutions**:
1. Use the full path with double directory:
   ```cpp
   #include "tf2_ros/tf2_ros/transform_listener.hpp"
   ```

2. Add include directory to CMakeLists.txt:
   ```cmake
   target_include_directories(target_name PUBLIC
     /opt/ros/jazzy/include
   )
   ```

3. Use simplified version without TF2:
   ```bash
   ros2 run occupancy_grid_generator occupancy_grid_generator_simple_node
   ```

#### Stale CMake Cache
**Error Message**:
```
[WARN] Stale CMake cache detected: ros2_ws/build/occupancy_grid_generator
```

**Root Cause**: Project was duplicated from template with cached absolute paths.

**Solution**:
```bash
# Option 1: Auto-clean on build
bash scripts/build.sh --ros-distro jazzy --auto-clean

# Option 2: Manual clean
bash scripts/clean.sh -y
```

#### Missing Dependencies
**Error Message**:
```
fatal error: rclcpp_components/register_node_macro.hpp: No such file or directory
```

**Solution**: Add missing dependency to CMakeLists.txt:
```cmake
ament_target_dependencies(target_name
  rclcpp
  rclcpp_components  # Add this
  # ... other dependencies
)
```

### 2. Runtime Issues

#### RViz Message Filter Warnings
**Warning Message**:
```
[INFO] [rviz]: Message Filter dropping message: frame 'base_scan' at time X for reason 'discarding message because the queue is full'
```

**Root Cause**: Missing TF transforms between frames.

**Solutions**:
1. For testing, ignore warnings (map still works)
2. Run fake_scan_publisher which publishes transforms:
   ```bash
   ros2 run occupancy_grid_generator fake_scan_publisher
   ```
3. For real robot, ensure TF tree is complete

#### No Map Visualization
**Symptoms**: Map topic exists but nothing shows in RViz.

**Checklist**:
1. Check if nodes are running:
   ```bash
   ros2 node list
   ```

2. Check if scan data is being published:
   ```bash
   ros2 topic hz /scan
   ```

3. Check if map is being published:
   ```bash
   ros2 topic hz /map
   ```

4. Verify RViz settings:
   - Fixed Frame: "map" or "odom"
   - Map topic: "/map"
   - LaserScan topic: "/scan"

#### Grid Not Updating
**Symptoms**: Map initializes but doesn't update with scan data.

**Debug Steps**:
1. Check scan callback is being called:
   ```bash
   ros2 run occupancy_grid_generator occupancy_grid_generator_simple_node --ros-args --log-level debug
   ```

2. Verify scan data is valid:
   ```bash
   ros2 topic echo /scan | head -20
   ```

3. Check parameter values:
   ```bash
   ros2 param list /occupancy_grid_generator_simple
   ```

### 3. Environment Issues

#### ROS 2 Not Sourced
**Error Message**:
```
Command 'ros2' not found
```

**Solution**:
```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
```

#### Colcon Build Warnings
**Warning Message**:
```
WARNING:colcon.colcon_defaults.argument_parser.defaults:Skipping unknown keys from 'colcon-defaults.yaml' for 'build': mixins
```

**Status**: This is harmless and can be ignored. The template uses mixins that may not be installed.

#### Environment Variable Warnings
**Warning Message**:
```
WARNING:colcon.colcon_ros.prefix_path.ament:The path '/path/to/install' doesn't exist
```

**Cause**: Old paths in environment from previous builds.

**Solution**: Start a new terminal or unset variables:
```bash
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
```

### 4. Performance Issues

#### High CPU Usage
**Symptoms**: System becomes sluggish when running nodes.

**Solutions**:
1. Reduce scan frequency:
   ```cpp
   timer_ = this->create_wall_timer(
     std::chrono::milliseconds(200),  // Increase from 100ms
     std::bind(&FakeScanPublisher::publish_scan, this));
   ```

2. Reduce grid resolution:
   ```bash
   ros2 param set /occupancy_grid_generator_simple resolution 0.1  # 10cm instead of 5cm
   ```

3. Reduce grid size:
   ```bash
   ros2 param set /occupancy_grid_generator_simple grid_width 1000
   ros2 param set /occupancy_grid_generator_simple grid_height 1000
   ```

#### Memory Issues
**Symptoms**: Out of memory errors or excessive memory usage.

**Analysis**:
- Grid memory: width × height × 1 byte = 2000 × 2000 = 4MB
- Log-odds: width × height × 8 bytes = 2000 × 2000 × 8 = 32MB
- Total per node: ~36MB minimum

**Solutions**:
1. Use smaller grid
2. Implement sparse grid representation
3. Clear old data periodically

### 5. Testing Issues

#### Fake Scan Publisher Not Working
**Symptoms**: No scan data being published.

**Debug**:
```bash
# Check if node is running
ros2 node list | grep fake

# Check publishing rate
ros2 topic hz /scan

# View actual data
ros2 topic echo /scan --once
```

#### Unit Tests Failing
**Build without tests**:
```bash
colcon build --packages-select occupancy_grid_generator --cmake-args -DBUILD_TESTING=OFF
```

### 6. Cross-Version Compatibility

#### Foxy to Jazzy Communication
**Issue**: TurtleBot3 runs Foxy, development environment uses Jazzy.

**Solutions**:
1. Use DDS discovery:
   ```bash
   export ROS_DOMAIN_ID=30  # Same on both systems
   ```

2. Use ros1_bridge equivalent for ROS 2:
   ```bash
   # On Foxy machine
   ros2 run ros_bridge parameter_bridge

   # On Jazzy machine
   ros2 topic list  # Should see Foxy topics
   ```

3. Use network bridge:
   ```bash
   # Configure DDS for network communication
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```

## Quick Diagnostic Commands

```bash
# System check
ros2 doctor

# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Check specific topic
ros2 topic info /scan
ros2 topic hz /scan
ros2 topic echo /scan --once

# Check parameters
ros2 param list /occupancy_grid_generator_simple
ros2 param get /occupancy_grid_generator_simple resolution

# Check services
ros2 service list

# Check TF tree
ros2 run tf2_tools view_frames
```

## When All Else Fails

1. **Clean Build**:
   ```bash
   bash scripts/clean.sh -y
   bash scripts/build.sh --ros-distro jazzy
   ```

2. **Check Dependencies**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Restart ROS Daemon**:
   ```bash
   ros2 daemon stop
   ros2 daemon start
   ```

4. **Check System Resources**:
   ```bash
   htop  # CPU and memory
   df -h  # Disk space
   ```

---
*Last Updated: September 26, 2025*
