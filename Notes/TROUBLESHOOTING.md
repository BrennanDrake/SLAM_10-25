# SLAM Project Troubleshooting Guide

## Quick Fixes

### Command Not Found
```bash
# If ros2 not found:
source /opt/ros/jazzy/setup.bash
source ~/Desktop/Dev/SLAM_10-25/ros2_ws/install/setup.bash

# Alias for quick sourcing (add to ~/.bashrc):
alias sjw='source /opt/ros/jazzy/setup.bash && source ~/Desktop/Dev/SLAM_10-25/ros2_ws/install/setup.bash'
```

### Clean Build
```bash
cd ~/Desktop/Dev/SLAM_10-25/ros2_ws
rm -rf build install log
colcon build --packages-select <package_name> --cmake-args -DBUILD_TESTING=OFF
```

---

## Common Build Errors

### Missing Dependencies
**Error**: `fatal error: tf2_ros/transform_listener.h: No such file or directory`

**Solution**:
```cmake
find_package(tf2_ros REQUIRED)
ament_target_dependencies(target_name tf2_ros)
```

### Parameter Not Declared
**Error**: `rclcpp::exceptions::ParameterNotDeclaredException`

**Solution**: Declare before using
```cpp
this->declare_parameter("param_name", default_value);
```

### Eigen Not Found
**Error**: `fatal error: Eigen/Dense: No such file or directory`

**Solution**:
```cmake
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
target_link_libraries(target Eigen3::Eigen)
```

---

## Runtime Issues

### Map Not Updating
**Symptoms**: Grid initializes but doesn't update

**Checklist**:
1. Check scan callback is triggered: `ros2 topic hz /scan`
2. Verify TF tree: `ros2 run tf2_tools view_frames`
3. Check occupancy threshold parameters
4. Ensure scan data is valid (not NaN/Inf)

### Scan Matching Not Converging
**Symptoms**: Warnings about convergence, drift in pose

**Solutions**:
1. Build initial map before matching (10-20 scans)
2. Lower score acceptance threshold (0.1 for static)
3. Check map has enough features (walls, corners)
4. Verify scan data consistency

### EKF Inconsistent
**Symptoms**: Covariance becomes non-positive definite

**Debug**:
```cpp
// Check covariance is symmetric
assert((P - P.transpose()).norm() < 1e-6);

// Check positive definite
Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P);
assert(es.eigenvalues().minCoeff() > 0);
```

---

## Visualization Issues

### RViz2 Message Filter Warnings
**Warning**: `Message Filter dropping message: frame 'laser' at time X`

**Fix**: Publish static transform
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser
```

### Markers Not Showing
**Checklist**:
- Correct frame_id in marker header
- Marker scale > 0
- Alpha > 0 (transparency)
- Topic subscribed in RViz2

---

## Performance Issues

### High CPU Usage
**Diagnosis**:
```bash
# Check which node is consuming CPU
top -H -p $(pgrep -d',' ros2)
```

**Common Causes**:
- Publishing at high rate unnecessarily
- Not using message filters
- Inefficient algorithms (use profiler)

### Memory Leaks
**Detection**:
```bash
valgrind --leak-check=full ros2 run <package> <node>
```

---

## Debugging Techniques

### Enable Debug Output
```cpp
// In your node
RCLCPP_DEBUG(this->get_logger(), "Debug message: %f", value);

// Set log level
ros2 run <package> <node> --ros-args --log-level debug
```

### Monitor Topics
```bash
# List all topics
ros2 topic list

# Check publishing rate
ros2 topic hz /topic_name

# View message content
ros2 topic echo /topic_name

# Check topic info
ros2 topic info /topic_name -v
```

### TF Debugging
```bash
# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Monitor specific transform
ros2 run tf2_ros tf2_echo map base_link
```

---

## SLAM-Specific Issues

### Drift Accumulation
**Causes**:
- No loop closure
- Poor odometry
- Insufficient features

**Mitigation**:
- Add external references (GPS, RFID)
- Improve feature detection
- Tune motion model noise

### Dynamic Objects
**Problem**: Moving objects corrupt map

**Solutions**:
- Filter by consistency over time
- Separate static/dynamic maps
- Use robust estimators (RANSAC)

### Lost Localization
**Recovery**:
```cpp
// Reset to known pose
geometry_msgs::msg::PoseWithCovarianceStamped reset_pose;
reset_pose.pose.pose.position.x = 0.0;
// ... publish to /initialpose
```

---

## Emergency Commands

### Kill All ROS Nodes
```bash
pkill -f ros2
```

### Clean Everything
```bash
cd ~/Desktop/Dev/SLAM_10-25
rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
rm -rf log build install  # If in wrong directory
```

### Reset Environment
```bash
unset ROS_DOMAIN_ID
source /opt/ros/jazzy/setup.bash
```

---

## Prevention Tips

1. **Always source in correct order**:
   - ROS first, then workspace

2. **Use launch files** instead of running nodes individually

3. **Set parameters in launch files** not hardcoded

4. **Add parameter validation** in nodes

5. **Use message filters** for synchronized topics

6. **Test incrementally** - don't write 1000 lines before testing

---

## Useful Aliases (.bashrc)

```bash
# Quick workspace navigation
alias slam='cd ~/Desktop/Dev/SLAM_10-25'
alias ws='cd ~/Desktop/Dev/SLAM_10-25/ros2_ws'

# Source everything
alias sjw='source /opt/ros/jazzy/setup.bash && source ~/Desktop/Dev/SLAM_10-25/ros2_ws/install/setup.bash'

# Quick build
alias cb='colcon build --packages-select'
alias cbs='colcon build --symlink-install --packages-select'

# Clean build
alias clean_build='rm -rf build install log && colcon build'
```

---

Last Updated: 2025-10-02
