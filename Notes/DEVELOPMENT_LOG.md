# SLAM Project Development Log

## Project Overview
- **Project Name**: SLAM_10-25
- **Purpose**: Research project for Simultaneous Localization and Mapping (SLAM) algorithms
- **ROS Version**: ROS 2 Jazzy
- **Start Date**: September 26, 2025

## Key Learnings and Solutions

### 1. ROS 2 Jazzy TF2 Include Path Issue
**Problem**: TF2 headers in ROS 2 Jazzy have a nested directory structure that causes compilation failures.

**Discovery**:
- TF2 headers are located at: `/opt/ros/jazzy/include/tf2_ros/tf2_ros/*.h` and `*.hpp`
- The double directory structure (`tf2_ros/tf2_ros/`) is required in Jazzy
- Compatibility headers without the double path don't exist in Jazzy
- When tf2_ros includes tf2 headers, it expects them at `tf2/buffer_core.hpp` but they're actually at `tf2/tf2/buffer_core.hpp`

**Solution**:
1. Add `/opt/ros/jazzy/include` to CMake include directories
2. Use the full path: `#include "tf2_ros/tf2_ros/transform_listener.hpp"`
3. For initial development, created a simplified version without TF2 dependencies

**Code Example**:
```cmake
target_include_directories(target_name PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /opt/ros/jazzy/include  # Required for TF2 headers
)
```

### 2. Template Project Duplication and Stale Cache
**Problem**: After duplicating a ROS 2 template project, CMake caches contain absolute paths pointing to the original template location.

**Solution**:
- The build script (`scripts/build.sh`) automatically detects stale caches
- Use `--auto-clean` flag to automatically clean on first build after duplication
- The warning about stale caches is expected and only occurs once after duplication

**Command**:
```bash
bash scripts/build.sh --ros-distro jazzy --auto-clean
```

### 3. Occupancy Grid Generator Architecture
**Simplified Version Created**:
- `occupancy_grid_generator_simple.hpp/cpp`: Core implementation without TF2
- Assumes robot at origin (0,0,0) for initial testing
- Implements probabilistic occupancy grid mapping using log-odds representation
- Uses Bresenham's line algorithm for ray tracing

**Key Parameters**:
- Resolution: 0.05m (5cm per pixel)
- Grid size: 2000x2000 cells (100m x 100m at 5cm resolution)
- Probability values: hit=0.7, miss=0.4, prior=0.5
- Occupancy thresholds: occupied>65, free<25 (on 0-100 scale)

### 4. ROS 2 Package Structure Best Practices
**Components Created**:
1. **Nodes**: Executable entry points
   - `occupancy_grid_generator_node.cpp`: Full version with TF2
   - `occupancy_grid_generator_simple_node.cpp`: Simplified version
   - `fake_scan_publisher.cpp`: Test data generator

2. **Libraries**: Reusable components
   - Component library for composition

3. **Configuration**:
   - Launch files for different scenarios
   - RViz configuration for visualization
   - Parameter files for tuning

### 5. Testing Infrastructure
**Fake Scan Publisher**:
- Simulates a rectangular room with obstacles
- Publishes to `/scan` topic at 10Hz
- Includes simulated robot movement
- Adds realistic noise to measurements

**Visualization Issues Encountered**:
- RViz2 shows "Message Filter dropping message" warnings
- This is due to missing TF transforms (expected with simplified version)
- Map still visualizes correctly despite warnings

## Development Phases Completed

### Phase 1: Basic Occupancy Grid Generation ✅
- [x] Create package structure
- [x] Implement scan subscriber
- [x] Implement occupancy grid publisher
- [x] Create fake scan publisher for testing
- [x] Basic probabilistic mapping algorithm
- [x] Bresenham line algorithm for ray tracing
- [x] Log-odds representation for Bayesian updates

## Next Steps

### Immediate Tasks
1. **Fix TF2 Integration**: Properly integrate TF2 for real robot pose tracking
2. **Test with Real Data**: Connect to TurtleBot3 (address Foxy/Jazzy compatibility)
3. **Parameter Tuning**: Optimize probability values for better mapping
4. **Add Dynamic Updates**: Implement map decay for dynamic environments

### Future Phases (Per PROJECT_DESIGN.md)
- Phase 2: Particle Filter Localization
- Phase 3: Graph-based SLAM
- Phase 4: Loop Closure Detection
- Phase 5: Multi-robot SLAM

## Useful Commands

### Build Commands
```bash
# Clean build with auto-clean
bash scripts/build.sh --ros-distro jazzy --auto-clean

# Build without tests (useful during development)
cd ros2_ws && colcon build --packages-select occupancy_grid_generator --cmake-args -DBUILD_TESTING=OFF

# Source the workspace
source ros2_ws/install/setup.bash
```

### Run Commands
```bash
# Terminal 1: Occupancy grid generator (simplified)
ros2 run occupancy_grid_generator occupancy_grid_generator_simple_node

# Terminal 2: Fake scan publisher
ros2 run occupancy_grid_generator fake_scan_publisher

# Terminal 3: Visualization
rviz2 -d ros2_ws/src/occupancy_grid_generator/config/occupancy_grid_rviz.rviz
```

### Debug Commands
```bash
# Check topics
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /map

# Check node info
ros2 node info /occupancy_grid_generator_simple
ros2 node info /fake_scan_publisher
```

## Technical Decisions

### Why Simplified Version First?
1. **Faster Development**: Avoid TF2 complexity initially
2. **Testing Focus**: Validate core algorithm without transform issues
3. **Incremental Complexity**: Add features progressively
4. **Educational Value**: Understand each component separately

### Algorithm Choices
1. **Log-Odds**: More numerically stable than direct probability
2. **Bresenham**: Efficient integer-based line drawing
3. **Probabilistic Updates**: Handles sensor uncertainty naturally
4. **Grid-based Representation**: Simple and sufficient for indoor environments

## Performance Considerations
- Grid size: 2000x2000 = 4M cells
- Memory usage: ~32MB for grid + log-odds
- Update rate: Can handle 10Hz scan rate comfortably
- Optimization opportunities: Sparse representation, GPU acceleration

## References and Resources
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Probabilistic Robotics by Thrun, Burgard, and Fox](http://www.probabilistic-robotics.org/)
- [Bresenham's Line Algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
- [Log-Odds Representation](https://en.wikipedia.org/wiki/Logit)

---
*Last Updated: September 26, 2025*
