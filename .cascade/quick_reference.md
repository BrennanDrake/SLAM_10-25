# SLAM Project Quick Reference Card

## 🚀 Session Start Checklist
```bash
# 1. Source environment
sjw  # Or: source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash

# 2. Check current phase status
cat Notes/PROJECT_ROADMAP.md | grep "Phase 4"

# 3. Open key files
code Notes/NEXT_SESSION_PLAN.md
code Notes/TECHNICAL_NOTES.md
```

## 🔨 Common Commands

### Build
```bash
# Single package
colcon build --packages-select ekf_slam --cmake-args -DBUILD_TESTING=OFF

# Clean build
rm -rf build install log && colcon build

# With symlinks (for Python changes)
colcon build --symlink-install --packages-select <package>
```

### Test
```bash
# Run node
ros2 run <package> <node>

# Launch file
ros2 launch <package> <launch_file>.launch.py

# With parameters
ros2 run <package> <node> --ros-args -p param:=value
```

### Debug
```bash
# Check topics
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /map

# Check TF
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link

# Node info
ros2 node list
ros2 node info /<node_name>

# Parameters
ros2 param list /<node_name>
ros2 param get /<node_name> <param>
```

### Visualize
```bash
# RViz
rviz2

# With config
rviz2 -d src/<package>/config/config.rviz

# Plot topics
ros2 run rqt_plot rqt_plot
```

## 📚 Key Files to Reference

| Need | File |
|------|------|
| Algorithm help | `Notes/TECHNICAL_NOTES.md` |
| Debug issue | `Notes/TROUBLESHOOTING.md` |
| Project status | `Notes/PROJECT_ROADMAP.md` |
| Next session | `Notes/NEXT_SESSION_PLAN.md` |
| Session history | `Notes/SESSION_LOGS.md` |

## 🎯 Current Focus

**Phase 4: Visual SLAM**
- [ ] Install RTAB-Map
- [ ] Test D455 camera
- [ ] Feature detection exercise
- [ ] Dataset testing

## 💡 Quick Fixes

### ROS not found
```bash
source /opt/ros/jazzy/setup.bash
```

### Package not found
```bash
source ros2_ws/install/setup.bash
```

### Transform error
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser
```

### High CPU
```bash
top -H -p $(pgrep -d',' ros2)
```

## 📝 Session Notes Template

```markdown
## Session N: [Topic]
Date: YYYY-MM-DD | Duration: X hours

### Goals:
- [ ] 

### Accomplished:
- 

### Key Learnings:
- 

### Issues & Solutions:
- 

### Next Time:
- 
```

## 🧮 Common SLAM Checks

```cpp
// Covariance symmetric?
assert((P - P.transpose()).norm() < 1e-6);

// Covariance positive definite?
Eigen::LLT<Eigen::MatrixXd> llt(P);
assert(llt.info() == Eigen::Success);

// Angle normalized?
angle = atan2(sin(angle), cos(angle));

// Valid measurement?
assert(!std::isnan(range) && !std::isinf(range));
assert(range > 0 && range < max_range);
```

## 🎓 Learning Reminders

1. **Understanding > Speed**: Take time to grasp concepts
2. **Ask "Why?"**: Don't just copy code
3. **Test incrementally**: Small changes, frequent tests
4. **Visualize everything**: Use RViz liberally
5. **Document insights**: Update notes while fresh

---
*Keep this open during sessions for quick reference!*
