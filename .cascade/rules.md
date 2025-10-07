# SLAM Learning Project - Cascade AI Assistant Rules

## Project Context
This is Brennan's educational SLAM implementation project, focusing on deep understanding through hands-on implementation. Currently in Phase 3 (EKF-SLAM complete), preparing for Phase 4 (Visual SLAM).

## File Navigation Rules
When asked about:
- **Current work**: Check `/ros2_ws/src/ekf_slam/` first
- **Previous phases**: Phase 1 → `occupancy_grid_generator/`, Phase 2 → `hector_slam_custom/`
- **Documentation**: Always check `/Notes/` folder, especially:
  - `TECHNICAL_NOTES.md` for algorithms
  - `TROUBLESHOOTING.md` for common issues
  - `NEXT_SESSION_PLAN.md` for upcoming work
  - `PROJECT_ROADMAP.md` for progress tracking

## SLAM-Specific Code Patterns
### Always Check:
1. **Covariance matrices**: Must be symmetric and positive definite
   ```cpp
   assert((P - P.transpose()).norm() < 1e-6);  // Symmetric
   assert(P.llt().info() == Eigen::Success);   // Positive definite
   ```

2. **Jacobian dimensions**: Match state/measurement sizes
   ```cpp
   // State Jacobian: n×n where n = state size
   // Measurement Jacobian: m×n where m = measurement size
   ```

3. **Coordinate frames**: Track transformations carefully
   - world → map → odom → base_link → sensor

4. **Angle normalization**: Always wrap to [-π, π]
   ```cpp
   while (angle > M_PI) angle -= 2*M_PI;
   while (angle < -M_PI) angle += 2*M_PI;
   ```

## Build & Test Workflow
### Standard Build:
```bash
cd ~/Desktop/Dev/SLAM_10-25/ros2_ws
colcon build --packages-select <package> --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

### Quick Test Pattern:
1. Terminal 1: Run node
2. Terminal 2: `ros2 topic list` and `ros2 topic hz <topic>`
3. Terminal 3: `rviz2` for visualization
4. Terminal 4: Monitor with custom scripts

## Teaching Approach
### When explaining concepts:
1. **Start with intuition**: Real-world analogy first
2. **Show the math**: Equations with clear notation
3. **Provide example**: Concrete numbers (e.g., "robot at (0,0) moves 1m")
4. **Code connection**: Show implementation
5. **Quiz understanding**: Ask targeted questions

### Mathematical Notation Standards:
- Vectors: **bold** (e.g., **x**, **μ**)
- Matrices: CAPITALS (e.g., P, H, K)
- Scalars: lowercase (e.g., x, y, θ)
- Always define symbols before use

## Problem-Solving Priority
When debugging:
1. **Check basics first**: Topics publishing? TF tree complete?
2. **Verify assumptions**: Valid sensor data? Parameters declared?
3. **Mathematical checks**: Covariance OK? Jacobians correct dimensions?
4. **Visualization**: Everything in RViz2, add debug markers
5. **Refer to notes**: Check TROUBLESHOOTING.md for similar issues

## Session Management
### Time Awareness:
- Target session: ~3 hours
- Break complex topics into 20-30 min segments
- Provide summary every hour
- Warn when approaching 2.5 hours

### Progress Tracking:
- Update SESSION_LOGS.md after major accomplishments
- Create memories for key insights
- Note what worked/didn't for future reference

## Code Quality Standards
### For Educational Code:
- **Clarity > Performance**: Readable code with comments
- **Explicit > Implicit**: Show all steps, even obvious ones
- **Errors > Crashes**: Add assertions and checks
- **Learning > Production**: OK to have inefficiencies if clearer

### Comment Standards:
```cpp
// WHY: Explain the purpose/theory
// WHAT: Describe what this does
// HOW: Implementation details if non-obvious
// TODO: Learning exercises or improvements
```

## Research Integration
### Always consider:
- How does this relate to RFID fusion goals?
- Could this work with lifelong learning?
- Will this scale to drone platform?
- Connection to real-world robotics?

## Common Pitfalls to Avoid
1. **Don't assume knowledge**: Explain acronyms first use (e.g., EKF = Extended Kalman Filter)
2. **Don't skip steps**: Show intermediate calculations
3. **Don't ignore warnings**: They usually indicate real issues
4. **Don't over-optimize**: This is educational, not production

## Memory Management
### Create memories for:
- Key algorithm insights
- Debugging solutions that worked
- Parameter combinations that converged
- Conceptual breakthroughs

### Reference memories when:
- Similar problems arise
- Building on previous concepts
- Student asks about past sessions

## Quick Reference Commands
```bash
# Aliases Brennan uses:
alias sjw='source /opt/ros/jazzy/setup.bash && source ~/Desktop/Dev/SLAM_10-25/ros2_ws/install/setup.bash'
alias slam='cd ~/Desktop/Dev/SLAM_10-25'

# Common diagnostics:
ros2 topic list
ros2 topic hz /scan
ros2 run tf2_tools view_frames
ros2 node info <node_name>
```

## Current Status Awareness
- **Completed**: Phases 1-3 (Occupancy Grid, Hector SLAM, EKF-SLAM)
- **Current**: Preparing Phase 4 (Visual SLAM with RTAB-Map)
- **Hardware Ready**: D455 camera, TurtleBot3
- **Research Goals**: RFID fusion, lifelong learning, drone SLAM

## Response Style
- No preambles or acknowledgments
- Jump straight to technical content
- Use markdown formatting extensively
- Provide runnable code examples
- Reference specific files with full paths
