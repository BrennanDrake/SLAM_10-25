# ROS 2 Workspace

This is the project-local ROS 2 workspace. It follows the standard layout:

- `ros2_ws/src/` — put your ROS 2 packages here (one folder per package)
- Build outputs will be created by colcon in:
  - `ros2_ws/build/`
  - `ros2_ws/install/`
  - `ros2_ws/log/`

Quickstart

1) Source ROS 2:
```bash
source /opt/ros/jazzy/setup.bash
```

2) Install deps for packages you add to `src/` (ensure rosdep is initialized on this machine; if not, run `sudo rosdep init || true` then `rosdep update` once):
```bash
rosdep install -i --from-paths src --rosdistro jazzy -y
```

3) (First time only) Add colcon mixins (for release/debug profiles):
```bash
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update
```

4) Build:
```bash
colcon build --mixin release
```

5) Source local overlay (in a new terminal or after build completes):
```bash
source install/setup.bash
```

6) Run the template pub/sub demo (if `template_pkg` exists):
```bash
ros2 launch template_pkg pub_sub.launch.py
```

Notes
- This project’s Windsurf configuration denies editing generated directories (`build/`, `install/`, `log/`) and caches.
- Keep package metadata complete in each `package.xml` (license, maintainer, repository URL).
- Prefer ROS-native logging and proper lifecycle init/shutdown in nodes.
