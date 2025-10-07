# Renaming the Template Package

This checklist helps you safely rename the template package (`template_pkg`) to a new package name.

## 1) Rename the directory
- `ros2_ws/src/template_pkg/` → `ros2_ws/src/<new_name>/`

## 2) Update package metadata and build files
- `package.xml`
  - `<name>template_pkg</name>` → `<name><new_name></name>`
  - Verify `<maintainer>`, `<license>`, and `<url>` entries
- `CMakeLists.txt`
  - `project(template_pkg)` → `project(<new_name>)`
  - Targets can be left as `talker`/`listener` or renamed; they install under `${PROJECT_NAME}`

## 3) Update code and launch filenames (optional)
- C++ nodes
  - In `src/talker.cpp`, node name is currently `template_pkg_talker`; update the string if desired
  - In `src/listener.cpp`, node name is `template_pkg_listener`; update if desired
- Launch files in `launch/`
  - Update `package='template_pkg'` → `package='<new_name>'`
  - Filenames may remain the same (`talker.launch.py`, `pub_sub.launch.py`)

## 4) Search & replace references
- Grep the workspace for `template_pkg` and update docs, paths, and any scripts

## 5) Rebuild and verify
```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --mixin release
source install/setup.bash
ros2 launch <new_name> pub_sub.launch.py
```

## Tips
- Keep node names configurable through launch arguments to reduce renames in code
- For large renames, consider `git mv` to preserve history
