# Project Template Notes

This workspace is a template you can duplicate to start new projects quickly and safely. It assumes you will create a `ros2_ws/` directory inside the project folder for ROS 2 development.

## What’s included
- **Windsurf allow/deny policy**: see `.windsurf/allow_deny.yaml`.
  - Allows editing typical source/config assets: `ros2_ws/src/**`, `**/*.launch.py`, `**/*.msg|*.srv|*.action`, `**/package.xml`, `**/CMakeLists.txt`, `**/*.yaml`, `**/*.urdf|*.xacro`, etc.
  - Auto-denies generated artifacts and caches: `ros2_ws/build/**`, `ros2_ws/install/**`, `ros2_ws/log/**`, `**/.colcon/**`, `**/.ros/**`, etc.
  - Blocks risky commands by default (e.g., `curl|bash`, `sudo apt install`, destructive `git`).
  - Provides scoped, safe commands (read-only status/listing) constrained to `ros2_ws/`.
- **ROS 2 rules file (template)**: `.windsurf/rules/ros_rules.md` (authoritative).
  - Language-scoped hints for Python, C++, CMake, package.xml, YAML, and launch files.
  - Encourages proper ROS lifecycle init/shutdown and ROS-native logging.
- **General rules file (template)**: `.windsurf/rules/general_rules.md` (authoritative).
  - General coding hygiene (docstrings/JSDoc, duplicate import/line cleanup, HTML meta hygiene).
  - Note: legacy copies at the project root may exist for discoverability and can be removed.

### Helper scripts (added for convenience)
- `scripts/verify.sh` — read-only checks: sources ROS, runs `ros2 doctor --report`, `colcon list`, and `rosdep check`.
- `scripts/build.sh` — builds from `ros2_ws/` with `--mixin release` (falls back to `-DCMAKE_BUILD_TYPE=Release`).
- `scripts/run_pubsub.sh` — launches `template_pkg pub_sub.launch.py` after sourcing overlay.
- `scripts/clean.sh` — safely removes `build/`, `install/`, and `log/` (top-level and `ros2_ws/*`) with confirmation.

## Quickstart (first ROS 2 run)

```bash
# 1) Trust project env (once per project)
direnv allow

# 2) Safe init (mixins + rosdep cache; no build)
bash scripts/init.sh --ros-distro jazzy --no-build

# 3) Build workspace
bash scripts/build.sh

# 4) Run demo
bash scripts/run_pubsub.sh
```

Or, if you prefer aliases:

```bash
sjw && ros2 launch template_pkg pub_sub.launch.py
```

## Common gotchas

- Stale CMake cache after duplicating the template (CMakeCache referencing a different directory). Fix:
  ```bash
  bash scripts/clean.sh -y && bash scripts/build.sh
  ```
- Environment not loaded in a new shell. Use `direnv allow` or the `sj`/`sjw` aliases above.
- Missing mixins. Re-run:
  ```bash
  bash scripts/init.sh --ros-distro jazzy --no-build
  ```

## Makefile shortcuts

From the project root, you can use these shortcuts:

- **init-no-build**: `make init-no-build` — safe init (mixins + rosdep cache, no build)
- **verify**: `make verify` — read-only checks (ros2 doctor, colcon list, rosdep check)
- **build**: `make build` — build workspace using release defaults
- **build-auto-clean**: `make build-auto-clean` — build and auto-clean stale caches if detected
- **run**: `make run` — launch `template_pkg` pub/sub demo
- **clean**: `make clean` — remove build/install/log with confirmation flag
- **doctor**: `make doctor` — read-only environment & workspace checks
- **create-pkg**: `make create-pkg NAME=my_pkg TYPE=ament_cmake DEPS="rclcpp std_msgs"`

## Create a new package

You can scaffold a package in `ros2_ws/src/` with:

```bash
bash scripts/create_pkg.sh --name my_pkg --type ament_cmake --deps "rclcpp std_msgs"
# or Python:
bash scripts/create_pkg.sh --name my_py_pkg --type ament_python --deps "rclpy std_msgs"
```

Using the Makefile:

```bash
make create-pkg NAME=my_pkg TYPE=ament_cmake DEPS="rclcpp std_msgs"
```

## How to use this template
1. Duplicate this project directory to a new location.
2. Create your workspace skeleton:
   - `mkdir -p ros2_ws/src`
3. Confirm rules are present under `.windsurf/rules/` (authoritative location). Root copies exist for discovery and can be removed.
4. Review `.windsurf/allow_deny.yaml` and adjust org allowlist for `vcs` imports if needed.
5. Source ROS 2 environment or add to shell profile:
   - `source /opt/ros/jazzy/setup.bash`

## Security posture (quick overview)
- System install paths like `/opt/ros/**` are not targeted by rules; avoid edits there.
- Generated build outputs are denied from edits to prevent accidental changes.
- Risky commands require explicit approval.

## Quick verification
- `ros2 doctor --report` (after `source /opt/ros/jazzy/setup.bash`)
- If needed, run demo:
  - Terminal A: `ros2 run demo_nodes_cpp talker`
  - Terminal B: `ros2 run demo_nodes_py listener`

## Maintenance checklist
- Keep `allow_deny.yaml` aligned with your team/org trust model (VCS import orgs).
- Update `ros_rules.md` and `general_rules.md` as coding standards evolve.
- Document parameter files and launch configurations as they grow.
- Re-run `rosdep` checks when adding packages: `rosdep check -i --from-paths ros2_ws/src`.

## First-time setup (mixins)
Colcon mixins provide convenient build profiles like `release`, `debug`, etc. Add the upstream mixin repository once per machine:

```bash
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update
```

Then you can build with:

```bash
colcon build --mixin release
```

## Template package quickstart
This template includes a sample package `template_pkg` (C++ talker/listener) under `ros2_ws/src/`.

Build and run both nodes:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --mixin release
ros2 launch template_pkg pub_sub.launch.py

### Why `bash -lc` in commands from the IDE?
Some IDE terminals default to non-Bash shells where `source` may not exist or login files aren’t loaded. Using `bash -lc "..."` ensures:
- Bash is used (so `source` works).
- `-c` runs the full chain in one shell so the sourced environment persists for subsequent commands in that line.
- `-l` loads login profiles if needed so tools like `colcon`/`ros2` are on PATH.

## Optional: automatic per-project environment with direnv
Direnv loads environment variables when you `cd` into the project, and unloads them when you leave.

1) Install and enable direnv (once per machine):

```bash
sudo apt install direnv -y
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
source ~/.bashrc
```

2) Allow this project’s `.envrc` (in the project root):

```bash
cd /path/to/your/project
direnv allow
```

3) Build, then re-enter the directory to auto-activate overlay:

```bash
cd ros2_ws && colcon build --mixin release && cd -
# direnv will source ros2_ws/install/setup.bash automatically when present
```

Notes:
- `.envrc` sources `/opt/ros/jazzy/setup.bash` and `ros2_ws/install/setup.bash` if they exist.
- If you use zsh, add `eval "$(direnv hook zsh)"` to `~/.zshrc` instead.

## Initialization script
Use the bootstrap script to set up common tooling and build in one step.

Run from project root:
```bash
bash scripts/init.sh --ros-distro jazzy [--install-deps] [--no-mixins] [--no-direnv-allow] [--no-build] [--verbose]
```

What it does by default:
- Sets up colcon mixins (adds/updates the default mixin repository)
- Runs `direnv allow` (if direnv is installed)
- Runs `rosdep update` (does not install deps unless you pass `--install-deps`)
- Builds the workspace with `--mixin release` (falls back to `-DCMAKE_BUILD_TYPE=Release` if needed)

Notes:
- `--install-deps` may prompt for sudo via rosdep; leave it off if you want a read-only check.
- After build, re-enter the project directory to let direnv auto-source the overlay.

### Renaming or relocating this repository
If you rename or move this project folder after cloning, previously generated CMake caches may point to the old path.

- Preferred: `make build-auto-clean` (auto-cleans stale caches before building)
- Or manually:
  ```bash
  bash scripts/clean.sh -y
  bash scripts/build.sh
  ```

Additionally, initialization is idempotent. The init stamp now records the project root and `scripts/init.sh` will skip only if the stored root matches the current one. If the directory changed, the script will re-run init steps and update the stamp automatically (or you can force with `--force-reinit`).

## Initialization stamp (idempotency)
To avoid repeating initialization on subsequent sessions, the init script writes a stamp file:

- Path: `.windsurf/state/init_done.json`
- Behavior: `scripts/init.sh` exits early if the stamp exists and matches the current project root. If the repository was moved/renamed, the script will re-initialize and update the stamp. Use `--force-reinit` to override.
- To re-run initialization: either delete the stamp file or run `bash scripts/init.sh --force-reinit`.
