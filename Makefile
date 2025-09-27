.PHONY: help init-no-build verify build run clean create-pkg

help:
	@echo "Targets:"
	@echo "  init-no-build   - Initialize mixins & rosdep cache (no build)"
	@echo "  verify          - Read-only environment checks"
	@echo "  build           - Build the workspace (release defaults)"
	@echo "  build-auto-clean- Build with auto-clean if stale caches are detected"
	@echo "  run             - Launch template_pkg pub/sub demo"
	@echo "  clean           - Remove build/install/log (with -y)"
	@echo "  doctor          - Quick environment & workspace checks (read-only)"
	@echo "  create-pkg      - Create a new package (NAME=my_pkg TYPE=ament_cmake|ament_python DEPS=\"rclcpp std_msgs\")"

init-no-build:
	bash scripts/init.sh --ros-distro jazzy --no-build

verify:
	bash scripts/verify.sh --ros-distro jazzy

build:
	bash scripts/build.sh --ros-distro jazzy --mixin release

build-auto-clean:
	bash scripts/build.sh --ros-distro jazzy --mixin release --auto-clean

run:
	bash scripts/run_pubsub.sh --ros-distro jazzy

clean:
	bash scripts/clean.sh -y

# Usage: make create-pkg NAME=my_pkg TYPE=ament_cmake DEPS="rclcpp std_msgs"
create-pkg:
	bash scripts/create_pkg.sh --name "$(NAME)" --type "$(TYPE)" --deps "$(DEPS)"

doctor:
	bash scripts/doctor.sh --ros-distro jazzy
