#!/bin/bash
# Test script for Hector SLAM with simulated data

echo "========================================="
echo "Hector SLAM Test Script"
echo "========================================="

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source ~/Desktop/Dev/SLAM_10-25/ros2_ws/install/setup.bash

echo ""
echo "Step 1: Building packages..."
cd ~/Desktop/Dev/SLAM_10-25/ros2_ws
colcon build --packages-select hector_slam_custom occupancy_grid_generator --cmake-args -DBUILD_TESTING=OFF

if [ $? -ne 0 ]; then
    echo "Build failed! Please fix compilation errors."
    exit 1
fi

echo ""
echo "Step 2: Sourcing workspace..."
source install/setup.bash

echo ""
echo "Step 3: Launching SLAM system..."
echo "Press Ctrl+C to stop"
echo ""

# Launch the SLAM system
ros2 launch hector_slam_custom test_slam.launch.py use_rviz:=false

echo ""
echo "Test completed!"
