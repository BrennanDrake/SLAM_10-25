#!/bin/bash
# Start Visual SLAM with D455 Camera
# Phase 4: Visual SLAM Session

set -e

echo "======================================"
echo "Visual SLAM with D455 Camera"
echo "======================================"
echo ""

# Check if RealSense is running
if ! ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw"; then
    echo "❌ RealSense camera not detected!"
    echo ""
    echo "Please start the camera first:"
    echo "  ros2 launch realsense2_camera rs_launch.py enable_depth:=true enable_color:=true align_depth.enable:=true"
    echo ""
    exit 1
fi

echo "✅ Camera detected and streaming"
echo ""

# Check for database
DB_PATH="$HOME/.ros/rtabmap_d455.db"
if [ -f "$DB_PATH" ]; then
    echo "📁 Existing database found: $DB_PATH"
    read -p "Delete and start fresh? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm "$DB_PATH"
        echo "🗑️  Database deleted"
        DELETE_FLAG="true"
    else
        echo "📂 Continuing with existing database"
        DELETE_FLAG="false"
    fi
else
    echo "📝 Creating new database: $DB_PATH"
    DELETE_FLAG="true"
fi

echo ""
echo "🚀 Starting RTAB-Map..."
echo ""
echo "Instructions:"
echo "1. Move the camera slowly around your environment"
echo "2. Ensure good lighting and textured surfaces"
echo "3. Watch the RTAB-Map visualization window"
echo "4. Green matches = good tracking"
echo "5. Press Ctrl+C to stop"
echo ""
echo "======================================"
echo ""

# Launch RTAB-Map
ros2 launch /home/brennan/Desktop/Dev/SLAM_10-25/scripts/visual_slam/launch_rtabmap_d455.py \
    delete_db_on_start:=$DELETE_FLAG
