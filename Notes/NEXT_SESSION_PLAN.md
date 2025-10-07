# Phase 4: Visual SLAM - Next Session Plan

## Session Goal
Transition from classical SLAM to production visual SLAM using RTAB-Map with hands-on learning.

## Session Structure (~3 hours)

### Part 1: Setup & Installation (30 min)
**Hands-on Activity**: Install RTAB-Map
```bash
sudo apt install ros-jazzy-rtabmap-ros
```

**Your Task**: 
- Install and verify RTAB-Map
- Test with sample bag file
- Understand the ROS 2 interface

---

### Part 2: Theory - Visual SLAM Fundamentals (30 min)

#### 2.1 Feature Detection (15 min)
**Interactive Demo**: I'll show you ORB features on your D455 live feed

**Concepts**:
- Corner detection (Harris, FAST)
- Feature descriptors (ORB, SIFT, SURF)
- Feature matching across frames
- Why features = landmarks in visual SLAM

**Your Coding Exercise**:
```python
# Write a simple ORB feature detector
import cv2
import numpy as np

def detect_orb_features(image):
    """
    TODO: Implement ORB feature detection
    1. Create ORB detector
    2. Detect keypoints
    3. Compute descriptors
    4. Return keypoints and descriptors
    """
    # Your code here
    pass

# Test with D455 camera feed
```

#### 2.2 Visual Odometry (15 min)
**Whiteboard Session**: How camera motion is estimated

**Concepts**:
- Epipolar geometry
- Essential matrix
- PnP (Perspective-n-Point)
- Connection to EKF-SLAM you just learned

**Quiz**: 
- How is visual odometry different from wheel odometry?
- Why do we need depth (RGB-D) vs just RGB?

---

### Part 3: D455 Camera Integration (45 min)

#### 3.1 Camera Calibration Check (10 min)
**Your Task**: Verify D455 calibration
```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108
```

#### 3.2 Live RTAB-Map with D455 (20 min)
**Hands-on Activity**: Run RTAB-Map with your camera

```bash
# Terminal 1: Start D455
ros2 launch realsense2_camera rs_launch.py \
    enable_depth:=true \
    enable_color:=true \
    enable_infra:=false

# Terminal 2: Start RTAB-Map
ros2 launch rtabmap_ros rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera/depth/image_rect_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    frame_id:=camera_link \
    approx_sync:=true

# Terminal 3: Visualization
rviz2 -d $(ros2 pkg prefix rtabmap_ros)/share/rtabmap_ros/launch/config/rgbd.rviz
```

**Your Experiment**:
1. Walk around your room with D455
2. Observe loop closure detection
3. Compare to your Phase 2 Hector SLAM

#### 3.3 Understanding the Output (15 min)
**Analysis Task**: Examine RTAB-Map database

```bash
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

**Questions to Answer**:
- How many features per frame?
- What's the loop closure threshold?
- How does 3D point cloud look?

---

### Part 4: Dataset Testing (30 min)

#### 4.1 TUM RGB-D Dataset
**Download**: https://vision.in.tum.de/data/datasets/rgbd-dataset

**Your Task**: Run RTAB-Map on pre-recorded data
```bash
# Download freiburg1_xyz dataset
wget https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.bag

# Convert to ROS 2 bag (if needed)
ros2 bag play rgbd_dataset_freiburg1_xyz.bag
```

**Analysis**:
- Compare trajectory to ground truth
- Measure ATE (Absolute Trajectory Error)
- Identify where SLAM fails

#### 4.2 Your Own Dataset
**Recording Activity**: Create your own dataset

```bash
# Record with D455
ros2 bag record -o my_room_dataset \
    /camera/depth/image_rect_raw \
    /camera/color/image_raw \
    /camera/color/camera_info \
    /camera/depth/camera_info
```

**Scenarios to Record** (5 min each):
1. **Easy**: Slow motion, good lighting, textured walls
2. **Medium**: Normal walking speed
3. **Hard**: Fast motion, low light, white walls
4. **Loop**: Walk in circle, return to start

**Comparison**: Run RTAB-Map on all 4, see which fails

---

### Part 5: Coding Exercise - Feature Matching (30 min)

**Your Implementation**: Write a simple feature matcher

```python
"""
feature_matcher.py - Compare to RTAB-Map's approach
"""

import cv2
import numpy as np
from typing import Tuple, List

class SimpleFeatureMatcher:
    def __init__(self):
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
    def match_frames(self, img1: np.ndarray, img2: np.ndarray) -> Tuple[List, List]:
        """
        TODO: Implement frame-to-frame matching
        
        Steps:
        1. Detect ORB features in both images
        2. Match descriptors
        3. Filter matches (ratio test, RANSAC)
        4. Return good matches
        
        Args:
            img1: First image
            img2: Second image
            
        Returns:
            matched_kp1: Keypoints from img1
            matched_kp2: Corresponding keypoints from img2
        """
        # Your code here
        pass
    
    def estimate_motion(self, kp1, kp2, K):
        """
        TODO: Estimate camera motion from matched features
        
        Use cv2.findEssentialMat() and cv2.recoverPose()
        
        Returns:
            R: Rotation matrix
            t: Translation vector
        """
        # Your code here
        pass

# Test with your D455 stream
if __name__ == "__main__":
    matcher = SimpleFeatureMatcher()
    # TODO: Capture two frames from D455
    # TODO: Match and visualize
```

**Learning Goal**: Understand what RTAB-Map does under the hood!

---

### Part 6: Comparison & Analysis (15 min)

**Discussion Questions**:

1. **EKF-SLAM vs Visual SLAM**:
   - What are the key differences?
   - When would you use each?
   
2. **Scalability**:
   - EKF: O(n²) with landmarks
   - Visual: How does RTAB-Map handle thousands of features?
   
3. **Your Research**:
   - How could you add RFID landmarks to RTAB-Map?
   - Where would lifelong learning fit?

**Whiteboard Session**: Design your RFID + Visual fusion system

---

## Homework Before Next Session

### 1. Install Software
```bash
# RTAB-Map
sudo apt install ros-jazzy-rtabmap-ros

# RealSense
sudo apt install ros-jazzy-realsense2-camera

# Visualization tools
sudo apt install ros-jazzy-rviz2
```

### 2. Test D455
```bash
# Verify camera works
ros2 launch realsense2_camera rs_launch.py
ros2 topic list | grep camera
```

### 3. Download Datasets
- TUM RGB-D: https://vision.in.tum.de/data/datasets/rgbd-dataset
- Download at least "freiburg1_xyz" (small, 30 seconds)

### 4. Review Concepts
- Watch: "Visual SLAM Tutorial" by Cyrill Stachniss (YouTube)
- Read: RTAB-Map paper (optional but recommended)

### 5. Prepare Questions
Think about:
- How to integrate RFID with visual features?
- What makes a good visual landmark?
- How to handle dynamic objects?

---

## Materials Needed

### Hardware:
- ✅ Intel RealSense D455
- ✅ TurtleBot3 (optional, for mobile testing)
- ✅ Laptop with ROS 2 Jazzy

### Software:
- ✅ RTAB-Map ROS 2 package
- ✅ OpenCV (for coding exercises)
- ✅ RViz2 (visualization)

### Datasets:
- ✅ TUM RGB-D benchmark
- ✅ Your own recordings

---

## Learning Outcomes

By end of session, you will:

1. ✅ **Understand** visual feature detection and matching
2. ✅ **Run** RTAB-Map with D455 camera
3. ✅ **Implement** basic feature matcher in Python
4. ✅ **Compare** classical vs visual SLAM approaches
5. ✅ **Design** your RFID + Visual fusion architecture
6. ✅ **Evaluate** SLAM performance on different datasets

---

## Success Criteria

- [ ] RTAB-Map running on D455 live feed
- [ ] Created 4 test datasets (easy/medium/hard/loop)
- [ ] Implemented feature matcher code
- [ ] Analyzed loop closure detection
- [ ] Designed RFID fusion approach

---

## Next Steps After This Session

**Phase 5: Sensor Fusion** (Your Novel Contribution!)
- Integrate RFID reader with RTAB-Map
- Implement confidence-based landmark selection
- Test on TurtleBot3 with both sensors

**Phase 6: Lifelong Learning**
- Implement map update strategies
- Handle dynamic environments
- Continuous learning from new data

**Phase 7: Drone Integration**
- Port to Skydio X10 platform
- Aerial-ground map fusion
- Real-world deployment

---

## Tips for Success

1. **Don't rush**: Understanding > completing all exercises
2. **Ask questions**: Interrupt me anytime
3. **Experiment**: Try breaking things to learn
4. **Document**: Take notes on what works/doesn't
5. **Connect**: Relate to your Phase 1-3 knowledge

---

## Emergency Backup Plan

If D455 has issues:
- Use webcam for feature detection exercises
- Use TUM datasets exclusively
- Focus more on theory and code

If RTAB-Map won't install:
- Use ORB-SLAM3 instead
- Or focus on feature detection only
- Save full SLAM for next session

---

**Estimated Time**: 3 hours
**Difficulty**: Intermediate (building on Phase 3 knowledge)
**Fun Factor**: High (real camera, real-time SLAM!)

See you next session! 🚀📸
