# Technical Notes - SLAM Algorithms

## Phase 1: Occupancy Grid Mapping

### Core Algorithm
**Log-Odds Update**:
```cpp
log_odds[cell] += log(p/(1-p))  // Numerical stability
```

**Bresenham Line Tracing**: Integer-only arithmetic for efficiency
- Mark cells along ray as free (p_miss = 0.4)
- Mark endpoint as occupied (p_hit = 0.7)

**Threshold Conversion**:
- Free: < 25% probability
- Occupied: > 65% probability  
- Unknown: -1

**World-to-Grid Transform**:
```cpp
grid_idx = (world_pos - origin) / resolution
```

### Key Insights
- Memory usage: ~36MB for 2000×2000 grid
- Origin shift by resolution/2 prevents alignment issues
- Use rounding (not floor) for grid conversion

---

## Phase 2: Hector SLAM (Scan Matching)

### Gauss-Newton Optimization

**Objective Function**:
```
f(x) = Σ M(S_i(x))²  // Sum of squared map values at scan points
```

**Update Rule**:
```
x_{k+1} = x_k - (H^T H)^{-1} H^T f
```
Where H is Jacobian of map values w.r.t. pose

### Critical Challenges & Solutions

1. **Initial Map Building**: Need 10-20 scans before scan matching
2. **Score Threshold**: Accept matches with score > 0.1
3. **Continuous Probabilities**: Required for gradients
4. **Parameter Sensitivity**: Environment-specific tuning crucial

### Why Scan Matching Failed

- Gauss-Newton needs good initial guess
- Sparse maps lack gradients
- Correlative scan matching more robust
- Production systems use multi-resolution pyramids

---

## Phase 3: EKF-SLAM

### State Representation
```
μ = [robot_x, robot_y, robot_θ, lm1_x, lm1_y, lm2_x, lm2_y, ...]
```
State grows as landmarks discovered!

### Covariance Matrix Structure
```
P = [ P_robot      P_robot_lm  ]
    [ P_lm_robot   P_landmark  ]
```
Captures correlations between robot and landmarks

### Motion Model
```cpp
x' = x + v*cos(θ)*dt
y' = y + v*sin(θ)*dt
θ' = θ + ω*dt
```

### Key Jacobians

**G (State Propagation)**:
```
G = [ 1    0   -v*sin(θ)*dt ]
    [ 0    1    v*cos(θ)*dt ]
    [ 0    0    1           ]
```
How pose uncertainty propagates

**V (Control Noise)**:
```
V = [ cos(θ)*dt    0  ]
    [ sin(θ)*dt    0  ]
    [ 0            dt ]
```
How motor noise affects position

### Prediction-Correction Cycle

**Prediction** (increases uncertainty):
```cpp
μ = motionModel(μ, u)
P = F*P*F^T + V*M*V^T
```

**Correction** (decreases uncertainty):
```cpp
K = P*H^T*(H*P*H^T + Q)^{-1}  // Kalman gain
μ = μ + K*(z - h(μ))          // State update
P = (I - K*H)*P               // Covariance update
```

### Critical Insights

1. **Heading Error Compounds**: 0.1 rad error → 1m position error after 10m travel
2. **Landmarks Become Anchors**: After multiple observations, landmarks more certain than robot
3. **Correlations Matter**: Observing one landmark helps localize everything
4. **O(n²) Complexity**: Covariance matrix grows quadratically with landmarks

---

## Comparison: Grid vs Feature-Based SLAM

| Aspect | Grid-Based (Hector) | Feature-Based (EKF) |
|--------|---------------------|---------------------|
| **Map Type** | Occupancy grid | Sparse landmarks |
| **Scalability** | Memory limited | Computationally limited |
| **Best For** | Dense environments | Sparse, structured |
| **Uncertainty** | Implicit | Explicit (covariance) |
| **Loop Closure** | Difficult | Natural |
| **Complexity** | O(mn) grid size | O(n²) landmarks |

---

## Mathematical Foundations

### Extended Kalman Filter
- **Linearization**: Use Jacobians for non-linear models
- **Assumption**: Gaussian noise and uncertainty
- **Limitation**: Single hypothesis (unimodal)

### Particle Filter (Future)
- **Advantage**: Multi-modal distributions
- **Method**: Sample-based representation
- **Trade-off**: Computational cost vs accuracy

### Graph-Based SLAM (Future)
- **Representation**: Poses as nodes, constraints as edges
- **Optimization**: Bundle adjustment
- **Advantage**: Better for loop closure

---

## Visual SLAM Concepts (Phase 4)

### Feature Detection
- **ORB**: Oriented FAST and Rotated BRIEF
- **Invariances**: Rotation, scale (limited), illumination
- **Matching**: Hamming distance for binary descriptors

### Essential Matrix
- **5-point algorithm**: Minimum for calibrated cameras
- **Decomposition**: Yields R and t (up to scale)
- **Ambiguity**: 4 solutions, need cheirality check

### PnP (Perspective-n-Point)
- **Problem**: Find pose from 3D-2D correspondences
- **Methods**: P3P, EPnP, DLS
- **RANSAC**: Robust to outliers

---

## Implementation Best Practices

1. **Coordinate Frames**
   - Always track frame transformations
   - Use TF2 for ROS systems
   - Robot frame vs world frame vs sensor frame

2. **Numerical Stability**
   - Use log-odds for probabilities
   - Normalize quaternions regularly
   - Check matrix conditioning

3. **Performance**
   - Profile before optimizing
   - Cache expensive computations
   - Use appropriate data structures

4. **Debugging SLAM**
   - Visualize everything (poses, landmarks, uncertainty)
   - Log key metrics (innovation, match scores)
   - Test with simple scenarios first

---

## Common Pitfalls & Solutions

### Problem: Scan matching doesn't converge
**Solution**: Build initial map, tune parameters, use correlative matching

### Problem: EKF becomes inconsistent
**Solution**: Check Jacobians, ensure positive definite covariance

### Problem: Drift accumulates
**Solution**: Loop closure, external references (GPS, RFID)

### Problem: Dynamic objects
**Solution**: Outlier rejection, separate static/dynamic maps

---

Last Updated: 2025-10-02 | Phase 3 Complete
