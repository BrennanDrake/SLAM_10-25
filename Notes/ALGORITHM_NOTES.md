# SLAM Algorithm Implementation Notes

## Occupancy Grid Mapping Algorithm

### Core Concepts

#### 1. Probabilistic Occupancy Grid
The occupancy grid represents the environment as a discrete grid where each cell stores the probability of being occupied.

**Key Properties**:
- Each cell has value in [0, 1] representing P(occupied)
- Unknown cells initialized to 0.5 (maximum uncertainty)
- Updated using Bayesian inference with sensor measurements

#### 2. Log-Odds Representation
Instead of storing probabilities directly, we use log-odds for numerical stability:

```
log_odds = log(p / (1 - p))
p = 1 / (1 + exp(-log_odds))
```

**Advantages**:
- Avoids numerical underflow/overflow
- Updates are simple additions
- Symmetric around zero (0 = unknown)

**Update Rule**:
```cpp
// For occupied cell (hit)
log_odds[cell] += log(prob_hit / (1 - prob_hit))

// For free cell (miss)
log_odds[cell] += log(prob_miss / (1 - prob_miss))
```

#### 3. Bresenham's Line Algorithm
Used for ray tracing from robot position to scan endpoints.

**Implementation**:
```cpp
void bresenham_line(int x0, int y0, int x1, int y1, vector<pair<int,int>>& cells) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        cells.push_back({x0, y0});
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}
```

**Why Bresenham?**:
- Integer arithmetic only (fast)
- Produces optimal line approximation
- No gaps in traced line

### Sensor Model

#### Laser Range Finder Model
For each laser beam, we model:
1. **Hit probability**: P(z|m,x) where z is measurement, m is map, x is pose
2. **Measurement noise**: Gaussian with σ = 0.01m
3. **Max range readings**: Treated as no detection (don't mark as occupied)

**Parameters**:
- `prob_hit = 0.7`: Probability of correct detection
- `prob_miss = 0.4`: Probability of false negative
- `prob_prior = 0.5`: Prior probability (unknown)

#### Inverse Sensor Model
Maps from sensor measurement to occupancy probability:

```
For each scan ray:
  1. Trace ray from robot to endpoint
  2. Mark all cells along ray as "free" (except last)
  3. Mark endpoint cell as "occupied" (if not max range)
  4. Update using log-odds
```

### Map Update Algorithm

#### Complete Update Cycle
```python
def update_map(scan_msg):
    # 1. Get robot pose (simplified: assume origin)
    robot_x, robot_y, robot_theta = 0, 0, 0
    
    # 2. Process each laser ray
    for i, range in enumerate(scan_msg.ranges):
        if not valid_range(range):
            continue
            
        # 3. Calculate ray endpoint
        angle = robot_theta + scan_msg.angle_min + i * scan_msg.angle_increment
        end_x = robot_x + range * cos(angle)
        end_y = robot_y + range * sin(angle)
        
        # 4. Ray trace to find cells
        cells = bresenham_line(robot_x, robot_y, end_x, end_y)
        
        # 5. Update cells
        for cell in cells[:-1]:  # All but last are free
            log_odds[cell] += log_odds_miss
        
        if range < max_range * 0.95:  # Last cell is occupied
            log_odds[cells[-1]] += log_odds_hit
```

#### Occupancy Threshold Conversion
Convert log-odds to occupancy grid values:

```cpp
if (probability < 0.25) {
    grid[cell] = 0;    // Free
} else if (probability > 0.65) {
    grid[cell] = 100;  // Occupied  
} else {
    grid[cell] = -1;   // Unknown
}
```

### Coordinate Transformations

#### World to Grid
```cpp
bool world_to_grid(double world_x, double world_y, int& grid_x, int& grid_y) {
    grid_x = (world_x - origin_x) / resolution;
    grid_y = (world_y - origin_y) / resolution;
    return (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height);
}
```

#### Grid to World
```cpp
void grid_to_world(int grid_x, int grid_y, double& world_x, double& world_y) {
    world_x = grid_x * resolution + origin_x;
    world_y = grid_y * resolution + origin_y;
}
```

### Optimization Techniques

#### 1. Sparse Representation
Instead of storing full grid, only store occupied/free cells:
```cpp
unordered_map<pair<int,int>, float> sparse_grid;
```

#### 2. Update Limiting
Only update cells that change significantly:
```cpp
if (abs(new_log_odds - old_log_odds) > threshold) {
    log_odds[cell] = new_log_odds;
    publish_update = true;
}
```

#### 3. Bounded Log-Odds
Prevent infinite certainty:
```cpp
log_odds[cell] = clamp(log_odds[cell], -max_log_odds, max_log_odds);
```

#### 4. Lazy Evaluation
Only convert log-odds to probability when needed:
```cpp
// Store as log-odds internally
// Convert only for publishing/visualization
```

### Advanced Features (Future Implementation)

#### 1. Dynamic Map Updates
Handle moving objects by decay:
```cpp
// Decay toward prior over time
log_odds[cell] *= decay_factor;
if (abs(log_odds[cell]) < threshold) {
    log_odds[cell] = 0;  // Reset to unknown
}
```

#### 2. Multi-Resolution Grids
Use quadtree or octree for variable resolution:
- High resolution near robot
- Lower resolution for distant areas
- Adaptive subdivision based on information content

#### 3. Frontier Detection
Identify exploration targets:
```cpp
// Frontier = free cells adjacent to unknown cells
vector<Point> find_frontiers() {
    vector<Point> frontiers;
    for (each free cell) {
        if (has_unknown_neighbor(cell)) {
            frontiers.push_back(cell);
        }
    }
    return frontiers;
}
```

#### 4. Map Merging
Combine maps from multiple robots:
```cpp
// Weighted average based on confidence
merged_log_odds = (w1 * log_odds1 + w2 * log_odds2) / (w1 + w2);
```

### Performance Metrics

#### Time Complexity
- Single scan update: O(n * m) where n = number of rays, m = average ray length in cells
- Full grid iteration: O(width * height)
- Bresenham line: O(max(dx, dy))

#### Space Complexity
- Dense grid: O(width * height)
- Sparse grid: O(occupied_cells)
- Log-odds array: O(width * height)

#### Typical Performance
- 2000x2000 grid at 0.05m resolution
- 360 laser rays per scan
- Update rate: 10Hz achievable on modern CPU
- Memory: ~36MB for dense representation

### Parameter Tuning Guide

#### Probability Parameters
- **prob_hit** (0.6-0.9): Higher = more confident in obstacles
- **prob_miss** (0.3-0.45): Lower = more confident in free space
- **prob_prior** (0.5): Keep at 0.5 for maximum uncertainty

#### Grid Parameters
- **resolution** (0.01-0.1m): Trade-off between detail and performance
- **grid_size**: Based on environment size and memory constraints
- **origin**: Center grid on expected operation area

#### Threshold Parameters
- **occupied_threshold** (0.6-0.8): Higher = more conservative obstacle detection
- **free_threshold** (0.2-0.4): Lower = more conservative free space marking

### Common Pitfalls and Solutions

1. **Numerical Instability**: Use log-odds instead of raw probabilities
2. **Ray Aliasing**: Ensure Bresenham covers all cells
3. **Max Range Artifacts**: Don't mark max-range readings as obstacles
4. **Coordinate Confusion**: Clearly distinguish grid vs world coordinates
5. **Update Rate**: Balance between map quality and computational load

### Testing and Validation

#### Unit Tests
1. Bresenham line correctness
2. Coordinate transformation accuracy
3. Log-odds conversion
4. Boundary conditions

#### Integration Tests
1. Known environment mapping
2. Loop closure consistency
3. Dynamic object handling
4. Multi-sensor fusion

#### Performance Tests
1. Update rate under load
2. Memory usage over time
3. Accuracy vs ground truth
4. Convergence rate

---
*Last Updated: September 26, 2025*
