---
title: RViz Visualization for VSLAM Debugging
chapter: 8
lesson: 6
proficiency_level: B2
learning_objectives:
  - Configure RViz displays for VSLAM diagnostics
  - Interpret feature tracking visualization (density, persistence)
  - Use trajectory paths to identify drift patterns
  - Debug VSLAM failures through real-time visualization
estimated_time: 90 minutes
skills:
  vslam-visualization:
    proficiency: B2
generated_by: content-implementer v1.1.0
source_spec: specs/book/part-3/chapter-08-plan.md
created: 2025-12-17
---

# RViz Visualization for VSLAM Debugging

VSLAM failures are invisible in log files—"Frame 127: Tracking lost" doesn't tell you *why* tracking failed. But visualization reveals the truth: feature distribution, trajectory divergence, loop closure edges, map structure. RViz transforms opaque VSLAM behavior into interpretable diagnostics.

This lesson teaches you to "see" VSLAM through RViz, building diagnostic skills essential for production deployment.

## RViz Basics for VSLAM

### Essential Displays for VSLAM Debugging

Launch RViz and configure these displays:

```bash
rviz2
```

**Display 1: Camera Image with Features**
- Type: `Image`
- Topic: `/visual_slam/tracking/image_with_features`
- Purpose: See which image regions contain detected features
- Diagnostic: Sparse features = tracking will fail

**Display 2: SLAM Trajectory**
- Type: `Path`
- Topic: `/visual_slam/tracking/slam_path`
- Purpose: Visualize estimated robot trajectory
- Diagnostic: Diverging path = accumulated drift

**Display 3: Feature Points 3D**
- Type: `PointCloud2`
- Topic: `/visual_slam/map/points`
- Purpose: View mapped 3D features
- Diagnostic: Sparse/noisy cloud = poor map quality

**Display 4: Pose Graph**
- Type: `MarkerArray`
- Topic: `/visual_slam/map/pose_graph`
- Purpose: See keyframes and loop closure edges
- Diagnostic: No red edges = loop closure not working

**Display 5: TF Frames**
- Type: `TF`
- Frames: `map`, `odom`, `base_link`, `camera_link`
- Purpose: Understand coordinate frame relationships
- Diagnostic: Missing transforms = configuration error

**Save this configuration**: File → Save Config As → `~/vslam_debug.rviz`

Next time: `rviz2 -d ~/vslam_debug.rviz`

## Interpreting Feature Visualization

### Healthy Feature Distribution

Run VSLAM on your textured environment bag:

```bash
# Terminal 1
ros2 launch ~/isaac_vslam_launch.py

# Terminal 2
ros2 bag play humanoid_walk_30s

# Terminal 3
rviz2 -d ~/vslam_debug.rviz
```

**In RViz Image display, observe**:

**Good tracking** (textured office):
```
+----------------------------------+
|  •  • •  •    •   •    •  •  •  |  <-- Features (green dots) distributed across image
|     •    •  •  •    •        •  |
|  •       •     •  •    • •      |
|    • •      •     •   •     •   |
|  •    •   •   •      •    •     |
+----------------------------------+
```

- **Dense coverage**: 400-500 features visible
- **Distributed**: Features across entire image (not clustered)
- **Persistent**: Same features visible across consecutive frames (tracking successful)

**Poor tracking** (low-texture hallway):
```
+----------------------------------+
|                                  |  <-- Mostly blank (white wall, no features)
|                                  |
|  •           • •                 |  <-- Sparse features (door frame, light switch)
|                                  |
|                    •  •          |  <-- &lt;100 features total
+----------------------------------+
```

- **Sparse**: &lt;100 features detected
- **Clustered**: Features only in small regions (door, window)
- **Unstable**: Features appear/disappear between frames (tracking loss imminent)

**Diagnostic insight**: Feature distribution predicts tracking stability *before* failure occurs. If you see clustering, reduce robot speed or adjust detector_threshold.

### Feature Persistence (Tracking Quality)

Enable feature trails in RViz:
- Image display → History Policy: `Keep All`
- Color by Age: Newer = bright green, older = faded green

**Good tracking**: Features show short trails (5-10 pixels across frames)
- Trails smooth and consistent direction
- Most features tracked for 10+ frames

**Failing tracking**: Features "jump" (no smooth trails)
- Dots disappear and reappear at different locations
- Few features persist beyond 2-3 frames

**Why this matters**: Visual odometry requires feature **correspondence** across frames. If features don't persist, motion estimation has no data.

## Trajectory Visualization for Drift Detection

### Comparing SLAM Path to Ground Truth

Add second Path display for ground truth:

**Display 6: Ground Truth Path**
- Type: `Path`
- Topic: `/tf_poses` (computed from `/tf` transforms)
- Color: Blue
- Purpose: Compare VSLAM estimate to ground truth

Create script `~/publish_gt_path.py`:

```python
#!/usr/bin/env python3
"""Publish ground truth path from tf for comparison."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

class GroundTruthPathPublisher(Node):
    def __init__(self):
        super().__init__('gt_path_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path = Path()
        self.path.header.frame_id = 'map'

        self.path_pub = self.create_publisher(Path, '/tf_poses', 10)
        self.create_timer(0.1, self.publish_path)

    def publish_path(self):
        """Query tf and append to path."""
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )

            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation

            self.path.poses.append(pose)
            self.path.header.stamp = self.get_clock().now().to_msg()

            self.path_pub.publish(self.path)
        except:
            pass

def main():
    rclpy.init()
    node = GroundTruthPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run comparison**:

```bash
# Terminal 4: Publish ground truth
python3 ~/publish_gt_path.py
```

**In RViz, observe two paths**:
- **Green** (VSLAM): Estimated trajectory
- **Blue** (ground truth): Actual trajectory from simulation

**Drift patterns**:

**1. Linear drift** (odometry bias):
```
Blue (GT):  ──────────────▶  Straight line
Green (VSLAM): ───────────────▶  Parallel but offset
                      ↑ Constant offset (e.g., 0.1m to the right)
```
**Cause**: Systematic bias in motion estimation (e.g., camera calibration error)
**Fix**: Recalibrate stereo cameras (check baseline distance)

**2. Rotational drift**:
```
Blue (GT):  ──────────────▶  Straight
Green (VSLAM): ─────────────▶  Curves to the side
                       Rotation accumulates
```
**Cause**: Bias in yaw estimation (rotation about vertical axis)
**Fix**: Check camera alignment (cameras not parallel) or improve feature distribution

**3. Sudden divergence**:
```
Blue (GT):  ──────┐
Green (VSLAM): ────────┘  Sudden jump at specific frame
```
**Cause**: Tracking loss at specific frame, followed by incorrect relocalization
**Fix**: Debug feature tracking at failure frame (likely motion blur or texture loss)

**4. Loop drift** (corrected by loop closure):
```
Before loop closure:
Blue (GT):  ●────────●  Loop closes
Green (VSLAM): ●────────▷  Open loop (drift)

After loop closure:
Green (VSLAM): ●────────●  Closes (trajectory adjusted)
```
**Cause**: Normal drift, corrected by loop closure detection
**Fix**: None needed (system working as designed)

## Map Point Cloud Visualization

### Understanding Map Structure

The 3D point cloud represents mapped features.

**PointCloud2 display settings**:
- Color Transform: `Intensity` (shows feature strength)
- Size (m): 0.02 (small dots)
- Decay Time: 60s (keep points visible)

**Healthy map** (office environment):
```
Top view:
    • •   •  •    •  •     <-- Desk outline
  •           •  •         <-- Chair
        • • •               <-- Wall edge
  •  •     •     •         <-- Window frame
```

- **Structured**: Points form recognizable shapes (walls, furniture)
- **Sparse but sufficient**: 2000-5000 points (not 50,000+)
- **Low noise**: Points cluster at real surfaces (not floating in mid-air)

**Degraded map** (low-texture or tracking issues):
```
Top view:
      • •      •            <-- Very sparse
  •                •        <-- Scattered randomly
         •    •             <-- No structure visible
    •      •         •      <-- &lt;500 points total
```

- **Too sparse**: &lt;500 points (insufficient for navigation)
- **Noisy**: Points not aligned to surfaces (depth estimation errors)
- **No structure**: Cannot identify walls or obstacles

**Diagnostic**: Map quality directly indicates VSLAM health. Degraded map = re-tune parameters or improve environment.

## Pose Graph and Loop Closure Edges

### Visualizing Keyframes and Connections

**Pose graph components** (MarkerArray display):
- **Green spheres**: Keyframes (selected frames added to map)
- **Green lines**: Odometry edges (sequential connections)
- **Red lines**: Loop closure edges (non-sequential connections)

**Loop closure detection in real-time**:

1. Watch RViz while robot navigates loop
2. Initially: Only green lines (sequential trajectory)
3. At loop detection: Red line suddenly appears connecting distant keyframes
4. Green trajectory adjusts (all keyframes shift slightly to satisfy loop constraint)

**Example**:

```
Frame sequence: 1 → 2 → 3 → ... → 398 → 399 → 400

Before loop closure:
  1───2───3─ ─ ─ ─398───399───400  (only green lines)
  ●   ●   ●         ●     ●     ●

After loop closure (Frame 400 matches Frame 1):
  1───2───3─ ─ ─ ─398───399───400  (green lines)
  ●═══●   ●         ●     ●═══●    (red line: 1↔400)
```

**False loop closure visualization**:

```
  1───2───3───4───5
  ●   ●   ●   ●═══●  <-- Red line connects Frame 2 ↔ Frame 5

  (But robot never returned to Frame 2's location)
  (Contradictory constraint breaks map geometry)
```

**After false loop, map distorts**:
- Walls bend at impossible angles
- Point cloud shows overlapping surfaces
- Trajectory has non-physical kinks

**Diagnostic**: Watch for red lines. If they appear but map degrades, that loop was false positive → increase `loop_closure_threshold` or `geometric_inlier_threshold`.

## Real-Time Failure Diagnosis

### Scenario-Based Debugging with RViz

**Scenario 1: Tracking Lost During Fast Rotation**

**Symptoms in RViz**:
- Image display: Features detected but jumping between frames (no trails)
- Path display: Trajectory stops updating (robot moving but SLAM frozen)
- Point cloud: No new points added during rotation

**Diagnosis**: Motion blur → feature matching failure
**Fix**: Reduce rotation speed or increase detector sensitivity

---

**Scenario 2: Drift in Long Hallway**

**Symptoms in RViz**:
- Path display: SLAM trajectory (green) diverging from ground truth (blue)
- Image display: Sparse features (white walls)
- Point cloud: Very few points added in hallway section

**Diagnosis**: Low-texture environment → insufficient features for accurate odometry
**Fix**: Lower `detector_threshold` (Lesson 3) or add texture to environment

---

**Scenario 3: Map Suddenly Distorts**

**Symptoms in RViz**:
- Pose graph: New red line (loop closure) appears
- Point cloud: Walls suddenly overlap or bend
- Path: Trajectory kinks at loop closure frame

**Diagnosis**: False loop closure → incorrect constraint
**Fix**: Increase `loop_closure_threshold` to reduce false positives

---

**Scenario 4: No Loop Closure Detected**

**Symptoms in RViz**:
- Path: SLAM and ground truth diverge at loop closure point
- Pose graph: No red lines even though robot returned to start
- Image display: Features detected and tracking stable

**Diagnosis**: Loop closure threshold too high OR lighting changed between visits
**Fix**: Lower `loop_closure_threshold` (Lesson 5) or investigate environmental changes

## Creating Custom RViz Configurations

Save different RViz setups for different debugging scenarios.

### Configuration 1: Feature Tracking Debug

**Focus**: Diagnose feature detection and tracking issues

**Displays**:
- Image with features (large view)
- Feature count graph (custom plugin)
- Small path for trajectory reference

**Use case**: Debugging low-texture failures (Lesson 3)

### Configuration 2: Drift Analysis

**Focus**: Compare SLAM vs ground truth trajectories

**Displays**:
- Two Path displays (SLAM and ground truth, contrasting colors)
- TF frames
- No image (not needed for trajectory analysis)

**Use case**: Quantifying drift, tuning parameters

### Configuration 3: Loop Closure Validation

**Focus**: Verify loop closure detection and map consistency

**Displays**:
- Pose graph with keyframes and edges
- Point cloud (to see map distortion after false loops)
- Path (to see trajectory correction)

**Use case**: Tuning `loop_closure_threshold` (Lesson 5)

**Save each**: File → Save Config As → `~/vslam_[scenario].rviz`

## Checkpoint

Verify your RViz diagnostic skills:

- [ ] You can identify sparse vs dense feature distribution
- [ ] You can detect drift by comparing SLAM path to ground truth
- [ ] You can interpret map point cloud quality
- [ ] You can spot loop closures through pose graph red edges
- [ ] You can diagnose VSLAM failures from visualization patterns

## Try With AI

**Prompt 1: Visualization Interpretation**
```
I'm observing my VSLAM in RViz and see this pattern:
- Features: ~500 detected, well-distributed across image
- Trajectory: SLAM path diverges 15cm to the right of ground truth over 10m
- Point cloud: Structured, shows wall/furniture clearly
- Pose graph: Only green edges (no loop closures yet)

Based on this visualization, is my VSLAM performing well? What's causing the 15cm drift?
```

**Prompt 2: Custom Display Design**
```
I want to create a custom RViz display showing "feature tracking health" as a real-time score (0-100%). This would combine:
- Feature count (should be 400-600)
- Feature persistence (% tracked across 5+ frames)
- Feature distribution (coverage across image regions)

How would I implement this as an RViz plugin? Provide architecture overview and key metrics.
```

**Prompt 3: Failure Pattern Recognition**
```
Teach me to recognize VSLAM failure *before* it happens by watching RViz. What early warning signs in feature distribution, trajectory, or point cloud predict imminent tracking loss in the next 1-2 seconds?
```

**Reflection**: Visualization transforms VSLAM from a black box into an observable system. Which visualization surprised you most in its diagnostic power—feature distribution, trajectory comparison, or pose graph edges?

---

Next lesson: Encode VSLAM debugging patterns into a reusable skill for future robotics projects (Layer 3: Intelligence Design).
