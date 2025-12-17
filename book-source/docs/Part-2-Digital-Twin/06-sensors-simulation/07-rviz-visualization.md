---
title: "Lesson 7: RViz Visualization and Sensor Analysis"
chapter: 6
lesson: 7
proficiency_level: B2
learning_objectives:
  - Design comprehensive RViz setups for multi-sensor visualization
  - Create reusable visualization skill for sensor debugging
  - Display raw and processed sensor data simultaneously
  - Debug perception algorithms using visualization
estimated_time: 90 minutes
generated_by: content-implementer v1.0.0
created: 2025-12-16
---

# Lesson 7: RViz Visualization and Sensor Analysis

## Multi-Sensor Visualization Strategy

Effective debugging requires seeing multiple data streams simultaneously:

**Raw sensor layer**: Point clouds, depth images, IMU arrows
**Processing layer**: Detected obstacles, estimated orientation
**Reference layer**: Robot model, coordinate frames, expected boundaries

**Layout strategy**: Arrange panels by information hierarchy

```
┌─────────────────────────────────────────┐
│  3D View (large, center)                │
│  - Robot model with TF frames            │
│  - Point cloud (LiDAR) in color         │
│  - Detected obstacles as markers        │
└─────────────────────────────────────────┘
┌────────────────┬──────────────────┬────┐
│ Depth image    │ RGB camera       │IMU │
│ (raw)          │ (live)           │data│
└────────────────┴──────────────────┴────┘
```

## RViz Configuration File

Create `config/sensor_debug.rviz`:

```yaml
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_common/RobotModel
      Enabled: true
      Links:
        All Links:
          Radius: 0.1

    - Class: rviz_common/PointCloud2
      Name: LiDAR
      Topic: /lidar/points
      Enabled: true
      Style: Points
      Size (Pixels): 2
      Decay Time: 0  # Keep all points

    - Class: rviz_common/Image
      Name: Depth Image
      Topic: /depth_camera/depth
      Enabled: true

    - Class: rviz_common/Image
      Name: RGB Image
      Topic: /camera/rgb/image_raw
      Enabled: true

    - Class: rviz_common/Imu
      Name: IMU
      Topic: /imu/data_raw
      Enabled: true

    - Class: rviz_common/Marker
      Name: Detected Obstacles
      Topic: /obstacles_marker
      Enabled: true

    - Class: rviz_common/TF
      Enabled: true
```

## Creating the Sensor Visualization Skill

This lesson creates a reusable skill for future chapters.

**Skill file**: `.claude/skills/gazebo-sensor-visualization/SKILL.md`

```markdown
# Gazebo Sensor Visualization Skill

## Persona
Think like a robotics engineer debugging perception systems. Your goal is to understand what sensors see and whether processing produces correct results.

## Questions to Guide Visualization Design

1. **What data am I trying to understand?**
   - Raw sensor streams (point clouds, images)?
   - Processed results (obstacles, orientation)?
   - Comparisons (raw vs filtered)?

2. **What should I see in the 3D view?**
   - Robot model with TF frames
   - Point clouds colored by range/intensity/height
   - Detected objects as bounding boxes or markers

3. **How do I check data quality?**
   - Compare multiple sensors simultaneously
   - Watch for noise, artifacts, dropouts
   - Validate algorithm outputs

## Principles

**Clarity**: Each sensor type visually distinct
**Completeness**: Raw and processed data visible
**Performance**: Don't overwhelm rendering
**Consistency**: Standard symbols and colors

## Implementation: Display Patterns

### LiDAR Point Cloud Display
```
Topic: /lidar/points
Style: Points (not squares)
Size: 2-3 pixels (depends on monitor)
Color: Rainbow by range (red=close, blue=far)
Decay: 0 seconds (keep all points)
```

### Depth Camera Display
```
RGB topic: /camera/rgb/image_raw
Depth topic: /camera/depth_registered/image_raw
Layout: Side by side for comparison
Colorize depth: Jet colormap (red=close, blue=far)
```

### IMU Orientation
```
Topic: /imu/data_raw
Display: Axes showing current orientation
Update rate: 100 Hz (fast updates)
Scale: 1.0 meter (large enough to see rotation)
```

### Detected Objects
```
Topic: /detected_obstacles
Style: Cube markers for bounding boxes
Color: Green (good), Red (uncertain)
Size: Proportional to detection confidence
```

## Validation Checklist

- [ ] Can I see all three sensor types simultaneously?
- [ ] Do point clouds appear in correct position (head sensor)?
- [ ] Do depth images update smoothly?
- [ ] Do IMU axes rotate when robot rotates?
- [ ] Are detected objects correctly positioned?
- [ ] Does visualization run at >10 FPS?

## Reuse Instructions

To visualize sensors in new project:
1. Copy this skill
2. Adapt topic names to your nodes
3. Adjust display scales/colors for visibility
4. Extend with project-specific markers
```

## Visualizing Sensor Data Quality Issues

**Common problems and visualization clues**:

| Problem | Visual Indicator |
|---------|-----------------|
| LiDAR not moving | Point cloud stationary, doesn't follow obstacles |
| Depth camera upside down | Image appears inverted or rotated 180° |
| IMU not updating | Axes frozen, don't rotate |
| Bad calibration | Point clouds offset from actual environment |
| Sensor noise too high | Point cloud appears scattered, blurry |
| Missing points | Holes in point cloud, gaps in depth image |

## Processing Visualization Markers

Create ROS 2 node to publish detected obstacles as markers:

```python
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class ObstacleVisualizer(Node):
    def __init__(self):
        super().__init__('obstacle_visualizer')
        self.marker_pub = self.create_publisher(
            MarkerArray, '/obstacles_marker', 10)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.process_lidar, 10)

    def process_lidar(self, msg):
        # Detect obstacles (cluster points)
        obstacles = self.detect_obstacles(msg)

        # Create markers for visualization
        markers = MarkerArray()
        for i, obs in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = obs['x']
            marker.pose.position.y = obs['y']
            marker.pose.position.z = obs['z']

            # Size
            marker.scale.x = 0.1  # Width
            marker.scale.y = 0.1  # Height
            marker.scale.z = 0.1  # Depth

            # Color (green)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7

            markers.markers.append(marker)

        self.marker_pub.publish(markers)

    def detect_obstacles(self, point_cloud):
        # Parse point cloud, cluster nearby points
        points = self.pointcloud2_to_array(point_cloud)

        # Simple clustering: group points close together
        obstacles = []
        for point in points:
            if len(obstacles) == 0:
                obstacles.append([point])
            else:
                found_cluster = False
                for cluster in obstacles:
                    for pt in cluster:
                        dist = np.sqrt(
                            (point[0]-pt[0])**2 +
                            (point[1]-pt[1])**2 +
                            (point[2]-pt[2])**2
                        )
                        if dist < 0.1:  # 10cm threshold
                            cluster.append(point)
                            found_cluster = True
                            break
                    if found_cluster:
                        break
                if not found_cluster:
                    obstacles.append([point])

        # Convert clusters to obstacle centers
        result = []
        for cluster in obstacles:
            if len(cluster) > 5:  # Filter small clusters
                points_array = np.array(cluster)
                center = np.mean(points_array, axis=0)
                result.append({
                    'x': center[0],
                    'y': center[1],
                    'z': center[2],
                    'size': len(cluster)
                })

        return result
```

## Exercise: Create Sensor Debugging Visualization

**Task**: Launch humanoid with all three sensors and create RViz setup showing:

1. Robot model with TF frames
2. Point cloud from LiDAR (colored by range)
3. Depth image from camera
4. IMU orientation axes
5. Detected obstacles as markers

```bash
# Terminal 1: Simulation
ros2 launch humanoid_bringup sim_with_sensors.launch.py

# Terminal 2: Obstacle detection node
ros2 run humanoid_perception obstacle_detector

# Terminal 3: RViz with visualization
ros2 run rviz2 rviz2 -d config/sensor_debug.rviz
```

Verify in RViz:
- [ ] Robot appears in correct pose
- [ ] LiDAR points visible around robot
- [ ] Depth image updates (colors change as camera moves)
- [ ] IMU axes rotate when robot rotates
- [ ] Green boxes appear where obstacles detected

## Try With AI

**Prompt 1: Visualization design**
"I'm debugging a perception system with LiDAR, depth camera, and IMU. What's the best RViz layout to see all three simultaneously? Should I prioritize the 3D view or detailed 2D image windows?"

**Prompt 2: Data quality assessment**
"Looking at my LiDAR point cloud in RViz, I see stripes (regular gaps) in the data. What does this indicate about sensor configuration or data processing?"

**Prompt 3: Marker-based visualization**
"I want to visualize detected obstacles as color-coded boxes (green=confident, yellow=uncertain, red=error). How should I use RViz markers to implement this? What topics should I publish?"

**Expected outcome**: Design effective multi-sensor visualization for debugging and validation.

**Safety note**: High-frequency visualization (LiDAR at 20 Hz with many points) can slow RViz. Use data decay or point size adjustment to maintain responsive rendering.
