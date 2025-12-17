---
title: "Lesson 8: Sensor Data Processing in ROS 2"
chapter: 6
lesson: 8
proficiency_level: B2
learning_objectives:
  - Build reusable sensor processing skill for obstacle detection
  - Implement point cloud and image processing pipelines
  - Fuse multiple sensors for robust perception
  - Design production-ready processing nodes
estimated_time: 90 minutes
generated_by: content-implementer v1.0.0
created: 2025-12-16
---

# Lesson 8: Sensor Data Processing in ROS 2

## Processing Pipeline Architecture

Data flows through layered processing:

```
Raw Sensor Data
    ↓
[LiDAR → Denoise → Downsample → Cluster]
[Depth  → Register → Threshold → Extract planes]
[IMU    → Filter → Drift correct → Estimate pose]
    ↓
Fused Results
    ↓
[Obstacle detection] [Orientation estimation] [Navigation inputs]
```

## Creating the Sensor Processing Skill

**Skill file**: `.claude/skills/gazebo-sensor-processing/SKILL.md`

```markdown
# Gazebo Sensor Processing Skill

## Persona
Think like a robotics engineer building robust perception pipelines that handle noisy real-world sensor data.

## Questions to Guide Processing Design

1. **What information does each sensor provide?**
   - LiDAR: 360° range measurements → Obstacle map
   - Depth: RGB-D images → 3D object detection
   - IMU: Acceleration + angular velocity → Orientation

2. **What processing reduces noise while preserving detail?**
   - LiDAR: Statistical filtering, clustering
   - Depth: Bilateral filtering (preserves edges), outlier removal
   - IMU: Complementary filtering, sensor fusion

3. **How do I fuse multiple sensors?**
   - Early fusion: Combine raw data before processing
   - Late fusion: Process separately, combine results

## Principles

**Robustness**: Handle sensor failures, missing data
**Timeliness**: Process faster than sensor update rate
**Accuracy**: Validate outputs match expectations
**Composability**: Nodes chain together easily

## Implementation: Processing Patterns

### LiDAR Processing Pipeline
```
1. Denoise: Remove isolated points (likely errors)
2. Downsample: Every Nth point (reduce compute)
3. Ground removal: Filter out floor plane
4. Clustering: Group nearby points into objects
5. Publishing: Send clusters as markers or PointCloud2
```

### Depth Image Processing
```
1. Bilateral filter: Denoise while preserving edges
2. Registration: Align RGB and depth
3. Plane detection: Find floor, walls
4. Object extraction: Isolated foreground objects
```

### IMU Processing
```
1. Bias estimation: Compute sensor offset
2. Complementary filtering: Fuse gyro + accel
3. Drift correction: Use gravity as reference
4. Orientation estimation: Output quaternion
```

## Validation Checklist

- [ ] LiDAR processing runs faster than sensor rate?
- [ ] Detected obstacles match visual observation in RViz?
- [ ] Depth processing outputs are noise-free?
- [ ] IMU orientation doesn't drift over time?
- [ ] System handles missing messages gracefully?
- [ ] Processing uses <50% CPU on Jetson/robot platform?

## Reuse Instructions

To process sensors in new project:
1. Copy this skill
2. Adapt to your sensor types/update rates
3. Validate outputs in RViz
4. Tune filtering thresholds for your environment
```

## Implementing Obstacle Detection

Complete ROS 2 node:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import struct
from sklearn.cluster import DBSCAN

class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')

        self.obstacle_pub = self.create_publisher(
            MarkerArray, '/detected_obstacles', 10)
        self.filtered_cloud_pub = self.create_publisher(
            PointCloud2, '/lidar_filtered', 10)

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.lidar_callback, 10)

        self.get_logger().info('Obstacle detector started')

    def lidar_callback(self, msg):
        # Convert to numpy array
        points = self.pointcloud2_to_array(msg)

        if len(points) == 0:
            return

        xyz = np.array([[p[0], p[1], p[2]] for p in points])

        # Step 1: Remove ground plane (y > -0.3, assuming flat floor)
        ground_mask = xyz[:, 2] > -0.3
        above_ground = xyz[~ground_mask]

        # Step 2: Remove ceiling/high points
        max_height_mask = above_ground[:, 2] < 2.0
        filtered_points = above_ground[max_height_mask]

        # Step 3: Cluster points into obstacles (DBSCAN)
        if len(filtered_points) > 10:
            clusters = DBSCAN(eps=0.1, min_samples=5).fit(filtered_points)
            labels = clusters.labels_

            # Step 4: Create markers
            markers = self.create_obstacle_markers(filtered_points, labels)

            self.obstacle_pub.publish(markers)
            self.get_logger().info(f'Obstacles detected: {len(set(labels)) - 1}')

    def create_obstacle_markers(self, points, labels):
        markers = MarkerArray()
        unique_labels = set(labels)
        unique_labels.discard(-1)  # Remove noise cluster

        for label_id in unique_labels:
            cluster_points = points[labels == label_id]

            # Compute cluster center
            center = np.mean(cluster_points, axis=0)

            # Create marker
            marker = Marker()
            marker.header.frame_id = "lidar"
            marker.id = int(label_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])

            # Size proportional to cluster
            size = 0.05 + 0.1 * len(cluster_points) / 100
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = size

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            markers.markers.append(marker)

        return markers

    def pointcloud2_to_array(self, msg):
        """Convert PointCloud2 to numpy array"""
        points = []
        x_offset = None
        y_offset = None
        z_offset = None

        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset

        if x_offset is None or y_offset is None or z_offset is None:
            return []

        for i in range(msg.width * msg.height):
            offset = i * msg.point_step
            x = struct.unpack('f', msg.data[offset+x_offset:offset+x_offset+4])[0]
            y = struct.unpack('f', msg.data[offset+y_offset:offset+y_offset+4])[0]
            z = struct.unpack('f', msg.data[offset+z_offset:offset+z_offset+4])[0]
            points.append([x, y, z])

        return np.array(points)

def main(args=None):
    rclpy.init(args=args)
    detector = LidarObstacleDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Implementing Orientation Estimation

IMU fusion node:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation

class ImuOrientationEstimator(Node):
    def __init__(self):
        super().__init__('imu_orientation_estimator')

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Complementary filter parameters
        self.alpha = 0.98  # Gyro weight (fast), 1-alpha = accel weight (slow)

        # State
        self.q_fused = Rotation.from_quat([0, 0, 0, 1])  # Identity
        self.last_time = None

    def imu_callback(self, msg):
        # Current time
        current_time = (msg.header.stamp.sec +
                       msg.header.stamp.nanosec / 1e9)

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        # Extract measurements
        ax, ay, az = (msg.linear_acceleration.x,
                     msg.linear_acceleration.y,
                     msg.linear_acceleration.z)
        wx, wy, wz = (msg.angular_velocity.x,
                     msg.angular_velocity.y,
                     msg.angular_velocity.z)

        # Gyro integration (fast but drifts)
        q_gyro = self.q_fused.copy()
        gyro_quat = self.gyro_to_quaternion_delta(wx, wy, wz, dt)
        q_gyro = q_gyro * gyro_quat

        # Accelerometer-based orientation (noisy but reference)
        q_accel = self.accelerometer_to_quaternion(ax, ay, az)

        # Complementary filter: alpha*gyro + (1-alpha)*accel
        q_accel_quat = Rotation.from_quat(q_accel)
        q_gyro.as_quat_slerp(q_accel_quat, 1 - self.alpha)

        self.q_fused = q_gyro.slerp(q_accel_quat, 1 - self.alpha)

        # Broadcast transform
        self.broadcast_orientation(msg.header.frame_id, self.q_fused)

    def gyro_to_quaternion_delta(self, wx, wy, wz, dt):
        """Convert angular velocity to quaternion delta"""
        angle = np.sqrt(wx**2 + wy**2 + wz**2) * dt

        if angle < 1e-6:
            return Rotation.from_quat([0, 0, 0, 1])

        axis = np.array([wx, wy, wz]) / np.sqrt(wx**2 + wy**2 + wz**2)
        return Rotation.from_rotvec(axis * angle)

    def accelerometer_to_quaternion(self, ax, ay, az):
        """Estimate orientation from acceleration (gravity reference)"""
        # Normalize
        a_norm = np.sqrt(ax**2 + ay**2 + az**2)
        ax, ay, az = ax/a_norm, ay/a_norm, az/a_norm

        # Assume gravity is (0, 0, -9.81)
        # Find quaternion that rotates (0, 0, -1) to (ax, ay, az)
        r = Rotation.from_rotvec([ay, -ax, 0]) if (ax**2 + ay**2) > 0.01 else Rotation.from_quat([0,0,0,1])

        return r.as_quat()

    def broadcast_orientation(self, frame_id, rotation):
        """Broadcast as TF2 transform"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "imu_frame"

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        quat = rotation.as_quat()
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    estimator = ImuOrientationEstimator()
    rclpy.spin(estimator)
    estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise: Build Processing Pipeline

**Task**: Create sensor processing node combining all three sensors.

```bash
# 1. Implement obstacle detection node
# 2. Implement orientation estimation node
# 3. Launch with humanoid simulator
ros2 launch humanoid_bringup sim_with_sensors.launch.py

# 4. Run your nodes
ros2 run humanoid_perception obstacle_detector
ros2 run humanoid_perception orientation_estimator

# 5. Visualize in RViz
ros2 run rviz2 rviz2 -d config/sensor_debug.rviz

# 6. Verify:
ros2 topic echo /detected_obstacles
ros2 topic echo /tf
```

## Try With AI

**Prompt 1: Processing pipeline design**
"I have LiDAR, depth camera, and IMU publishing data. Should I process each independently or fuse them together? What's the best architecture?"

**Prompt 2: Noise handling**
"My obstacle detection uses point cloud clustering (DBSCAN), but it produces false positives with noisy LiDAR data. Should I filter the point cloud before clustering, or adjust DBSCAN parameters?"

**Prompt 3: Performance optimization**
"My processing node receives 720×32=23,040 LiDAR points at 10 Hz (230k points/sec). How should I structure the code to process fast enough for real-time control? Should I downsample?"

**Expected outcome**: Understand sensor processing pipelines and implement robust perception algorithms.

**Safety note**: Processing latency affects control stability. Monitor node timing with `ros2 monitor` to ensure processing doesn't create delays.
