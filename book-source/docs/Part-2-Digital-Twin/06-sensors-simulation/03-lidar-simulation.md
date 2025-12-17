---
title: "Lesson 3: LiDAR Simulation"
chapter: 6
lesson: 3
proficiency_level: B2
learning_objectives:
  - Configure LiDAR ray casting in Gazebo
  - Generate point clouds with realistic resolution and noise
  - Process PointCloud2 messages in Python
  - Debug LiDAR output and optimize performance
estimated_time: 120 minutes
generated_by: content-implementer v1.0.0
created: 2025-12-16
---

# Lesson 3: LiDAR Simulation

## Ray Casting: How LiDAR Works

LiDAR emits laser pulses and measures time to reflection. In simulation:

1. **Ray emission**: From sensor center, fire rays in a grid (720 horizontal × 32 vertical samples)
2. **Intersection detection**: Check what each ray hits (terrain, obstacles, robot links)
3. **Distance calculation**: Convert hit distance to depth measurement
4. **Noise addition**: Add realistic measurement error
5. **Point cloud generation**: Package all points (x, y, z, intensity) into PointCloud2 message

**Gazebo plugin** (libgazebo_ros_gpu_lidar.so) handles this automatically when configured.

## Running LiDAR Simulation

**Launch file** (launch/lidar_test.launch.py):
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    gazebo = ExecuteProcess(
        cmd=['gazebo', 'worlds/office_with_humanoid.sdf'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', 'config/default.rviz'],
        output='screen'
    )

    return LaunchDescription([gazebo, rviz])
```

**Start simulation**:
```bash
cd ~/humanoid_ws
ros2 launch humanoid_bringup lidar_test.launch.py
```

**Verify LiDAR is publishing**:
```bash
ros2 topic list | grep lidar
# Output: /lidar/points

# Inspect point cloud (first few points)
ros2 topic echo /lidar/points --no-arr | head -30
```

## Processing Point Clouds in Python

Create a node to process LiDAR data:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.lidar_callback,
            10
        )
        self.get_logger().info('LiDAR processor started')

    def lidar_callback(self, msg):
        """Process incoming point cloud"""
        # Extract point cloud dimensions
        points_count = msg.width * msg.height

        # Parse point data (x, y, z, intensity)
        points = self.pointcloud2_to_array(msg)

        # Extract coordinates and intensity
        x = points['x']
        y = points['y']
        z = points['z']
        intensity = points['intensity']

        # Calculate distances from sensor origin
        distances = np.sqrt(x**2 + y**2 + z**2)

        # Detect obstacles (anything closer than 2 meters)
        obstacles = np.where(distances < 2.0)[0]

        self.get_logger().info(
            f'Points: {len(points)}, Obstacles detected: {len(obstacles)}'
        )

    def pointcloud2_to_array(self, msg):
        """Convert PointCloud2 message to structured numpy array"""
        # Parse field offsets
        offset = {f.name: f.offset for f in msg.fields}
        datatype = {f.name: f.datatype for f in msg.fields}

        # Read binary data
        data = msg.data

        # Extract points
        points = np.zeros(msg.width * msg.height, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.uint8)
        ])

        for i in range(len(points)):
            idx = i * msg.point_step
            points[i]['x'] = struct.unpack('f', data[idx:idx+4])[0]
            points[i]['y'] = struct.unpack('f', data[idx+4:idx+8])[0]
            points[i]['z'] = struct.unpack('f', data[idx+8:idx+12])[0]
            points[i]['intensity'] = data[idx+16]

        return points

def main(args=None):
    rclpy.init(args=args)
    processor = LidarProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Test output**:
```
[lidar_processor] LiDAR processor started
[lidar_processor] Points: 23040, Obstacles detected: 0
[lidar_processor] Points: 23040, Obstacles detected: 245
[lidar_processor] Points: 23040, Obstacles detected: 189
```

(23040 = 720 × 32 points)

## Tuning LiDAR Parameters

**Parameter effects on performance**:

| Parameter | Low Value | High Value | Tradeoff |
|-----------|-----------|------------|----------|
| Horizontal samples | 360 | 1080 | More detail vs slower |
| Vertical samples | 16 | 64 | Wider view vs more compute |
| Update rate | 5 Hz | 20 Hz | Smoother vs CPU load |
| Range max | 30m | 100m | Sees farther vs less accurate |
| Noise stddev | 0.005m | 0.05m | Realistic vs clean data |

**Optimization example**: If simulation runs too slow:

```xml
<!-- Original (high load) -->
<horizontal>
  <samples>1440</samples>  <!-- Every 0.25 degrees -->
</horizontal>
<update_rate>20</update_rate>

<!-- Optimized (lower load) -->
<horizontal>
  <samples>720</samples>  <!-- Every 0.5 degrees -->
</horizontal>
<update_rate>10</update_rate>

<!-- Result: 4x fewer points, half the update rate, still 360-degree coverage -->
```

## Real-World Validation

Compare simulation to real LiDAR specifications:

**Velodyne HDL-32E** (real sensor):
- Horizontal resolution: 0.2° (1800 points per revolution)
- Vertical: 32 layers
- Range: 0.8–100m
- Noise: ±2cm at 10m, ±6cm at 50m
- Update rate: 10–20 Hz

**Our simulation**:
- Horizontal: 0.5° (720 points)
- Vertical: 32 layers
- Range: 0.15–50m
- Noise: 2cm constant
- Update rate: 10 Hz

**Differences**:
- Lower horizontal resolution (but still usable)
- Shorter range (but sufficient for indoor robotics)
- Constant noise (real LiDAR noise increases with distance)

**Improvement**: Match real noise curve:

```python
# In LiDAR plugin configuration, add distance-dependent noise
def apply_distance_noise(point_x, point_y, point_z):
    distance = np.sqrt(point_x**2 + point_y**2 + point_z**2)
    # Real LiDAR: noise proportional to distance
    noise_stddev = 0.01 + 0.0005 * distance  # 1cm + 0.05% of distance
    noise = np.random.normal(0, noise_stddev)
    return point_z + noise
```

## Collaborative LiDAR Tuning

**Scenario**: Your simulated LiDAR produces point clouds, but obstacle detection algorithm struggles.

**Initial investigation**: Ask an AI colleague:
"My LiDAR point cloud has 23,040 points per scan (720×32), but obstacle detection is noisy. Should I increase sampling for better resolution, or add filtering?"

**AI suggests**: "Before increasing sampling (more compute), try filtering. Your noise is probably higher than necessary. What's your typical distance to obstacles?"

**You respond**: "Most obstacles are within 3 meters. Everything beyond that is background."

**AI adapts**: "Then focus on near-range accuracy. Set update_rate to 20 Hz and stddev to 0.01m (1cm noise). This matches real LiDAR in your range. Filter out anything beyond 5m—reduces noise and compute."

**Result**: You modify configuration:
```xml
<update_rate>20</update_rate>
<noise>
  <stddev>0.01</stddev>  <!-- Changed from 0.02 -->
</noise>
<range>
  <max>5</max>  <!-- Changed from 50 -->
</range>
```

**Outcome**: Obstacle detection improves because:
1. Lower noise in near range
2. Fewer irrelevant far-range points
3. Higher update rate catches obstacles sooner
4. Processing faster (fewer total points)

This is collaboration: AI suggested filtering-first, you provided constraint (3m range), AI adapted recommendation to your specific case.

## Exercise: Debug LiDAR Output

**Task**: Verify LiDAR data quality with a simple test.

```bash
# Terminal 1: Start simulation
ros2 launch humanoid_bringup lidar_test.launch.py

# Terminal 2: Record a few seconds of LiDAR data
ros2 bag record -o lidar_test /lidar/points -d 5  # Record for 5 seconds

# Terminal 3: Analyze the bag
python3 << 'EOF'
import rosbag2_py
import numpy as np
from rosidl_runtime_py.utilities import get_message

bag = rosbag2_py.SequentialReader()
bag.open('lidar_test')

storage_options = rosbag2_py.StorageOptions(uri='lidar_test', storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('cdr', 'cdr')

bag = rosbag2_py.SequentialReader()
bag.open(storage_options, converter_options)

count = 0
while bag.has_next():
    topic, message, timestamp = bag.read_next()
    if topic == '/lidar/points':
        print(f'Frame {count}: {message.width}x{message.height} points')
        count += 1

print(f'Total frames: {count}')
EOF
```

## Try With AI

**Prompt 1: Troubleshooting**
"My LiDAR in Gazebo publishes at 10 Hz, but my obstacle detection node only receives ~3 messages per second. What's happening, and how do I debug it?"

**Prompt 2: Optimization**
"I need LiDAR coverage from 0.3–10m for my indoor robot. Current config has 720 horizontal samples and 32 vertical layers. Should I increase to 1440 samples for finer detail, or keep 720 and increase vertical layers to 64?"

**Prompt 3: Validation**
"Here's my real sensor spec: Sick TiM781S with 270° field of view, 0.25° resolution, 25m range, ±60mm accuracy. How should I configure my simulated LiDAR to match? What tradeoffs are acceptable?"

**Expected outcome**: Understand LiDAR parameter tuning and how point cloud quality affects downstream algorithms.

**Safety note**: Simulation speed degrades with very high LiDAR sampling rates. If simulation runs below real-time, reduce sampling or update rate to maintain synchronization with control algorithms.
