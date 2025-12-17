---
title: "Lesson 9: Capstone - Complete Sensor Suite with Processing Pipeline"
chapter: 6
lesson: 9
proficiency_level: B2
learning_objectives:
  - Design specification for complete multi-sensor perception system
  - Integrate all sensors with processing pipeline
  - Validate system against acceptance criteria
  - Deploy complete sensor suite to humanoid
estimated_time: 150 minutes
generated_by: content-implementer v1.0.0
created: 2025-12-16
---

# Lesson 9: Capstone - Complete Sensor Suite with Processing Pipeline

## Specification-First Approach

This capstone teaches the PRIMARY skill of AI-native development: **writing clear specifications BEFORE implementation**.

Your specification will define the complete system. Then, AI handles implementation using skills from Lessons 7-8.

## Part 1: Writing the System Specification

Create `specs/chapter-6-capstone/spec.md`:

```markdown
# Humanoid Sensor Suite and Perception System Specification

## Intent

Equip humanoid robot with complete perception system (LiDAR + depth camera + IMU) that generates realistic sensor data and processes it to enable autonomous navigation and obstacle avoidance.

## Success Criteria

- All three sensors publishing data at specified rates
- Obstacle detection working (LiDAR and depth camera)
- Orientation estimation working (IMU with sensor fusion)
- Sensor data recording to ROS 2 bags
- System runs in real-time (min 20 Hz perception updates)
- Visualization in RViz shows all sensor streams
- Documentation of processing decisions

## Constraints

- Gazebo simulation on single Ubuntu 22.04 machine
- GPU for Gazebo rendering recommended but not required
- LiDAR: 360° coverage, indoor range (0.15-50m)
- Depth camera: Forward-facing, RGB-D output
- IMU: 100 Hz update rate for balance control
- Processing on ROS 2 Humble or later
- No external hardware (pure simulation)

## Non-Goals

- Real robot deployment (simulation validation only)
- Advanced SLAM or mapping (perception pipeline only)
- Multi-robot coordination
- Optimization for power consumption
- Integration with locomotion controller

## Acceptance Tests

### Test 1: Sensor Publishing
```bash
# All sensors must publish continuously
- /lidar/points @ 10 Hz, 23,040 points per scan
- /depth_camera/depth @ 30 Hz, 640x480 depth images
- /depth_camera/rgb @ 30 Hz, 640x480 RGB images
- /imu/data_raw @ 100 Hz, acceleration + angular velocity
```

### Test 2: Obstacle Detection
```bash
# Place obstacle at 1m distance
# Run obstacle_detector node
# Verify detection:
- ros2 topic echo /detected_obstacles
- Should show markers at obstacle location
- Detection latency < 200ms
```

### Test 3: Orientation Estimation
```bash
# Rotate humanoid around each axis
# Monitor IMU-based orientation estimation
# Verify:
- TF frame /imu_frame rotates with robot
- No drift for 5-minute stationary period
- Accel-based gravity correction prevents gyro drift
```

### Test 4: Sensor Data Recording
```bash
# Record all sensors for 30 seconds
ros2 bag record -o capstone_test /lidar/points /depth_camera/depth /imu/data_raw -d 30

# Playback and verify
ros2 bag play capstone_test
# All nodes should receive replayed data
```

### Test 5: Real-Time Performance
```bash
# Monitor CPU usage while all systems running
# Verify:
- Simulation runs at >20 Hz (50ms per frame)
- Processing pipeline keeps up with sensor rate
- No dropped messages in ROS 2 node monitoring
```

## Architecture

### Components

1. **Simulation**: Gazebo world with humanoid + sensors
2. **Sensor plugins**: Publish LiDAR, depth, IMU to ROS 2 topics
3. **Processing nodes**:
   - Obstacle detector (subscribes /lidar/points)
   - Orientation estimator (subscribes /imu/data_raw)
4. **Visualization**: RViz with multi-sensor display
5. **Recording**: ROS 2 bag recorder

### Data Flow

```
[Gazebo simulation]
  ├─→ /lidar/points → [Obstacle detector] → /detected_obstacles → [RViz]
  ├─→ /depth_camera/depth → [Depth processor] → /filtered_depth
  ├─→ /depth_camera/rgb → [RViz]
  └─→ /imu/data_raw → [Orientation estimator] → /tf → [RViz]
```

### Reusable Components

From previous lessons:
- gazebo-sensor-visualization-skill (Lesson 7)
- gazebo-sensor-processing-skill (Lesson 8)

## Deliverables

1. Humanoid URDF with all three sensors
2. Gazebo world file with sensor plugins
3. ROS 2 launch file coordinating all components
4. Obstacle detection node (with processing skill)
5. Orientation estimation node (with processing skill)
6. RViz configuration for visualization
7. Test validation results (all acceptance tests pass)
8. Documentation explaining processing decisions

## Timeline

- Phase 1 (30 min): Specification review and clarification
- Phase 2 (30 min): URDF and world setup
- Phase 3 (30 min): Sensor plugin configuration
- Phase 4 (30 min): Obstacle detection implementation
- Phase 5 (30 min): Orientation estimation implementation
- Phase 6 (30 min): Integration and validation
- Phase 7 (30 min): Testing and documentation
```

## Part 2: Prompting AI with Your Specification

**Prompt to AI**:

```
I need to implement a humanoid robot perception system in Gazebo with ROS 2.
Here's my specification:

[Copy full spec.md above]

Key requirements:
1. Three sensors (LiDAR on head, depth camera on torso, IMU at center of mass)
2. Realistic simulation with noise matching real sensors
3. Processing pipeline detecting obstacles and estimating orientation
4. All components running in ROS 2 with real-time performance

Please:
1. Suggest which reusable skills apply from Lessons 7-8
2. Identify components that need new implementation
3. Outline the integration steps
4. Flag any specification gaps I should clarify

Start with high-level architecture before diving into code.
```

**AI's collaborative response**:

AI will likely suggest:
- Using gazebo-sensor-visualization-skill for RViz setup
- Using gazebo-sensor-processing-skill for obstacle detection pipeline
- Creating new orientation_estimator node (not covered by skills yet)
- Requesting clarification: "How should orientation drift be handled during 5-minute test?"

**Your refinement**:

"For drift handling, use complementary filter: 98% gyro + 2% accelerometer correction. Accelerometer provides gravity reference to prevent gyro drift over time."

AI now has clearer specification and will implement accordingly.

## Part 3: Integration and Validation

### Step 1: Prepare URDF with All Sensors

From Lessons 1-2, your humanoid has all three sensors defined.

```bash
# Verify URDF parses correctly
urdf_to_graphviz humanoid_with_sensors.urdf > humanoid.gv
```

### Step 2: Configure Gazebo World

`worlds/capstone_with_sensors.sdf`:

```xml
<sdf version="1.9">
  <world name="capstone_world">
    <!-- Standard world elements (gravity, light) -->
    <gravity>0 0 -9.81</gravity>
    <light>...</light>

    <!-- Include humanoid with sensors -->
    <include>
      <uri>model://humanoid_with_sensors</uri>
      <name>humanoid</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <!-- Sensor plugins -->
    <plugin name="gazebo_ros2_lidar"
            filename="libgazebo_ros_gpu_lidar.so">
      <sensor_name>lidar_sensor</sensor_name>
      <ros>
        <namespace>/</namespace>
        <remapping>~/out:=/lidar/points</remapping>
      </ros>
    </plugin>

    <plugin name="gazebo_ros2_camera"
            filename="libgazebo_ros_camera.so">
      <sensor_name>depth_camera_sensor</sensor_name>
      <ros>
        <namespace>/</namespace>
        <remapping>~/image:=/depth_camera/depth</remapping>
      </ros>
    </plugin>

    <plugin name="gazebo_ros2_imu"
            filename="libgazebo_ros_imu_sensor.so">
      <sensor_name>imu_sensor</sensor_name>
      <ros>
        <namespace>/</namespace>
        <remapping>~/out:=/imu/data_raw</remapping>
      </ros>
    </plugin>
  </world>
</sdf>
```

### Step 3: Create Launch File

`launch/capstone_perception.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_bringup')

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo',
             os.path.join(pkg_share, 'worlds', 'capstone_with_sensors.sdf')],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'capstone.rviz')],
        output='screen'
    )

    # Obstacle detection
    detector = Node(
        package='humanoid_perception',
        executable='obstacle_detector',
        output='screen'
    )

    # Orientation estimation
    orient_est = Node(
        package='humanoid_perception',
        executable='orientation_estimator',
        output='screen'
    )

    # Bag recording
    bag_recorder = Node(
        package='rosbag2',
        executable='record',
        arguments=['-o', 'capstone_data',
                   '/lidar/points', '/depth_camera/depth',
                   '/depth_camera/rgb', '/imu/data_raw'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rviz,
        detector,
        orient_est,
        bag_recorder,
    ])
```

### Step 4: Run and Validate

```bash
# Terminal 1: Launch everything
ros2 launch humanoid_bringup capstone_perception.launch.py

# Terminal 2: Verify sensors are publishing
ros2 topic list | grep -E "lidar|depth|imu"
# Expected:
# /depth_camera/depth
# /depth_camera/rgb
# /imu/data_raw
# /lidar/points

# Terminal 3: Run acceptance tests
python3 << 'EOF'
import subprocess
import time

def test_sensor_publishing():
    """Test 1: Verify sensors publishing"""
    result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
    topics = result.stdout.split('\n')

    required = ['/lidar/points', '/depth_camera/depth', '/imu/data_raw']
    found = [t for t in required if t in topics]

    if len(found) == len(required):
        print("✅ Test 1 PASSED: All sensors publishing")
        return True
    else:
        print(f"❌ Test 1 FAILED: Missing {set(required) - set(found)}")
        return False

def test_obstacle_detection():
    """Test 2: Verify obstacle detection"""
    result = subprocess.run(['ros2', 'topic', 'echo', '/detected_obstacles', '--no-arr'],
                          capture_output=True, text=True, timeout=5)
    if 'markers' in result.stdout:
        print("✅ Test 2 PASSED: Obstacle detection working")
        return True
    else:
        print("❌ Test 2 FAILED: No obstacle markers published")
        return False

# Run tests
results = [
    test_sensor_publishing(),
    test_obstacle_detection(),
]

print(f"\n{sum(results)}/{len(results)} tests passed")
EOF
```

## Part 4: Analysis and Reflection

**Document your implementation**:

```markdown
# Capstone Implementation Report

## Sensors Configured

- LiDAR: 720×32 samples, 10 Hz, 2cm noise
- Depth camera: 640×480, 30 Hz, intrinsics matched to RealSense D435
- IMU: 100 Hz, complementary filter fusion

## Processing Pipeline

1. **Obstacle detection**: DBSCAN clustering on LiDAR point cloud
   - Preprocessing: Remove ground plane, ceiling, downsampling
   - Clustering: 10cm threshold, min 5 points per cluster
   - Performance: 10ms processing time < 100ms sensor period ✓

2. **Orientation estimation**: Complementary filter (IMU fusion)
   - 98% gyro weight + 2% accelerometer correction
   - Prevents gyro drift over long periods
   - Validation: Stationary test shows <0.1° drift over 5 minutes ✓

## Lessons Learned

- Realistic noise is critical for algorithm development
- Sensor fusion (accel-based gravity correction) prevents drift
- Processing latency affects control stability (keep <100ms)
- RViz visualization essential for debugging multi-sensor systems

## What Would Transfer to Real Hardware

✓ Processing algorithms (obstacle detection, IMU fusion)
✓ ROS 2 node architecture
✓ Sensor plugin configuration patterns

✗ Exact noise parameters (will differ per real sensor)
✗ Gazebo physics (real dynamics more complex)
✗ Processing latencies (depend on real computer)

## Improvements for Production

1. Add dynamic parameter tuning (adjust clustering threshold)
2. Implement sensor health monitoring (detect failures)
3. Add more sophisticated point cloud filtering (statistical)
4. Integrate with actual humanoid control loop
5. Validate on real robot with same sensors
```

## Try With AI

**Prompt 1: Specification review**
"Review my perception system specification. What important requirements might I be missing? What could cause sim-to-real transfer failure?"

**Prompt 2: Architecture feedback**
"I'm planning to record all sensor data to ROS 2 bags for post-analysis. Is this the best approach, or should I implement online analysis of sensor quality?"

**Prompt 3: Validation strategy**
"My acceptance tests check that sensors are publishing and obstacles are detected. What additional tests would validate that my system is truly production-ready?"

**Prompt 4: Production readiness**
"Before deploying this perception system to a real humanoid, what validation should I perform? How do I ensure simulation results transfer to real hardware?"

**Expected outcome**: Complete, working multi-sensor perception system validated against specification. Understand spec-driven development and system integration at scale.

**Safety note**: When deploying to real robots, validate that:
1. Sensor mounting is mechanically sound
2. Sensor calibration matches simulation assumptions
3. Processing latency is acceptable for control loop
4. Noise characteristics match real hardware
5. Failure modes are handled gracefully

---

**Capstone Summary**:

You've gone from understanding sensor architecture (Lesson 1) through configuring individual sensors (Lessons 2-6), creating reusable skills (Lessons 7-8), to designing and implementing a complete perception system (Lesson 9).

This is the journey from learning components to building integrated systems. In your future work with humanoid robots, this sensor suite foundation will support navigation, manipulation, and autonomous decision-making.
