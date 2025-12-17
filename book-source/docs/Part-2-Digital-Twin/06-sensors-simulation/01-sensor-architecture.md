---
title: "Lesson 1: Sensor Plugin Architecture"
chapter: 6
lesson: 1
proficiency_level: B2
learning_objectives:
  - Understand Gazebo sensor plugin architecture and data flow
  - Recognize how sensors integrate with simulation and publish to ROS 2
  - Identify ROS 2 message types for different sensor modalities
  - Design sensor configurations for robotics applications
estimated_time: 90 minutes
generated_by: content-implementer v1.0.0
created: 2025-12-16
---

# Lesson 1: Sensor Plugin Architecture

## Data Flow: From Simulation to ROS 2

Imagine a humanoid standing in a simulated office. Its eyes (camera), ears (microphone), and inner balance system (accelerometer) should all feed information about the environment. In Gazebo, this happens through a pipeline:

**Simulation physics engine** → **Sensor simulator** → **ROS 2 message publisher** → **Visualization/processing**

Each sensor has three roles:
1. **Data generation**: Physics engine calculates what the sensor should measure (ray casting for LiDAR, perspective projection for cameras)
2. **Measurement simulation**: Plugin adds realistic effects (noise, delay, accuracy limits)
3. **Publication**: Data packaged into ROS 2 messages on topics

Without understanding this flow, you can't debug sensor misconfigurations. Let's trace it.

## Gazebo Sensor Plugin Architecture

A Gazebo sensor has these components:

**World file (SDF)**:
```xml
<world name="office">
  <model name="humanoid">
    <!-- Sensor definition (we'll write in URDF, Gazebo converts it) -->
  </model>
  <plugin name="gazebo_ros2_camera" filename="libgazebo_ros_camera.so">
    <!-- Plugin configuration: which sensor, which topic, publishing frequency -->
  </plugin>
</world>
```

**URDF sensor definition**:
```xml
<sensor name="camera_sensor" type="camera">
  <pose relative_to="head">0 0 0.05 0 0 0</pose>
  <camera>
    <horizontal_fov>1.5708</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
</sensor>
```

**Plugin lifecycle**:

1. **Initialization**: Plugin loads, identifies sensor in URDF, creates ROS 2 publisher
2. **Update loop**: Every simulation step, plugin:
   - Calls sensor's `Update()` method (generates measurement)
   - Adds noise if configured
   - Packages data into ROS 2 message
   - Publishes at configured rate (e.g., 10 Hz for LiDAR, 30 Hz for cameras)
3. **Shutdown**: Plugin destroys publisher, releases resources

**Key insight**: The plugin is the bridge. Without it, sensor data stays in Gazebo. With it, data flows to ROS 2 nodes.

## ROS 2 Message Types for Sensors

Different sensors publish different message types:

**LiDAR** → `sensor_msgs/msg/PointCloud2`
- Contains: Point coordinates (x, y, z), intensity, optionally RGB
- Use case: Obstacle detection, 3D mapping
- Topic example: `/lidar/points`

**Depth camera** → `sensor_msgs/msg/Image` (depth) + `sensor_msgs/msg/Image` (RGB)
- Depth channel: Pixel value = distance from camera
- RGB channel: Standard color image
- Use case: Object detection, 3D reconstruction
- Topic examples: `/depth/image_raw`, `/camera/rgb/image_raw`

**IMU** → `sensor_msgs/msg/Imu`
- Contains: Linear acceleration, angular velocity, orientation (as quaternion)
- Optional: Magnetic field
- Use case: Motion estimation, balance control
- Topic example: `/imu/data_raw`

**Camera intrinsics** → `sensor_msgs/msg/CameraInfo`
- Contains: Focal length, principal point, distortion coefficients
- Pairs with Image messages to enable 3D reconstruction
- Topic example: `/camera/camera_info`

**Output**: Show what these look like in practice:

```bash
# Inspect LiDAR point cloud structure
ros2 topic echo /lidar/points --no-arr | head -20

# Output shows:
# header:
#   seq: 12345
#   stamp:
#     sec: 10
#     nsec: 500000000
# height: 32
# width: 720
# fields:
# - name: x
# - name: y
# - name: z
# - name: intensity
# data: [binary point data]
```

## Sensor Configuration Parameters

When defining a sensor in URDF, key parameters control output:

**LiDAR configuration**:
```xml
<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0.3 0 0 0</pose>
  <update_rate>10</update_rate>  <!-- Publish 10 times per second -->
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>  <!-- Horizontal resolution -->
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>6.28</max_angle>  <!-- Full 360 degrees -->
      </horizontal>
      <vertical>
        <samples>32</samples>  <!-- Vertical layers -->
        <resolution>1</resolution>
        <min_angle>-0.26</min_angle>
        <max_angle>0.26</max_angle>  <!-- ±15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.15</min>  <!-- Minimum detectable distance -->
      <max>100</max>  <!-- Maximum range -->
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev>  <!-- 2cm noise -->
    </noise>
  </ray>
</sensor>
```

**Parameter effects**:
- Higher `samples` → Better angular resolution, more points, slower processing
- Larger `range` → Sees farther, but less accurate at distance
- Higher `stddev` → More noisy data, more realistic

**Depth camera configuration**:
```xml
<sensor name="depth_camera" type="depth_camera">
  <pose>0 0 0.1 0 0 0</pose>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>  <!-- Minimum depth -->
      <far>10.0</far>   <!-- Maximum depth -->
    </clip>
    <!-- Intrinsics (focal length, principal point) -->
    <distortion>
      <k1>0.0</k1>
      <k2>0.0</k2>
      <p1>0.0</p1>
      <p2>0.0</p2>
    </distortion>
  </camera>
</sensor>
```

**IMU configuration**:
```xml
<sensor name="imu" type="imu">
  <pose>0 0 0 0 0 0</pose>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>  <!-- 0.01 rad/s noise -->
        </noise>
      </x>
      <!-- Similar for y, z -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.1</stddev>  <!-- 0.1 m/s² noise -->
        </noise>
      </x>
      <!-- Similar for y, z -->
    </linear_acceleration>
  </imu>
</sensor>
```

## Message Publishing Frequency

Real sensors have physical constraints on how fast they update:

**Camera**: ~30 Hz (standard video frame rate)
**LiDAR**: ~10 Hz (mechanical rotation speed) to ~20 Hz (modern fast LiDARs)
**IMU**: ~100 Hz (high-frequency motion tracking)

**Simulation note**: Published rate in `update_rate` parameter. But actual rate depends on:
- Simulation speed (if simulation runs slow, published rate decreases)
- CPU load (if processing is heavy, Gazebo may skip updates)
- ROS 2 publisher queue size (messages may buffer if subscriber is slow)

**Rule of thumb**: Set `update_rate` 1.5–2x faster than what you want to guarantee. This provides buffer for occasional delays.

## Plugin Integration Workflow

When you run a Gazebo world with sensors:

1. **Parse URDF**: Gazebo reads sensor definitions from URDF
2. **Load plugins**: Each `<plugin>` element loads the specified .so file
3. **Plugin initialization**: Plugin scans world for matching sensors, creates ROS 2 publishers
4. **Simulation loop**: For each timestep:
   - Physics engine steps (updates robot pose, etc.)
   - Sensor updates (LiDAR fires rays, camera renders, IMU calculates accelerations)
   - Plugin publishes (packages data into messages, sends to ROS 2)
5. **ROS 2 reception**: Subscriber nodes receive messages on topics

**Common failure modes**:
- Plugin .so file not found → Check `libgazebo_ros_*` filename matches installed version
- Sensor type mismatch → Plugin expects `camera` but URDF defines `depth_camera`
- Update rate too high → Gazebo slows down because CPU can't keep up
- Publisher not created → ROS 2 node doesn't find topic (check namespace)

## Checkpoint: Explaining Sensor Architecture

To validate your understanding:

**Describe the data flow**: From physics simulation through sensor plugin to ROS 2 message on a topic.

**Explain why configuration matters**: Why does LiDAR sample count affect obstacle detection quality?

**Predict plugin behavior**: If you set LiDAR `update_rate` to 100 Hz but Gazebo only runs at 50 Hz simulation speed, what happens to published data?

## Try With AI

**Setup**: Open ChatGPT or Claude and explore sensor architecture concepts.

**Prompt 1: Clarify architecture**
"Explain the difference between sensor update rate (how often Gazebo calculates measurements) and ROS 2 publisher rate (how often messages appear on topics). What happens if update_rate is slower than needed?"

**Prompt 2: Debug scenario**
"I configured a LiDAR with 1000 samples per revolution and 100 Hz update rate. Gazebo runs at 1000 Hz simulation speed. My perception node subscribes to /lidar/points. Why would the node receive fewer messages than expected?"

**Prompt 3: Validate thinking**
"For my humanoid robot simulation, I need:
- LiDAR on head for 360 obstacle detection (moving robot)
- Depth camera on torso (stationary object detection)
- IMU at center of mass (balance control at 100 Hz)

What update rates and sample counts would you recommend for each? What are the tradeoffs?"

**Expected outcome**: You should understand that sensors are simulated at high frequency but published at controlled rates, and that plugin configuration directly affects both data quality and computational load.

**Safety note**: When configuring sensor update rates, balance realism with computational feasibility. Too-high rates make simulation slow without benefit.
