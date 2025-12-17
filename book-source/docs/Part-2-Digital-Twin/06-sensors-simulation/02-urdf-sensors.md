---
title: "Lesson 2: Adding Sensors to URDF"
chapter: 6
lesson: 2
proficiency_level: B2
learning_objectives:
  - Modify humanoid URDF to include LiDAR, depth camera, and IMU sensors
  - Understand sensor frame attachment and coordinate systems
  - Configure sensor parameters for realistic simulation
  - Validate URDF sensor definitions
estimated_time: 90 minutes
generated_by: content-implementer v1.0.0
created: 2025-12-16
---

# Lesson 2: Adding Sensors to URDF

## Starting Point: The Humanoid Structure

From Chapter 4, you have a humanoid.urdf with this structure:

```xml
<robot name="humanoid_simple">
  <!-- Links: base_link, torso, head, arms, legs -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.3 0.3 0.05"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.3 0.3 0.05"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="torso">
    <!-- Torso definition -->
  </link>

  <link name="head">
    <!-- Head definition -->
  </link>

  <!-- Joints connecting links -->
</robot>
```

Now you'll add sensors to specific links. Each sensor needs:

1. **Sensor definition** in URDF: Specifies type, pose, parameters
2. **Mounting location**: Which link the sensor attaches to
3. **Pose adjustment**: Position and orientation relative to link

## Step 1: Adding LiDAR to Head

LiDAR should be on the head for maximum visibility.

**Why head?**
- Highest point on humanoid (clears obstacles)
- Can observe full environment (360 degrees)
- Natural position for autonomous navigation sensors

**URDF sensor definition**:

```xml
<!-- Inside <robot>, after link definitions, before </robot> -->

<!-- LiDAR sensor on head -->
<sensor name="lidar_sensor" type="gpu_lidar">
  <parent link="head"/>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>  <!-- 15cm above head center -->
  <update_rate>10</update_rate>  <!-- 10 Hz -->
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>  <!-- 360 degrees / 0.5 degree resolution -->
        <min_angle>0</min_angle>
        <max_angle>6.28</max_angle>
      </horizontal>
      <vertical>
        <samples>32</samples>  <!-- 32 vertical lines (like real LiDAR) -->
        <min_angle>-0.2618</min_angle>  <!-- -15 degrees -->
        <max_angle>0.2618</max_angle>   <!-- +15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.15</min>
      <max>50</max>  <!-- 50 meter range -->
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev>  <!-- 2cm noise standard deviation -->
    </noise>
  </ray>
</sensor>

<!-- LiDAR plugin configuration (in world file, but shown for reference) -->
<!-- <plugin name="gazebo_ros2_lidar" filename="libgazebo_ros_lidar.so">
  <sensor>lidar_sensor</sensor>
  <ros>
    <remapping>~/out:=/lidar/points</remapping>
  </ros>
</plugin> -->
```

**Understanding the parameters**:
- `720 samples` horizontal = 0.5 degree resolution (360° / 720 = 0.5°)
- `32 samples` vertical = Real LiDAR profile (Velodyne HDL-32, etc.)
- `origin xyz="0 0 0.15"` = Mounted 15cm above head center in world frame
- `stddev=0.02` = ±2cm noise, typical for real LiDAR at 10m distance

**Test output**: After Gazebo launches, check:
```bash
ros2 topic echo /lidar/points --no-arr | head -5
# Should show: height: 32, width: 720 (or similar, order may vary)
```

## Step 2: Adding Depth Camera to Torso

Depth camera should face forward on the torso (like a chest-mounted camera).

**Why torso?**
- Closer to robot center of mass (more stable mounting)
- Forward-facing (detects obstacles in path of travel)
- Useful for object manipulation (if picking things)

**URDF sensor definition**:

```xml
<!-- Depth camera on torso, facing forward (+X direction) -->
<sensor name="depth_camera_sensor" type="depth_camera">
  <parent link="torso"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>  <!-- 10cm above torso center, facing forward -->
  <update_rate>30</update_rate>  <!-- 30 Hz (video frame rate) -->
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees field of view -->
    <image>
      <width>640</width>
      <height>480</height>  <!-- Standard VGA resolution -->
      <format>RGB8</format>  <!-- Color format: 8-bit RGB -->
    </image>
    <clip>
      <near>0.1</near>   <!-- Minimum depth: 10cm -->
      <far>10.0</far>    <!-- Maximum depth: 10 meters -->
    </clip>
    <!-- Camera intrinsics (these are typical values, real camera differs) -->
    <distortion>
      <k1>0.0</k1>
      <k2>0.0</k2>
      <p1>0.0</p1>
      <p2>0.0</p2>
    </distortion>
  </camera>
</sensor>
```

**Understanding the parameters**:
- `horizontal_fov=1.047` = ~60 degrees (1.047 radians ≈ 60°)
- `near=0.1` and `far=10.0` = Depth range (can't see closer than 10cm, can't see farther than 10m)
- `640x480` = Lower resolution than modern cameras, but faster processing for simulation
- `update_rate=30` = 30 Hz matches typical USB camera (or decimated high-speed camera)

**Camera intrinsics explained**:
```
Focal length (f): Links pixel coordinates to real-world angles
Principal point (cx, cy): Pixel coordinate of camera center
Distortion (k1, k2, p1, p2): Lens distortion parameters
```

In this simulation, we set distortion to 0 (perfect lens). Real cameras have slight distortion.

**Intrinsics calculation** (if you wanted to match real camera):
```
f_x = (width / 2) / tan(horizontal_fov / 2)
For 640x480, 60°: f_x = 320 / tan(30°) ≈ 554 pixels
```

**Test output**: After Gazebo launches, check:
```bash
ros2 topic list | grep -E "depth|camera"
# Should show: /depth_camera/image_raw, /depth_camera/depth, /depth_camera/camera_info
```

## Step 3: Adding IMU to Base Link

IMU should be at the center of mass for accurate measurement.

**Why base_link?**
- Typically located at center of mass (or very close)
- Rotation measured at center point (not offset by mounting)
- All body motion (acceleration, rotation) measurable

**URDF sensor definition**:

```xml
<!-- IMU at center of mass (base_link) -->
<sensor name="imu_sensor" type="imu">
  <parent link="base_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>  <!-- At center of mass -->
  <update_rate>100</update_rate>  <!-- 100 Hz (high-frequency motion tracking) -->
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>  <!-- 0.001 rad/s noise -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.05</stddev>  <!-- 0.05 m/s² noise -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.05</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.05</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

**Understanding the parameters**:
- `angular_velocity` noise = Gyroscope noise (0.001 rad/s = typical low-cost IMU)
- `linear_acceleration` noise = Accelerometer noise (0.05 m/s² = typical)
- `update_rate=100` = 100 Hz (used for high-frequency balance control)
- `origin xyz="0 0 0"` = Exactly at link center (important for accurate rotation measurement)

**Noise interpretation**: Stddev of 0.05 m/s² means:
- 68% of measurements within ±0.05 m/s² of true value
- 95% within ±0.10 m/s²
- Real gravity (9.81 m/s²) measured as 9.81 ± 0.05 m/s²

## Step 4: Verify URDF Structure

Complete sensor-equipped humanoid structure:

```xml
<?xml version="1.0"?>
<robot name="humanoid_with_sensors">
  <!-- Links (torso, head, base_link, arms, legs) -->
  <link name="base_link">
    <!-- ... -->
  </link>

  <link name="torso">
    <!-- ... -->
  </link>

  <link name="head">
    <!-- ... -->
  </link>

  <!-- Joints connecting links -->
  <joint name="torso_to_head" type="revolute">
    <!-- ... -->
  </joint>

  <!-- SENSORS SECTION -->

  <!-- LiDAR on head -->
  <sensor name="lidar_sensor" type="gpu_lidar">
    <parent link="head"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <min_angle>0</min_angle>
          <max_angle>6.28</max_angle>
        </horizontal>
        <vertical>
          <samples>32</samples>
          <min_angle>-0.2618</min_angle>
          <max_angle>0.2618</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.15</min>
        <max>50</max>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.02</stddev>
      </noise>
    </ray>
  </sensor>

  <!-- Depth camera on torso -->
  <sensor name="depth_camera_sensor" type="depth_camera">
    <parent link="torso"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>RGB8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
      <distortion>
        <k1>0.0</k1>
        <k2>0.0</k2>
        <p1>0.0</p1>
        <p2>0.0</p2>
      </distortion>
    </camera>
  </sensor>

  <!-- IMU at center of mass -->
  <sensor name="imu_sensor" type="imu">
    <parent link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><mean>0.0</mean><stddev>0.001</stddev></noise></x>
        <y><noise type="gaussian"><mean>0.0</mean><stddev>0.001</stddev></noise></y>
        <z><noise type="gaussian"><mean>0.0</mean><stddev>0.001</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><mean>0.0</mean><stddev>0.05</stddev></noise></x>
        <y><noise type="gaussian"><mean>0.0</mean><stddev>0.05</stddev></noise></y>
        <z><noise type="gaussian"><mean>0.0</mean><stddev>0.05</stddev></noise></z>
      </linear_acceleration>
    </imu>
  </sensor>

</robot>
```

## Coordinate Systems: Checking Your Work

Sensors must be positioned correctly relative to their parent links.

**LiDAR position check**:
- Origin: 15cm above head (should clear torso)
- Direction: Faces upward (rays shoot outward in all directions)
- Expected: Can see ground, ceiling, surrounding walls

**Depth camera position check**:
- Origin: 10cm above torso, facing forward
- Direction: Rays project forward (along +X axis in local frame)
- Expected: Can see obstacles in robot's path

**IMU position check**:
- Origin: At link center (base_link center of mass)
- Direction: Measures accelerations at this point
- Expected: Rotation measured accurately without offset

**Validation command**:
```bash
# Parse and validate URDF syntax
ros2 model_state --model-name humanoid_with_sensors

# Visualize URDF structure
urdf_to_graphviz humanoid_with_sensors.urdf > humanoid.gv
dot -Tsvg humanoid.gv -o humanoid.svg
# View humanoid.svg to see sensor mounting points
```

## Exercise: Modify Sensor Parameters

**Task**: Modify the URDF with these changes and predict the effects:

1. **LiDAR**: Change `samples` from 720 to 360 (half resolution)
   - Effect: Fewer points per scan, faster processing, less detail

2. **Depth camera**: Change resolution from 640x480 to 320x240
   - Effect: Quarter as many pixels, faster processing, lower detail

3. **IMU**: Change `update_rate` from 100 to 50
   - Effect: Only 50 measurements per second (still fast enough for most balance control)

**Validation**: Save modified URDF, describe the tradeoffs you've made.

## Try With AI

**Setup**: Use ChatGPT or Claude to discuss URDF sensor placement decisions.

**Prompt 1: Sensor placement**
"I'm designing a humanoid robot's sensor suite. Should the depth camera be mounted:
a) On the head (with LiDAR) for maximum field of view?
b) On the torso, facing forward only?
c) Distributed (one on head, one on torso)?

What are the tradeoffs for obstacle detection, object manipulation, and power consumption?"

**Prompt 2: Parameter tuning**
"For real-time obstacle avoidance, I need depth camera updates at least every 100ms (10 Hz minimum). What resolution and frame rate should I use? I have a GPU but want to keep power consumption reasonable."

**Prompt 3: Noise calibration**
"Real sensor datasheet says my LiDAR has 2cm accuracy at 10m distance. My IMU accelerometer has ±0.2g accuracy. How should I set noise parameters in the URDF to match real hardware? Why does simulation noise matter if algorithms will handle it anyway?"

**Expected outcome**: You should understand sensor placement strategies and parameter tradeoffs between accuracy, update rate, and computational load.

**Safety note**: When adding sensors to humanoid, ensure sensor poses don't create visual artifacts (cameras inside robot body, LiDAR firing through links). Check with visualization tools (RViz, Gazebo).
