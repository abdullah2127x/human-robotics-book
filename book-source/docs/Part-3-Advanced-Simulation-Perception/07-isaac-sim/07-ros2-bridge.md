# Lesson 7: Configuring Isaac Sim ROS 2 Bridge

Your synthetic data pipeline is working (Replicator generating annotated images). Now connect Isaac Sim to your ROS 2 ecosystem—enabling sensor data to flow from simulation to navigation and perception stacks you'll build in Chapters 8-9.

Isaac Sim's ROS 2 bridge uses **OmniGraph** (visual node-based programming) to configure publishers, subscribers, and transformations.

## Understanding OmniGraph Architecture

**OmniGraph** is Isaac Sim's computational graph system for creating data flows.

**Concept**: Instead of writing Python scripts for every ROS 2 connection, you create visual graphs:
```
[Camera Sensor] → [ROS 2 Camera Publisher] → /camera/image_raw (ROS 2 topic)
     ↓
[Transform] → [TF Publisher] → /tf (coordinate frames)
```

**Why OmniGraph?** Visual graphs are easier to debug (see data flow), modify without code changes, and save/reuse across projects.

## Prerequisites: ROS 2 Humble Installation

**Verify ROS 2 installed** (from Part 1):
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

**Expected**: Should see ROS 2 CLI working (no command not found).

## Creating Your First ROS 2 Bridge

Let's publish camera data from Isaac Sim to ROS 2.

### Step 1: Enable ROS 2 Bridge Extension

In Isaac Sim:
1. **Window → Extensions**
2. Search: **"ROS2 Bridge"**
3. Enable: **omni.isaac.ros2_bridge**
4. Wait for activation (5-10 seconds)

**Verify**: Extension panel shows "ROS2 Bridge" with green checkmark.

### Step 2: Create OmniGraph for Camera Publishing

**Manual OmniGraph creation** (for learning; later lessons will script this):

1. **Window → Visual Scripting → Action Graph**
2. New graph window opens
3. Right-click in graph → **Add Node**

**Add these nodes** (search by name):

1. **On Playback Tick**
   - Triggers graph every simulation step

2. **Isaac Read Camera Info**
   - Reads camera parameters (resolution, FOV)
   - Inputs:
     - Render Product: `/Render/RenderProduct` (path to camera)

3. **ROS2 Publisher (Camera Info)**
   - Publishes camera_info message
   - Inputs:
     - Topic Name: `/camera/camera_info`
     - QoS Profile: `SENSOR_DATA` (default for sensors)

4. **Isaac Read RGB**
   - Reads RGB image from camera
   - Inputs:
     - Render Product: `/Render/RenderProduct`

5. **ROS2 Publisher (Image)**
   - Publishes sensor_msgs/Image
   - Inputs:
     - Topic Name: `/camera/image_raw`
     - QoS Profile: `SENSOR_DATA`

**Connect nodes**:
- **On Playback Tick** → (Tick) → **Isaac Read Camera Info** (Exec In)
- **Isaac Read Camera Info** → (Camera Info) → **ROS2 Publisher (Camera Info)** (Data)
- **Isaac Read RGB** → (RGB Image) → **ROS2 Publisher (Image)** (Data)

**Click Play** in Isaac Sim.

### Step 3: Verify ROS 2 Topics

**In separate terminal**:
```bash
source /opt/ros/humble/setup.bash

# List topics
ros2 topic list

# Should see:
# /camera/image_raw
# /camera/camera_info
```

**View camera stream**:
```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/camera/image_raw` from dropdown.

**Expected**: Real-time camera feed from Isaac Sim appears in rqt window.

## Publishing Rate and QoS Configuration

**Your first observation**: Camera publishes at 60 Hz (simulation timestep), but ROS 2 subscriber lags.

### Discovering Rate Mismatches

**Your question**:
```
My Isaac Sim camera publishes at 60 Hz, but my ROS 2 subscriber (running on same machine) only receives 20-30 messages/second and sometimes drops frames. What's causing this?
```

**AI's diagnosis**:
```
**Common ROS 2 bridge bottlenecks**:

1. **QoS mismatch**: Publisher and subscriber must have compatible QoS policies
   - SENSOR_DATA profile: Best effort, volatile (fast but can drop)
   - RELIABLE profile: Guaranteed delivery (slower, queues)

2. **Publication rate too high**: 60 Hz camera can overwhelm network/CPU
   - Many perception algorithms don't need 60 Hz (30 Hz often sufficient)
   - Reduce publication rate to match subscriber processing speed

3. **Message size**: 1280x720 RGB images are ~2.7 MB each
   - At 60 Hz, that's 162 MB/s bandwidth
   - Consider:
     - Reducing resolution (640x480 = 0.9 MB)
     - Publishing compressed images (JPEG, ~200 KB)
     - Lowering framerate (30 Hz)

**Try reducing publication rate first** (cheapest fix).
```

**Your refinement**:
```
For my use case (visual SLAM in Chapter 8), what framerate and resolution should I target? I have RTX 4070 and standard desktop CPU (no bandwidth constraints).
```

**AI's adapted recommendation**:
```
For visual SLAM specifically:
- **Framerate**: 20-30 Hz sufficient (VSLAM algorithms don't benefit from 60 Hz)
- **Resolution**: 640x480 to 1280x720 (higher = better feature detection, but slower processing)
- **Compression**: Raw (uncompressed) for VSLAM (JPEG compression loses features)

Recommended: 30 Hz at 848x480 (standard stereo camera resolution).

This gives VSLAM algorithms enough frame rate for motion estimation while maintaining good feature quality.
```

**What emerged**: Through questioning your specific use case (VSLAM), you and AI converged on parameters tailored to Chapter 8 requirements.

### Implementing Rate Limiting

**Option 1: Reduce Simulation Framerate**

In Isaac Sim:
- **Edit → Preferences → Physics**
- **Time Steps Per Second**: 30 (was 60)

**Effect**: Entire simulation runs at 30 Hz (cheaper, but affects physics accuracy).

**Option 2: Throttle Camera Publication**

**Add throttle node in OmniGraph**:
1. Add node: **Isaac Throttle**
2. Configure:
   - **Target FPS**: 30
   - Connect between camera read and publisher

**Effect**: Simulation still runs at 60 Hz (physics accurate), but camera publishes every other frame.

**Preferred**: Option 2 (doesn't compromise physics).

## Publishing LiDAR Data

**Next sensor**: LiDAR for obstacle detection (Chapter 9 Nav2 integration).

### Adding LiDAR to Scene

**Step 1: Create LiDAR Sensor**

In Isaac Sim:
1. **Isaac Sensors → Rotating LiDAR**
2. Configure:
   - **Parent prim**: `/World/humanoid/base_link` (attach to robot)
   - **Position**: (0, 0, 0.3) (30cm above base)
   - **Rotation**: (0, 0, 0)
   - **Num Channels**: 16 (typical Velodyne VLP-16)
   - **Horizontal FOV**: 360°
   - **Max Range**: 100m

### Step 2: Create LiDAR OmniGraph

**Similar to camera, add nodes**:

1. **Isaac Read LiDAR Point Cloud**
   - Reads 3D points from sensor
   - Inputs:
     - Lidar Prim: `/World/humanoid/base_link/lidar`

2. **ROS2 Publisher (Point Cloud)**
   - Publishes sensor_msgs/PointCloud2
   - Inputs:
     - Topic Name: `/lidar/points`
     - Frame ID: `lidar_frame`
     - QoS Profile: `SENSOR_DATA`

**Connect and test**:
```bash
# Verify topic
ros2 topic echo /lidar/points --once

# Visualize in RViz
rviz2
# Add: PointCloud2 display
# Topic: /lidar/points
```

**Expected**: 3D point cloud of scene visible in RViz.

## Transform Frames (TF) Publishing

**Critical for ROS 2 integration**: Publish coordinate frame transforms so ROS 2 knows spatial relationships (camera relative to robot base, robot base relative to world).

### Understanding TF Requirements

**ROS 2 navigation stack needs**:
```
map (world frame)
  └─ odom (odometry frame)
      └─ base_link (robot base)
          ├─ camera_link (camera)
          └─ lidar_link (LiDAR)
```

**Isaac Sim USD hierarchy** (different structure):
```
/World (stage root)
  └─ humanoid (articulation root)
      ├─ base_link
      ├─ left_leg
      └─ sensors
          ├─ camera
          └─ lidar
```

**Bridge task**: Convert Isaac USD transforms → ROS 2 TF tree.

### Publishing TF Transforms

**Add TF publisher nodes**:

1. **Isaac Read Simulation Time**
   - Gets current simulation timestamp

2. **Isaac Compute Odometry**
   - Computes robot odometry from base_link motion
   - Inputs:
     - Chassis Prim: `/World/humanoid/base_link`

3. **ROS2 Publisher (TF)**
   - Publishes geometry_msgs/TransformStamped
   - Inputs:
     - Topic Name: `/tf`
     - Parent Frame: `odom`
     - Child Frame: `base_link`

4. **ROS2 Publisher (Static TF)**
   - Publishes static transforms (camera/LiDAR relative to base)
   - Topic: `/tf_static`
   - Transforms:
     - `base_link` → `camera_link`: (0, 0, 0.5)
     - `base_link` → `lidar_link`: (0, 0, 0.3)

**Verify TF tree**:
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo odom base_link
```

**Expected**: TF tree shows all frames connected.

## Clock Synchronization

**Problem**: Isaac Sim uses simulation time, but some ROS 2 nodes use system wall time.

### Teaching Clock Requirements

**Your constraint**:
```
I recorded a ROS 2 bag from Isaac Sim and tried to play it back. My VSLAM node (Chapter 8) processes frames at wrong rate—either too fast or too slow. I think it's a clock synchronization issue. How do I ensure Isaac Sim time matches ROS 2 time?
```

**AI's explanation**:
```
**ROS 2 has two time sources**:
1. **Wall time** (system clock): Real-world time
2. **Simulation time** (/clock topic): Simulated time (can run faster/slower than real-time)

**For bag replay and simulation**:
- Isaac Sim must publish simulation time to /clock topic
- ROS 2 nodes must use simulation time (parameter: use_sim_time=true)

**Configure Isaac Sim to publish clock**:
```python
# In OmniGraph or Python
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

# Enable clock publishing
import omni.isaac.ros_bridge
omni.isaac.ros_bridge.set_publish_clock(True)
```

**Launch ROS 2 nodes with simulation time**:
```bash
ros2 launch my_vslam vslam.launch.py use_sim_time:=true
```

Without this, nodes process messages as if they arrived in real-time (ignoring simulation timestamps).
```

**Implementation**:
1. Add **ROS2 Publisher (Clock)** node to OmniGraph
2. Configure topic: `/clock`
3. Ensure all downstream nodes have `use_sim_time:=true`

## Complete Bridge Example

**Full OmniGraph** (camera + LiDAR + TF + clock):

```python
# Python API to create OmniGraph programmatically (alternative to visual editor)
import omni.graph.core as og

# Create graph
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadCameraInfo", "omni.isaac.sensor.IsaacReadCameraInfo"),
            ("ReadRGB", "omni.isaac.sensor.IsaacReadRGB"),
            ("PublishImage", "omni.isaac.ros2_bridge.ROS2PublishImage"),
            ("ReadLidar", "omni.isaac.sensor.IsaacReadLidarPointCloud"),
            ("PublishPointCloud", "omni.isaac.ros2_bridge.ROS2PublishPointCloud"),
            ("ComputeOdometry", "omni.isaac.sensor.IsaacComputeOdometry"),
            ("PublishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
            ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock")
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "ReadCameraInfo.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ReadRGB.inputs:execIn"),
            ("ReadRGB.outputs:data", "PublishImage.inputs:data"),
            ("OnPlaybackTick.outputs:tick", "ReadLidar.inputs:execIn"),
            ("ReadLidar.outputs:data", "PublishPointCloud.inputs:data"),
            ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
            ("ComputeOdometry.outputs:transform", "PublishTF.inputs:transforms"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn")
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishImage.inputs:topicName", "/camera/image_raw"),
            ("PublishPointCloud.inputs:topicName", "/lidar/points"),
            ("PublishTF.inputs:topicName", "/tf"),
            ("PublishClock.inputs:topicName", "/clock")
        ]
    }
)

print("ROS 2 bridge OmniGraph created!")
```

**Save graph**: **File → Save** (graph persists in USD scene).

## Validation Checklist

- [ ] **Camera publishes** to `/camera/image_raw` at 30 Hz
- [ ] **LiDAR publishes** to `/lidar/points` at 20 Hz
- [ ] **TF tree complete**: odom → base_link → camera_link/lidar_link
- [ ] **Clock published** to `/clock` (simulation time)
- [ ] **RViz visualization** shows camera and LiDAR data correctly
- [ ] **ROS 2 bag recording** works (play back without issues)

## Exercise: Record and Replay ROS 2 Bag

**Task**: Record sensor data from Isaac Sim, replay for offline processing.

**Steps**:
1. Configure bridge (camera + LiDAR + TF + clock)
2. Start Isaac Sim, click Play
3. Record bag for 30 seconds:
   ```bash
   ros2 bag record /camera/image_raw /lidar/points /tf /clock -o test_bag
   ```
4. Stop recording, stop Isaac Sim
5. Replay bag:
   ```bash
   ros2 bag play test_bag --clock
   ```
6. View in RViz (should look identical to live simulation)

**Success criteria**:
- [ ] Bag recorded without errors
- [ ] Replay shows camera and LiDAR data
- [ ] TF transforms available during replay
- [ ] Timestamps match original recording

## Try With AI

**Setup**: Open ChatGPT or Claude.

**Part 1: QoS Troubleshooting**
Ask AI:
```
My Isaac Sim camera publishes to /camera/image_raw with QoS profile SENSOR_DATA. My ROS 2 subscriber uses RELIABLE QoS. I'm seeing "incompatible QoS" warnings and no data. What's the mismatch and how do I fix it?
```

**Part 2: Namespace Organization**
Tell AI your multi-robot scenario:
```
I'm simulating 3 humanoid robots in Isaac Sim. Each needs separate camera and LiDAR topics:
- Robot1: /robot1/camera/image_raw, /robot1/lidar/points
- Robot2: /robot2/camera/image_raw, /robot2/lidar/points
- Robot3: /robot3/camera/image_raw, /robot3/lidar/points

How do I configure OmniGraph to namespace topics per robot?
```

**Part 3: Clock Sync Debugging**
After implementing clock publishing:
```
I enabled /clock publishing from Isaac Sim. My ROS 2 node has use_sim_time:=true. But timestamps in my bag file show wall time, not simulation time. What am I missing?
```

**Part 4: Convergence Check**
After completing bridge setup:
- What publication rates emerged from your use case constraints?
- How did AI help debug QoS and clock issues?
- What namespace organization did you implement?
- Is your bridge ready for Chapter 8 (VSLAM integration)?

**Safety note**: ROS 2 bags with high-resolution sensors can grow large quickly (1GB+ for 1 minute of 1280x720 camera at 30 Hz). Monitor disk space during recording.
