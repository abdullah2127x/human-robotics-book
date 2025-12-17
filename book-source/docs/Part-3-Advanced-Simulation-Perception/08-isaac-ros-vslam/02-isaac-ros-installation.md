---
title: Installing and Configuring Isaac ROS
chapter: 8
lesson: 2
proficiency_level: B2
learning_objectives:
  - Install Isaac ROS packages and dependencies
  - Understand GXF (Graph Execution Framework) architecture
  - Configure Isaac Visual SLAM parameters
  - Record stereo camera ROS 2 bags from Isaac Sim
estimated_time: 120 minutes
skills:
  isaac-ros-setup:
    proficiency: B2
generated_by: content-implementer v1.1.0
source_spec: specs/book/part-3/chapter-08-plan.md
created: 2025-12-17
---

# Installing and Configuring Isaac ROS

Now that you understand what Visual SLAM solves, let's install NVIDIA's GPU-accelerated implementation. Unlike traditional ROS packages, Isaac ROS uses CUDA and the Graph Execution Framework (GXF) for real-time performance—which means installation requires careful dependency management.

This lesson walks through installation, explains the GXF architecture, and prepares sensor data for VSLAM testing in upcoming lessons.

## Prerequisites Check

Before installing Isaac ROS, verify these requirements:

### Hardware

```bash
# Check NVIDIA GPU
lspci | grep -i nvidia
# Expected: Shows your NVIDIA GPU model (e.g., RTX 4070)

# Check GPU memory
nvidia-smi
# Expected: Shows GPU with 8GB+ VRAM, Driver Version 535+
```

If `nvidia-smi` fails, install NVIDIA drivers:

```bash
# Ubuntu 22.04 NVIDIA driver installation
sudo apt update
sudo apt install nvidia-driver-535
sudo reboot
```

### Software

```bash
# Check ROS 2 Humble
ros2 --version
# Expected: ros2 cli version 0.18.x (Humble)

# Check CUDA Toolkit
nvcc --version
# Expected: Cuda compilation tools, release 12.x
```

If CUDA Toolkit is missing:

```bash
# Install CUDA 12.x
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install cuda-toolkit-12-2
```

**Checkpoint**: All commands above succeed? Proceed to Isaac ROS installation.

## Installing Isaac ROS Packages

NVIDIA provides Isaac ROS through APT repositories for easy installation.

### Step 1: Add Isaac ROS APT Repository

```bash
# Add NVIDIA Isaac ROS repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add Isaac ROS package repository
sudo apt-get update && sudo apt-get install -y \
    curl \
    gnupg \
    lsb-release

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

### Step 2: Install Isaac ROS Visual SLAM

```bash
# Install Isaac ROS Visual SLAM and dependencies
sudo apt install ros-humble-isaac-ros-visual-slam

# Install stereo image processing tools
sudo apt install ros-humble-isaac-ros-stereo-image-proc

# Install necessary message types
sudo apt install ros-humble-vision-msgs

# Install RViz plugins for VSLAM visualization
sudo apt install ros-humble-isaac-ros-rviz-plugin
```

**Output**: Installation downloads ~500MB of packages. Takes 5-10 minutes depending on connection speed.

### Step 3: Verify Installation

```bash
# Source ROS 2 workspace
source /opt/ros/humble/setup.bash

# Check Isaac ROS Visual SLAM package exists
ros2 pkg list | grep isaac_ros_visual_slam
# Expected output: isaac_ros_visual_slam

# Verify GXF runtime installed
ls /opt/nvidia/graph-composer/
# Expected: Shows GXF installation directory
```

**If verification fails**: Check error messages. Common issues:
- CUDA not in PATH: Add `export PATH=/usr/local/cuda-12.2/bin:$PATH` to `~/.bashrc`
- Missing dependencies: Run `rosdep install --from-paths . --ignore-src -r -y`

## Understanding GXF Architecture

Before configuring VSLAM, you need to understand how Isaac ROS processes data—it's fundamentally different from traditional ROS 2 nodes.

### Traditional ROS 2 Node Architecture

```
Camera Topic → ROS 2 Node (Python/C++) → Processing → Output Topic
                    ↑
              Single-threaded CPU processing
```

**Bottleneck**: Data copies between CPU and GPU for every operation.

### GXF (Graph Execution Framework) Architecture

```
Camera Topic → GXF Graph (CUDA kernels on GPU) → Output Topic
                    ↑
              Pipelined GPU processing with minimal data copying
```

**Key concepts**:

**1. Codelets**: Processing units (like ROS 2 nodes but GPU-accelerated)
**2. Entities**: Data containers flowing through graph
**3. Connections**: Data paths between codelets (zero-copy when possible)
**4. Scheduler**: Manages parallel execution across GPU streams

**Why GXF matters for VSLAM**:

Traditional VSLAM pipeline:
```
Feature detection (CPU) → Copy to GPU → Descriptor extraction (GPU) →
Copy to CPU → Feature matching (CPU)
```

Isaac ROS GXF pipeline:
```
Feature detection (GPU) → Descriptor extraction (GPU) → Feature matching (GPU)
(All on GPU, minimal copying)
```

**Result**: 5-10x performance improvement.

You don't write GXF code directly—Isaac ROS packages provide pre-built GXF graphs you configure via YAML files.

## Configuring Isaac Visual SLAM

Isaac ROS Visual SLAM uses YAML configuration files to control algorithm parameters.

### Basic Configuration File

Create `~/isaac_vslam_config.yaml`:

```yaml
/**:
  ros__parameters:
    # Camera parameters
    num_cameras: 2  # Stereo camera (left + right)

    # Feature detection parameters
    detector_type: "ORB"  # ORB features (fast, rotation-invariant)
    num_features: 500     # Features to track per frame
    detector_threshold: 20 # Lower = more features (but slower)

    # Visual odometry parameters
    vo_enable: true           # Enable frame-to-frame tracking
    vo_min_parallax: 0.01     # Minimum parallax for new keyframe (radians)

    # Loop closure parameters
    loop_closure_enable: true       # Enable drift correction
    loop_closure_threshold: 0.1     # Similarity threshold (0-1)
    loop_closure_frequency: 1.0     # Check for loops every 1.0 seconds

    # Map parameters
    map_frame: "map"              # TF frame for global map
    odom_frame: "odom"            # TF frame for odometry
    base_frame: "base_link"       # Robot base frame

    # Performance tuning
    num_threads: 4                # CPU threads for non-GPU operations
    enable_gpu_stereo: true       # Use GPU for stereo matching

    # Debugging
    enable_debug_mode: false      # Disable for production (adds latency)
    enable_slam_visualization: true  # Publish visualization markers
```

**Key parameters explained**:

**`num_features` (500)**: How many corners to track per frame
- Higher values → Better tracking in feature-rich environments, slower processing
- Lower values → Faster processing, risk of tracking loss in sparse environments
- Default 500 balances performance and robustness

**`detector_threshold` (20)**: Sensitivity of corner detection
- Lower values (10-15) → Detect more features (weaker corners included)
- Higher values (30-50) → Only strong corners (fewer but more reliable)
- You'll tune this in Lesson 3 when debugging tracking failures

**`loop_closure_threshold` (0.1)**: How similar images must be to trigger loop closure
- Lower values (0.05-0.1) → Aggressive loop closure (may have false positives)
- Higher values (0.2-0.3) → Conservative (misses some valid loops)
- You'll tune this in Lesson 5 when debugging map drift

### Stereo Camera Configuration

Isaac VSLAM requires stereo camera calibration parameters. For Isaac Sim cameras, these are provided automatically. For real hardware, you'd calibrate manually.

Create `~/stereo_camera_info.yaml`:

```yaml
left_camera:
  image_width: 1280
  image_height: 720
  camera_matrix:
    rows: 3
    cols: 3
    data: [700.0, 0.0, 640.0,
           0.0, 700.0, 360.0,
           0.0, 0.0, 1.0]
  distortion_model: plumb_bob
  distortion_coefficients:
    rows: 1
    cols: 5
    data: [0.0, 0.0, 0.0, 0.0, 0.0]  # Isaac Sim cameras are already rectified

right_camera:
  image_width: 1280
  image_height: 720
  camera_matrix:
    rows: 3
    cols: 3
    data: [700.0, 0.0, 640.0,
           0.0, 700.0, 360.0,
           0.0, 0.0, 1.0]
  distortion_model: plumb_bob
  distortion_coefficients:
    rows: 1
    cols: 5
    data: [0.0, 0.0, 0.0, 0.0, 0.0]

  # Stereo transformation (right camera relative to left)
  T:  # Translation: 0.1m baseline (distance between cameras)
    [0.1, 0.0, 0.0]
  R:  # Rotation: Identity (cameras parallel)
    [1.0, 0.0, 0.0,
     0.0, 1.0, 0.0,
     0.0, 0.0, 1.0]
```

**Stereo baseline (0.1m)**: Distance between left and right cameras. Larger baseline improves depth accuracy but limits close-range performance.

## Creating Isaac ROS Launch File

Launch files coordinate ROS 2 nodes and GXF graphs. Create `~/isaac_vslam_launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch Isaac ROS Visual SLAM with stereo cameras."""

    # Launch configuration
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'config_file',
            default_value='~/isaac_vslam_config.yaml',
            description='Path to Isaac VSLAM configuration file'
        ),

        # Isaac ROS Visual SLAM node
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='isaac_vslam',
            parameters=[config_file],
            remappings=[
                ('stereo_camera/left/image', '/humanoid/camera/left/image_raw'),
                ('stereo_camera/right/image', '/humanoid/camera/right/image_raw'),
                ('stereo_camera/left/camera_info', '/humanoid/camera/left/camera_info'),
                ('stereo_camera/right/camera_info', '/humanoid/camera/right/camera_info'),
            ],
            output='screen'
        ),
    ])
```

**Topic remapping**: Connects Isaac VSLAM inputs to your Isaac Sim humanoid camera topics. Adjust `/humanoid/camera/*` paths to match your robot's actual topic names.

**Verification**:

```bash
# Test launch file (without running VSLAM)
ros2 launch ~/isaac_vslam_launch.py --show-args
# Expected: Shows launch arguments, no errors
```

## Recording Stereo Camera Data from Isaac Sim

Before running VSLAM, you need test data. Let's record a stereo camera ROS 2 bag from Isaac Sim.

### Step 1: Start Isaac Sim with Humanoid

```bash
# Launch Isaac Sim (from Chapter 7)
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac_sim.sh

# In Isaac Sim GUI:
# 1. Open your humanoid scene from Chapter 7
# 2. Verify stereo cameras are configured on humanoid head
# 3. Start simulation (Play button)
```

### Step 2: Verify Camera Topics

In a terminal:

```bash
source /opt/ros/humble/setup.bash

# List active topics
ros2 topic list | grep camera
# Expected output:
#   /humanoid/camera/left/image_raw
#   /humanoid/camera/right/image_raw
#   /humanoid/camera/left/camera_info
#   /humanoid/camera/right/camera_info

# Check camera publishing rate
ros2 topic hz /humanoid/camera/left/image_raw
# Expected: ~30 Hz (standard camera framerate)
```

### Step 3: Record ROS 2 Bag

```bash
# Create directory for test data
mkdir -p ~/isaac_vslam_test_data
cd ~/isaac_vslam_test_data

# Record stereo camera topics for 30 seconds
ros2 bag record \
  /humanoid/camera/left/image_raw \
  /humanoid/camera/right/image_raw \
  /humanoid/camera/left/camera_info \
  /humanoid/camera/right/camera_info \
  /tf \
  /tf_static \
  --duration 30 \
  --output humanoid_walk_30s

# In Isaac Sim: Control humanoid to walk forward in environment
# (Use your Chapter 7 navigation controls)
```

**Recording tips**:
- Walk humanoid through **textured environment** (not blank hallways)
- Include **moderate rotation** (not just straight line)
- Avoid **rapid movements** (causes motion blur)
- 30 seconds provides ~900 frames at 30 Hz (sufficient for testing)

**Output**:

```
[INFO] [rosbag2_recorder]: Subscribed to topic '/humanoid/camera/left/image_raw'
[INFO] [rosbag2_recorder]: Subscribed to topic '/humanoid/camera/right/image_raw'
[INFO] [rosbag2_recorder]: Recording to 'humanoid_walk_30s'
[INFO] [rosbag2_recorder]: Recording stopped
```

### Step 4: Verify Bag Contents

```bash
# Inspect recorded bag
ros2 bag info humanoid_walk_30s

# Expected output shows:
# Duration: 30.x s
# Topics:
#   /humanoid/camera/left/image_raw: ~900 messages (sensor_msgs/msg/Image)
#   /humanoid/camera/right/image_raw: ~900 messages
#   /humanoid/camera/left/camera_info: ~900 messages
#   /humanoid/camera/right/camera_info: ~900 messages
#   /tf: Variable messages
#   /tf_static: 1-2 messages
```

**If message counts are low (&lt;100)**: Camera wasn't publishing during recording. Verify Isaac Sim simulation was running.

**If bag size is huge (&gt;2GB)**: Image compression not enabled. For testing, this is fine. For production, compress images:

```bash
# Record with compression (optional)
ros2 bag record [...topics...] --compression-mode file --compression-format zstd
```

## Testing Isaac ROS Installation

Let's verify Isaac VSLAM can process your recorded bag (without analyzing output quality yet—that's Lesson 3).

### Quick Smoke Test

```bash
# Terminal 1: Play recorded bag (slowly for initial testing)
ros2 bag play humanoid_walk_30s --rate 0.5

# Terminal 2: Launch Isaac VSLAM
ros2 launch ~/isaac_vslam_launch.py config_file:=~/isaac_vslam_config.yaml

# Terminal 3: Check VSLAM output topics
ros2 topic list | grep visual_slam
# Expected output:
#   /visual_slam/tracking/slam_path
#   /visual_slam/tracking/odometry
#   /visual_slam/map/points
```

**Success indicators**:
- No error messages in Terminal 2
- VSLAM publishes to output topics
- GPU utilization increases (check `nvidia-smi` in Terminal 4)

**Common errors**:

**Error: "CUDA out of memory"**
- Reduce `num_features` in config (500 → 250)
- Close other GPU-using applications

**Error: "Failed to initialize GXF graph"**
- Check CUDA Toolkit installed: `nvcc --version`
- Verify GPU drivers: `nvidia-smi`

**Error: "No camera_info received"**
- Bag recording missing camera_info topics
- Re-record with all 4 camera topics

## Configuration Files Summary

You now have three critical files:

1. **`~/isaac_vslam_config.yaml`**: Algorithm parameters (features, thresholds, loop closure)
2. **`~/stereo_camera_info.yaml`**: Camera calibration (intrinsics, stereo baseline)
3. **`~/isaac_vslam_launch.py`**: Launch file (node configuration, topic remapping)

In Lesson 3, you'll tune parameters in `isaac_vslam_config.yaml` while debugging tracking failures.

## Checkpoint

Verify you've completed setup:

- [ ] Isaac ROS Visual SLAM installed (`ros2 pkg list | grep isaac_ros_visual_slam` succeeds)
- [ ] CUDA Toolkit verified (`nvcc --version` shows CUDA 12.x)
- [ ] Configuration files created (3 files in home directory)
- [ ] Launch file tested (`ros2 launch ~/isaac_vslam_launch.py --show-args` succeeds)
- [ ] Stereo camera bag recorded (30 seconds, 4 topics, ~900 messages each)
- [ ] Smoke test passed (Isaac VSLAM processes bag without errors)

## Try With AI

**Setup**: Open ChatGPT or your preferred AI tool.

**Prompt 1: Troubleshooting Installation**
```
I'm installing Isaac ROS Visual SLAM on Ubuntu 22.04 with ROS 2 Humble and an RTX 4070 GPU. After running `sudo apt install ros-humble-isaac-ros-visual-slam`, I get this error when launching:

[ERROR] [isaac_ros_visual_slam]: Failed to load GXF extension

What are the three most likely causes and how do I diagnose each?
```

**Prompt 2: Parameter Understanding**
```
In Isaac ROS Visual SLAM config, I have two parameters:
- detector_threshold: 20
- num_features: 500

Explain the relationship between these parameters. If I lower detector_threshold to 10, should I also change num_features? Why?
```

**Prompt 3: Recording Optimal Test Data**
```
I'm about to record a ROS 2 bag of stereo camera data for testing Visual SLAM. My humanoid robot is in an Isaac Sim office environment. What specific movements should I perform during the 30-second recording to create challenging but valid test data for VSLAM debugging?
```

**Self-check**: Can you explain your configuration choices based on VSLAM fundamentals from Lesson 1? If parameters feel like "magic numbers," revisit the Feature Detection and Tracking sections.

---

Next lesson, you'll run Isaac Visual SLAM on your recorded bag and intentionally cause tracking failures to understand how feature detection and motion estimation work (and fail) in practice.
