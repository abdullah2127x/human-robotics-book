---
title: "Lesson 4: Depth Camera Simulation"
chapter: 6
lesson: 4
proficiency_level: B2
learning_objectives:
  - Configure depth camera for RGB-D data generation
  - Understand camera intrinsics and 3D reconstruction
  - Process depth images in Python
  - Apply intrinsic calibration parameters
estimated_time: 120 minutes
generated_by: content-implementer v1.0.0
created: 2025-12-16
---

# Lesson 4: Depth Camera Simulation

## RGB-D Fundamentals

Depth cameras (like RealSense, Kinect) output two synchronized streams:

**RGB Image**: Standard color image (8-bit per channel, 24-bit total)
**Depth Image**: Grayscale image where pixel value = distance from camera

Example from 640×480 camera:
```
RGB pixel at (320, 240):  [R:200, G:180, B:160]  (brownish)
Depth pixel at (320, 240): 1500  (1.5 meters from camera)

Together: "There's a brown surface 1.5 meters away at image center"
```

## Camera Intrinsics: The Math

Camera intrinsics define the relationship between 3D world coordinates and 2D image coordinates:

```
[u]   [f_x   0  c_x] [X]
[v] = [ 0  f_y  c_y] [Y]
[1]   [ 0   0    1 ] [Z]

Where:
- (u, v) = pixel coordinates in image (0 to width, 0 to height)
- (X, Y, Z) = 3D point relative to camera
- f_x, f_y = focal lengths (pixels)
- c_x, c_y = principal point (image center, pixels)
```

**Practical example**:

Humanoid's depth camera:
- Resolution: 640×480
- Horizontal FOV: 60°
- Assume square pixels (f_x = f_y = f)

Calculate focal length:
```
f = (width / 2) / tan(FOV / 2)
f = 320 / tan(30°)
f = 320 / 0.577
f ≈ 554 pixels
```

Camera info message:
```python
camera_info = CameraInfo()
camera_info.width = 640
camera_info.height = 480
camera_info.f_x = 554.0  # focal length x
camera_info.f_y = 554.0  # focal length y
camera_info.c_x = 320.0  # principal point x (center)
camera_info.c_y = 240.0  # principal point y (center)
camera_info.distortion = [0, 0, 0, 0, 0]  # No lens distortion (ideal lens)
```

## Depth Image Format

Depth values stored as 16-bit integers (millimeters):

```
Depth value: 1500 (16-bit unsigned)
Interpretation: 1500 mm = 1.5 meters
Missing/invalid depth: 0 or max value (65535 mm = 65.5 m)
```

Python code to process:

```python
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(
            Image, '/depth_camera/depth', self.depth_callback, 10)

    def depth_callback(self, msg):
        # Convert ROS message to numpy array
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

        # Convert to meters (divide by 1000)
        depth_meters = depth_image.astype(np.float32) / 1000.0

        # Find closest point
        valid_depths = depth_meters[depth_meters > 0]
        if len(valid_depths) > 0:
            closest = np.min(valid_depths)
            self.get_logger().info(f'Closest obstacle: {closest:.2f} m')

        # Find obstacles within 1 meter
        close_obstacles = np.sum(depth_image < 1000)  # < 1000 mm
        self.get_logger().info(f'Pixels closer than 1m: {close_obstacles}')
```

## Gazebo Depth Camera Configuration

**URDF sensor definition** (from Lesson 2, with detail):

```xml
<sensor name="depth_camera_sensor" type="depth_camera">
  <parent link="torso"/>
  <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
  <update_rate>30</update_rate>
  <camera>
    <!-- Field of view (larger = wider view) -->
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees radians -->

    <!-- Image resolution (more pixels = more detail, slower processing) -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>RGB8</format>  <!-- RGB color format -->
    </image>

    <!-- Depth range (near = closest, far = farthest detectable) -->
    <clip>
      <near>0.1</near>   <!-- 10 cm minimum (closer = error) -->
      <far>10.0</far>    <!-- 10 m maximum -->
    </clip>

    <!-- Lens distortion (0 = perfect lens, real cameras have some) -->
    <distortion>
      <k1>0.0</k1>   <!-- Radial distortion 1st order -->
      <k2>0.0</k2>   <!-- Radial distortion 2nd order -->
      <k3>0.0</k3>   <!-- Radial distortion 3rd order -->
      <p1>0.0</p1>   <!-- Tangential distortion 1st -->
      <p2>0.0</p2>   <!-- Tangential distortion 2nd -->
    </distortion>
  </camera>
</sensor>
```

## 3D Reconstruction from Depth

Convert 2D depth image to 3D point cloud:

```python
def depth_image_to_pointcloud(depth_image, camera_info, rgb_image=None):
    """
    Convert depth image to 3D point cloud using camera intrinsics

    Args:
        depth_image: (h, w) depth values in meters
        camera_info: CameraInfo with focal length, principal point
        rgb_image: (h, w, 3) optional RGB image for coloring

    Returns:
        points: (n_points, 3) xyz coordinates
        colors: (n_points, 3) RGB colors (or None)
    """
    h, w = depth_image.shape
    fx = camera_info.K[0]  # focal length x
    fy = camera_info.K[4]  # focal length y
    cx = camera_info.K[2]  # principal point x
    cy = camera_info.K[5]  # principal point y

    # Create pixel coordinate grid
    u = np.arange(w)
    v = np.arange(h)
    uu, vv = np.meshgrid(u, v)

    # Project to 3D using intrinsics
    z = depth_image  # depth values (meters)
    x = (uu - cx) * z / fx
    y = (vv - cy) * z / fy

    # Stack into point array
    points = np.stack([x, y, z], axis=-1)
    points = points.reshape(-1, 3)

    # Remove invalid points (zero depth)
    valid = z.flatten() > 0
    points = points[valid]

    # Optionally add colors
    if rgb_image is not None:
        rgb = rgb_image.reshape(-1, 3)
        colors = rgb[valid]
    else:
        colors = None

    return points, colors
```

## Real-World Camera Validation

**Intel RealSense D435** (real depth camera):
- Resolution: 848×480 (or less for faster processing)
- Horizontal FOV: 87°
- Depth range: 0.1–10m
- Noise: ±1–2cm at 1m
- Update rate: 30 fps

**Our simulation**:
- Resolution: 640×480
- FOV: 60°
- Range: 0.1–10m
- Noise: 0 (perfect lens, clean measurements)
- Update rate: 30 fps

**Differences**:
- Lower resolution (fewer pixels, faster)
- Narrower FOV (less coverage)
- No noise (too clean, unrealistic)

**Improvement**: Add realistic noise:

```xml
<!-- In plugin configuration -->
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>  <!-- 1cm noise -->
</noise>
```

## Collaborative Depth Camera Tuning

**Scenario**: Your object detection algorithm works in simulation but fails on real robot with depth camera.

**Your initial attempt**: "I'll match real camera resolution (848×480) to improve accuracy."

**AI colleague checks**: "Before increasing resolution, what's the actual failure mode? Is it missing objects or false positives?"

**You investigate**: "False positives—detecting obstacles that aren't really there."

**AI responds**: "That's often noise, not resolution. Real cameras have measurement noise. Your simulation is too clean. Let me ask: at what distance are false positives happening?"

**You check**: "Mostly at far distances (>3m). Close range is fine."

**AI suggests**: "Distance-dependent noise is key. Real cameras have worse accuracy far away. Try this: noise standard deviation proportional to depth. In simulation, add depth-dependent noise, or clip depth beyond 3m if that's your working range."

**You modify**:
```xml
<!-- Instead of increasing resolution -->
<!-- Add realistic noise and working range limit -->
<clip>
  <near>0.1</near>
  <far>3.5</far>  <!-- Changed from 10 to focus on working range -->
</clip>
<noise>
  <type>gaussian</type>
  <stddev>0.02</stddev>  <!-- 2cm noise, more realistic -->
</noise>
```

**Result**: Fewer false positives because:
1. Noise now similar to real camera
2. No spurious far-range measurements
3. Algorithm tuned for realistic data

## Exercise: Calculate Focal Length

**Given**:
- Camera resolution: 1280×720
- Horizontal FOV: 45 degrees

**Calculate**:
1. Maximum image coordinate on x-axis: 1280/2 = 640
2. Half FOV angle: 45/2 = 22.5°
3. Focal length: f_x = 640 / tan(22.5°) = 640 / 0.414 ≈ 1545 pixels

**Verify**: At 1 meter distance, a point at image edge (pixel 1280) should be at:
```
x = (1280 - 640) * 1 / 1545 = 640 / 1545 ≈ 0.41 meters
angle = atan2(0.41, 1) ≈ 22.3°  (matches expected FOV/2)
```

## Try With AI

**Prompt 1: Intrinsics calibration**
"My depth camera has a 60° horizontal FOV and outputs 640×480 images. Calculate the focal length assuming square pixels centered in image. If I detect an object at pixel (320, 100) and depth 2.0m, what are its 3D coordinates?"

**Prompt 2: Noise modeling**
"Real sensor datasheet: 'Depth accuracy ±1% of distance at 0.5–3m range.' My object detection fails when objects are 4–5m away in simulation. Should I add noise, clip the depth range, or increase resolution?"

**Prompt 3: Practical debugging**
"My simulated RGB-D camera shows point cloud in RViz, but it has gaps and noise. Real depth cameras often fail at transparent surfaces and bright sunlight. How should I simulate these failure modes in Gazebo?"

**Expected outcome**: Understand intrinsic calibration, depth-to-3D conversion, and realistic sensor simulation.

**Safety note**: When processing depth images, always validate depth values. Zero or max-value pixels indicate measurement failure (should be ignored), not actual obstacles.
