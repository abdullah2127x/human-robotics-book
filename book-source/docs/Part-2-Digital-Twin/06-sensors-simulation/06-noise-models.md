---
title: "Lesson 6: Sensor Noise Models and Realistic Data"
chapter: 6
lesson: 6
proficiency_level: B2
learning_objectives:
  - Configure sensor noise matching real hardware specifications
  - Validate simulation data against sensor datasheets
  - Apply filtering and smoothing to reduce noise
  - Design noise-aware algorithms
estimated_time: 120 minutes
generated_by: content-implementer v1.0.0
created: 2025-12-16
---

# Lesson 6: Sensor Noise Models and Realistic Data

## Why Noise Matters for Robotics

**Poor noise modeling leads to sim-to-real transfer failure:**

- **Too clean (no noise)**: Algorithm works perfectly in simulation, fails on real robot
- **Too noisy (unrealistic)**: Algorithm struggles even in simulation, unnecessarily fragile
- **Wrong noise profile (doesn't match hardware)**: Algorithm tuned for wrong error characteristics

**Correct noise modeling:**
- Validates algorithms against realistic sensor behavior
- Prevents over-optimizing for simulation
- Builds robust algorithms that transfer to hardware

## Real Sensor Specifications

Let's examine actual datasheets and match simulation:

**Sick TiM781S LiDAR**:
```
Measurement accuracy: ±60mm @ 1m, ±6% @ >2m
Resolution: 0.25°
Max range: 25m
Typical noise: Gaussian ~25mm
```

**Intel RealSense D435**:
```
Depth accuracy: ±1% of distance (0.5-3m) or ±2% (>3m)
Example: 1m depth ±10mm, 3m depth ±30mm
Noise: ~2-3mm @ 1m, ~15-20mm @ 3m
Lateral resolution: 0.4mm/pixel @ 1m
```

**MPU-6050 IMU**:
```
Accelerometer: ±16g range, noise density 50 µg/√Hz
Gyroscope: ±2000°/s range, noise density 0.005°/s/√Hz
Bias stability: ±50 mg (accel), ±0.3°/s (gyro)
```

## Configuring Realistic Noise

**Step 1: Find sensor datasheet**

Real sensors have published specifications. Example search:
```
"Velodyne HDL-32E" + "specification" + "noise" + "accuracy"
```

**Step 2: Extract accuracy specifications**

From datasheet, identify:
- Noise type (Gaussian, fixed, range-dependent)
- Magnitude (standard deviation or absolute error)
- Frequency dependence (is noise constant or scale-dependent)

**Step 3: Apply to Gazebo configuration**

```xml
<!-- Real sensor accuracy: ±2cm @ 10m -->
<sensor name="lidar" type="gpu_lidar">
  <ray>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev>  <!-- 2cm -->
    </noise>
  </ray>
</sensor>

<!-- Real sensor accuracy: ±1% of distance + 50mm baseline -->
<!-- This requires custom plugin code (Gazebo doesn't support formula-based noise directly) -->
```

**Gazebo noise types** (from SDF specification):
- `gaussian`: Normal distribution (configurable mean, stddev)
- `custom`: Custom implementation (requires plugin code)

## Distance-Dependent Noise Implementation

Real depth sensors have noise that increases with distance:

```python
# In custom depth processing
def add_realistic_depth_noise(depth_value):
    """
    Real sensor: ±1% of distance + 5mm baseline
    """
    noise_sigma = np.sqrt((0.01 * depth_value)**2 + (0.005)**2)
    noise = np.random.normal(0, noise_sigma)
    noisy_depth = depth_value + noise
    return np.clip(noisy_depth, 0.1, 10.0)  # Clip to valid range
```

Apply in ROS 2 processing node:

```python
class DepthNoiseSimulator(Node):
    def depth_callback(self, msg):
        # Convert message to numpy
        depth_image = self.bridge.imgmsg_to_cv2(msg)

        # Apply realistic noise
        noisy_depth = np.zeros_like(depth_image)
        for i in range(depth_image.shape[0]):
            for j in range(depth_image.shape[1]):
                depth_m = depth_image[i, j] / 1000.0  # mm to m
                noisy_depth[i, j] = int(1000 * self.add_noise(depth_m))

        # Publish noisy version
        noisy_msg = self.bridge.cv2_to_imgmsg(noisy_depth, encoding="16UC1")
        self.depth_publisher.publish(noisy_msg)

    def add_noise(self, depth_m):
        """Realistic depth noise: ±1% + 5mm baseline"""
        sigma = np.sqrt((0.01 * depth_m)**2 + 0.005**2)
        return depth_m + np.random.normal(0, sigma)
```

## Filtering Noisy Sensor Data

**Moving average filter** (simple smoothing):

```python
class SimpleFilter(Node):
    def __init__(self):
        super().__init__('simple_filter')
        self.window_size = 5
        self.buffer = deque(maxlen=self.window_size)

    def sensor_callback(self, msg):
        self.buffer.append(msg.value)

        if len(self.buffer) == self.window_size:
            filtered = np.mean(self.buffer)
            self.get_logger().info(f'Raw: {msg.value:.3f}, Filtered: {filtered:.3f}')
```

**Kalman filter** (optimal for Gaussian noise):

```python
class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter')
        self.x_est = 0.0  # Estimated state
        self.p_est = 1.0  # Estimation error covariance
        self.q = 0.01     # Process noise
        self.r = 0.1      # Measurement noise

    def sensor_callback(self, msg):
        z = msg.value  # Measurement

        # Predict
        x_pred = self.x_est
        p_pred = self.p_est + self.q

        # Update
        k = p_pred / (p_pred + self.r)  # Kalman gain
        self.x_est = x_pred + k * (z - x_pred)
        self.p_est = (1 - k) * p_pred

        self.get_logger().info(f'Filtered: {self.x_est:.3f}')
```

## Outlier Detection and Rejection

Real sensors sometimes produce bad measurements (reflections, shadows):

```python
class OutlierRejection(Node):
    def __init__(self):
        super().__init__('outlier_rejection')
        self.prev_value = 0.0
        self.max_delta = 0.5  # Max change per measurement

    def sensor_callback(self, msg):
        z = msg.value

        # Reject if change too large
        if abs(z - self.prev_value) > self.max_delta:
            self.get_logger().warn(f'Rejected outlier: {z}')
            z = self.prev_value  # Use previous value

        self.prev_value = z
        self.get_logger().info(f'Accepted: {z:.3f}')
```

## Validation: Comparing Simulation to Real

**Method**: Collect data from both real and simulated sensors, compare distributions.

```python
import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt

# Load real sensor bag
real_bag = rosbag2_py.SequentialReader()
real_bag.open(rosbag2_py.StorageOptions(uri='real_sensor_data', storage_id='sqlite3'))
real_values = []

while real_bag.has_next():
    topic, msg, _ = real_bag.read_next()
    if topic == '/sensor_data':
        real_values.append(msg.value)

# Load simulated bag
sim_bag = rosbag2_py.SequentialReader()
sim_bag.open(rosbag2_py.StorageOptions(uri='sim_sensor_data', storage_id='sqlite3'))
sim_values = []

while sim_bag.has_next():
    topic, msg, _ = sim_bag.read_next()
    if topic == '/sensor_data':
        sim_values.append(msg.value)

# Compare statistics
real_values = np.array(real_values)
sim_values = np.array(sim_values)

print('Real sensor:')
print(f'  Mean: {np.mean(real_values):.4f}')
print(f'  Std: {np.std(real_values):.4f}')
print(f'  Min: {np.min(real_values):.4f}')
print(f'  Max: {np.max(real_values):.4f}')

print('Simulated sensor:')
print(f'  Mean: {np.mean(sim_values):.4f}')
print(f'  Std: {np.std(sim_values):.4f}')
print(f'  Min: {np.min(sim_values):.4f}')
print(f'  Max: {np.max(sim_values):.4f}')

# Plot histograms
plt.figure(figsize=(12, 4))
plt.subplot(1, 2, 1)
plt.hist(real_values, bins=50, alpha=0.7, label='Real')
plt.hist(sim_values, bins=50, alpha=0.7, label='Simulated')
plt.xlabel('Sensor Value')
plt.ylabel('Count')
plt.legend()

plt.subplot(1, 2, 2)
plt.plot(real_values[:100], label='Real', alpha=0.7)
plt.plot(sim_values[:100], label='Simulated', alpha=0.7)
plt.xlabel('Sample')
plt.ylabel('Value')
plt.legend()
plt.tight_layout()
plt.savefig('sensor_comparison.png')
```

## Collaborative Noise Calibration

**Scenario**: Your algorithm works well with clean simulated data but fails with filtered real sensor data.

**Initial hypothesis**: "Real sensors are too noisy. I need better noise filtering."

**AI colleague suggests**: "Before assuming filtering is the solution, let's analyze the real noise. What's the actual distribution?"

**You run analysis**: Statistics show real sensor noise is not Gaussian—it has occasional spikes.

**AI responds**: "That's common. Real sensors have non-Gaussian noise components. Outliers and glitches aren't Gaussian. You need adaptive filtering, not just Gaussian smoothing."

**You implement**:
```python
# Instead of pure Kalman (assumes Gaussian noise)
# Use hybrid approach: Kalman + outlier rejection

class AdaptiveFilter(Node):
    def sensor_callback(self, msg):
        z = msg.value

        # Step 1: Outlier detection
        if abs(z - self.prev_value) > 2 * self.expected_noise:
            self.get_logger().warn(f'Outlier detected: {z}')
            z = self.prev_value  # Reject

        # Step 2: Kalman filtering (Gaussian noise)
        self.x_est = self.kalman_update(z)

        # Step 3: Trust algorithm output (less sensitivity to remaining noise)
        self.algorithm_output = self.algorithm(self.x_est, confidence=0.8)
```

**Result**: Algorithm now handles realistic sensor behavior—both Gaussian noise and outliers.

## Exercise: Implement Noise Validation

**Task**: Configure sensors with realistic noise and validate against specification.

```bash
# 1. Modify URDF to add realistic noise
# 2. Record sensor data for 60 seconds
ros2 bag record -o noise_validation /lidar/points /depth_camera/depth /imu/data_raw -d 60

# 3. Analyze with script
python3 << 'EOF'
import rosbag2_py
import numpy as np

# Your task: Calculate statistics and verify they match real sensor specs
# Print: mean, std, min, max for each sensor
# Compare to real specifications from datasheets
EOF
```

## Try With AI

**Prompt 1: Noise configuration**
"Real LiDAR spec: ±6cm @ 10m distance. How should I configure Gazebo LiDAR noise to match? My Gazebo only supports constant noise (stddev), not distance-dependent."

**Prompt 2: Filter design**
"My depth camera data is noisy (±3cm), but my object detection is robust to noise. Should I add filtering in ROS 2, or configure sensor noise lower in Gazebo? What are the tradeoffs?"

**Prompt 3: Sim-to-real transfer**
"I tuned my obstacle detection algorithm in clean Gazebo simulation (no noise). It fails on real robot (noisy sensors). What's the best strategy: (a) add realistic noise to simulation? (b) Filter data in ROS 2? (c) Both?"

**Expected outcome**: Understand noise modeling, filtering strategies, and validation of simulation realism against real hardware.

**Safety note**: Simulation with realistic noise can reveal algorithm fragility before deploying to real robots. Use this to build robust, production-ready perception systems.
