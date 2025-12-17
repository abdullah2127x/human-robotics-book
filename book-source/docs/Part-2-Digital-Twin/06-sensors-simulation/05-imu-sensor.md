---
title: "Lesson 5: IMU Sensor Simulation"
chapter: 6
lesson: 5
proficiency_level: B2
learning_objectives:
  - Configure IMU with accelerometer, gyroscope, and magnetometer
  - Implement realistic noise models (white noise, bias, random walk)
  - Process IMU data for orientation estimation
  - Understand sensor drift and bias compensation
estimated_time: 120 minutes
generated_by: content-implementer v1.0.0
created: 2025-12-16
---

# Lesson 5: IMU Sensor Simulation

## IMU Data Components

IMU measures three types of motion:

**Accelerometer**: Linear acceleration (including gravity)
- Message: `sensor_msgs/msg/Imu.linear_acceleration`
- Units: m/s²
- Range: Typically ±16g (±160 m/s²) for humanoids
- Gravity always present: 9.81 m/s² downward (sensor is affected by gravity)

**Gyroscope**: Angular velocity (rotation rate)
- Message: `sensor_msgs/msg/Imu.angular_velocity`
- Units: rad/s
- Range: Typically ±500 rad/s or more
- Measures: Rotation about x, y, z axes

**Magnetometer**: Magnetic field (optional in many IMUs)
- Message: Not part of standard Imu message (separate MagneticField message)
- Units: Tesla or Gauss
- Provides: Heading reference (points toward magnetic north)

## Gazebo IMU Configuration

From Lesson 2, the IMU definition:

```xml
<sensor name="imu_sensor" type="imu">
  <parent link="base_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <update_rate>100</update_rate>  <!-- High frequency for balance control -->
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>  <!-- rad/s noise -->
        </noise>
      </x>
      <!-- y, z similar -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.05</stddev>  <!-- m/s² noise -->
        </noise>
      </x>
      <!-- y, z similar -->
    </linear_acceleration>
  </imu>
</sensor>
```

## Processing IMU Data

ROS 2 IMU message:

```python
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import numpy as np

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)

    def imu_callback(self, msg):
        # Linear acceleration (m/s²) - includes gravity
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Angular velocity (rad/s)
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        # Orientation (quaternion from Gazebo simulation)
        q = msg.orientation  # [x, y, z, w]

        self.get_logger().info(
            f'Accel: [{ax:.2f}, {ay:.2f}, {az:.2f}] '
            f'Angular: [{wx:.3f}, {wy:.3f}, {wz:.3f}]'
        )
```

## Gravity Offset and Calibration

Accelerometer includes gravity (9.81 m/s² downward). For meaningful acceleration:

```python
def remove_gravity(accel_x, accel_y, accel_z, orientation_quat):
    """Remove gravity component from acceleration"""
    # Convert quaternion to rotation matrix
    qx, qy, qz, qw = orientation_quat
    R = np.array([
        [1-2*(qy**2+qz**2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)],
        [2*(qx*qy+qw*qz), 1-2*(qx**2+qz**2), 2*(qy*qz-qw*qx)],
        [2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx**2+qy**2)]
    ])

    # Acceleration in world frame
    accel_body = np.array([accel_x, accel_y, accel_z])
    accel_world = R @ accel_body

    # Remove gravity (9.81 m/s² downward in world frame)
    accel_world[2] -= 9.81

    # Convert back to body frame
    accel_true = R.T @ accel_world

    return accel_true
```

## Realistic Noise Models

**White Gaussian noise**: Random variation, normally distributed
```python
noise = np.random.normal(0, stddev)
measurement = true_value + noise
```

**Bias (constant offset)**: Sensor always off by fixed amount
```python
measurement = true_value + bias  # bias ≈ constant per sensor
```

**Random walk (drift)**: Bias changes slowly over time
```python
bias_prev = 0.0
for t in range(n_samples):
    bias = bias_prev + np.random.normal(0, drift_stddev)
    measurement = true_value + bias
    bias_prev = bias
```

**Real IMU example** (MPU-6050 accelerometer):
- White noise: ±0.05 m/s² (1-sigma)
- Bias: ±0.5 m/s² (typical ±50 mg)
- Bias stability: ~0.01 m/s² over 1 hour (very slow drift)

## Orientation Estimation from IMU

**Simple integration** (not recommended—accumulates error):
```python
q_prev = Quaternion(x=0, y=0, z=0, w=1)  # Identity
dt = 0.01  # 100 Hz sensor

while True:
    wx, wy, wz = get_angular_velocity()

    # Quaternion derivative
    qx_dot = 0.5 * (q_prev.w*wx - q_prev.z*wy + q_prev.y*wz)
    qy_dot = 0.5 * (q_prev.z*wx + q_prev.w*wy - q_prev.x*wz)
    qz_dot = 0.5 * (-q_prev.y*wx + q_prev.x*wy + q_prev.w*wz)
    qw_dot = 0.5 * (-q_prev.x*wx - q_prev.y*wy - q_prev.z*wz)

    # Integrate
    q_curr = Quaternion(
        x=q_prev.x + qx_dot * dt,
        y=q_prev.y + qy_dot * dt,
        z=q_prev.z + qz_dot * dt,
        w=q_prev.w + qw_dot * dt
    )

    # Normalize
    norm = np.sqrt(q_curr.x**2 + q_curr.y**2 + q_curr.z**2 + q_curr.w**2)
    q_curr = Quaternion(
        x=q_curr.x/norm, y=q_curr.y/norm,
        z=q_curr.z/norm, w=q_curr.w/norm
    )

    q_prev = q_curr
```

Problem: Gyro noise and bias accumulate → orientation drifts over time.

**Better approach**: Kalman filter or complementary filter
```python
# Complementary filter (gyro + accelerometer fusion)
alpha = 0.95  # Weight for gyro vs accelerometer

# Gyro-based orientation update (fast, but drifts)
q_gyro = integrate_gyro(wx, wy, wz, q_prev, dt)

# Accelerometer-based orientation update (noisy, but reference)
q_accel = estimate_from_accel(ax, ay, az)

# Fuse: Trust gyro more short-term, accel for long-term correction
q_fused = slerp(q_gyro, q_accel, alpha)
```

## Collaborative IMU Debugging

**Scenario**: Simulated humanoid loses balance during walking—doesn't correct properly.

**Initial observation**: "Simulation works perfectly with ideal sensors. Adding realistic IMU noise breaks balance."

**AI colleague investigates**: "Noise from sensors alone shouldn't break control. What does your balance controller use from IMU?"

**You explain**: "It integrates gyro to estimate orientation, then corrects based on error."

**AI responds**: "That's the problem. Gyro integration drifts with bias and noise. Does your controller use accelerometer for correction?"

**You check**: "No, it only uses gyro. I thought that was more reliable."

**AI adapts**: "Short-term, gyro is good. Long-term, accelerometer provides gravity reference for heading correction. Add a complementary filter to fuse them."

**You implement**:
```python
class BalanceController(Node):
    def imu_callback(self, msg):
        # Gyro integration for fast response
        self.q_gyro = self.integrate_gyro(msg)

        # Accel-based heading correction (slow, drifts less)
        self.q_accel = self.estimate_from_accel(msg)

        # Complementary filter: 95% gyro, 5% accel correction
        self.q_fused = slerp(self.q_gyro, self.q_accel, 0.95)

        # Control using fused orientation
        self.apply_balance_correction(self.q_fused)
```

**Result**: Balance improves because controller now handles sensor fusion properly.

## Exercise: Analyze IMU Noise

**Task**: Record IMU data while humanoid stands still, analyze noise characteristics.

```bash
# Record for 30 seconds
ros2 bag record -o imu_noise_test /imu/data_raw -d 30

# Analyze
python3 << 'EOF'
import rosbag2_py
import numpy as np

bag = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri='imu_noise_test', storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('cdr', 'cdr')
bag.open(storage_options, converter_options)

accel_x = []
while bag.has_next():
    topic, msg, _ = bag.read_next()
    if topic == '/imu/data_raw':
        accel_x.append(msg.linear_acceleration.x)

accel_x = np.array(accel_x)
print(f'Mean: {np.mean(accel_x):.4f}')
print(f'Std: {np.std(accel_x):.4f}')
print(f'Min: {np.min(accel_x):.4f}')
print(f'Max: {np.max(accel_x):.4f}')
EOF
```

Expected output (if properly calibrated):
```
Mean: 0.0500 (small bias)
Std: 0.0502 (matches configured stddev 0.05)
Min: -0.1234
Max: 0.1456
```

## Try With AI

**Prompt 1: Sensor fusion**
"I'm fusing IMU (gyro + accel) with magnetometer for heading. How should I weight each sensor in a complementary filter? Gyro is fast but drifts. Accel is noisy but stable. Magnetometer sometimes fails indoors."

**Prompt 2: Bias compensation**
"My humanoid stands still but reports acceleration of 0.3 m/s² in x-direction (should be 0). Is this sensor bias? How do I estimate and remove it without a calibration procedure?"

**Prompt 3: Real-world validation**
"Real sensor specs: MPU-9250 with gyro bias stability 0.01°/s. How should I configure Gazebo noise for realistic simulation? Should I model the bias as constant offset or random walk?"

**Expected outcome**: Understand IMU noise, sensor fusion, and drift compensation for realistic humanoid simulation.

**Safety note**: High IMU noise can cause control instability. Always validate that configured noise matches real sensor specs, or add filtering before passing to control algorithms.
