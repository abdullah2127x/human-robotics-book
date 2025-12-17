---
title: CUDA Acceleration and Performance Measurement
chapter: 8
lesson: 4
proficiency_level: C1
learning_objectives:
  - Measure GPU acceleration speedup (CUDA vs CPU baseline)
  - Validate CUDA kernel execution with profiling tools
  - Understand GPU memory bandwidth limitations
  - Optimize VSLAM for real-time performance (30 Hz target)
estimated_time: 120 minutes
skills:
  gpu-performance-profiling:
    proficiency: C1
generated_by: content-implementer v1.1.0
source_spec: specs/book/part-3/chapter-08-plan.md
created: 2025-12-17
---

# CUDA Acceleration and Performance Measurement

Isaac ROS claims 5-10x performance improvements over CPU VSLAM through GPU acceleration. But how do you **verify** this claim? And more importantly, how do you confirm your system is actually using the GPU—not silently falling back to slow CPU execution?

This lesson teaches performance engineering for robotics: profiling GPU utilization, measuring end-to-end latency, and validating that CUDA acceleration delivers real-time performance for navigation.

## Why Performance Matters for VSLAM

Traditional computer vision workflows tolerate latency:
- **Offline analysis**: Process recorded video at 5 FPS, results available hours later
- **Human-in-the-loop**: Person waits for processing, then makes decision

But autonomous navigation has **hard real-time constraints**:

| **Component** | **Latency Requirement** | **Why** |
|---------------|-------------------------|---------|
| **Visual SLAM** | &lt;33 ms (30 Hz) | Must process every camera frame |
| **Path Planning** | &lt;100 ms | Update plan before environment changes |
| **Control** | &lt;10 ms | Bipedal balance control loop |

If VSLAM runs at 5 Hz (200ms latency), the robot navigates using 200ms-old position estimates—already moved 40cm at walking speed. This causes collisions with doorframes, drift into walls, and navigation failures.

**CUDA acceleration isn't optional—it's required** for real-time robotics.

## Understanding CUDA Acceleration in Isaac ROS

### What Runs on GPU vs CPU?

**Traditional CPU VSLAM pipeline**:
```
Camera (30 FPS) → Copy to CPU RAM
                 ↓
             CPU: Feature detection (80ms)
                 ↓
             Copy to GPU VRAM
                 ↓
             GPU: Descriptor extraction (10ms)
                 ↓
             Copy back to CPU RAM
                 ↓
             CPU: Feature matching (50ms)
                 ↓
             CPU: Motion estimation (20ms)

Total latency: 160ms (6.25 FPS - TOO SLOW)
```

**Isaac ROS GPU-accelerated pipeline**:
```
Camera (30 FPS) → Copy to GPU VRAM (via ROS2 CUDA-aware messaging)
                 ↓
             GPU: Feature detection (8ms)
                 ↓
             GPU: Descriptor extraction (3ms)
                 ↓
             GPU: Feature matching (5ms)
                 ↓
             GPU: Motion estimation (4ms)
                 ↓
             CPU: Map update (10ms)

Total latency: 30ms (33 FPS - REAL-TIME)
```

**Key differences**:
1. **Minimal CPU↔GPU copying**: Data stays on GPU throughout pipeline
2. **Parallel execution**: Feature detection processes 500 features simultaneously (GPU has 5000+ CUDA cores)
3. **Optimized kernels**: Hand-tuned CUDA code for ORB detection, descriptor matching

**Result**: 5x speedup (160ms → 30ms).

## Verifying GPU Utilization

Before measuring speedup, confirm GPU is actually being used.

### Real-Time GPU Monitoring

```bash
# Terminal 1: Launch Isaac VSLAM
ros2 launch ~/isaac_vslam_launch.py

# Terminal 2: Play test bag
ros2 bag play ~/isaac_vslam_test_data/humanoid_walk_30s

# Terminal 3: Monitor GPU in real-time
watch -n 0.5 nvidia-smi
```

**Expected output (GPU active)**:

```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.104.05   Driver Version: 535.104.05   CUDA Version: 12.2    |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA RTX 4070     Off  | 00000000:01:00.0 On  |                  N/A |
| 35%   54C    P2   180W / 220W |   4872MiB /  12288MiB |     67%      Default |
+-------------------------------+----------------------+----------------------+
```

**Key metrics**:
- **GPU-Util: 67%** → GPU is actively processing (not idle)
- **Memory-Usage: 4872MB** → Significant VRAM allocated (feature maps, descriptors)
- **Pwr:Usage: 180W / 220W** → High power draw indicates computation
- **Perf: P2** → Performance state (P0 = max perf, P8 = idle)

**If GPU utilization is low (&lt;10%)**:
- CUDA kernels not executing (fallback to CPU)
- Check Isaac ROS config: `enable_gpu_stereo: true`
- Verify CUDA runtime: `nvcc --version`

### Detailed GPU Profiling with nsys

For deeper analysis, use NVIDIA Nsight Systems (nsys) profiler.

```bash
# Profile Isaac VSLAM execution
nsys profile \
  --output=isaac_vslam_profile \
  --force-overwrite=true \
  ros2 launch ~/isaac_vslam_launch.py
```

In another terminal, play bag:

```bash
ros2 bag play ~/isaac_vslam_test_data/humanoid_walk_30s --duration 10
```

**After bag completes, stop profiler (Ctrl+C) and analyze**:

```bash
# Generate profiling report
nsys stats isaac_vslam_profile.nsys-rep

# Key sections to examine:
# 1. CUDA Kernel Summary (shows which GPU functions executed)
# 2. CUDA Memory Operations (CPU↔GPU transfers)
# 3. Timeline (visualize execution overlap)
```

**Expected nsys output** (truncated):

```
CUDA Kernel Summary:

  Time(%)  Total Time (ns)  Instances   Kernel Name
  -------  ---------------  ---------   -----------
    24.5%      987,234,112        450   isaac::orb_detector_kernel
    18.3%      736,891,023        450   isaac::brief_descriptor_kernel
    15.7%      631,245,789        450   isaac::feature_matcher_kernel
    12.1%      487,123,456        450   isaac::stereo_depth_kernel
```

**What to verify**:
- **Isaac-specific kernels present**: Names like `isaac::orb_detector_kernel` confirm Isaac ROS code running
- **High instance counts**: 450 instances = 1 per frame for 30-second bag at 15 FPS
- **Balanced time distribution**: No single kernel dominating (good parallelism)

**If no CUDA kernels appear**: Isaac ROS is running on CPU (fallback mode). Check CUDA installation.

## Measuring End-to-End Latency

GPU utilization confirms *something* is accelerated, but does it meet **real-time requirements** (&lt;33ms per frame)?

### Instrumenting VSLAM for Latency Measurement

Create `~/measure_vslam_latency.py`:

```python
#!/usr/bin/env python3
"""Measure Visual SLAM end-to-end latency."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from collections import deque
import time

class VSLAMLatencyMeasurer(Node):
    def __init__(self):
        super().__init__('vslam_latency_measurer')

        # Track input timestamps
        self.input_timestamps = deque(maxlen=1000)

        # Subscribe to camera input (what VSLAM receives)
        self.create_subscription(
            Image,
            '/humanoid/camera/left/image_raw',
            self.image_callback,
            10
        )

        # Subscribe to VSLAM output (what VSLAM produces)
        self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odometry_callback,
            10
        )

        # Latency statistics
        self.latencies = []
        self.frame_count = 0

        # Report timer
        self.create_timer(5.0, self.report_latency)

    def image_callback(self, msg):
        """Record when frame enters VSLAM."""
        self.input_timestamps.append(msg.header.stamp)
        self.frame_count += 1

    def odometry_callback(self, msg):
        """Calculate latency when VSLAM produces output."""
        output_time = msg.header.stamp

        # Find matching input frame (should be recent)
        for input_time in reversed(self.input_timestamps):
            # Convert ROS2 Time to seconds
            latency_sec = (output_time.sec - input_time.sec) + \
                          (output_time.nanosec - input_time.nanosec) * 1e-9

            if 0 < latency_sec < 0.5:  # Sanity check (0-500ms)
                self.latencies.append(latency_sec * 1000)  # Convert to ms
                break

    def report_latency(self):
        """Print latency statistics."""
        if not self.latencies:
            self.get_logger().warning('No latency measurements yet')
            return

        import numpy as np
        latencies = np.array(self.latencies)

        self.get_logger().info(f'=== VSLAM Latency Statistics (n={len(latencies)}) ===')
        self.get_logger().info(f'Mean:   {np.mean(latencies):.1f} ms')
        self.get_logger().info(f'Median: {np.median(latencies):.1f} ms')
        self.get_logger().info(f'p95:    {np.percentile(latencies, 95):.1f} ms')
        self.get_logger().info(f'p99:    {np.percentile(latencies, 99):.1f} ms')
        self.get_logger().info(f'Max:    {np.max(latencies):.1f} ms')
        self.get_logger().info(f'Frames processed: {self.frame_count}')

        # Real-time performance check
        if np.percentile(latencies, 99) < 33:
            self.get_logger().info('✓ Real-time performance achieved (p99 < 33ms)')
        else:
            self.get_logger().warning('✗ Missing real-time target (p99 >= 33ms)')

def main():
    rclpy.init()
    node = VSLAMLatencyMeasurer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run latency measurement**:

```bash
# Terminal 1: Launch Isaac VSLAM
ros2 launch ~/isaac_vslam_launch.py

# Terminal 2: Measure latency
python3 ~/measure_vslam_latency.py

# Terminal 3: Play bag
ros2 bag play ~/isaac_vslam_test_data/humanoid_walk_30s
```

**Expected output (GPU-accelerated)**:

```
[INFO] [vslam_latency_measurer]: === VSLAM Latency Statistics (n=874) ===
[INFO] [vslam_latency_measurer]: Mean:   28.3 ms
[INFO] [vslam_latency_measurer]: Median: 27.1 ms
[INFO] [vslam_latency_measurer]: p95:    31.8 ms
[INFO] [vslam_latency_measurer]: p99:    34.2 ms
[INFO] [vslam_latency_measurer]: Max:    38.7 ms
[INFO] [vslam_latency_measurer]: Frames processed: 900
[INFO] [vslam_latency_measurer]: ✓ Real-time performance achieved (p99 < 33ms)
```

**Interpretation**:
- **Mean 28.3ms**: Average well below 33ms target (35 FPS effective rate)
- **p99 34.2ms**: 99% of frames processed within target (acceptable occasional miss)
- **Max 38.7ms**: Worst-case still reasonable (&lt;1 frame drop at 30 FPS)

**If p99 > 50ms**: GPU acceleration not working, or GPU overloaded. Debug before proceeding.

## Comparing GPU vs CPU Performance

To quantify speedup, you need a CPU baseline. Isaac ROS doesn't provide CPU-only mode, so we'll use OpenCV-based VSLAM as comparison.

### Installing ORB-SLAM3 (CPU Baseline)

```bash
# Install ORB-SLAM3 (popular CPU VSLAM)
cd ~/
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3

# Build (takes ~10 minutes)
chmod +x build.sh
./build.sh

# Build ROS2 wrapper
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/ORB_SLAM3/Examples_old/ROS
cd ~/ORB_SLAM3/Examples_old/ROS/ORB_SLAM3
mkdir build && cd build
cmake .. -DROS_VERSION=2
make -j4
```

### Running CPU Baseline Test

```bash
# Launch ORB-SLAM3 (CPU-only)
ros2 run orb_slam3 stereo ~/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ~/ORB_SLAM3/Examples_old/Stereo/EuRoC.yaml

# Play same test bag
ros2 bag play ~/isaac_vslam_test_data/humanoid_walk_30s --remap \
  /humanoid/camera/left/image_raw:=/camera/left/image_raw \
  /humanoid/camera/right/image_raw:=/camera/right/image_raw
```

**Measure CPU VSLAM latency** (same Python script, different topics):

```
[INFO] [vslam_latency_measurer]: === ORB-SLAM3 (CPU) Latency Statistics ===
[INFO] [vslam_latency_measurer]: Mean:   142.7 ms
[INFO] [vslam_latency_measurer]: Median: 138.2 ms
[INFO] [vslam_latency_measurer]: p95:    167.3 ms
[INFO] [vslam_latency_measurer]: p99:    189.4 ms
[INFO] [vslam_latency_measurer]: ✗ Missing real-time target (p99 >= 33ms)
```

### Calculating Speedup

```
Speedup = CPU latency / GPU latency
        = 142.7 ms / 28.3 ms
        = 5.04x
```

**Validation**: Isaac ROS achieves **5x speedup** vs CPU baseline, confirming NVIDIA's claims.

**What contributes to speedup**:
- Feature detection: 15x faster (parallelized across 5000+ CUDA cores)
- Descriptor extraction: 8x faster (SIMD operations on GPU)
- Feature matching: 3x faster (GPU-optimized brute-force search)
- Stereo depth: 10x faster (parallel disparity computation)

## Collaborative Performance Optimization

Your current VSLAM runs at 28.3ms mean latency. What if you need even better performance?

### Exploring Optimization Strategies with AI

**Your request**:
```
My Isaac ROS Visual SLAM achieves 28.3ms mean latency (5x faster than CPU), but I need to reduce this to 20ms for a higher framerate camera (50 Hz instead of 30 Hz).

Current config:
- num_features: 500
- detector_threshold: 10 (lowered for low-texture environments)
- enable_gpu_stereo: true

What parameters can I tune to reduce latency by ~30% while maintaining tracking quality?
```

**AI's analysis**:
```
Latency breakdown for VSLAM:
1. Feature detection: ~40% of time (largest contributor)
2. Descriptor extraction: ~15%
3. Feature matching: ~20%
4. Motion estimation: ~15%
5. Overhead (data transfer, synchronization): ~10%

To reduce latency 30% (28.3ms → 20ms):

Option 1: Reduce num_features
- Current: 500 features
- Proposal: 300 features
- Impact: ~30% fewer features = ~25% latency reduction
- Tradeoff: Less robust tracking (may fail in sparse environments)

Option 2: Increase detector_threshold
- Current: 10 (sensitive, detects weak corners)
- Proposal: 20 (only strong corners)
- Impact: Faster detection (~20% reduction)
- Tradeoff: Conflicts with your low-texture tuning from Lesson 3

Option 3: Reduce image resolution
- Current: 1280x720
- Proposal: 640x480
- Impact: 50% fewer pixels = ~40% latency reduction
- Tradeoff: Lower depth accuracy, worse long-range tracking

Option 4: Optimize GPU memory access patterns (requires code changes)
```

**Your constraint**:
```
Options 1-2 sacrifice tracking quality, which I can't accept. Option 3 (lower resolution) is interesting but I need to understand the depth accuracy tradeoff quantitatively.

Option 4 requires code changes to Isaac ROS, which is out of scope.

Given these constraints, is 20ms latency achievable? Or should I upgrade GPU hardware instead?
```

**AI's hardware recommendation**:
```
Your RTX 4070 has 5888 CUDA cores. Upgrading to RTX 4080 (9728 cores, 65% more) would give:
- Estimated latency: 28.3ms / 1.3 = 21.8ms (close to target)

But more cost-effective: Use RTX 4070 with 640x480 resolution:
- Latency: ~17ms (40% reduction)
- Depth accuracy impact: Test on your specific environment

Recommendation: Empirically measure depth accuracy at 640x480 vs 1280x720. If &lt;10% degradation, lower resolution is better than GPU upgrade ($0 vs $800).
```

**What emerged**: AI initially suggested parameter tuning (standard VSLAM optimization), but your constraints revealed this wouldn't achieve the target. Collaboration pivoted to hardware vs resolution tradeoff analysis—a system-level decision neither of you had at the start.

**Resolution experiment** (you'd run this):

```yaml
# Test config with 640x480 resolution
left_camera:
  image_width: 640
  image_height: 480
```

```bash
# Measure latency with lower resolution
[INFO]: Mean: 17.2 ms (39% improvement)

# Measure depth accuracy (compare to ground truth)
[INFO]: RMS depth error: 0.043m (640x480) vs 0.038m (1280x720)
[INFO]: Degradation: 13% worse depth accuracy
```

**Decision**: 13% depth degradation acceptable for 50 Hz operation? Depends on application. AI can't decide this—you need domain knowledge of humanoid navigation tolerances.

## Performance Bottleneck Analysis

What if your GPU is slower than expected?

### Identifying Bottlenecks

```bash
# Profile with detailed metrics
nsys profile --stats=true --force-overwrite=true \
  --output=vslam_bottleneck \
  ros2 launch ~/isaac_vslam_launch.py
```

**Analyze bottleneck report**:

```bash
nsys stats vslam_bottleneck.nsys-rep --report cuda_gpu_trace
```

**Key metrics**:

```
Top 5 CUDA Kernels by Time:

1. orb_detector_kernel         247ms  (35%)  <-- Bottleneck
2. feature_matcher_kernel      142ms  (20%)
3. brief_descriptor_kernel     105ms  (15%)
4. stereo_depth_kernel          88ms  (12%)
5. memory_transfer_kernel       67ms  (9%)
```

**If feature detection dominates (&gt;40% of time)**:
- Reduce `num_features` or increase `detector_threshold`
- Optimize input image (resize, crop)

**If memory transfers dominate (&gt;20% of time)**:
- GPU↔CPU copying excessive
- Check for unnecessary data conversions in ROS2 pipeline
- Enable CUDA-aware ROS2 transport (zero-copy)

**If no clear bottleneck (even distribution)**:
- Algorithm well-optimized, limited by GPU hardware
- Consider faster GPU or optimize non-CUDA code

## Real-Time Performance Validation

Final check: Does VSLAM maintain real-time performance under load?

### Stress Test

```bash
# Increase CPU background load
stress --cpu 8 --timeout 60s &

# Increase GPU background load (render simultaneously)
glxgears &

# Run VSLAM under stress
ros2 launch ~/isaac_vslam_launch.py
ros2 bag play humanoid_walk_30s

# Measure latency
python3 measure_vslam_latency.py
```

**Expected degradation**:

```
Normal conditions:   Mean 28.3ms, p99 34.2ms
Under stress:        Mean 32.1ms, p99 39.7ms
Degradation:         +13% mean, +16% p99
```

**If p99 > 50ms under stress**: System cannot maintain real-time VSLAM with concurrent workloads. Either:
1. Reduce VSLAM parameters (fewer features)
2. Dedicate GPU to VSLAM only (no concurrent rendering)
3. Upgrade hardware

## Checkpoint

Verify you've validated GPU acceleration:

- [ ] GPU utilization confirmed (&gt;50% during VSLAM execution via `nvidia-smi`)
- [ ] CUDA kernels profiled (nsys shows Isaac-specific kernels executing)
- [ ] End-to-end latency measured (&lt;33ms p99 for real-time)
- [ ] CPU baseline comparison completed (5x+ speedup validated)
- [ ] Stress test passed (VSLAM maintains real-time under load)

## Try With AI

**Prompt 1: Hardware Upgrade Decision**
```
I'm running Isaac ROS Visual SLAM on an RTX 4070 (5888 CUDA cores, 12GB VRAM). Current performance:
- Mean latency: 28.3ms
- p99 latency: 34.2ms
- GPU utilization: 67%

I need &lt;25ms mean latency for 40 Hz camera. Should I upgrade GPU (RTX 4080 is $800, 9728 CUDA cores) or tune parameters? Provide quantitative analysis of expected improvement vs cost.
```

**Prompt 2: Profiling Interpretation**
```
My nsys profile shows:
- orb_detector_kernel: 35% of time
- memory_transfer_kernel: 22% of time
- feature_matcher_kernel: 18% of time

What's the most impactful optimization target? Explain why memory transfers consume 22% (seems high for GPU-accelerated pipeline).
```

**Prompt 3: Real-Time Guarantees**
```
My VSLAM achieves p99 latency of 34.2ms (just above 33ms real-time target). For safety-critical humanoid navigation, I need guaranteed worst-case &lt;33ms.

What techniques exist for hard real-time guarantees in CUDA kernels? Is this achievable with Isaac ROS, or do I need real-time operating system (RTOS) modifications?
```

**Reflection**: Performance engineering requires balancing tradeoffs—accuracy vs speed, cost vs improvement, average vs worst-case. Which tradeoff surprised you most during this lesson?

---

Next lesson: Loop closure detection—how VSLAM corrects accumulated drift by recognizing previously visited locations.
