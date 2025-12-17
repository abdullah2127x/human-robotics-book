---
title: "Capstone: Real-Time VSLAM on Humanoid Robot"
chapter: 8
lesson: 8
proficiency_level: C1
learning_objectives:
  - Write specification-first for real-time VSLAM system
  - Compose accumulated skills (vslam-debugging, performance validation)
  - Implement real-time VSLAM with &lt;33ms latency guarantee
  - Validate VSLAM quality (&lt;5% drift, loop closure functional)
  - Integrate VSLAM with humanoid navigation (Chapter 9 prep)
estimated_time: 180 minutes
skills:
  vslam-system-integration:
    proficiency: C1
generated_by: content-implementer v1.1.0
source_spec: specs/book/part-3/chapter-08-plan.md
created: 2025-12-17
---

# Capstone: Real-Time VSLAM on Humanoid Robot

You've built VSLAM understanding across seven lessons:
- **Lesson 1**: VSLAM fundamentals (algorithm stages)
- **Lesson 2**: Isaac ROS installation (GPU acceleration)
- **Lesson 3**: Visual odometry debugging (feature tracking failures)
- **Lesson 4**: Performance measurement (CUDA profiling, latency)
- **Lesson 5**: Loop closure tuning (drift correction)
- **Lesson 6**: RViz diagnostics (visualization-based debugging)
- **Lesson 7**: Debugging skill creation (reusable intelligence)

Now integrate everything: deploy real-time Visual SLAM on your humanoid robot navigating Isaac Sim environments. This is **spec-driven development** (Layer 4)—write specification FIRST, then implement using accumulated knowledge.

## Why Specification-First for VSLAM Integration?

**Without specification**, VSLAM projects fail because:
- Unclear quality requirements ("good enough" is subjective)
- Missing constraints (real-time? offline? which GPU?)
- Scope creep (adding features without validating core functionality)

**With specification**, you define success criteria BEFORE building:
- Measurable quality targets (drift &lt;5%, latency &lt;33ms)
- Explicit constraints (hardware, environment, framerate)
- Non-goals (what you're NOT building)

This capstone uses specification-first methodology to orchestrate your accumulated VSLAM knowledge.

## Phase 1: Write the Specification

Create `~/capstone_vslam_spec.md`:

```markdown
# Real-Time Visual SLAM for Humanoid Indoor Navigation

**Version**: 1.0.0
**Created**: 2025-12-17
**Author**: [Your name]

## Intent

Implement real-time Visual SLAM providing accurate localization and mapping for humanoid robot navigating indoor office environments. VSLAM must meet stringent performance and accuracy requirements to enable autonomous navigation (integrated with Nav2 in Chapter 9).

## Success Criteria

### 1. Real-Time Performance
- [ ] **Processing latency**: p99 ≤ 33ms (30 Hz camera framerate)
- [ ] **No frame drops**: Process 95%+ of camera frames (dropped frames = outdated localization)
- [ ] **GPU acceleration verified**: &gt;60% GPU utilization during execution

**Validation method**: Run `measure_vslam_latency.py` script (Lesson 4) on 60-second navigation bag. Report p50, p95, p99 latency.

### 2. Trajectory Accuracy
- [ ] **Drift**: ≤5% of distance traveled (on loop trajectories)
- [ ] **RMS error**: ≤0.10m average position error vs ground truth
- [ ] **Loop closure functional**: Detects 80%+ of valid loops, &lt;2% false positives

**Validation method**: Navigate 20m square loop (return to start). Compare VSLAM trajectory to ground truth using `compute_vslam_error.py` (Lesson 3).

### 3. Map Quality
- [ ] **3D point cloud**: Contains recognizable environment structure (walls, furniture visible)
- [ ] **Feature density**: 2000-5000 mapped features (not &lt;500 sparse, not &gt;10000 excessive)
- [ ] **Map consistency**: No impossible geometry (overlapping walls, floating points)

**Validation method**: Visual inspection in RViz. Map should allow human observer to identify room layout.

### 4. Robustness
- [ ] **Low-texture tolerance**: Maintains tracking in hallways (white walls, &lt;150 features)
- [ ] **Loop closure reliability**: Corrects drift when returning to start (error &lt;0.2m after correction)
- [ ] **Recovery from tracking loss**: Relocalizes within 5 seconds if tracking lost

**Validation method**: Test bags in three environments (textured office, low-texture hallway, mixed). Tracking must not fail catastrophically.

## Constraints

### Hardware Requirements
- **GPU**: NVIDIA RTX 4070 (5888 CUDA cores, 12GB VRAM) — minimum spec
- **Camera**: Stereo camera (10cm baseline, 1280x720 @ 30 Hz)
- **CPU**: 8+ cores for non-GPU tasks (map management, loop closure database)
- **RAM**: 16GB+ (Isaac ROS + RViz + Isaac Sim)

### Software Environment
- **OS**: Ubuntu 22.04 LTS (native, not VM or WSL2)
- **ROS 2**: Humble
- **Isaac ROS**: isaac_ros_visual_slam package
- **CUDA**: 12.x with compatible NVIDIA drivers (535+)

### Environmental Constraints
- **Indoor only**: Office buildings, labs, homes (not outdoor)
- **Static environment**: No crowds of moving people (violates static world assumption)
- **Moderate lighting**: Not complete darkness, not direct sunlight through windows
- **Textured surfaces**: At least 30% of environment has detectable features

### Real-Time Constraints
- **Localization latency**: &lt;100ms end-to-end (camera capture → VSLAM output → navigation planner)
- **Localization update rate**: 20 Hz minimum (faster = better for bipedal balance control)
- **Computational budget**: VSLAM cannot monopolize GPU (Nav2 needs resources for path planning)

## Non-Goals (Explicit Exclusions)

### Out of Scope for This Capstone
- ❌ **Outdoor navigation**: Sunlight, high dynamic range, GPS fusion (future work)
- ❌ **Dense reconstruction**: Full 3D mesh (VSLAM provides sparse feature map only)
- ❌ **Multi-robot SLAM**: Single humanoid only (no collaborative mapping)
- ❌ **Dynamic object tracking**: Moving people/objects ignored (static world assumed)
- ❌ **Loop closure implementation**: Use Isaac ROS built-in (don't reimplement DBoW2)

**Why excluded**: Scope management for C1 proficiency. These topics require dedicated advanced chapters.

### Deferred to Chapter 9 (Nav2 Integration)
- ❌ **Path planning**: Nav2 handles planning (VSLAM provides localization only)
- ❌ **Obstacle avoidance**: Costmap building in Chapter 9
- ❌ **Behavior trees**: Navigation logic in Chapter 9

**Why deferred**: VSLAM is localization/mapping. Navigation is separate concern.

## Component Composition

This capstone composes accumulated knowledge from Lessons 1-7:

### From Lesson 1: VSLAM Fundamentals
- **Applied**: Understanding of 4-stage pipeline (detect → track → estimate → map)
- **Used for**: Interpreting failure modes (which stage broke?)

### From Lesson 2: Installation & Configuration
- **Applied**: Isaac ROS setup, GXF architecture understanding
- **Used for**: Launching VSLAM with optimized configuration

### From Lesson 3: Feature Tracking Debugging
- **Applied**: Parameter tuning for low-texture environments
- **Used for**: `detector_threshold` optimization, adaptive thresholding

### From Lesson 4: Performance Measurement
- **Applied**: Latency measurement, GPU profiling methodology
- **Used for**: Validating real-time constraint (p99 &lt;33ms)

### From Lesson 5: Loop Closure Tuning
- **Applied**: Threshold tuning for precision/recall tradeoff
- **Used for**: `loop_closure_threshold` calibration, false positive prevention

### From Lesson 6: RViz Diagnostics
- **Applied**: Visualization-based debugging (feature distribution, trajectory, pose graph)
- **Used for**: Real-time monitoring during capstone validation

### From Lesson 7: Debugging Skill
- **Applied**: Systematic diagnostic workflow (classify → analyze → fix)
- **Used for**: Debugging failures during capstone implementation

### Skill Invocation Strategy

When failures occur during capstone:
1. **vslam-debugging skill**: Classify failure, apply diagnostic workflow
2. **isaac-ros-performance skill**: If latency degrades, profile GPU

## Acceptance Tests

### Test 1: Initialization Test
**Scenario**: Launch VSLAM in Isaac Sim office environment
**Expected**:
- VSLAM initializes within 5 seconds
- Feature count stable at 450-550
- No error messages in logs

### Test 2: Straight-Line Navigation
**Scenario**: Humanoid walks 10m straight line
**Expected**:
- VSLAM tracks continuously (no "tracking lost" messages)
- Drift &lt;5% (RMS error &lt;0.5m over 10m)
- Latency p99 &lt;33ms

### Test 3: Loop Closure Validation
**Scenario**: Humanoid walks 4m square loop (return to start)
**Expected**:
- Loop closure detects return (red edge in pose graph)
- Final position error &lt;0.2m from start
- No false loop closures (ground truth validation)

### Test 4: Low-Texture Robustness
**Scenario**: Humanoid navigates hallway with white walls
**Expected**:
- Tracking maintains (even if features drop to 150-200)
- No catastrophic failure (tracking doesn't diverge &gt;1m)
- Recovery within 5s if tracking temporarily lost

### Test 5: Real-Time Under Load
**Scenario**: VSLAM runs concurrently with RViz + Isaac Sim rendering
**Expected**:
- Latency remains &lt;33ms p99 (no degradation)
- GPU utilization 60-70% (VSLAM not starved)
- No OOM errors (memory management stable)

## Implementation Workflow

### Step 1: Environment Setup (15 minutes)
- Verify hardware (GPU, drivers, CUDA)
- Verify software (ROS 2, Isaac ROS, Isaac Sim)
- Create workspace directory for capstone artifacts

### Step 2: Parameter Configuration (30 minutes)
- Create optimized `vslam_capstone_config.yaml` based on Lessons 3-5 tuning
- Configure camera calibration (stereo baseline, intrinsics)
- Set real-time performance targets in config

### Step 3: Test Environment Creation (20 minutes)
- Design Isaac Sim scene with diversity:
  - Textured office area (desks, posters, furniture)
  - Low-texture hallway (white walls, minimal features)
  - Loop path (square or figure-8 trajectory)

### Step 4: Baseline Testing (45 minutes)
- Record 3 test bags (textured, low-texture, loop)
- Run VSLAM with default parameters
- Measure baseline metrics (latency, drift, feature count)

### Step 5: Iterative Tuning (60 minutes)
- Apply lessons 3-5 tuning strategies
- Re-measure metrics after each parameter change
- Document what worked vs what didn't

### Step 6: Acceptance Testing (30 minutes)
- Run all 5 acceptance tests
- Record pass/fail for each criterion
- Generate validation report

### Step 7: Integration Preparation (15 minutes)
- Configure VSLAM for Nav2 integration (Chapter 9)
- Document TF frame configuration
- Test localization topic publishing

---

**Total estimated time**: 180 minutes (3 hours)

## Deliverables

1. **Configuration file**: `vslam_capstone_config.yaml` (optimized parameters)
2. **Test bags**: 3 ROS2 bags (textured, low-texture, loop scenarios)
3. **Validation report**: Markdown documenting acceptance test results
4. **RViz configuration**: `vslam_capstone.rviz` for real-time monitoring
5. **Performance benchmarks**: Latency, drift, GPU utilization measurements

---

**This specification defines success.** Implementation follows.
```

## Phase 2: Implement Against Specification

Now that specification is written, implement systematically.

### Configuration File Creation

Create `~/vslam_capstone_config.yaml` incorporating Lessons 3-5 optimizations:

```yaml
/**:
  ros__parameters:
    # === Performance Tuning (Lesson 4) ===
    num_threads: 6              # CPU threads for non-GPU tasks
    enable_gpu_stereo: true     # GPU acceleration for stereo matching
    enable_debug_mode: false    # Disable logging overhead (production mode)

    # === Feature Detection (Lesson 3) ===
    detector_type: "ORB"
    num_features: 500           # Balanced for RTX 4070
    detector_threshold: 12      # Tuned for mixed environments (textured + low-texture)
    adaptive_threshold: true    # Auto-adjust for environment
    threshold_range: [6, 30]    # Min/max bounds for adaptive tuning

    # === Visual Odometry ===
    vo_enable: true
    vo_min_parallax: 0.008      # Keyframe creation threshold (radians)
    vo_ransac_iterations: 100   # Motion estimation robustness

    # === Loop Closure (Lesson 5) ===
    loop_closure_enable: true
    loop_closure_threshold: 0.11           # Tuned for 97% precision, 90% recall
    loop_closure_frequency: 2.0            # Check every 0.5s (was 1.0s)
    geometric_inlier_threshold: 0.78       # Stricter than default 0.7
    loop_closure_min_distance: 3.0         # Minimum distance for valid loop (meters)

    # === Map Management ===
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    max_keyframes: 1000         # Limit memory usage
    keyframe_culling_enable: true  # Remove redundant keyframes

    # === Real-Time Constraints ===
    enable_slam_visualization: true   # RViz markers (diagnostic)
    publish_frequency: 30.0           # Match camera framerate

    # === Robustness ===
    min_tracking_features: 100        # Trigger recovery if below
    relocalization_enable: true       # Attempt recovery on tracking loss
    fallback_to_odometry: false       # No wheel odometry on humanoid (rely on VSLAM only)
```

### Isaac Sim Environment Setup

Create three test environments in Isaac Sim:

**Environment 1: Textured Office**
- Furniture: Desks, chairs, bookshelves
- Walls: Posters, whiteboards with writing
- Floor: Patterned carpet or tiles
- **Purpose**: Ideal VSLAM conditions (baseline test)

**Environment 2: Low-Texture Hallway**
- Walls: Plain white paint
- Floor: Uniform gray
- Minimal furniture (1-2 benches)
- **Purpose**: Stress test feature detection

**Environment 3: Loop Path**
- Figure-8 or square trajectory
- Mix of textured and low-texture regions
- Return to starting position
- **Purpose**: Loop closure validation

### Recording Test Bags

```bash
# Environment 1: Textured Office (60 seconds)
ros2 bag record \
  /humanoid/camera/left/image_raw \
  /humanoid/camera/right/image_raw \
  /humanoid/camera/left/camera_info \
  /humanoid/camera/right/camera_info \
  /tf /tf_static \
  --duration 60 \
  --output ~/capstone_test_textured_60s

# Environment 2: Low-Texture Hallway (30 seconds)
ros2 bag record [...same topics...] \
  --duration 30 \
  --output ~/capstone_test_lowtexture_30s

# Environment 3: Loop Path (45 seconds - complete loop)
ros2 bag record [...same topics...] \
  --duration 45 \
  --output ~/capstone_test_loop_45s
```

**Ensure**:
- Ground truth available (tf from Isaac Sim)
- Camera topics publishing at 30 Hz
- Complete loop in Environment 3 (return to within 0.5m of start)

## Phase 3: Validation Against Specification

Run acceptance tests systematically.

### Test 1: Initialization (Pass/Fail)

```bash
# Launch VSLAM
ros2 launch ~/isaac_vslam_launch.py config_file:=~/vslam_capstone_config.yaml

# Play bag
ros2 bag play ~/capstone_test_textured_60s --start 0 --duration 10

# Observe
# ✓ VSLAM publishes within 5 seconds?
# ✓ Feature count stable (450-550)?
# ✓ No errors in logs?
```

**Result**: [ ] Pass  [ ] Fail — Document if failed: _________________

### Test 2: Straight-Line Accuracy (Quantitative)

```bash
# Terminal 1: Launch VSLAM
ros2 launch ~/isaac_vslam_launch.py config_file:=~/vslam_capstone_config.yaml

# Terminal 2: Measure accuracy
python3 ~/compute_vslam_error.py

# Terminal 3: Play bag (first 30s = straight walk)
ros2 bag play ~/capstone_test_textured_60s --duration 30
```

**Results**:
```
RMS Error: ______ m  (target: &lt;0.5m over 10m)
Drift:     ______%   (target: &lt;5%)
```

**Result**: [ ] Pass  [ ] Fail

### Test 3: Loop Closure (Qualitative + Quantitative)

```bash
# Play loop bag
ros2 bag play ~/capstone_test_loop_45s

# Observe in RViz:
# ✓ Red loop closure edge appears?
# ✓ Trajectory closes (end near start)?

# Measure final position error
[Final VSLAM position] - [Ground truth start] = ______ m
(Target: &lt;0.2m)
```

**Results**:
- Loop detected: [ ] Yes  [ ] No
- Final error: ______ m (target: &lt;0.2m)

**Result**: [ ] Pass  [ ] Fail

### Test 4: Low-Texture Robustness

```bash
# Play low-texture bag
ros2 bag play ~/capstone_test_lowtexture_30s

# Observe:
# Feature count during hallway: ______ (acceptable if &gt;100)
# Tracking lost events: ______ (target: 0)
# Max drift: ______ m (target: &lt;1.0m over 30s)
```

**Result**: [ ] Pass  [ ] Fail

### Test 5: Real-Time Performance

```bash
# Measure latency
python3 ~/measure_vslam_latency.py

# Play bag (concurrent with Isaac Sim + RViz rendering)
ros2 bag play ~/capstone_test_textured_60s
```

**Results**:
```
Latency p50:  ______ ms
Latency p95:  ______ ms
Latency p99:  ______ ms (target: &lt;33ms)
GPU Util:     ______%  (target: &gt;60%)
```

**Result**: [ ] Pass  [ ] Fail

### Validation Report

Create `~/capstone_vslam_validation_report.md`:

```markdown
# VSLAM Capstone Validation Report

**Date**: 2025-12-17
**System**: Isaac ROS Visual SLAM on RTX 4070

## Configuration
[Paste vslam_capstone_config.yaml]

## Acceptance Test Results

| Test | Criterion | Target | Actual | Pass/Fail |
|------|-----------|--------|--------|-----------|
| 1. Initialization | Time to first pose | &lt;5s | ___s | ___ |
| 2. Straight-line | Drift | &lt;5% | ___%  | ___ |
| 2. Straight-line | RMS error | &lt;0.5m | ___m | ___ |
| 3. Loop closure | Detected | Yes | ___ | ___ |
| 3. Loop closure | Final error | &lt;0.2m | ___m | ___ |
| 4. Low-texture | Tracking maintained | Yes | ___ | ___ |
| 4. Low-texture | Max drift | &lt;1.0m | ___m | ___ |
| 5. Real-time | Latency p99 | &lt;33ms | ___ms | ___ |
| 5. Real-time | GPU utilization | &gt;60% | ___% | ___ |

**Overall Pass Rate**: ____ / 8 criteria

## Issues Encountered

[Document any failures and debugging steps taken]

## Parameter Tuning History

[Document which parameters changed and why]

## Lessons Applied

- Lesson 3 (Feature Tracking): [How applied]
- Lesson 4 (Performance): [How applied]
- Lesson 5 (Loop Closure): [How applied]
- Lesson 7 (Debugging Skill): [When invoked]

## Recommendations for Production Deployment

[Based on test results, what would you change for real-world deployment?]
```

## Phase 4: Debugging with Accumulated Skills

If any acceptance test fails, apply debugging skills systematically.

### Failure Scenario Example

**Test 2 fails**: Drift is 8.2% (exceeds 5% target)

**Apply vslam-debugging skill**:

1. **Classify failure mode**: Map drift (trajectory diverges from ground truth)
2. **Check loop closure**: Was bag a loop? Did loop closure trigger? (No = explains drift)
3. **Environment analysis**: Low-texture regions in test bag? (Check RViz feature distribution)
4. **Parameter diagnosis**: Is `loop_closure_threshold` too high? (Check similarity scores in logs)
5. **Fix hierarchy**:
   - Immediate: None (offline test)
   - Short-term: Lower `loop_closure_threshold` from 0.11 to 0.09
   - Long-term: Improve environment texture
6. **Validate fix**: Re-run test, measure drift (should reduce to &lt;5%)

**Document in report**: "Initial drift 8.2% due to missed loop closure (threshold too high). Lowered threshold to 0.09, drift reduced to 3.1%."

## Checkpoint

Before proceeding to Chapter 9, verify capstone completion:

- [ ] Specification written BEFORE implementation (spec-first)
- [ ] Configuration file optimized (incorporating Lessons 3-5)
- [ ] Three test bags recorded (textured, low-texture, loop)
- [ ] All 5 acceptance tests run (documented results)
- [ ] Validation report created (includes pass/fail, tuning history)
- [ ] Debugging skills applied (if failures occurred)
- [ ] VSLAM meets specification criteria (80%+ tests passing)

## Chapter 9 Integration Prep

Your working VSLAM system now provides localization for navigation. Chapter 9 uses these topics:

**VSLAM Outputs (for Nav2)**:
- `/visual_slam/tracking/odometry` → Localization updates
- `/visual_slam/map/points` → Obstacle detection (optional)
- TF: `map → odom → base_link` → Coordinate transforms

**Verify these publish** before Chapter 9:

```bash
ros2 topic list | grep visual_slam
# Expected:
#   /visual_slam/tracking/odometry
#   /visual_slam/tracking/slam_path
#   /visual_slam/map/points
#   ...

ros2 topic hz /visual_slam/tracking/odometry
# Expected: ~30 Hz (matches camera framerate)
```

## Try With AI

**Prompt 1: Specification Critique**
```
Review my capstone VSLAM specification:
[paste your spec.md]

Critique for completeness:
- Are success criteria measurable and falsifiable?
- Are constraints explicit enough?
- What non-goals am I missing?
- What acceptance tests would you add?
```

**Prompt 2: Failure Root Cause Analysis**
```
My capstone failed Test 5 (Real-time Performance):
- Latency p99: 47ms (target was &lt;33ms)
- GPU utilization: 82% (high, but still slow)
- Test environment: Textured office (should be easy)

Using the vslam-debugging skill, what are the three most likely root causes and how would I diagnose each?
```

**Prompt 3: Production Deployment Planning**
```
My capstone VSLAM passes all acceptance tests in Isaac Sim. What additional validation is needed before deploying on a real physical humanoid robot? Consider:
- Simulation-to-reality gaps
- Hardware differences (real cameras vs simulated)
- Environmental variability (real offices vs Isaac Sim)
```

**Reflection**: This capstone integrated 7 lessons of accumulated knowledge through specification-first orchestration. Which aspect was most challenging—writing the spec, tuning parameters, or debugging failures? That challenge reveals where your expertise needs reinforcement.

---

**You've completed Chapter 8.** Your humanoid now has real-time Visual SLAM providing accurate localization. Chapter 9 adds Nav2 path planning to enable autonomous navigation: VSLAM answers "Where am I?" while Nav2 answers "How do I get there?"
