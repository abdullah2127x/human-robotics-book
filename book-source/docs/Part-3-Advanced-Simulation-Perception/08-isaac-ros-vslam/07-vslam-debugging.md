---
title: Creating a VSLAM Debugging Skill
chapter: 8
lesson: 7
proficiency_level: C1
learning_objectives:
  - Extract reusable debugging patterns from Lessons 3-6
  - Design a VSLAM debugging skill using Persona + Questions + Principles
  - Encode failure mode classification and diagnostic workflows
  - Validate skill effectiveness on novel VSLAM failures
estimated_time: 90 minutes
skills:
  vslam-debugging:
    proficiency: C1
generated_by: content-implementer v1.1.0
source_spec: specs/book/part-3/chapter-08-plan.md
created: 2025-12-17
---

# Creating a VSLAM Debugging Skill

You've debugged VSLAM failures across four lessons—feature tracking loss (Lesson 3), loop closure drift (Lesson 5), and visualization diagnostics (Lesson 6). These aren't isolated incidents; they're **recurring patterns** in any Visual SLAM deployment.

Rather than debugging from scratch every time, encode these patterns into a reusable skill. This is Layer 3: Intelligence Design—transforming tacit debugging knowledge into explicit, reusable guidance.

## What Makes Debugging Knowledge Reusable?

### The Debugging Pattern Recognition Challenge

Traditional debugging documentation:
```
"If VSLAM fails, check logs for errors."
```

**Problem**: Too vague. Which logs? What errors? What do I do after finding errors?

**Skill-based debugging**:
```
Persona: Think like a SLAM engineer diagnosing production failures

Question 1: What failure mode am I observing?
- Feature tracking loss (features detected but not matched)
- Map drift (trajectory diverges from ground truth)
- Loop closure failure (valid loops missed or false positives)
- Performance degradation (latency exceeds real-time threshold)

Question 2: What sensor data reveals the issue?
- RViz image: Feature distribution (sparse = low-texture problem)
- RViz path: Trajectory divergence pattern (linear = bias, sudden = loss)
- Logs: Feature count over time (stable = good, dropping = tracking loss imminent)

Principle: Classify failure mode first, then apply mode-specific diagnostic workflow.
```

**Result**: Systematic debugging process, not trial-and-error.

## Extracting Patterns from Lessons 3-6

### Pattern 1: Feature Tracking Loss

**When**: Feature count drops below threshold, tracking "lost" messages

**Diagnostic workflow** (from Lesson 3):
1. Check RViz image: Are features detected but not matched? (motion blur)
2. Check RViz image: Are features too sparse? (&lt;100 features = low-texture)
3. Check environment: Lighting changes? Fast motion? Featureless surfaces?
4. Check parameters: `detector_threshold` too high? (increase sensitivity)

**Solution hierarchy**:
- Immediate: Reduce robot speed (buy time for parameter tuning)
- Short-term: Tune `detector_threshold` (lower for more features)
- Long-term: Add texture to environment OR switch sensor modality (RGB-D camera)

### Pattern 2: Map Drift Without Loop Closure

**When**: Trajectory diverges from ground truth, no loop closure events

**Diagnostic workflow** (from Lesson 5):
1. Check RViz pose graph: Are there red edges? (no = loop closure not detecting)
2. Check logs: Loop closure candidates? (similarity scores just below threshold?)
3. Check environment: Did lighting/viewpoint change between visits?
4. Check parameters: `loop_closure_threshold` too high? (lower for better recall)

**Solution hierarchy**:
- Immediate: Manually trigger relocalization (if SLAM supports it)
- Short-term: Tune `loop_closure_threshold` (balance precision vs recall)
- Long-term: Improve place recognition (add distinctive landmarks to environment)

### Pattern 3: False Loop Closure (Map Distortion)

**When**: Map suddenly distorts, pose graph has contradictory constraints

**Diagnostic workflow** (from Lesson 5):
1. Check RViz pose graph: Recent red edge added?
2. Check ground truth: Distance between matched frames (should be &lt;0.5m, is it &gt;2m?)
3. Check parameters: `loop_closure_threshold` too low? (false positives)
4. Check environment: Repetitive structures? (identical hallways = perceptual aliasing)

**Solution hierarchy**:
- Immediate: Reject last loop closure (if manual control available)
- Short-term: Increase `loop_closure_threshold` OR `geometric_inlier_threshold`
- Long-term: Improve geometric verification (stricter RANSAC inlier ratio)

### Pattern 4: Real-Time Performance Degradation

**When**: Latency exceeds 33ms, frame drops occur

**Diagnostic workflow** (from Lesson 4):
1. Check GPU utilization: Is GPU actually being used? (`nvidia-smi` during run)
2. Profile with nsys: Which kernel dominates time? (feature detection? matching?)
3. Check parameters: Too many features? (`num_features` excessive for GPU?)
4. Check concurrent processes: Other GPU workloads? (rendering, video encoding)

**Solution hierarchy**:
- Immediate: Kill concurrent GPU processes (dedicated GPU to VSLAM)
- Short-term: Reduce `num_features` (fewer features = faster processing)
- Long-term: Upgrade GPU (more CUDA cores) OR optimize GXF graph (requires code changes)

## Designing the VSLAM Debugging Skill

Create `.claude/skills/vslam-debugging/SKILL.md`:

```markdown
---
name: vslam-debugging
version: 1.0.0
description: Systematic debugging for Visual SLAM failures in production robotics
created: 2025-12-17
applies_to:
  - Isaac ROS Visual SLAM
  - ORB-SLAM3
  - RTAB-Map
  - Any feature-based Visual SLAM system
---

# VSLAM Debugging Skill

## Persona

Think like a SLAM engineer debugging production robot failures in the field. Your goal is to diagnose Visual SLAM failures systematically—classifying the failure mode, gathering diagnostic evidence from multiple sources (logs, visualization, ground truth), and applying the correct fix hierarchy (immediate workarounds → parameter tuning → long-term architectural changes).

You are NOT guessing randomly. You follow a diagnostic decision tree, eliminating possibilities until root cause is identified.

## Analysis Questions

### Question 1: Failure Mode Classification

**What category of failure am I observing?**

- **Feature tracking loss**: Robot moving but SLAM position frozen OR sudden position jumps
- **Map drift**: SLAM trajectory diverges from ground truth over time (linear or rotational)
- **Loop closure failure**: Robot returned to known location but SLAM didn't correct drift
- **False loop closure**: Map suddenly distorts, impossible geometry appears
- **Performance degradation**: Processing latency exceeds real-time threshold (&gt;33ms for 30 Hz)
- **Initialization failure**: SLAM never initializes (no map created at start)

**How to determine**:
- Check RViz path: Is trajectory tracking ground truth? (drift = diverging paths)
- Check logs: "Tracking lost" messages? (feature loss)
- Check logs: "Loop closure detected" vs ground truth (did robot actually return?)
- Check latency: Mean &gt;33ms? (performance issue)

**Classify FIRST before proceeding.** Different failure modes require different diagnostics.

### Question 2: Sensor Data Analysis

**What does the raw data reveal about the failure?**

- **RViz image with features**: Feature distribution (sparse? clustered? absent?)
- **Feature count over time**: Stable (400-600) or dropping (300 → 150 → 50)?
- **Trajectory visualization**: Divergence pattern (linear bias? sudden jump? gradual drift?)
- **Point cloud map**: Structured (recognizable walls/furniture) or noisy (random scatter)?
- **Pose graph**: Loop closure edges present (red lines)? Where do they connect?
- **Ground truth comparison**: How large is error? Where did drift start?

**Key principle**: Look at VISUALIZATION and GROUND TRUTH, not just logs. Logs tell you THAT failure occurred; visualization tells you WHY.

### Question 3: Environmental Factors

**Is the environment causing this failure?**

- **Texture**: White walls? Glass? Mirrors? (low-texture → feature loss)
- **Lighting**: Changed between visits? (loop closure miss)
- **Motion**: Fast rotation? (motion blur → matching failure)
- **Repetition**: Identical hallways? (perceptual aliasing → false loops)
- **Dynamics**: Moving objects? (violates static world assumption)

**Check**: Record similar bag in different environment. Does failure persist? (parameter issue) or disappear? (environmental issue)

### Question 4: Parameter Validation

**Are current parameters appropriate for this environment?**

- **Detector sensitivity**: `detector_threshold` too high for low-texture? (default 20)
- **Feature count**: `num_features` too low for complex environment? (default 500)
- **Loop closure threshold**: `loop_closure_threshold` too high (misses loops) or too low (false positives)? (default 0.1)
- **Geometric verification**: `geometric_inlier_threshold` strict enough? (default 0.7)

**Validation**: Compare parameters to known-good baseline. Tune ONE parameter at a time (scientific method).

### Question 5: Hardware and Performance

**Is hardware the bottleneck?**

- **GPU utilization**: `nvidia-smi` shows &gt;60% utilization during VSLAM? (yes = using GPU)
- **CUDA execution**: `nsys profile` shows Isaac CUDA kernels? (yes = GPU accelerated)
- **CPU load**: Other processes competing for GPU? (`htop` + `nvidia-smi` combined view)
- **Memory**: VRAM usage &lt;80% of total? (&gt;80% = potential OOM crashes)

**If GPU utilization &lt;20%**: CUDA not executing, check installation (`nvcc --version`, driver version).

## Diagnostic Principles

### Principle 1: Classify Before Diagnosing

**Never start with parameter tuning.** Always classify failure mode first. Feature tracking loss requires different fixes than loop closure failures.

**Decision tree approach**:
```
IF feature_count_drops_suddenly:
    → Diagnose feature tracking loss (texture? motion blur?)
ELSE IF trajectory_diverges_gradually:
    → Diagnose drift (odometry bias? loop closure disabled?)
ELSE IF map_suddenly_distorts:
    → Diagnose false loop closure (threshold too low? repetitive environment?)
ELSE IF latency_exceeds_threshold:
    → Diagnose performance (GPU not used? too many features?)
```

### Principle 2: Sensor Data First, Parameters Second

**Check RViz visualization and ground truth BEFORE changing parameters.**

Many failures are environmental (low texture, lighting changes) and cannot be solved by parameter tuning. Blindly adjusting `detector_threshold` when the real issue is a white wall wastes time.

**Workflow**:
1. Observe failure in RViz (feature distribution, trajectory)
2. Compare to ground truth (quantify error magnitude)
3. Identify environmental factors (texture, lighting, motion)
4. THEN tune parameters (with specific target: "detect 200 more features")

### Principle 3: Immediate → Short-term → Long-term Solutions

**Failure fixes have hierarchy based on implementation time:**

**Immediate** (0-5 minutes):
- Reduce robot speed (buys time for diagnosis)
- Kill competing GPU processes (dedicate resources to VSLAM)
- Switch to known-good configuration (previously saved parameters)

**Short-term** (10-60 minutes):
- Tune parameters (`detector_threshold`, `loop_closure_threshold`, `num_features`)
- Adjust sensor settings (exposure, framerate)
- Enable/disable features (loop closure, IMU integration)

**Long-term** (hours to days):
- Modify environment (add texture, improve lighting)
- Upgrade hardware (faster GPU, better camera)
- Switch algorithm (RGB-D SLAM, LiDAR SLAM)
- Code modifications (custom GXF graphs, optimized kernels)

**Apply appropriate tier**: If robot is deployed in production and failing NOW, implement immediate fix first (even if suboptimal), then iterate to long-term solution.

### Principle 4: One Parameter at a Time (Scientific Method)

**Never tune multiple parameters simultaneously.** You won't know which change caused improvement (or regression).

**Experimental approach**:
1. Record baseline bag (known environment, quantify failure)
2. Change ONE parameter (e.g., `detector_threshold: 20 → 15`)
3. Replay bag, measure metrics (feature count, drift, latency)
4. If improved: Keep change. If worse: Revert.
5. Repeat for next parameter.

**Document each experiment**: Parameter → Result → Keep/Revert

### Principle 5: Validate with Ground Truth

**Subjective assessment ("looks better") is insufficient.** Compare to ground truth quantitatively.

**Metrics to track**:
- **RMS error**: Position error averaged over trajectory
- **Drift percentage**: Error / distance_traveled × 100%
- **Feature count**: Mean features per frame
- **Loop closure precision**: Correct loops / total loops detected
- **Latency**: p99 latency (99th percentile)

**Before/after comparison**:
```
Baseline:    Drift 3.2%, Feature count 427, Latency 28ms
After tuning: Drift 1.8%, Feature count 512, Latency 31ms
→ Improvement: Drift reduced, features increased, latency acceptable
```

## Usage Workflow

### Step 1: Reproduce Failure

Record ROS2 bag with failure occurring. Ensure bag contains:
- Stereo camera images
- Camera_info (calibration)
- TF (ground truth from simulation)
- Duration covers failure event (not just aftermath)

### Step 2: Classify Failure Mode

Play bag in RViz, observe:
- Feature visualization (distribution, tracking)
- Trajectory (drift pattern)
- Logs (error messages)
- Ground truth comparison

Assign failure to ONE category: Feature loss, drift, false loop, performance, or initialization.

### Step 3: Apply Mode-Specific Diagnostic

**If Feature Tracking Loss**:
- Check feature count trend (dropping?)
- Check RViz image (sparse features?)
- Measure texture (image gradient magnitude)
- Tune `detector_threshold` (lower = more sensitive)

**If Map Drift**:
- Check loop closure events (missing?)
- Check similarity scores (just below threshold?)
- Tune `loop_closure_threshold` (lower = more loops detected)

**If False Loop Closure**:
- Check matched frame distance (ground truth)
- Check pose graph (contradictory edges?)
- Tune `geometric_inlier_threshold` (higher = stricter verification)

**If Performance Issue**:
- Check GPU utilization (`nvidia-smi`)
- Profile with nsys (kernel bottleneck?)
- Reduce `num_features` (lower computational cost)

### Step 4: Implement Fix Hierarchy

**Immediate**: What can I do in 5 minutes to restore function?
**Short-term**: What parameter tuning fixes root cause?
**Long-term**: What architectural changes prevent recurrence?

### Step 5: Validate Fix

Replay same failure bag. Measure metrics:
- Did feature count increase? (feature loss fix)
- Did drift reduce? (loop closure fix)
- Did false loops disappear? (threshold fix)
- Did latency decrease? (performance fix)

**If metrics improved**: Fix validated. Document parameter changes.
**If metrics unchanged or worse**: Fix incorrect. Revert, try alternative.

## Common Pitfalls

### Pitfall 1: Tuning Without Measuring

**Don't**: Change `detector_threshold` from 20 to 15 "because it seems low"
**Do**: Measure feature count (baseline: 437), change to 15, measure again (new: 512), validate improvement

### Pitfall 2: Ignoring Environmental Factors

**Don't**: Spend hours tuning parameters when real issue is white wall (no amount of tuning will create features that don't exist)
**Do**: Identify environmental constraint first (low texture), then choose appropriate solution (add texture OR switch sensor)

### Pitfall 3: Multiple Parameter Changes

**Don't**: Change `detector_threshold` AND `num_features` AND `loop_closure_threshold` simultaneously
**Do**: Change detector_threshold only, measure, then iterate to next parameter

### Pitfall 4: Insufficient Test Data

**Don't**: Test on single 10-second bag and declare success
**Do**: Test on diverse conditions (textured, low-texture, loops, fast motion) to validate robustness

## Validation

This skill is effective when you can:
- Classify VSLAM failures within 2 minutes of observation
- Identify root cause (parameter vs environment vs hardware) systematically
- Select appropriate fix tier (immediate/short/long) based on deployment context
- Tune parameters with measurable improvement (not subjective "better")
- Document debugging process for reproducibility

## Related Skills

- **isaac-ros-performance-skill** (Lesson 7, complementary): Performance profiling
- **sensor-processing-skill** (Part 2 Ch6): Camera calibration and data validation

---

**Usage**: Invoke this skill when VSLAM behaves unexpectedly. Follow diagnostic workflow, don't skip classification step.
```

## Validating the Skill on Novel Failure

To prove the skill works, apply it to a VSLAM failure you haven't debugged yet.

### Novel Scenario: VSLAM Works Indoors, Fails Near Windows

**Observation**: Robot navigates successfully in interior rooms but loses tracking when approaching windows (outdoor views).

**Applying vslam-debugging skill**:

**Step 1: Classify Failure Mode**
- RViz visualization: Features sparse near windows
- Logs: "Tracking lost" messages when facing window
- Classification: **Feature tracking loss**

**Step 2: Sensor Data Analysis (Question 2)**
- RViz image: Bright overexposure in window region (sky blown out)
- Feature count: Drops from 500 → 120 when window in view
- Environmental factor: Bright outdoor light overwhelming indoor exposure settings

**Step 3: Environmental Factors (Question 3)**
- Lighting: High dynamic range (dark interior + bright exterior)
- Camera settings: Fixed exposure (cannot adapt to HDR)
- Motion: Normal (not motion blur)

**Step 4: Diagnostic Principle 2 Application**
- This is ENVIRONMENTAL (HDR lighting), not parameter issue
- Tuning `detector_threshold` won't create features in overexposed regions

**Step 5: Fix Hierarchy (Principle 3)**
- **Immediate**: Avoid windows (path planning constraint)
- **Short-term**: Enable HDR camera mode (if hardware supports)
- **Long-term**: Add automatic exposure control (sensor upgrade)

**Result**: Skill correctly diagnosed environmental issue (HDR) and avoided wasting time on parameter tuning.

## Checkpoint

Verify you've created reusable debugging intelligence:

- [ ] Skill encoded with Persona + Questions + Principles structure
- [ ] Diagnostic decision tree documented (classify → analyze → fix)
- [ ] Fix hierarchy defined (immediate/short-term/long-term)
- [ ] Skill validated on novel failure (not seen in Lessons 3-6)
- [ ] Skill documented in `.claude/skills/vslam-debugging/`

## Try With AI

**Prompt 1: Skill Application Practice**
```
I'm using the vslam-debugging skill to diagnose this failure:
- Robot navigating office, VSLAM initializes successfully
- After 45 seconds, tracking suddenly freezes (position not updating)
- RViz shows 487 features detected in last frame before freeze
- Logs show "Motion estimation failed: insufficient parallax"

Walk me through the skill's diagnostic workflow for this scenario. Which failure mode is this? What does "insufficient parallax" indicate?
```

**Prompt 2: Skill Extension**
```
The vslam-debugging skill covers feature-based Visual SLAM. How would I extend it to include:
- RGB-D SLAM (using depth cameras instead of stereo)
- LiDAR SLAM (using laser scanners)
- Visual-Inertial Odometry (fusing camera + IMU)

What principles transfer directly? What new diagnostic questions are needed?
```

**Prompt 3: Skill Composition**
```
I have two skills:
- vslam-debugging (this lesson)
- isaac-ros-performance (Lesson 4)

Design a workflow that composes both skills to diagnose: "VSLAM tracking quality degrades under computational load (when Nav2 path planner runs concurrently)."
```

**Reflection**: Encoding debugging patterns as a skill transforms expertise from implicit (in your head after Lessons 3-6) to explicit (documented and reusable). What pattern surprised you most when writing it down formally?

---

Final lesson: Capstone project—implement real-time VSLAM on humanoid using specification-first approach, composing all accumulated knowledge and skills.
