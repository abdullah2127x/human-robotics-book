---
title: Creating Nav2 Humanoid Configuration Skill
chapter: 9
lesson: 7
learning_objectives:
  - Integrate Isaac Visual SLAM with Nav2 for accurate humanoid localization
  - Create nav2-humanoid-config skill encoding bipedal navigation patterns
  - Validate skill reusability on different humanoid platform
  - Apply specification-first approach to skill development
estimated_time: 180 minutes
skills:
  nav2-humanoid-config:
    proficiency_level: B2
  skill-creation:
    proficiency_level: C1
generated_by: content-implementer v1.1.0
source_spec: specs/book/part-3/chapter-09-plan.md
created: 2025-12-17
---

# Creating Nav2 Humanoid Configuration Skill

Through Lessons 2-6, you configured Nav2 for bipedal humanoid navigation: costmaps, planners, controllers, behavior trees, and dynamic obstacles. Each configuration required understanding humanoid-specific constraints—limited lateral movement, balance-dependent velocities, careful recovery behaviors.

This lesson has two goals:
1. **Integrate Isaac Visual SLAM** (Chapter 8) with Nav2 for accurate localization
2. **Create a reusable skill** that encodes humanoid navigation configuration patterns

The skill you create will help you (and others) configure Nav2 for humanoid robots without repeating the manual tuning process. This is Layer 3 of our pedagogical framework—transforming expertise into reusable intelligence.

## Part 1: Isaac Visual SLAM Integration

### Why VSLAM for Nav2 Localization

Nav2 requires two inputs:
1. **Map**: Where are obstacles? (global costmap source)
2. **Localization**: Where is the robot? (pose in map frame)

**Traditional approach**: Use AMCL (particle filter localization) with prebuilt map.

**Our approach**: Use Isaac Visual SLAM (Chapter 8) for simultaneous localization and mapping.

**Advantages of VSLAM for humanoids**:
- No prebuilt map required
- Adapts to environment changes
- GPU-accelerated on NVIDIA hardware
- Provides odometry and map simultaneously

### VSLAM Integration Architecture

```
Isaac Sim (Chapter 7)
    ↓ RGB-D Camera
Isaac Visual SLAM (Chapter 8)
    ↓ /tf (map → odom → base_link)
    ↓ /map (OccupancyGrid)
Nav2 (Chapter 9)
    ↓ Uses VSLAM pose for localization
    ↓ Uses VSLAM map for global costmap
Humanoid Controller
```

### Configuring Nav2 to Use VSLAM

#### Step 1: TF Tree Setup

VSLAM publishes transforms:
- `map` → `odom` (drift correction)
- `odom` → `base_link` (visual odometry)

**Verify TF tree**:

```bash
# Check TF is publishing
ros2 run tf2_ros tf2_echo map base_link

# Expected output: Transform from map to base_link
# Translation: [x, y, z]
# Rotation: [qx, qy, qz, qw]
```

**If transform missing**: VSLAM not running or not publishing TF. Start Isaac Visual SLAM (Chapter 8, Lesson 3).

#### Step 2: Map Topic Configuration

VSLAM publishes occupancy grid on `/map`:

```bash
# Verify map is publishing
ros2 topic echo /map --once

# Expected: nav_msgs/OccupancyGrid with reasonable dimensions
```

**Configure global costmap to use VSLAM map**:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map                    # VSLAM provides map frame
      robot_base_frame: base_link

      plugins: ["static_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
        map_topic: /map                    # VSLAM map topic
        subscribe_to_updates: true         # Update when map changes
```

#### Step 3: Disable AMCL

If you previously used AMCL for localization, disable it:

```xml
<!-- In launch file: comment out or remove AMCL node -->
<!--
<node pkg="nav2_amcl" exec="amcl" name="amcl" output="screen">
  ...
</node>
-->
```

Nav2 will use TF directly from VSLAM instead.

#### Step 4: Verify Integration

```bash
# Terminal 1: Launch Isaac Sim with humanoid
# (Chapter 7 launch procedure)

# Terminal 2: Launch Isaac Visual SLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Terminal 3: Launch Nav2 (without AMCL)
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=~/humanoid_ws/src/humanoid_nav2_config/config/nav2_params.yaml \
    use_lifecycle_manager:=true \
    autostart:=true

# Terminal 4: Verify in RViz
ros2 run rviz2 rviz2
```

**In RViz, verify**:
- Robot model appears in correct location
- Global costmap shows VSLAM-generated map
- Local costmap updates with sensor data
- TF tree shows: map → odom → base_link

### Troubleshooting VSLAM Integration

#### Issue 1: Robot Position Incorrect in RViz

**Symptom**: Robot model at origin, not at actual position.

**Diagnosis**: TF `map → base_link` not being published.

**Fix**:
- Verify VSLAM is running: `ros2 node list | grep visual_slam`
- Check TF: `ros2 run tf2_ros tf2_monitor map base_link`
- Restart VSLAM if transform stopped

#### Issue 2: Global Costmap Empty

**Symptom**: RViz shows empty global costmap (no obstacles).

**Diagnosis**: VSLAM map not reaching costmap.

**Fix**:
- Verify `/map` topic: `ros2 topic hz /map`
- Check `map_topic` in global costmap config
- Ensure `subscribe_to_updates: true`

#### Issue 3: Localization Drift

**Symptom**: Robot position slowly drifts from actual position.

**Diagnosis**: VSLAM loop closure not working, accumulated odometry error.

**Fix**:
- Apply Chapter 8 VSLAM debugging skill
- Improve lighting conditions
- Add more visual features to environment

## Part 2: Creating Nav2 Humanoid Configuration Skill

### Why Create a Skill?

You've learned to configure Nav2 for humanoids through experimentation:
- Footprint calculation from stance dimensions
- Velocity limits from gait stability
- TEB planner for smooth paths
- Conservative recovery behaviors

This knowledge is valuable but scattered across configuration files. A **skill** codifies this knowledge so:
1. You can apply it quickly to new humanoid robots
2. Others can benefit from your expertise
3. AI assistants can use it consistently

### Skill Structure Review

From Chapter 5, skills follow the Persona + Questions + Principles template:

```markdown
# [Skill Name]

## Persona
Who is the expert applying this skill?

## Questions
What information do we need before starting?

## Principles
What rules/patterns guide the decisions?
```

### Extracting Patterns from Lessons 2-6

Before writing the skill, identify patterns from our Nav2 configuration:

#### Pattern 1: Footprint from Stance Dimensions

**Input**: Stance width, stance depth
**Formula**:
- Footprint width = stance_width × 1.15 (15% safety margin)
- Footprint depth = stance_depth × 1.15
- Vertices: rectangular polygon centered on base_link

#### Pattern 2: Velocity Limits from Gait

**Input**: Step length, step frequency
**Formula**:
- max_vel_x = step_length × step_frequency
- max_vel_y = 0.0 (bipedal constraint)
- max_vel_theta = 0.5 rad/s (typical humanoid turning)

#### Pattern 3: Acceleration for Balance

**Heuristic**:
- acc_lim_x = 0.3-0.5 m/s² (prevents center-of-mass shift)
- decel_lim_x = 1.2 × acc_lim_x (faster stopping for safety)
- acc_lim_theta = 0.4-0.5 rad/s²

#### Pattern 4: Planner Selection

**Rule**: Use TEB planner for humanoids (trajectory optimization, smooth paths)
- Never Navfn (sharp corners)
- Smac acceptable but TEB preferred

#### Pattern 5: Recovery Behavior Speeds

**Rules**:
- Spin: max 0.3 rad/s, small angles (45-90°)
- Backup: max 0.05 m/s, short distances (0.1-0.2m)
- Wait before movement (standing is safe)

#### Pattern 6: Inflation Radius

**Formula**:
- inflation_radius = (max_vel_x × reaction_time) + safety_buffer
- Typical: 1.5-2.5m for humanoids at 0.4 m/s

### Writing the Nav2 Humanoid Configuration Skill

Create skill file:

```bash
mkdir -p ~/humanoid_ws/skills/nav2-humanoid-config
```

Create `SKILL.md`:

```markdown
# Nav2 Humanoid Configuration Skill

## Persona

You are a robotics engineer specialized in configuring Nav2 navigation stack for bipedal humanoid robots. You understand that humanoids have unique kinematic constraints: they cannot move laterally, have limited turning rates due to stepping gait, and must maintain balance during all movements. Your configurations prioritize stability over speed, smooth trajectories over shortest paths, and conservative recovery behaviors that won't cause falls.

## Questions

Before configuring Nav2 for a humanoid robot, gather this information:

### Physical Dimensions
1. **Stance width**: Distance between feet during normal stance (meters)
2. **Stance depth**: Front-to-back dimension of foot placement (meters)
3. **Robot height**: For obstacle detection height filtering (meters)

### Gait Parameters
4. **Step length**: Distance covered per step during walking (meters)
5. **Step frequency**: Steps per second during comfortable walking (Hz)
6. **Maximum stable walking speed**: If known from gait analysis (m/s)

### Sensor Configuration
7. **Primary depth sensor**: Type (RGB-D camera, LiDAR) and topic name
8. **Sensor frame**: TF frame name for depth sensor
9. **Sensor range**: Maximum reliable range (meters)

### Environment
10. **Localization source**: VSLAM, AMCL, or other?
11. **Map topic**: If using pre-built map or VSLAM map
12. **Expected dynamic obstacles**: People, doors, furniture?

## Principles

Apply these rules when configuring Nav2 for humanoid robots:

### Footprint Configuration

```
PRINCIPLE: Footprint = physical_dimensions × safety_margin

- Calculate footprint width: stance_width × 1.15
- Calculate footprint depth: stance_depth × 1.15
- Define as rectangular polygon in base_link frame
- NEVER use circular footprint for humanoids (doesn't represent stance geometry)
```

### Velocity Limits

```
PRINCIPLE: Velocity must not exceed gait stability limits

- max_vel_x = min(step_length × step_frequency, known_stable_speed)
- max_vel_y = 0.0 (MANDATORY for bipedal robots)
- max_vel_theta = 0.5 rad/s (typical, reduce if robot has stability issues)
- NEVER allow lateral velocity (humanoids cannot strafe)
```

### Acceleration Limits

```
PRINCIPLE: Smooth acceleration prevents balance disruption

- acc_lim_x = 0.3 to 0.5 m/s² (start conservative)
- decel_lim_x = acc_lim_x × 1.2 to 1.5 (faster stopping is safer)
- acc_lim_theta = 0.4 rad/s²
- If robot stumbles during acceleration, REDUCE acc_lim_x
```

### Path Planner

```
PRINCIPLE: Use trajectory-optimization planners, not grid-based

- ALWAYS use TEB planner for humanoids
- Set weight_shortest_path = 0.0 (smoothness > distance)
- Set weight_kinematics_nh >= 1000 (enforce non-holonomic)
- Set min_turning_radius based on humanoid turning capability (0.5-1.0m)
- NEVER use Navfn alone (produces sharp corners)
```

### Controller

```
PRINCIPLE: DWB with humanoid-safe parameters

- Configure DWB controller
- vy_samples = 1 (only sample zero lateral velocity)
- Oscillation critic enabled (prevents wobbling)
- PathAlign scale high (32+) for accurate tracking
- Goal tolerance: 0.25m position, 0.25 rad orientation
```

### Recovery Behaviors

```
PRINCIPLE: Safest behaviors first, slowest speeds always

Recovery order (safest to riskiest):
1. Wait (5s) - standing is stable
2. Clear costmap - no movement
3. Wait (10s) - more time for dynamic obstacles
4. Slow spin - 0.3 rad/s max, 45-90° max
5. Slow backup - 0.05 m/s max, 0.15m max
6. Abort

NEVER use default Nav2 recovery speeds (too fast for humanoids)
```

### Costmap Configuration

```
PRINCIPLE: Appropriate update rates and inflation for bipedal response time

Global costmap:
- update_frequency: 1-2 Hz (map changes slowly)
- inflation_radius: 2.0m (accounts for humanoid stopping distance)

Local costmap:
- update_frequency: 10 Hz (dynamic obstacles)
- rolling_window: true
- size: 4m × 4m (sufficient for reactive control)
- inflation_radius: 1.5m
```

### VSLAM Integration

```
PRINCIPLE: Use VSLAM transforms directly, disable AMCL

- Configure global_frame: map
- Use /map topic from VSLAM for static_layer
- Verify TF: map → odom → base_link published by VSLAM
- Disable AMCL node (redundant with VSLAM localization)
```

## Configuration Template

When applying this skill, generate configuration files following this structure:

### nav2_params.yaml (master config)

```yaml
# Humanoid Nav2 Configuration
# Generated using nav2-humanoid-config skill
# Robot: [ROBOT_NAME]
# Date: [DATE]

# ===== CALCULATED FROM QUESTIONS =====
# Footprint: [FOOTPRINT_VERTICES]
# Max velocity: [MAX_VEL] m/s
# Inflation radius: [INFLATION] m

# Include costmap, planner, controller, BT configs...
```

### Validation Checklist

After configuration, verify:
- [ ] max_vel_y = 0.0 in all configs
- [ ] TEB planner selected (not Navfn)
- [ ] Recovery spin_speed ≤ 0.3 rad/s
- [ ] Recovery backup_speed ≤ 0.05 m/s
- [ ] Inflation radius ≥ 1.5m
- [ ] Local costmap update_frequency ≥ 10 Hz
```

### Saving the Skill

Save to project skills directory:

```bash
# Create skill directory
mkdir -p .claude/skills/nav2-humanoid-config

# Save skill file
cp ~/humanoid_ws/skills/nav2-humanoid-config/SKILL.md \
   .claude/skills/nav2-humanoid-config/SKILL.md
```

## Exercise 1: Extract Configuration Patterns

**Objective**: Document patterns from your Lessons 2-6 configurations.

Review your configuration files and document:

| Parameter | Your Value | Pattern/Formula |
|-----------|------------|-----------------|
| Footprint width | | stance × 1.15 |
| max_vel_x | | step_length × freq |
| max_vel_y | | 0.0 (mandatory) |
| acc_lim_x | | |
| TEB min_turning_radius | | |
| Spin speed | | ≤ 0.3 rad/s |
| Backup speed | | ≤ 0.05 m/s |
| Local costmap update | | ≥ 10 Hz |

**Success criteria**:
- All critical parameters documented
- Patterns match humanoid constraints

## Exercise 2: Write Skill Using Template

**Objective**: Create complete skill file following Persona + Questions + Principles format.

**Steps**:

1. Copy skill template from above
2. Customize based on your experience (Lessons 2-6)
3. Add any additional patterns you discovered
4. Save to `.claude/skills/nav2-humanoid-config/SKILL.md`

**Success criteria**:
- [ ] Persona clearly defined
- [ ] All critical questions included
- [ ] Principles match humanoid constraints
- [ ] Configuration template provided
- [ ] Validation checklist complete

## Exercise 3: Test Skill on Different Humanoid

**Objective**: Validate skill reusability by applying to different robot.

**Scenario**: Configure Nav2 for a different humanoid with these specs:
- Stance width: 0.40m (wider than original)
- Stance depth: 0.20m (shorter than original)
- Step length: 0.25m
- Step frequency: 2.0 Hz
- Sensor: Intel RealSense D435

**Steps**:

1. Apply skill questions to gather info
2. Apply skill principles to calculate parameters
3. Generate configuration files
4. Compare to your original configuration

**Expected results**:

| Parameter | Original Robot | New Robot | Skill-Calculated |
|-----------|----------------|-----------|------------------|
| Footprint width | 0.40m | | 0.40 × 1.15 = 0.46m |
| Footprint depth | 0.29m | | 0.20 × 1.15 = 0.23m |
| max_vel_x | 0.4 m/s | | 0.25 × 2.0 = 0.5 m/s |
| max_vel_y | 0.0 | | 0.0 (mandatory) |

**Success criteria**:
- [ ] Skill produces valid configuration for new robot
- [ ] Parameters differ appropriately based on robot specs
- [ ] Core principles (no lateral velocity, TEB planner) maintained

## Using the Skill with AI

Your skill is now ready for AI-assisted configuration:

**Example prompt**:

```
"Using the nav2-humanoid-config skill, configure Nav2 for my new humanoid robot:
- Stance width: 0.35m
- Stance depth: 0.28m
- Step length: 0.30m
- Step frequency: 1.5 Hz
- Depth camera: /camera/depth/points
- Using Isaac Visual SLAM for localization

Generate complete configuration files."
```

**AI will**:
1. Follow skill persona (humanoid robotics expert)
2. Answer questions from provided specs
3. Apply principles to calculate parameters
4. Generate configuration template
5. Provide validation checklist

## Summary: Skill Creation for Humanoid Navigation

**What you accomplished**:

1. **Integrated VSLAM with Nav2**: Accurate localization from Chapter 8 now drives navigation
2. **Created reusable skill**: Encoded humanoid configuration patterns
3. **Validated skill portability**: Tested on different humanoid specs

**Skill value**:

| Without Skill | With Skill |
|---------------|------------|
| Hours of manual tuning | Minutes with guided questions |
| Trial-and-error configuration | Principle-based calculation |
| Knowledge in your head | Documented, shareable patterns |
| One robot at a time | Any humanoid robot |

**Next lesson**: You'll create a second skill—behavior-tree-design—encoding the recovery escalation patterns from Lesson 5.

## Checkpoint: Skill Validation

Before proceeding to Lesson 8 (BT design skill), validate:

- [ ] **VSLAM integrated**: Nav2 uses VSLAM localization and map
- [ ] **Skill file created**: `.claude/skills/nav2-humanoid-config/SKILL.md` exists
- [ ] **Persona defined**: Clear expert identity
- [ ] **Questions complete**: All critical parameters covered
- [ ] **Principles match constraints**: Humanoid-specific rules encoded
- [ ] **Skill tested on new robot**: Different specs produce valid config

**If all checks pass**, your nav2-humanoid-config skill is ready for reuse.
