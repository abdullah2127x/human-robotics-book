---
title: Nav2 Architecture and Component Overview
chapter: 9
lesson: 1
learning_objectives:
  - Understand Nav2 navigation stack architecture and component relationships
  - Identify roles of planner plugins, controller plugins, and behavior tree executor
  - Examine Nav2 configuration files manually to build mental model
  - Install Nav2 packages and launch sample navigation successfully
estimated_time: 90 minutes
skills:
  nav2-architecture-understanding:
    proficiency_level: B2
generated_by: content-implementer v1.1.0
source_spec: specs/book/part-3/chapter-09-plan.md
created: 2025-12-17
---

# Nav2 Architecture and Component Overview

Before configuring Nav2 for your bipedal humanoid, you need to understand how its components fit together. Nav2 is not a monolithic system—it's a modular plugin architecture where planners, controllers, costmaps, and behavior trees coordinate through well-defined interfaces.

Think of Nav2 like an orchestral performance: each plugin is an instrument (planner, controller, recovery behavior), the behavior tree is the conductor (orchestrating when each plays), and the navigation server is the concert hall (providing the infrastructure). Understanding this architecture prevents "random tuning syndrome"—blindly tweaking parameters without understanding system interactions.

This lesson builds the mental model you need before any configuration work. No AI assistance yet—you'll manually explore Nav2's structure to develop intuition about how autonomous navigation works.

## Why Autonomous Navigation Needs a Stack

Imagine trying to navigate your humanoid robot through a building manually:

1. **Localization**: Where am I? (VSLAM from Chapter 8 provides this)
2. **Global Planning**: What's the high-level path from here to goal? (avoid walls, furniture)
3. **Local Planning**: Given sensor data RIGHT NOW, how do I adjust my path? (dynamic obstacles)
4. **Control**: What velocity commands achieve desired trajectory? (execute plan)
5. **Recovery**: I'm stuck—what do I do? (rotate, backup, replan)
6. **Orchestration**: When do I plan vs execute vs recover? (state machine logic)

Implementing all six functions from scratch would take months. Nav2 provides these as reusable, well-tested components you can configure for your robot.

## Nav2 Component Architecture

Nav2 organizes navigation into these functional components:

### 1. Navigation Server (nav2_bt_navigator)

The navigation server is the entry point for navigation requests. When you send a "navigate to goal" action:

1. Navigation server receives goal pose (x, y, θ in map frame)
2. Delegates to behavior tree executor for orchestration
3. Monitors progress, handles cancellation requests
4. Returns success/failure result when navigation completes

**Interface**: ROS 2 action server (`NavigateToPose`, `NavigateThroughPoses`)

**Why it matters**: This is how external systems (like voice command handlers in Part 4) request navigation. Understanding the action interface is critical for integration.

### 2. Planner Plugins (Global Path Planning)

Planner plugins compute high-level paths from start to goal using the global costmap (typically static map + inflation).

**Common planners:**
- **Navfn**: Dijkstra-based, fast, grid-based (good for simple environments)
- **Smac**: State-lattice planner with kinematic constraints (smoother paths than Navfn)
- **TEB**: Timed Elastic Band, trajectory optimization (best for bipedal, considers dynamics)

**When it runs**: At navigation start, and when replanning is triggered (e.g., path blocked)

**Output**: Sequence of poses (x, y, θ) forming path from start to goal

**Why it matters**: Planner selection affects path smoothness and computational cost. Bipedal robots benefit from trajectory-optimizing planners (TEB, Smac) that avoid sharp turns.

### 3. Controller Plugins (Local Trajectory Tracking)

Controller plugins track the global path by computing velocity commands based on local costmap (sensor data RIGHT NOW).

**Common controllers:**
- **DWB (Dynamic Window Approach)**: Samples velocity space, scores trajectories, good for dynamic environments
- **Regulated Pure Pursuit**: Follows path by steering toward lookahead point, simpler than DWB
- **TEB Local Planner**: Optimizes local trajectory in real-time (can replace both planner and controller)

**When it runs**: Every control loop iteration (typically 10-20 Hz)

**Output**: Velocity commands (linear and angular velocities) sent to robot

**Why it matters**: Controller determines trajectory tracking quality and responsiveness to obstacles. For humanoids, velocity limits must respect gait stability.

### 4. Costmap Layers (Obstacle Representation)

Costmaps represent environment as 2D grids where each cell has cost (0 = free space, 255 = obstacle):

**Layer types:**
- **Static Layer**: Pre-built map from SLAM (walls, furniture)
- **Obstacle Layer**: Real-time sensor data (LiDAR, depth camera)
- **Inflation Layer**: Expands obstacles with safety margin (robot shouldn't hug walls)
- **Voxel Layer**: 3D obstacle tracking (for tall obstacles, overhangs)

**Global vs Local Costmap:**
- **Global**: Large area, lower update frequency (5 Hz), uses static map + inflation
- **Local**: Small area around robot, high update frequency (10 Hz), uses sensors + inflation

**Why it matters**: Costmap configuration determines obstacle safety margins and computational cost. Humanoid footprints need accurate representation for collision-free navigation.

### 5. Behavior Tree Executor (Navigation Logic)

Behavior trees orchestrate navigation: when to plan, when to follow path, when to recover from failures. Think of behavior trees as hierarchical state machines:

```
Navigate to Goal
├─ Compute Path (planner)
├─ Follow Path (controller)
│   └─ If stuck → Try Recovery
│       ├─ Rotate Recovery (spin in place)
│       ├─ Backup Recovery (reverse)
│       └─ Clear Costmap (reset obstacle layer)
└─ Success (reached goal) or Abort (unrecoverable failure)
```

**Why it matters**: Behavior tree design determines how your robot handles failures. For bipedal robots, recovery behaviors must prevent tipping over (aggressive backup might destabilize humanoid).

### 6. Recovery Behaviors

Recovery plugins handle stuck situations:

- **Spin Recovery**: Rotate 360° to clear costmap and find new path
- **Backup Recovery**: Move backward to escape local minima
- **Wait Recovery**: Pause for dynamic obstacles to clear
- **Clear Costmap Recovery**: Reset obstacle layer (sensor glitch recovery)

**Why it matters**: Bipedal robots need gentler recoveries than wheeled robots (aggressive maneuvers risk tipping).

## Component Interaction Flowchart

```
User/Application
    ↓ (sends NavigateToPose action)
Navigation Server (nav2_bt_navigator)
    ↓ (loads behavior tree)
Behavior Tree Executor
    ↓
    ├→ Planner Plugin (compute global path using global costmap)
    ↓
    ├→ Controller Plugin (track path, compute velocities using local costmap)
    ↓       ↑
    ↓       └──── Costmap Layers (static + obstacles + inflation)
    ↓
    └→ Recovery Behaviors (if stuck: spin, backup, clear costmap, replan)
    ↓
Success or Failure Result
```

**Key insight**: Costmaps feed both planner and controller. VSLAM provides localization (pose estimate) that all components use to transform sensor data into map frame.

## Nav2 Lifecycle Management

Nav2 uses ROS 2 lifecycle nodes, which have explicit state transitions:

1. **Unconfigured** → Configure parameters
2. **Inactive** → Activate (start processing)
3. **Active** → Deactivate (pause processing)
4. **Finalized** → Shutdown

**Why this matters**: You can't send navigation goals until Nav2 nodes are in "Active" state. Launch files handle lifecycle transitions automatically, but understanding the states helps debug launch failures.

## Installing Nav2

Let's install Nav2 packages for ROS 2 Humble:

```bash
# Install Nav2 packages
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install TurtleBot3 packages (for testing Nav2 architecture)
sudo apt install ros-humble-turtlebot3*

# Source ROS 2 workspace
source /opt/ros/humble/setup.bash
```

**Verification:**
```bash
# Check Nav2 packages installed
ros2 pkg list | grep nav2_

# Expected output (partial):
# nav2_behavior_tree
# nav2_bt_navigator
# nav2_controller
# nav2_costmap_2d
# nav2_lifecycle_manager
# nav2_planner
# nav2_util
```

## Examining Nav2 Directory Structure

Navigate to Nav2 configuration examples to see how parameters are organized:

```bash
# Navigate to Nav2 parameter examples
cd /opt/ros/humble/share/nav2_bringup/params/

# List parameter files
ls -lh

# You'll see:
# nav2_params.yaml  ← Default Nav2 configuration
```

**Examine the configuration file:**

```bash
# Open default Nav2 params (read-only, don't edit yet)
cat nav2_params.yaml | less
```

**What you should notice** (without modifying anything):

1. **Node organization**: Parameters grouped by node (`controller_server`, `planner_server`, `bt_navigator`)
2. **Plugin specification**: Each functional component (planner, controller) specifies which plugin to use
3. **Footprint definition**: Robot shape defined as polygon vertices or radius
4. **Costmap configuration**: Separate configs for global and local costmaps
5. **Behavior tree XML path**: Points to XML file defining navigation logic

**Key parameters to locate** (just observe, don't change):
- `robot_base_frame`: Which TF frame represents robot base (usually `base_link`)
- `robot_radius` or `footprint`: Robot geometry for collision checking
- `expected_planner_frequency`: How often to replan (e.g., 1 Hz)
- `expected_controller_frequency`: How often to compute velocities (e.g., 20 Hz)

## Launching Sample Nav2 Navigation

To solidify your architecture understanding, launch Nav2 with TurtleBot3 (wheeled robot, simpler than humanoid):

### Step 1: Set TurtleBot3 Model

```bash
# Set TurtleBot3 model (burger is simplest)
export TURTLEBOT3_MODEL=burger
```

### Step 2: Launch Gazebo Simulation

```bash
# Launch TurtleBot3 in Gazebo world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**What you should see**: Gazebo opens with TurtleBot3 in an obstacle-filled environment.

### Step 3: Launch Nav2 Stack

**In a new terminal:**

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

# Launch Nav2
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True
```

**What's happening:**
- Nav2 nodes start (planner, controller, costmap, behavior tree)
- Lifecycle manager transitions nodes to Active state
- Navigation server waits for goals

### Step 4: Launch RViz Visualization

**In a new terminal:**

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

# Launch RViz with Nav2 configuration
ros2 launch nav2_bringup rviz_launch.py
```

**What you should see in RViz:**
- Robot model (TurtleBot3)
- Global costmap (static map + inflation layer)
- Local costmap (sensor data + inflation layer)
- Global plan (path from planner)
- Local plan (controller trajectory)

### Step 5: Send Navigation Goal

**In RViz:**
1. Click "2D Goal Pose" button (top toolbar)
2. Click on map to set goal position
3. Drag to set goal orientation (arrow direction)

**What you should observe:**
- Planner computes path (green line on global costmap)
- Controller follows path (cyan line on local costmap)
- TurtleBot3 navigates to goal autonomously
- If obstacles detected, local costmap updates and controller adjusts

**Checkpoint**: Can you see the architectural components working?
- Planner ran once (global path computed)
- Controller runs continuously (velocity commands)
- Costmaps update (local costmap changes as robot moves)
- Behavior tree orchestrates (you see planner → controller → success sequence)

## Understanding Behavior Tree Execution

To see behavior tree logic in action, intentionally create a "stuck" situation:

1. Send navigation goal across the environment
2. While TurtleBot3 is moving, manually place obstacle in its path (Gazebo: Insert → Box model, drag to robot's path)
3. Observe behavior tree recovery:
   - Controller tries to reach path (fails, obstacle blocking)
   - Behavior tree triggers recovery (robot spins or backs up)
   - Planner recomputes path (new route around obstacle)
   - Controller follows new path

**Key insight**: You didn't program this recovery logic—Nav2's behavior tree handled it automatically. Your job (in Lesson 5) is to customize recovery strategies for bipedal constraints.

## Configuration File Walkthrough

Let's examine key sections of `nav2_params.yaml` conceptually (don't edit yet):

### Planner Server Configuration

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0  # Plan every 1 second
    planner_plugins: ["GridBased"]   # List of planner plugins

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"  # Which planner plugin
      tolerance: 0.5  # How close to goal counts as success
      use_astar: true # A* vs Dijkstra (A* is faster)
```

**What you're seeing**: Planner server can have multiple planner plugins (e.g., "GridBased", "TEB", "Smac"). Each plugin has its own configuration section.

### Controller Server Configuration

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Compute velocities 20 times per second
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"  # DWB controller
      max_vel_x: 0.26  # Maximum linear velocity (m/s)
      max_vel_theta: 1.0  # Maximum angular velocity (rad/s)
      # ... many more parameters ...
```

**What you're seeing**: Controller runs at higher frequency than planner (20 Hz vs 1 Hz) because it needs to react to obstacles quickly. Velocity limits are critical for bipedal adaptation.

### Costmap Configuration (Global)

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0  # Update every 1 second (static map changes slowly)
      robot_radius: 0.22  # Robot size for collision checking (meters)

      plugins: ["static_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: "/map"  # SLAM-generated map

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.55  # Expand obstacles by this distance
```

**What you're seeing**: Global costmap uses static map (from SLAM) and inflates obstacles. No sensor layers (global plan doesn't need real-time obstacle detection).

### Costmap Configuration (Local)

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0  # Update 5 times per second (sensor data)
      width: 3  # 3 meters wide
      height: 3  # 3 meters tall
      robot_radius: 0.22

      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan  # LiDAR data
        scan:
          topic: /scan
          clearing: True  # Remove obstacles when not detected
          marking: True   # Add obstacles when detected
```

**What you're seeing**: Local costmap uses sensor data (LiDAR `/scan` topic) and updates more frequently than global costmap. This enables dynamic obstacle avoidance.

## Nav2 Actions and Services

Nav2 exposes these ROS 2 interfaces:

**Actions** (long-running tasks with feedback):
- `/navigate_to_pose` (NavigateToPose): Navigate to single goal
- `/navigate_through_poses` (NavigateThroughPoses): Navigate through waypoints
- `/follow_path` (FollowPath): Execute precomputed path

**Services** (synchronous requests):
- `/reinitialize_global_localization`: Reset localization (if lost)
- `/clear_global_costmap`: Reset global costmap
- `/clear_local_costmap`: Reset local costmap

**Topics** (publish for debugging):
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands to robot
- `/plan` (nav_msgs/Path): Current global plan
- `/local_plan` (nav_msgs/Path): Current local plan

**Why this matters**: In Lesson 9 capstone, you'll programmatically send goals using action clients. Understanding these interfaces is prerequisite for integration.

## Checkpoint: Architecture Understanding

Before moving to configuration (Lesson 2), verify your mental model:

**Question 1**: What's the difference between planner and controller?
<details>
<summary>Answer</summary>
Planner computes high-level path from start to goal (runs infrequently, ~1 Hz). Controller tracks that path by computing velocity commands (runs frequently, ~20 Hz, reacts to dynamic obstacles).
</details>

**Question 2**: Why do we need both global and local costmaps?
<details>
<summary>Answer</summary>
Global costmap covers large area for long-term planning (uses static map, updates slowly). Local costmap covers small area around robot for reactive control (uses sensors, updates quickly for dynamic obstacles).
</details>

**Question 3**: What triggers recovery behaviors?
<details>
<summary>Answer</summary>
Behavior tree monitors navigation progress. If no progress toward goal for timeout period (e.g., 15 seconds), behavior tree transitions to recovery behaviors (spin, backup, clear costmap, replan).
</details>

**Question 4**: How does Nav2 receive localization data?
<details>
<summary>Answer</summary>
Nav2 listens to `/tf` topic for transforms between `map` and `base_link` frames. VSLAM (from Chapter 8) publishes these transforms, providing Nav2 with real-time pose estimates.
</details>

**If you can answer these confidently**, you've built the mental model needed for configuration work. If not, re-read component sections and re-run TurtleBot3 demo observing specific components.

## Summary: What You've Learned

You manually explored Nav2's architecture:

- **Component roles**: Planner (global path), controller (local trajectory), costmap (obstacles), behavior tree (orchestration)
- **Configuration structure**: Parameters organized by node, plugins specified per function
- **Lifecycle management**: ROS 2 lifecycle states (Unconfigured → Inactive → Active)
- **Interface understanding**: Actions for navigation, services for maintenance, topics for commands
- **Hands-on observation**: TurtleBot3 demo showed all components working together

**No AI collaboration yet**—you built foundational understanding manually. This mental model is critical for Lessons 2-6 where you'll configure Nav2 for bipedal humanoid constraints.

**Next lesson**: You'll configure costmaps for your humanoid's footprint, defining collision geometry and inflation parameters appropriate for bipedal robots.

## Try With AI

Now that you understand Nav2's architecture, explore conceptual questions to deepen your intuition:

**Understanding Component Interactions**

Ask your AI assistant:
```
"I'm learning Nav2 for bipedal humanoid navigation. Explain the interaction between global planner, local controller, and costmaps. Why does the controller use local costmap instead of global costmap?"
```

**Exploring Bipedal Adaptations**

Ask your AI assistant:
```
"What Nav2 components need modification when adapting from wheeled robots to bipedal humanoids? Consider footprint geometry, velocity limits, and recovery behaviors."
```

**Behavior Tree Design**

Ask your AI assistant:
```
"Describe a Nav2 behavior tree for humanoid navigation. How should recovery escalation work differently for bipedal robots compared to wheeled robots?"
```

**Self-Reflection**

After discussing with AI:
- Can you now explain Nav2 architecture to someone unfamiliar with ROS 2?
- Do you understand why planner runs at 1 Hz while controller runs at 20 Hz?
- Can you identify which parameters will need tuning for humanoid constraints?

These conceptual explorations prepare you for hands-on configuration work in the next lessons.
