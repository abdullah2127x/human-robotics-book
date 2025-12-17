---
title: Costmap Configuration for Humanoid Footprint
chapter: 9
lesson: 2
learning_objectives:
  - Configure global and local costmaps with humanoid-specific footprint geometry
  - Calculate inflation radius based on sensor range and reaction time
  - Integrate Isaac Sim depth camera sensor data into obstacle layer
  - Visualize costmaps in RViz to validate configuration correctness
estimated_time: 120 minutes
skills:
  costmap-configuration:
    proficiency_level: B2
generated_by: content-implementer v1.1.0
source_spec: specs/book/part-3/chapter-09-plan.md
created: 2025-12-17
---

# Costmap Configuration for Humanoid Footprint

Costmaps are Nav2's representation of the environment—2D grids where each cell has a cost indicating obstacle presence. For autonomous navigation, your humanoid needs accurate costmaps that respect its footprint geometry and provide sufficient safety margins around obstacles.

This lesson focuses on **specification-first configuration**: you'll define footprint safety requirements BEFORE editing YAML configs. Unlike wheeled robots with circular footprints, bipedal humanoids have rectangular stances that change during walking. Your costmap configuration must account for this complexity.

You'll work manually in this lesson (no AI yet), building intuition about how costmap layers compose and how inflation parameters affect safety vs mobility tradeoffs.

## Why Costmaps Matter for Bipedal Robots

Consider these failure modes if costmaps are misconfigured:

**Footprint too small**: Robot's actual footprint collides with obstacles even though planner thought path was clear (false clearance).

**Footprint too large**: Robot refuses paths through doorways it could actually fit through (false collision).

**Inflation too aggressive**: Inflated obstacles block valid paths, robot declares environment unnavigable (over-conservative).

**Inflation too timid**: Robot plans paths too close to obstacles, doesn't account for localization error or dynamic movement (unsafe).

**Sensor integration broken**: Local costmap doesn't update with sensor data, robot collides with dynamic obstacles not in static map (perception failure).

Proper costmap configuration balances safety (adequate margins) with mobility (not overly conservative). For bipedal robots, this balance is critical because humanoids have limited maneuverability compared to wheeled robots.

## Costmap Layer Architecture

Nav2 costmaps use layered composition—multiple layers contribute costs to the final costmap:

### Layer 1: Static Layer

**Purpose**: Represents pre-built map from SLAM (walls, furniture, permanent obstacles)

**Update frequency**: Low (1 Hz), map changes rarely

**Data source**: `/map` topic (nav_msgs/OccupancyGrid from SLAM)

**Use case**: Global costmap (long-term planning)

**Why it matters**: Provides base layer for global path planning. Humanoids need accurate static maps for long-distance navigation.

### Layer 2: Obstacle Layer

**Purpose**: Represents dynamic obstacles detected by sensors (people, moved furniture, doors)

**Update frequency**: High (5-10 Hz), sensor data updates constantly

**Data source**: Sensor topics (LiDAR `/scan`, depth camera `/camera/depth/points`)

**Use case**: Local costmap (reactive control)

**Behavior**:
- **Marking**: When sensor detects obstacle, mark corresponding costmap cells
- **Clearing**: When sensor sees free space, clear previously marked cells
- **Raycasting**: Trace rays from sensor through costmap to determine free space

**Why it matters**: Enables dynamic obstacle avoidance. Humanoids navigate human-populated environments where obstacles move unpredictably.

### Layer 3: Inflation Layer

**Purpose**: Expands obstacles with safety margin (robot shouldn't hug walls)

**Algorithm**: Inflates occupied cells outward by inflation radius, with costs decaying toward edges

**Cost function**:
- Cell at obstacle: Cost = 254 (lethal obstacle)
- Cells within inflation radius: Cost decays exponentially (254 → 0)
- Cells beyond inflation radius: Cost = 0 (free space)

**Why it matters**: Accounts for localization uncertainty, robot dynamics, and safety margins. Humanoids need conservative inflation due to tipping risk if they collide.

### Layer 4: Voxel Layer (Optional)

**Purpose**: 3D obstacle representation (detects overhangs, low obstacles)

**Use case**: Environments with 3D structure (tables humanoid can walk under, low-hanging objects)

**Why it matters**: 2D obstacle layer might miss low obstacles (ankle-height) or overhangs (head-height). For humanoid navigation, 3D awareness prevents collisions.

## Global vs Local Costmap Configuration

Nav2 uses two costmaps with different characteristics:

| Characteristic | Global Costmap | Local Costmap |
|----------------|----------------|---------------|
| **Coverage** | Large area (entire known map) | Small area (3-5m around robot) |
| **Update Frequency** | Low (1-5 Hz) | High (10-20 Hz) |
| **Layers** | Static + Inflation | Obstacles + Inflation |
| **Purpose** | Long-term path planning | Reactive obstacle avoidance |
| **Resolution** | Coarser (0.05m) | Finer (0.025m) |

**Why separate costmaps?**
- Global planning doesn't need real-time sensor data (uses static map)
- Local control needs high-frequency updates (reacts to dynamic obstacles)
- Computational efficiency: global costmap is large but updates slowly; local costmap is small but updates quickly

## Defining Humanoid Footprint: Specification First

Before writing configuration files, specify footprint requirements:

### Footprint Requirements Specification

**Intent**: Define collision geometry representing humanoid's physical footprint during navigation.

**Constraints**:
1. **Footprint must encompass stance width and depth** during normal walking
2. **Safety margin**: Add 10-15% to measured dimensions (account for foot swing, arm movement)
3. **Simplification**: Use rectangular approximation (exact foot shape too complex for costmap)
4. **Coordinate frame**: Footprint defined in `base_link` frame (robot center)

**Measurement procedure**:
1. Measure humanoid stance width: Distance between feet during normal stance
2. Measure stance depth: Front-to-back foot dimension
3. Add safety margin: Multiply by 1.1-1.15
4. Define rectangle corners: Vertices in base_link frame

**Example calculation** (generic humanoid):

Measured dimensions:
- Stance width: 0.35m (left foot center to right foot center)
- Stance depth: 0.25m (heel to toe)

With 15% safety margin:
- Footprint width: 0.35 × 1.15 = 0.40m
- Footprint depth: 0.25 × 1.15 = 0.29m

Rectangle vertices (base_link at center):
```
Front-left:  [+0.145,  +0.20]  # (+depth/2, +width/2)
Front-right: [+0.145,  -0.20]  # (+depth/2, -width/2)
Back-right:  [-0.145, -0.20]  # (-depth/2, -width/2)
Back-left:   [-0.145, +0.20]  # (-depth/2, +width/2)
```

**Success criteria**:
- [ ] Footprint encompasses all parts that might collide during walking
- [ ] Safety margin accounts for foot swing and localization error
- [ ] Vertices are in correct coordinate frame (base_link)
- [ ] Costmap planner can check collisions efficiently (simple polygon)

### Inflation Radius Calculation

**Intent**: Calculate inflation radius providing adequate safety margin around obstacles.

**Formula**:
```
Inflation radius = Sensor range × Velocity × Reaction time + Safety buffer
```

**For humanoid walking at 0.4 m/s with depth camera (range 10m), reaction time 0.5s**:
```
Inflation = 10m × 0.4 m/s × 0.5s + 0.3m
          = 2.0m + 0.3m
          = 2.3m (round to 2.5m for safety)
```

**However**, this formula is for wheeled robots. Humanoids have additional constraints:

**Humanoid-specific adjustments**:
1. **Lower velocity** (0.3-0.5 m/s vs 1-2 m/s for wheeled): Reduces required inflation
2. **Limited maneuverability** (can't strafe sideways): Needs more lateral clearance
3. **Tipping risk** (collision might cause fall): Needs conservative margins

**Adjusted calculation for humanoid**:
```
Base inflation: Sensor range × Velocity × Reaction time
              = 10m × 0.4 m/s × 0.5s = 2.0m

Humanoid adjustment: Increase for limited maneuverability
                   = 2.0m × 1.25 = 2.5m

Safety buffer: Additional margin for localization error
             = 2.5m + 0.5m = 3.0m (round to 2.5-3.0m)
```

**Practical value**: Start with 2.0m inflation radius, tune upward if robot gets too close to obstacles.

**Success criteria**:
- [ ] Inflation accounts for sensor range and robot velocity
- [ ] Humanoid maneuverability limitations considered
- [ ] Safety buffer prevents collisions even with localization error

## Creating Humanoid Costmap Configuration

Let's create Nav2 costmap configuration for your humanoid robot.

### Directory Setup

```bash
# Create Nav2 config directory for your humanoid
mkdir -p ~/humanoid_ws/src/humanoid_nav2_config/config

# Navigate to config directory
cd ~/humanoid_ws/src/humanoid_nav2_config/config
```

### Global Costmap Configuration

Create `global_costmap.yaml`:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      # Frame definitions
      global_frame: map         # VSLAM provides map frame
      robot_base_frame: base_link

      # Update rates (global updates slowly)
      update_frequency: 1.0     # 1 Hz update rate
      publish_frequency: 1.0    # Publish for visualization

      # Costmap size (covers entire known map)
      rolling_window: false     # Static map, not rolling
      width: 50                 # 50 meters wide
      height: 50                # 50 meters tall
      resolution: 0.05          # 5cm grid resolution

      # Footprint definition (humanoid rectangular stance)
      footprint: "[[0.145, 0.20], [0.145, -0.20], [-0.145, -0.20], [-0.145, 0.20]]"

      # Layers (static map + inflation for global planning)
      plugins: ["static_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
        map_topic: /map           # VSLAM map from Chapter 8

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0  # Cost decay rate (higher = steeper)
        inflation_radius: 2.0     # Inflate obstacles by 2.0m (humanoid-appropriate)
        inflate_unknown: false    # Don't inflate unexplored areas
        inflate_around_unknown: true  # Inflate near unknown areas
```

**Key configuration decisions**:

1. **Footprint vertices**: Match your humanoid's measured dimensions (adjust values)
2. **Inflation radius**: 2.0m provides safety margin for 0.4 m/s walking speed
3. **Resolution**: 0.05m (5cm) balances accuracy vs computational cost
4. **Static layer only**: Global costmap doesn't need sensor data (uses SLAM map)

### Local Costmap Configuration

Create `local_costmap.yaml`:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # Frame definitions
      global_frame: odom        # Local costmap in odom frame (faster)
      robot_base_frame: base_link

      # Update rates (local updates quickly for dynamic obstacles)
      update_frequency: 5.0     # 5 Hz update rate
      publish_frequency: 2.0    # Publish for visualization

      # Costmap size (small area around robot)
      rolling_window: true      # Follows robot as it moves
      width: 4                  # 4 meters wide (2m each side)
      height: 4                 # 4 meters tall (2m front/back)
      resolution: 0.025         # 2.5cm resolution (finer than global)

      # Footprint (same as global)
      footprint: "[[0.145, 0.20], [0.145, -0.20], [-0.145, -0.20], [-0.145, 0.20]]"

      # Layers (obstacles from sensors + inflation)
      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: depth_camera  # Isaac Sim depth camera

        depth_camera:
          topic: /camera/depth/points      # Point cloud from Isaac Sim
          sensor_frame: camera_link        # Depth camera frame
          data_type: "PointCloud2"         # Point cloud message type
          marking: true                    # Mark obstacles
          clearing: true                   # Clear free space
          obstacle_max_range: 5.0          # Ignore points beyond 5m
          obstacle_min_range: 0.2          # Ignore points closer than 0.2m
          raytrace_max_range: 6.0          # Clear to 6m
          raytrace_min_range: 0.0          # Raytrace from 0m
          max_obstacle_height: 2.0         # Ignore obstacles above 2m (ceiling)
          min_obstacle_height: 0.1         # Ignore obstacles below 0.1m (floor noise)

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0  # Steeper decay for local (more aggressive)
        inflation_radius: 1.5     # Smaller inflation for local (reactive)
        inflate_unknown: false
        inflate_around_unknown: false  # Don't inflate in local (sensor clears)
```

**Key configuration decisions**:

1. **Rolling window**: Local costmap follows robot (centered on robot continuously)
2. **Smaller size**: 4m × 4m is sufficient for reactive control (computational efficiency)
3. **Obstacle layer**: Integrates depth camera from Isaac Sim (adjust topic name to match your setup)
4. **Smaller inflation**: 1.5m for local (controller needs tighter paths, planner already applied 2.0m global inflation)
5. **Obstacle height filtering**: Ignores floor (< 0.1m) and ceiling (> 2.0m)

### Combining Configurations

Create master `nav2_costmap_params.yaml` that includes both:

```yaml
# Master costmap configuration for humanoid navigation

/**:
  ros__parameters:
    use_sim_time: true  # Using Isaac Sim time

# Include global costmap config
<< : *global_costmap

# Include local costmap config
<< : *local_costmap
```

**Alternative** (if YAML anchors don't work):

Create separate files and reference them in launch file (Lesson 3).

## Integrating Isaac Sim Depth Camera

Your Isaac Sim humanoid (from Chapter 7) should publish depth camera data. Verify topic:

```bash
# Source ROS 2 workspace
source ~/humanoid_ws/install/setup.bash

# List topics (look for depth camera)
ros2 topic list | grep -i depth

# Expected output:
# /camera/depth/image_raw
# /camera/depth/points  ← Point cloud for obstacle layer
```

**If depth topic missing**, verify Isaac Sim ROS 2 bridge configuration (Chapter 7, Lesson 7).

**Test point cloud visualization**:

```bash
# Visualize depth point cloud in RViz
ros2 run rviz2 rviz2

# In RViz:
# 1. Add → PointCloud2
# 2. Topic: /camera/depth/points
# 3. Fixed Frame: camera_link
```

**You should see**: 3D point cloud representing depth camera view (obstacles, floor, walls).

## Visualizing Costmaps in RViz

Launch Isaac Sim with your humanoid, then start Nav2 costmap nodes to visualize configuration:

### Step 1: Launch Isaac Sim (Humanoid Scene)

```bash
# Launch Isaac Sim (from Chapter 7)
# Follow Chapter 7 launch procedure to start humanoid in indoor environment
```

### Step 2: Launch Costmap Nodes

Create launch file `costmap_visualization.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to costmap config files
    config_dir = os.path.join(
        get_package_share_directory('humanoid_nav2_config'),
        'config'
    )

    global_costmap_config = os.path.join(config_dir, 'global_costmap.yaml')
    local_costmap_config = os.path.join(config_dir, 'local_costmap.yaml')

    return LaunchDescription([
        # Declare use_sim_time argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time from Isaac Sim'
        ),

        # Global costmap node
        Node(
            package='nav2_costmap_2d',
            executable='costmap_2d_node',
            name='global_costmap',
            output='screen',
            parameters=[
                global_costmap_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('/map', '/map'),  # VSLAM map from Chapter 8
            ]
        ),

        # Local costmap node
        Node(
            package='nav2_costmap_2d',
            executable='costmap_2d_node',
            name='local_costmap',
            output='screen',
            parameters=[
                local_costmap_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('/camera/depth/points', '/camera/depth/points'),
            ]
        ),
    ])
```

**Launch costmaps**:

```bash
# Build workspace
cd ~/humanoid_ws
colcon build --packages-select humanoid_nav2_config

# Source workspace
source install/setup.bash

# Launch costmap visualization
ros2 launch humanoid_nav2_config costmap_visualization.launch.py
```

### Step 3: Visualize in RViz

**Launch RViz**:

```bash
ros2 run rviz2 rviz2
```

**Configure RViz**:

1. **Fixed Frame**: Set to `map` (or `odom` for local costmap)

2. **Add Global Costmap**:
   - Add → Map
   - Topic: `/global_costmap/costmap`
   - Color Scheme: costmap

3. **Add Local Costmap**:
   - Add → Map
   - Topic: `/local_costmap/costmap`
   - Color Scheme: costmap

4. **Add Robot Footprint**:
   - Add → Polygon
   - Topic: `/global_costmap/footprint` (or `/local_costmap/footprint`)

5. **Add Robot Model**:
   - Add → RobotModel
   - Description Topic: `/robot_description`

**What you should see**:

- **Global costmap**: Entire known environment with inflated obstacles (2.0m inflation)
- **Local costmap**: Small area around robot updating with depth camera data
- **Footprint polygon**: Rectangular humanoid footprint (vertices you configured)
- **Robot model**: Humanoid URDF visualized at current pose

**Validation checks**:

✅ **Footprint covers robot**: Polygon encompasses humanoid's stance width and depth
✅ **Inflation visible**: Obstacles have gradient cost zones around them (not binary occupied/free)
✅ **Local costmap updates**: When robot moves or obstacles move, local costmap changes
✅ **Obstacles detected**: Depth camera data marks obstacles in local costmap (place object in front of robot, observe)

## Troubleshooting Common Costmap Issues

### Issue 1: Footprint Too Small (Collision Despite "Clear" Path)

**Symptom**: Planner computes path, but robot collides with obstacles during execution.

**Diagnosis**: Visualize footprint polygon in RViz—does it encompass entire robot?

**Fix**: Increase footprint dimensions, add more safety margin (10% → 15% or 20%).

### Issue 2: Footprint Too Large (Overly Conservative Navigation)

**Symptom**: Robot refuses to navigate through doorways or narrow passages it should fit through.

**Diagnosis**: Measure actual doorway width, compare to footprint width.

**Fix**: Reduce footprint dimensions to measured size + 10% (not 20%).

### Issue 3: Inflation Too Aggressive (Environment "Unnavigable")

**Symptom**: Costmap shows nearly entire environment as occupied (red), planner fails to find any path.

**Diagnosis**: Check inflation radius—if it's too large (> 3m for humanoid), obstacles fill entire space.

**Fix**: Reduce inflation radius (try 1.5m - 2.0m for humanoids at 0.4 m/s).

### Issue 4: Local Costmap Not Updating (Static, Ignores Obstacles)

**Symptom**: Place obstacle in front of robot (in Isaac Sim), local costmap doesn't show it.

**Diagnosis**:
1. Check depth camera topic publishing: `ros2 topic hz /camera/depth/points` (should be > 10 Hz)
2. Check obstacle layer enabled: `enabled: true` in config
3. Check sensor frame transform: `ros2 run tf2_ros tf2_echo base_link camera_link` (should print transform)

**Fix**:
- If topic not publishing: Fix Isaac Sim ROS 2 bridge (Chapter 7)
- If obstacle layer disabled: Set `enabled: true` in `local_costmap.yaml`
- If transform missing: Add camera link to URDF (Chapter 7), publish `base_link → camera_link` transform

### Issue 5: Obstacles Appear in Wrong Location (TF Frame Issues)

**Symptom**: Obstacles marked in costmap, but not where depth camera actually sees them (offset or rotated).

**Diagnosis**: Transform between `base_link` and `camera_link` incorrect.

**Fix**:
1. Verify camera link in URDF: Check joint origin (xyz, rpy)
2. Publish correct transform: `ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll base_link camera_link`
3. Validate: `ros2 run tf2_tools view_frames.py` (generates PDF showing all transforms)

## Checkpoint: Costmap Validation

Before proceeding to Lesson 3 (path planners), validate your costmap configuration:

**Validation Checklist**:

- [ ] **Footprint defined**: Vertices match humanoid dimensions + safety margin
- [ ] **Global costmap visible**: RViz shows global costmap covering known environment
- [ ] **Local costmap visible**: RViz shows local costmap updating with sensor data
- [ ] **Inflation working**: Obstacles have gradient cost zones (not binary)
- [ ] **Depth camera integrated**: Local costmap marks obstacles detected by depth camera
- [ ] **Footprint visualization**: Polygon overlaid on costmap matches robot size
- [ ] **Dynamic updates**: Local costmap changes when robot moves or obstacles move

**If all checks pass**, your costmaps are correctly configured for humanoid navigation.

**If checks fail**, revisit configuration sections and apply troubleshooting procedures.

## Summary: What You've Accomplished

You configured Nav2 costmaps for bipedal humanoid navigation:

1. **Specification-first approach**: Defined footprint requirements before editing configs
2. **Footprint calculation**: Measured humanoid dimensions, added safety margin, defined polygon vertices
3. **Inflation tuning**: Calculated inflation radius accounting for sensor range, velocity, and humanoid maneuverability
4. **Global costmap**: Configured for long-term planning (static map + inflation)
5. **Local costmap**: Configured for reactive control (depth camera obstacles + inflation)
6. **Isaac Sim integration**: Integrated depth camera point cloud into obstacle layer
7. **RViz visualization**: Validated costmap behavior observing real-time updates

**No AI collaboration yet**—you manually configured costmaps to build intuition about layer composition and parameter effects.

**Next lesson**: You'll collaborate with AI to select and tune path planners for bipedal gait constraints, comparing Navfn (fast, jagged) vs TEB (smooth, trajectory-optimized).

## Try With AI

Now that you've configured costmaps manually, explore inflation tuning with AI collaboration:

**Calculating Optimal Inflation Radius**

Ask your AI assistant:
```
"I configured Nav2 costmap for humanoid walking at 0.4 m/s with depth camera (10m range). My inflation radius is 2.0m. Given humanoid's limited maneuverability and tipping risk if collision occurs, should I increase or decrease inflation? What's the tradeoff?"
```

**Troubleshooting Local Costmap Updates**

If your local costmap isn't updating with depth camera data, ask AI:
```
"My Nav2 local costmap obstacle layer isn't marking obstacles detected by Isaac Sim depth camera. Topic /camera/depth/points is publishing at 30 Hz. What are common causes of obstacle layer not integrating sensor data?"
```

**Footprint Geometry Optimization**

Ask AI:
```
"My humanoid has measured stance width 0.35m and depth 0.25m. I added 15% safety margin (footprint 0.40m × 0.29m). Robot sometimes collides with door frames. Should I increase safety margin or is this a different issue?"
```

**Self-Reflection**

After configuring costmaps and discussing with AI:
- Can you explain why global and local costmaps have different update frequencies?
- Do you understand the tradeoff between inflation radius (safety) and navigation mobility?
- Can you diagnose whether costmap issues are configuration vs sensor integration problems?

These reflections prepare you for path planner selection in Lesson 3, where costmap quality directly impacts planning success.
