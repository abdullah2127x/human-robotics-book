---
title: Path Planner Selection for Bipedal Gait
chapter: 9
lesson: 3
learning_objectives:
  - Compare Navfn, Smac, and TEB planners for bipedal humanoid navigation
  - Configure TEB planner parameters for smooth, kinematically-feasible paths
  - Understand bipedal kinematic constraints and their impact on path quality
  - Collaborate with AI using Three Roles to tune planner cost weights
estimated_time: 150 minutes
skills:
  path-planner-selection:
    proficiency_level: B2
generated_by: content-implementer v1.1.0
source_spec: specs/book/part-3/chapter-09-plan.md
created: 2025-12-17
---

# Path Planner Selection for Bipedal Gait

Path planning determines how your humanoid navigates from current position to goal—the sequence of waypoints defining the trajectory. Unlike wheeled robots that can turn on the spot and strafe sideways, bipedal humanoids have kinematic constraints that require smooth, curvature-limited paths.

This lesson compares three Nav2 planners (Navfn, Smac, TEB) and explains why trajectory-optimization planners like TEB are preferred for bipedal robots. You'll configure all three planners, compare their outputs visually, and collaborate with AI to tune TEB parameters for humanoid-appropriate paths.

## Why Planner Choice Matters for Bipedal Robots

Consider a humanoid navigating around a table to reach a doorway:

**Navfn planner output**: Grid-based path with sharp 90° turns, shortest distance but kinematically infeasible—humanoid can't execute instant direction changes.

**TEB planner output**: Smooth curved path respecting turning radius constraints, longer distance but actually executable—humanoid can follow without stumbling.

**The difference**: Navfn optimizes *distance*, TEB optimizes *trajectory feasibility*. For bipedal robots, feasibility trumps distance because falling is catastrophic.

### Bipedal Kinematic Constraints

Humanoid robots have constraints that wheeled robots don't:

**Turning radius**: Humanoids can't pivot instantly. Minimum turning radius depends on gait width and stride length (typically 0.5-1.0m for human-sized robots).

**Lateral movement**: Limited or impossible. Wheeled robots can strafe; humanoids must turn and walk.

**Velocity coupling**: Forward velocity and angular velocity are coupled—turning while walking requires coordination, can't be arbitrary.

**Step continuity**: Path must be followable with discrete footsteps. Sharp corners might require stop-turn-walk sequences that break flow.

These constraints mean planners optimizing pure distance produce paths that are either:
1. Infeasible (humanoid can't execute)
2. Suboptimal (humanoid must stop, turn, restart—slower than smooth curve)

## Nav2 Planner Architecture

Before comparing planners, understand how Nav2 integrates them:

```
Navigation Goal (RViz / Action Client)
         ↓
    Navigator
         ↓
    Planner Server
         ↓
   ┌────────────────────────────────────┐
   │    Plugin: Planner Interface       │
   │   - Navfn (Dijkstra/A*)            │
   │   - Smac (State Lattice)           │
   │   - TEB (Trajectory Optimization)  │
   └────────────────────────────────────┘
         ↓
    Global Path (nav_msgs/Path)
         ↓
    Controller Server (follows path)
```

**Key insight**: Planners are *plugins*—you can swap them by changing configuration, no code changes required. This allows experimentation with different planners on same navigation task.

## Planner 1: Navfn (Dijkstra/A*)

### Algorithm Overview

Navfn implements classic grid-based search:

1. **Grid representation**: Costmap cells as graph nodes
2. **Dijkstra expansion**: Expand from start, find minimum-cost path to goal
3. **A* variant**: Optional heuristic (Euclidean distance) to speed up search

**Advantages**:
- Fast computation (milliseconds for typical environments)
- Guarantees shortest path on grid
- Well-understood, widely used

**Disadvantages**:
- Produces jagged paths (follows grid edges)
- No kinematic awareness (ignores turning radius)
- Sharp corners require smoothing post-processing

### Navfn Configuration

Create `navfn_planner.yaml`:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5                  # Goal tolerance (meters)
      use_astar: true                 # Use A* (faster than Dijkstra)
      allow_unknown: false            # Don't plan through unknown space
      use_final_approach_orientation: true  # Use goal orientation
```

**When Navfn is appropriate**:
- Wheeled robots with high maneuverability
- Environments requiring shortest path (warehouse logistics)
- When post-processing smoothing is applied

**When Navfn is inappropriate**:
- Bipedal robots (sharp corners are infeasible)
- Environments requiring smooth trajectories (human-populated spaces)
- When path quality matters more than computation time

## Planner 2: Smac Planner (State Lattice)

### Algorithm Overview

Smac planners use state lattice primitives—pre-computed motion patterns that are kinematically feasible:

1. **Motion primitives**: Library of short, feasible trajectories (curves, straights)
2. **Lattice search**: Connect start to goal using primitive combinations
3. **Kinematic awareness**: Primitives respect turning radius, curvature limits

**Variants**:
- **Smac 2D**: 2D lattice (x, y), no heading consideration
- **Smac Hybrid-A***: 2D + heading (x, y, θ), car-like kinematics
- **Smac Lattice**: Full state lattice with custom primitives

**Advantages**:
- Produces kinematically-feasible paths
- Respects turning radius constraints
- Smoother than Navfn (curves instead of grid edges)

**Disadvantages**:
- More computation than Navfn (larger search space)
- Requires primitive design for specific robot kinematics
- Not optimized for trajectory time/velocity

### Smac Hybrid-A* Configuration

Create `smac_planner.yaml`:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["SmacHybrid"]

    SmacHybrid:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.5                          # Goal tolerance
      downsample_costmap: false               # Use full resolution
      downsampling_factor: 1

      # Kinematic constraints
      minimum_turning_radius: 0.8             # Humanoid turning radius (meters)
      allow_reverse: false                    # Humanoids don't walk backward during navigation

      # Search parameters
      max_iterations: 100000                  # Search iteration limit
      max_on_approach_iterations: 1000        # Iterations when near goal
      max_planning_time: 5.0                  # Max planning time (seconds)

      # Path smoothing
      smooth_path: true                       # Enable post-smoothing

      # Cost weights
      cost_travel_multiplier: 1.0             # Weight for path length
      cost_penalty_cost_multiplier: 2.0       # Weight for costmap costs

      # Analytic expansion
      analytic_expansion_ratio: 3.5           # When to try direct connection
      analytic_expansion_max_length: 3.0      # Max length of direct connection
```

**Key parameters for humanoids**:

- `minimum_turning_radius: 0.8`: Prevents sharp turns (adjust based on your humanoid's gait)
- `allow_reverse: false`: Humanoids navigate forward, not backward
- `smooth_path: true`: Additional smoothing for natural movement

**When Smac is appropriate**:
- Car-like robots with Ackermann steering
- Robots with known kinematic models
- When turning radius constraint is primary concern

**When Smac is suboptimal for humanoids**:
- Doesn't optimize trajectory velocity/time
- Primitives designed for car kinematics, not bipedal gait
- Doesn't account for acceleration limits

## Planner 3: TEB (Timed Elastic Band)

### Algorithm Overview

TEB optimizes *trajectories*, not just *paths*—it considers time, velocity, and acceleration:

1. **Initial path**: Start with straight line or previous path
2. **Elastic band**: Treat path as series of poses connected by constraints
3. **Optimization**: Minimize cost function considering:
   - Distance from obstacles
   - Path smoothness (curvature)
   - Velocity limits
   - Acceleration limits
   - Time to goal

**Advantages**:
- Produces smooth, time-optimal trajectories
- Respects velocity and acceleration limits (critical for humanoids)
- Handles dynamic obstacles through replanning
- Optimizes trajectory, not just path geometry

**Disadvantages**:
- More computation than Navfn/Smac
- Requires careful tuning (many parameters)
- Can get stuck in local optima

### Why TEB is Best for Bipedal Humanoids

**Velocity awareness**: TEB plans trajectories with velocity profiles—critical because humanoids have stability-dependent speed limits.

**Acceleration limits**: Sharp accelerations can cause balance loss. TEB respects acceleration constraints.

**Smooth curvature**: TEB minimizes path curvature, producing natural walking trajectories.

**Time optimization**: TEB produces efficient trajectories balancing speed and safety.

### TEB Configuration for Humanoid

Create `teb_planner.yaml`:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["TEB"]

    TEB:
      plugin: "teb_local_planner/TebLocalPlannerROS"

      # Trajectory configuration
      teb_autosize: true
      dt_ref: 0.3                             # Desired temporal resolution
      dt_hysteresis: 0.1                      # Hysteresis for auto-resize
      max_samples: 500                        # Max trajectory samples
      global_plan_overwrite_orientation: true # Use goal orientation
      allow_init_with_backwards_motion: false # No backward walking
      max_global_plan_lookahead_dist: 3.0     # How far ahead to consider
      global_plan_viapoint_sep: 0.5           # Via-point separation
      global_plan_prune_distance: 1.0         # Prune old global path
      exact_arc_length: false                 # Approximation OK
      feasibility_check_no_poses: 4           # Feasibility check resolution
      publish_feedback: true                  # Publish optimization feedback

      # Robot configuration (CRITICAL for humanoid)
      max_vel_x: 0.4                          # Max forward velocity (m/s)
      max_vel_x_backwards: 0.0                # No backward walking
      max_vel_y: 0.0                          # No lateral movement (bipedal)
      max_vel_theta: 0.5                      # Max angular velocity (rad/s)
      acc_lim_x: 0.3                          # Forward acceleration limit
      acc_lim_y: 0.0                          # No lateral acceleration
      acc_lim_theta: 0.4                      # Angular acceleration limit

      # Turning constraints
      min_turning_radius: 0.5                 # Minimum turning radius (meters)
      wheelbase: 0.0                          # Not applicable for humanoid

      # Footprint (match costmap configuration)
      footprint_model:
        type: "polygon"
        vertices: [[0.145, 0.20], [0.145, -0.20], [-0.145, -0.20], [-0.145, 0.20]]

      # Goal tolerance
      xy_goal_tolerance: 0.2                  # Position tolerance (meters)
      yaw_goal_tolerance: 0.1                 # Orientation tolerance (radians)
      free_goal_vel: false                    # Stop at goal (don't coast)
      complete_global_plan: true              # Follow entire global path

      # Obstacle avoidance
      min_obstacle_dist: 0.5                  # Min distance to obstacles
      inflation_dist: 0.6                     # Inflation distance
      include_costmap_obstacles: true         # Use costmap for obstacles
      costmap_obstacles_behind_robot_dist: 1.5 # Consider obstacles behind
      obstacle_poses_affected: 30             # Poses affected by each obstacle

      # Optimization weights (TUNE THESE)
      weight_kinematics_nh: 1000.0            # Non-holonomic constraint weight
      weight_kinematics_forward_drive: 1.0   # Prefer forward motion
      weight_kinematics_turning_radius: 10.0 # Turning radius weight
      weight_optimaltime: 1.0                 # Time optimality weight
      weight_shortest_path: 0.0               # Shortest path weight (disabled)
      weight_obstacle: 100.0                  # Obstacle avoidance weight
      weight_inflation: 0.2                   # Inflation cost weight
      weight_dynamic_obstacle: 10.0           # Dynamic obstacle weight
      weight_dynamic_obstacle_inflation: 0.2  # Dynamic obstacle inflation
      weight_viapoint: 1.0                    # Via-point following weight
      weight_adapt_factor: 2.0                # Adaptation factor

      # Optimization parameters
      no_inner_iterations: 5                  # Inner optimization iterations
      no_outer_iterations: 4                  # Outer optimization iterations
      optimization_activate: true             # Enable optimization
      optimization_verbose: false             # Quiet output
      penalty_epsilon: 0.1                    # Penalty function epsilon

      # Homotopy (alternative paths)
      enable_homotopy_class_planning: false   # Single path (simpler)
      enable_multithreading: true             # Use multiple threads
```

**Critical parameters for humanoids**:

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `max_vel_x` | 0.4 | Humanoid walking speed (stability limit) |
| `max_vel_y` | 0.0 | No lateral movement (bipedal constraint) |
| `acc_lim_x` | 0.3 | Smooth acceleration (balance maintenance) |
| `min_turning_radius` | 0.5 | Gait-appropriate turning (feet clearance) |
| `weight_kinematics_nh` | 1000.0 | Strong non-holonomic enforcement |
| `weight_shortest_path` | 0.0 | Disabled—prioritize smoothness over distance |

### TEB Weight Tuning Guide

TEB optimization minimizes weighted sum of costs:

```
Total Cost = w₁×Obstacle + w₂×Time + w₃×Smoothness + w₄×Kinematics + ...
```

**Weight interpretation**:

- **Higher weight** = Stronger constraint (optimizer works harder to satisfy)
- **Lower weight** = Soft preference (optimizer may violate if other costs demand)

**Tuning strategy for humanoids**:

1. **Start with kinematics**: Set `weight_kinematics_nh` high (1000+) to ensure physically-feasible paths
2. **Obstacle safety**: Set `weight_obstacle` high (100+) for collision avoidance
3. **Smoothness**: Keep default weights for curvature/velocity smoothness
4. **Time**: Lower `weight_optimaltime` if robot rushes dangerously; raise if too slow
5. **Shortest path**: Keep at 0.0—humanoids don't need shortest paths, they need smooth paths

## Comparing Planners: Same Scenario, Different Results

Let's run all three planners on identical navigation scenario to visualize differences.

### Setup: Navigation Test Scenario

```bash
# Ensure Isaac Sim is running with humanoid in indoor environment
# Ensure VSLAM is running (provides /map and localization)
# Ensure costmaps are configured (Lesson 2)

# Source workspace
source ~/humanoid_ws/install/setup.bash
```

### Test Script: Compare Planners

Create `compare_planners.py`:

```python
#!/usr/bin/env python3
"""
Compare Nav2 planners on identical navigation scenario.
Visualizes path differences between Navfn, Smac, and TEB.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import os
import time

class PlannerComparison(Node):
    def __init__(self):
        super().__init__('planner_comparison')

        # Action client for planner server
        self.planner_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose'
        )

        # Publisher for path visualization
        self.path_pub = self.create_publisher(
            MarkerArray, '/planner_comparison/paths', 10
        )

        # Store results
        self.paths = {}

        self.get_logger().info('Planner comparison node initialized')

    def compute_path(self, planner_id: str, goal: PoseStamped) -> Path:
        """Compute path using specified planner."""

        # Wait for action server
        if not self.planner_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Planner server not available')
            return None

        # Create goal
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal
        goal_msg.planner_id = planner_id
        goal_msg.use_start = False  # Use current robot pose as start

        # Send goal
        self.get_logger().info(f'Computing path with {planner_id}...')
        start_time = time.time()

        future = self.planner_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{planner_id}: Goal rejected')
            return None

        # Get result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        elapsed = time.time() - start_time

        path = result.result.path
        self.get_logger().info(
            f'{planner_id}: {len(path.poses)} poses, {elapsed:.3f}s'
        )

        return path

    def visualize_paths(self):
        """Publish path visualization markers."""

        colors = {
            'GridBased': (1.0, 0.0, 0.0),   # Red: Navfn
            'SmacHybrid': (0.0, 1.0, 0.0),  # Green: Smac
            'TEB': (0.0, 0.0, 1.0)          # Blue: TEB
        }

        marker_array = MarkerArray()

        for i, (planner_id, path) in enumerate(self.paths.items()):
            if path is None:
                continue

            marker = Marker()
            marker.header = path.header
            marker.ns = planner_id
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05  # Line width
            marker.color.r, marker.color.g, marker.color.b = colors.get(
                planner_id, (1.0, 1.0, 1.0)
            )
            marker.color.a = 0.8

            # Add path points
            for pose in path.poses:
                marker.points.append(pose.pose.position)

            marker_array.markers.append(marker)

        self.path_pub.publish(marker_array)
        self.get_logger().info('Published path visualization')

    def run_comparison(self, goal_x: float, goal_y: float, goal_yaw: float):
        """Run comparison with specified goal."""

        # Create goal pose
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0

        # Convert yaw to quaternion (simplified)
        import math
        goal.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.orientation.w = math.cos(goal_yaw / 2.0)

        # Test each planner
        planners = ['GridBased', 'SmacHybrid', 'TEB']

        for planner_id in planners:
            self.paths[planner_id] = self.compute_path(planner_id, goal)

        # Visualize results
        self.visualize_paths()

        # Print summary
        self.print_comparison()

    def print_comparison(self):
        """Print path comparison summary."""

        print('\n' + '='*60)
        print('PLANNER COMPARISON RESULTS')
        print('='*60)

        for planner_id, path in self.paths.items():
            if path is None:
                print(f'{planner_id}: FAILED')
                continue

            # Calculate path length
            length = 0.0
            for i in range(1, len(path.poses)):
                dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x
                dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y
                length += (dx**2 + dy**2)**0.5

            # Calculate max curvature (approximation)
            max_curvature = 0.0
            for i in range(1, len(path.poses) - 1):
                # Angle change
                dx1 = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x
                dy1 = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y
                dx2 = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x
                dy2 = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y

                import math
                angle1 = math.atan2(dy1, dx1)
                angle2 = math.atan2(dy2, dx2)
                angle_change = abs(angle2 - angle1)
                if angle_change > math.pi:
                    angle_change = 2*math.pi - angle_change

                # Curvature = angle_change / segment_length
                seg_len = ((dx1**2 + dy1**2)**0.5 + (dx2**2 + dy2**2)**0.5) / 2
                if seg_len > 0.01:
                    curvature = angle_change / seg_len
                    max_curvature = max(max_curvature, curvature)

            print(f'{planner_id}:')
            print(f'  Path length: {length:.2f} m')
            print(f'  Waypoints: {len(path.poses)}')
            print(f'  Max curvature: {max_curvature:.2f} rad/m')
            print()

        print('='*60)
        print('INTERPRETATION:')
        print('- Lower curvature = smoother path (better for humanoids)')
        print('- Path length matters less than smoothness for bipedal robots')
        print('- TEB should show lowest curvature with reasonable length')
        print('='*60 + '\n')


def main():
    rclpy.init()

    node = PlannerComparison()

    # Run comparison: navigate to goal 5m ahead, 2m to the right
    # Adjust coordinates based on your Isaac Sim environment
    node.run_comparison(
        goal_x=5.0,
        goal_y=-2.0,
        goal_yaw=0.0
    )

    # Keep node alive for visualization
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Comparison

```bash
# Terminal 1: Launch Nav2 with all planners configured
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=~/humanoid_ws/src/humanoid_nav2_config/config/all_planners.yaml

# Terminal 2: Run comparison script
python3 compare_planners.py

# Terminal 3: Visualize in RViz
ros2 run rviz2 rviz2
# Add MarkerArray display for /planner_comparison/paths
```

### Expected Results

**Navfn (Red path)**:
- Shortest distance
- Sharp corners (high curvature peaks)
- Grid-aligned segments

**Smac Hybrid (Green path)**:
- Slightly longer than Navfn
- Smoother curves
- Respects turning radius

**TEB (Blue path)**:
- May be longest distance
- Smoothest curves (lowest max curvature)
- Velocity-feasible trajectory

**Visual comparison**: TEB path should look most "natural"—like how a human would walk, avoiding sharp direction changes.

## Exercise 1: Run All Three Planners

**Objective**: Observe visual differences between planner outputs.

**Steps**:

1. Ensure Isaac Sim humanoid is running with VSLAM and costmaps
2. Configure Nav2 with all three planner plugins:

Create `all_planners.yaml`:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased", "SmacHybrid", "TEB"]

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: false

    SmacHybrid:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.5
      minimum_turning_radius: 0.8
      allow_reverse: false
      smooth_path: true

    TEB:
      plugin: "teb_local_planner/TebLocalPlannerROS"
      max_vel_x: 0.4
      max_vel_y: 0.0
      max_vel_theta: 0.5
      acc_lim_x: 0.3
      min_turning_radius: 0.5
      weight_kinematics_nh: 1000.0
```

3. Run comparison script
4. Record observations in table:

| Metric | Navfn | Smac | TEB |
|--------|-------|------|-----|
| Path length (m) | | | |
| Max curvature (rad/m) | | | |
| Computation time (s) | | | |
| Sharp corners (count) | | | |

**Success criteria**:
- [ ] All three planners produce valid paths
- [ ] TEB shows lowest max curvature
- [ ] Visual differences are clearly observable in RViz

## Exercise 2: Configure TEB for Humanoid (with AI Assistance)

**Objective**: Tune TEB parameters for your specific humanoid using Three Roles collaboration.

This exercise demonstrates collaborative AI usage—you provide domain constraints (humanoid kinematics), AI helps optimize parameters.

**Prerequisites**: Completed Exercise 1, observed TEB path characteristics.

## Try With AI: Three Roles Demonstration

Now apply Three Roles collaboration pattern to tune TEB parameters.

### Role 1: AI as Teacher — Understanding Kinematic Constraints

You need to understand how humanoid kinematics map to TEB parameters. Ask AI:

```
"I'm configuring TEB planner for bipedal humanoid navigation. My humanoid has:
- Walking speed: 0.4 m/s max
- Step length: 0.3 m
- Stance width: 0.35 m
- Can't strafe sideways

Explain which TEB parameters I need to constrain and why. Focus on:
1. max_vel_x, max_vel_y, max_vel_theta
2. acc_lim_x, acc_lim_theta
3. min_turning_radius
4. weight_kinematics_nh"
```

**Expected learning**: AI explains how bipedal gait constraints translate to specific TEB parameter values, why `max_vel_y: 0.0` is mandatory, and how turning radius relates to stance width.

### Role 2: AI as Student — Validating Your Stability Requirements

You have domain knowledge about humanoid stability. Test whether AI understands your constraints:

```
"I want to prioritize stability over speed in my humanoid's navigation. My requirements:
1. No paths that require stopping and turning in place (momentum helps balance)
2. Prefer gentle curves over sharp corners
3. Velocity should ramp up/down gradually

Propose TEB weight adjustments to achieve this. I'll evaluate if your suggestions match my stability requirements."
```

**Expected interaction**: AI proposes weight changes (e.g., increase `weight_kinematics_turning_radius`, decrease `weight_optimaltime`). You evaluate whether proposals align with stability needs, correct if necessary.

**Your validation questions**:
- Does the proposed `acc_lim_x` allow smooth velocity changes?
- Will the turning radius constraint prevent stop-turn-walk sequences?
- Are obstacle weights high enough to maintain safety margins?

### Role 3: AI as Co-Worker — Balancing Cost Weights

You and AI collaborate to find optimal weight balance:

```
"Let's tune TEB weights together. My current configuration:
- weight_obstacle: 100.0
- weight_optimaltime: 1.0
- weight_kinematics_nh: 1000.0
- weight_shortest_path: 0.0

Problem: Paths are safe but robot takes very wide turns, adding significant time.

Let's find better balance:
1. You propose adjustment to reduce turn radius while maintaining safety
2. I'll evaluate if it works with humanoid gait
3. We iterate until we find good balance"
```

**Expected collaboration**: Iterative refinement where AI proposes weight changes, you test in simulation, provide feedback, AI refines further.

**Example iteration**:

*AI proposes*: "Try reducing `weight_obstacle` to 50.0 to allow tighter paths near obstacles."

*You test*: Robot passes closer to obstacles than comfortable.

*Your feedback*: "Paths too close to walls. Humanoid needs 0.5m+ clearance for arm swing during walking."

*AI refines*: "Keep `weight_obstacle: 100.0` but reduce `weight_kinematics_turning_radius` from 10.0 to 5.0. This allows tighter turns in open space while maintaining obstacle clearance."

*You test*: Better balance achieved.

## Exercise 3: Validate Smooth Paths

**Objective**: Confirm TEB produces humanoid-appropriate paths.

**Steps**:

1. Apply tuned TEB configuration from Exercise 2
2. Plan paths to multiple goals:
   - Straight ahead (5m forward)
   - Around obstacle (table, chair)
   - Through doorway (narrow passage)
   - 180° turn (return to start)

3. For each path, verify:

| Validation | Pass/Fail | Notes |
|------------|-----------|-------|
| No sharp corners (> 45° in < 0.5m) | | |
| Path clears obstacles by 0.5m+ | | |
| Curvature gradual (no sudden changes) | | |
| Path is executable (humanoid can follow) | | |

**Success criteria**:
- [ ] All paths pass validation
- [ ] Doorway navigation succeeds (humanoid fits)
- [ ] 180° turn uses smooth arc, not pivot-in-place

## Troubleshooting Planner Issues

### Issue 1: TEB Produces Infeasible Path (Sharp Corners)

**Symptom**: Despite TEB configuration, path still has sharp corners.

**Diagnosis**: Check `weight_kinematics_nh`—if too low, optimizer ignores kinematic constraints.

**Fix**: Increase `weight_kinematics_nh` to 1000+ (strong enforcement).

### Issue 2: TEB Takes Too Long (> 1 second planning)

**Symptom**: Navigation response is sluggish, planning takes seconds.

**Diagnosis**: Check `no_inner_iterations` and `no_outer_iterations`—too many iterations slow planning.

**Fix**:
- Reduce `no_inner_iterations` to 3-5
- Reduce `no_outer_iterations` to 3-4
- Reduce `max_samples` to 300

### Issue 3: Planner Fails to Find Path

**Symptom**: Planner returns empty path, goal unreachable.

**Diagnosis**:
1. Check costmap—is goal in obstacle/inflated zone?
2. Check inflation radius—too large blocks valid paths
3. Check turning radius—too large prevents navigation in tight spaces

**Fix**:
- Reduce inflation radius (costmap configuration)
- Reduce `min_turning_radius` in TEB
- Verify goal location is in free space

### Issue 4: Smac Ignores Turning Radius

**Symptom**: Smac produces paths with sharper turns than `minimum_turning_radius`.

**Diagnosis**: Smac Hybrid uses approximate kinematic model, may not strictly enforce turning radius.

**Fix**: Switch to TEB if strict turning radius enforcement is required.

## Summary: Planner Selection for Humanoid Navigation

**Key decisions**:

1. **Use TEB for bipedal robots**: Trajectory optimization produces smooth, kinematically-feasible paths
2. **Disable shortest path optimization**: `weight_shortest_path: 0.0`—smoothness matters more than distance
3. **Enforce non-holonomic constraints**: `weight_kinematics_nh: 1000.0+` ensures feasible paths
4. **Set appropriate velocity limits**: Match humanoid gait stability limits
5. **Zero lateral velocity**: `max_vel_y: 0.0`—humanoids don't strafe

**Planner comparison summary**:

| Planner | Best For | Humanoid Suitability |
|---------|----------|---------------------|
| Navfn | Wheeled robots, shortest path | ❌ Sharp corners |
| Smac | Car-like kinematics | ⚠️ Approximate |
| TEB | Bipedal robots, smooth trajectories | ✅ Recommended |

**Next lesson**: You'll configure the controller (DWB) to follow TEB's smooth paths while maintaining humanoid balance through velocity and acceleration limiting.

## Checkpoint: Path Planner Validation

Before proceeding to Lesson 4 (controllers), validate:

- [ ] **All planners configured**: Navfn, Smac, TEB configs created
- [ ] **TEB tuned for humanoid**: Velocity limits, turning radius, weights adjusted
- [ ] **Comparison completed**: Observed visual differences between planners
- [ ] **AI collaboration practiced**: Three Roles demonstration completed
- [ ] **Smooth paths validated**: TEB produces humanoid-appropriate trajectories
- [ ] **No sharp corners**: Max curvature within acceptable limits

**If all checks pass**, your path planner is correctly configured for humanoid navigation.
