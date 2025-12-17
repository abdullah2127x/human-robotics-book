---
title: Controller Configuration for Humanoid Tracking
chapter: 9
lesson: 4
learning_objectives:
  - Configure DWB controller for stable humanoid trajectory tracking
  - Set velocity and acceleration limits based on bipedal gait stability
  - Monitor and minimize cross-track and heading error during navigation
  - Collaborate with AI using Three Roles to tune controller parameters
estimated_time: 150 minutes
skills:
  controller-tuning:
    proficiency_level: B2
generated_by: content-implementer v1.1.0
source_spec: specs/book/part-3/chapter-09-plan.md
created: 2025-12-17
---

# Controller Configuration for Humanoid Tracking

The path planner (Lesson 3) produces *where* to go—a sequence of waypoints. The controller determines *how* to get there—the velocity commands that make the robot follow the path. For humanoid robots, controller configuration is critical because inappropriate velocity commands cause balance loss and falls.

This lesson focuses on the DWB (Dynamic Window approach with Behavior) controller, Nav2's default local planner/controller. You'll configure velocity limits based on humanoid gait stability, tune acceleration profiles for smooth movement, and monitor tracking error to ensure the robot follows planned paths accurately.

## Why Controller Tuning Matters for Bipedal Robots

Consider the path from Lesson 3: a smooth curve navigating around an obstacle. The planner produced excellent geometry. But the controller must translate this geometry into velocity commands:

**Velocity commands too fast**: Humanoid attempts to walk faster than gait allows, loses balance, falls.

**Velocity changes too abrupt**: Sharp acceleration causes center-of-mass shift, triggers stumble reflex.

**Turning commands too aggressive**: Angular velocity exceeds what stepping gait can achieve, robot tips sideways.

**Tracking error too large**: Robot drifts off path, misses doorway, collides with obstacle planner thought was avoided.

Proper controller tuning ensures the humanoid *actually follows* the planned path while *maintaining balance*. The planner plans; the controller executes.

## Controller Architecture in Nav2

```
Global Path (from Planner)
         ↓
    Controller Server
         ↓
   ┌────────────────────────────────────┐
   │    Plugin: Controller Interface    │
   │   - DWB (Dynamic Window)           │
   │   - Regulated Pure Pursuit         │
   │   - MPPI (Model Predictive)        │
   └────────────────────────────────────┘
         ↓
    Velocity Commands (geometry_msgs/Twist)
         ↓
    Robot Base (motors/actuators)
```

**Controller responsibility**: Given current pose and global path, compute velocity command (linear.x, angular.z) that moves robot toward path while avoiding obstacles.

**Update rate**: Controllers run at high frequency (20-50 Hz) for responsive obstacle avoidance.

## DWB Controller: Dynamic Window Approach

DWB extends the classic Dynamic Window Approach with configurable critics (cost functions):

### Algorithm Overview

1. **Sample velocity space**: Generate candidate (linear, angular) velocity pairs
2. **Simulate trajectories**: For each velocity, simulate short trajectory forward in time
3. **Evaluate critics**: Score each trajectory using cost functions (obstacle distance, path alignment, goal progress)
4. **Select best**: Choose velocity with lowest total cost
5. **Send command**: Publish selected velocity to robot

### Critics (Cost Functions)

DWB uses weighted sum of critic scores:

| Critic | Purpose | Humanoid Relevance |
|--------|---------|-------------------|
| `ObstacleFootprint` | Penalize collisions | Critical—falls are catastrophic |
| `PathAlign` | Reward alignment with path | Important—drift causes issues |
| `PathDist` | Reward progress along path | Important—reach goal |
| `GoalAlign` | Reward alignment with goal orientation | Important—final pose matters |
| `GoalDist` | Reward proximity to goal | Important—reach goal |
| `BaseObstacle` | Penalize proximity to obstacles | Important—safety margin |
| `RotateToGoal` | Enable rotation at goal | Needed for final orientation |

### Velocity Limits for Humanoid Stability

**Linear velocity limits**:

```
max_vel_x = Stable walking speed (typically 0.3-0.5 m/s for humanoids)
min_vel_x = 0.0 (can stop, no backward walking during navigation)
max_vel_y = 0.0 (no lateral movement—bipedal constraint)
```

**Angular velocity limits**:

```
max_vel_theta = Turning rate during walking (typically 0.3-0.6 rad/s)
```

**Why these limits matter**:

- **max_vel_x too high**: Humanoid attempts to run, gait becomes unstable
- **max_vel_theta too high**: Turning exceeds what footsteps can achieve, lateral instability
- **max_vel_y > 0**: Controller might command strafe, which humanoid cannot execute

### Acceleration Limits

Acceleration limits prevent abrupt velocity changes:

```
acc_lim_x = Linear acceleration (0.2-0.4 m/s² for humanoids)
acc_lim_theta = Angular acceleration (0.3-0.5 rad/s² for humanoids)
decel_lim_x = Deceleration (may be higher than acceleration for safety)
```

**Why acceleration limits matter**:

- **acc_lim_x too high**: Sudden forward acceleration shifts center-of-mass backward, causes backward fall
- **decel_lim_x too high**: Sudden stop shifts center-of-mass forward, causes forward stumble
- **acc_lim_theta too high**: Rapid rotation causes lateral center-of-mass shift

**Humanoid-specific consideration**: Humanoids have asymmetric acceleration capability—they can decelerate faster than accelerate (stepping to stop is easier than accelerating smoothly). Configure `decel_lim_x > acc_lim_x`.

## DWB Configuration for Humanoid

Create `dwb_controller.yaml`:

```yaml
controller_server:
  ros__parameters:
    # Controller plugin
    controller_plugins: ["FollowPath"]

    # General parameters
    controller_frequency: 20.0          # 20 Hz control loop
    min_x_velocity_threshold: 0.001     # Minimum velocity threshold
    min_y_velocity_threshold: 0.0       # No lateral movement
    min_theta_velocity_threshold: 0.001 # Minimum angular threshold
    failure_tolerance: 0.3              # Tolerance before failure
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]

    # Progress checker (detect stuck)
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5     # Must move 0.5m
      movement_time_allowance: 10.0     # Within 10 seconds

    # Goal checker
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      stateful: true
      xy_goal_tolerance: 0.25           # 25cm position tolerance
      yaw_goal_tolerance: 0.25          # 0.25 rad orientation tolerance

    # DWB controller configuration
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"

      # Debugging
      debug_trajectory_details: true
      publish_evaluation: true
      publish_global_plan: true
      publish_transformed_plan: true
      publish_local_plan: true
      publish_trajectories: true
      publish_cost_grid_pc: false       # Disable for performance

      # Trajectory generation
      trajectory_generator_name: "dwb_plugins::StandardTrajectoryGenerator"
      sim_time: 1.7                      # Simulate 1.7 seconds ahead
      vx_samples: 20                     # Velocity samples (linear)
      vy_samples: 1                      # Only 1 (no lateral movement)
      vtheta_samples: 20                 # Velocity samples (angular)
      linear_granularity: 0.05           # Trajectory point spacing
      angular_granularity: 0.025         # Angular spacing

      # Velocity limits (CRITICAL for humanoid stability)
      min_vel_x: 0.0                     # Can stop
      max_vel_x: 0.4                     # Max forward velocity (m/s)
      min_vel_y: 0.0                     # No lateral movement
      max_vel_y: 0.0                     # No lateral movement (bipedal)
      min_speed_xy: 0.0                  # Can stop
      max_speed_xy: 0.4                  # Same as max_vel_x
      min_speed_theta: 0.0               # Can stop rotation

      # Acceleration limits (CRITICAL for humanoid balance)
      acc_lim_x: 0.3                     # Forward acceleration (m/s²)
      acc_lim_y: 0.0                     # No lateral acceleration
      acc_lim_theta: 0.4                 # Angular acceleration (rad/s²)
      decel_lim_x: -0.5                  # Deceleration (faster than accel)
      decel_lim_y: 0.0                   # No lateral deceleration
      decel_lim_theta: -0.6              # Angular deceleration

      # Velocity thresholds
      min_vel_theta: -0.5                # Min angular velocity (negative = CW)
      max_vel_theta: 0.5                 # Max angular velocity (CCW)

      # Transform tolerance
      transform_tolerance: 0.2

      # Goal reaching
      short_circuit_trajectory_evaluation: true

      # Critics (cost functions)
      critics: [
        "RotateToGoal",
        "Oscillation",
        "BaseObstacle",
        "GoalAlign",
        "PathAlign",
        "PathDist",
        "GoalDist"
      ]

      # Critic configuration
      BaseObstacle:
        scale: 0.02                      # Weight for obstacle avoidance
        sum_scores: false

      PathAlign:
        scale: 32.0                      # Weight for path following
        forward_point_distance: 0.1      # Look-ahead for alignment

      PathDist:
        scale: 32.0                      # Weight for path progress

      GoalAlign:
        scale: 24.0                      # Weight for goal orientation
        forward_point_distance: 0.1

      GoalDist:
        scale: 24.0                      # Weight for goal proximity

      RotateToGoal:
        scale: 32.0                      # Weight for final rotation
        slowing_factor: 5.0              # Slow down near goal
        lookahead_time: -1.0             # Use path end

      Oscillation:
        scale: 1.0                       # Penalize oscillation
        oscillation_reset_dist: 0.25    # Reset after moving 0.25m
        oscillation_reset_angle: 0.2    # Reset after rotating 0.2 rad
        oscillation_reset_time: -1.0    # No time-based reset
        x_only_threshold: 0.05          # X oscillation threshold
        theta_oscillation_threshold: 0.2 # Theta oscillation threshold
```

### Key Parameter Explanations

**Velocity limits** (lines 50-57):

```yaml
max_vel_x: 0.4        # Humanoid walking speed
max_vel_y: 0.0        # CRITICAL: No lateral movement (bipedal constraint)
max_vel_theta: 0.5    # Turning rate while walking
```

**Acceleration limits** (lines 59-64):

```yaml
acc_lim_x: 0.3        # Gentle acceleration (balance)
decel_lim_x: -0.5     # Faster deceleration (safety stopping)
acc_lim_theta: 0.4    # Gentle turning acceleration
```

**Asymmetric deceleration**: `decel_lim_x` magnitude > `acc_lim_x` because stopping quickly (emergency) is more important than accelerating smoothly.

**Critics** (lines 72-78):

```yaml
critics: ["RotateToGoal", "Oscillation", "BaseObstacle", ...]
```

Order matters for some critics. `Oscillation` critic prevents back-and-forth movement that could destabilize humanoid.

## Understanding Tracking Error

Tracking error measures how well the robot follows the planned path:

### Cross-Track Error (XTE)

**Definition**: Perpendicular distance from robot to nearest path point.

**Acceptable range for humanoids**: < 0.3m (robot stays within 30cm corridor around path)

**Causes of high XTE**:
- Velocity limits too restrictive (can't turn fast enough)
- Path has sharp corners controller can't follow
- Localization error (robot thinks it's somewhere else)

### Heading Error

**Definition**: Angle between robot heading and path tangent direction.

**Acceptable range for humanoids**: < 0.3 rad (about 17°)

**Causes of high heading error**:
- Angular velocity limit too low
- Path changes direction faster than controller responds
- Goal orientation different from path direction

### Monitoring Tracking Error

Create tracking error visualization:

```python
#!/usr/bin/env python3
"""
Monitor and visualize DWB controller tracking error.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import math

class TrackingErrorMonitor(Node):
    def __init__(self):
        super().__init__('tracking_error_monitor')

        # Subscriptions
        self.global_path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10
        )

        # Publishers
        self.xte_pub = self.create_publisher(Float64, '/tracking/cross_track_error', 10)
        self.heading_pub = self.create_publisher(Float64, '/tracking/heading_error', 10)

        # State
        self.current_path = None
        self.max_xte = 0.0
        self.max_heading = 0.0

        # Timer for statistics
        self.create_timer(5.0, self.print_stats)

        self.get_logger().info('Tracking error monitor started')

    def path_callback(self, msg):
        """Store current global path."""
        self.current_path = msg
        # Reset statistics for new path
        self.max_xte = 0.0
        self.max_heading = 0.0

    def pose_callback(self, msg):
        """Calculate and publish tracking errors."""
        if self.current_path is None or len(self.current_path.poses) < 2:
            return

        # Find nearest path point
        robot_x = msg.pose.position.x
        robot_y = msg.pose.position.y

        min_dist = float('inf')
        nearest_idx = 0

        for i, pose in enumerate(self.current_path.poses):
            dx = robot_x - pose.pose.position.x
            dy = robot_y - pose.pose.position.y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i

        # Cross-track error is distance to nearest point
        xte = min_dist

        # Heading error (angle to path tangent)
        heading_error = 0.0
        if nearest_idx < len(self.current_path.poses) - 1:
            # Path tangent direction
            p1 = self.current_path.poses[nearest_idx].pose.position
            p2 = self.current_path.poses[nearest_idx + 1].pose.position
            path_angle = math.atan2(p2.y - p1.y, p2.x - p1.x)

            # Robot heading (from quaternion)
            q = msg.pose.orientation
            robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            heading_error = abs(path_angle - robot_yaw)
            if heading_error > math.pi:
                heading_error = 2 * math.pi - heading_error

        # Update maxima
        self.max_xte = max(self.max_xte, xte)
        self.max_heading = max(self.max_heading, heading_error)

        # Publish
        xte_msg = Float64()
        xte_msg.data = xte
        self.xte_pub.publish(xte_msg)

        heading_msg = Float64()
        heading_msg.data = heading_error
        self.heading_pub.publish(heading_msg)

        # Log warnings if errors exceed thresholds
        if xte > 0.3:
            self.get_logger().warn(f'High cross-track error: {xte:.2f}m')
        if heading_error > 0.3:
            self.get_logger().warn(f'High heading error: {heading_error:.2f}rad')

    def print_stats(self):
        """Print tracking statistics."""
        self.get_logger().info(
            f'Tracking stats - Max XTE: {self.max_xte:.2f}m, '
            f'Max heading: {self.max_heading:.2f}rad'
        )


def main():
    rclpy.init()
    node = TrackingErrorMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercise 1: Run DWB with Default Parameters

**Objective**: Observe baseline controller behavior (may show instability).

**Steps**:

1. Launch Nav2 with default DWB configuration
2. Send navigation goal
3. Observe robot behavior in Isaac Sim

**Expected observations with default params**:
- Robot may accelerate too quickly (jerky start)
- Turns may be too aggressive (lateral sway)
- May attempt lateral movement (instability)

Record observations:

| Observation | Default Behavior | Issue? |
|-------------|------------------|--------|
| Start acceleration | | |
| Turn smoothness | | |
| Lateral movement | | |
| Stop behavior | | |

## Exercise 2: Tune Velocity and Acceleration with AI

**Objective**: Apply humanoid-appropriate limits using Three Roles collaboration.

## Try With AI: Three Roles Demonstration

### Role 1: AI as Teacher — Velocity Limits from Gait Analysis

You need to derive velocity limits from humanoid gait characteristics. Ask AI:

```
"I need to set max_vel_x for DWB controller on bipedal humanoid. My humanoid has:
- Step length: 0.3 m
- Step frequency: 1.5 Hz (during stable walking)
- Mass: 60 kg
- Height: 1.6 m

Calculate appropriate max_vel_x. Also explain:
1. Why going faster than gait allows causes instability
2. How to calculate max_vel_theta from turning gait
3. What happens if I set max_vel_y > 0 for bipedal robot"
```

**Expected learning**: AI derives `max_vel_x = step_length × step_frequency = 0.3 × 1.5 = 0.45 m/s`, explains that faster velocities require running gait (dynamic stability) vs walking gait (static stability), and warns that `max_vel_y > 0` will cause controller to command impossible movements.

### Role 2: AI as Student — Validating Acceleration Safety

You have domain knowledge about balance dynamics. Test AI understanding:

```
"I want to verify your proposed acceleration limits are safe. My constraints:
1. Center-of-mass must stay within support polygon during acceleration
2. Human-sized humanoid can typically handle 0.3-0.5 m/s² without balance disturbance
3. Deceleration can be faster because stepping to stop is more controlled than accelerating

Propose acc_lim_x, acc_lim_theta, decel_lim_x. Explain how each affects balance."
```

**Expected interaction**: AI proposes values, you evaluate against your knowledge of humanoid balance dynamics, correct any misconceptions.

**Your validation checks**:
- Does `acc_lim_x` account for center-of-mass dynamics?
- Is `decel_lim_x` appropriately larger (faster stopping)?
- Does `acc_lim_theta` prevent lateral tip-over during turns?

### Role 3: AI as Co-Worker — Tuning Critic Weights

Collaborate with AI to balance controller critics:

```
"Let's tune DWB critic weights together. My current issues:
1. Robot hugs obstacles too closely (needs more BaseObstacle weight?)
2. Robot overshoots path curves (PathAlign too low?)
3. Robot oscillates near goal (Oscillation weight?)

Current weights:
- BaseObstacle.scale: 0.02
- PathAlign.scale: 32.0
- Oscillation.scale: 1.0

Propose adjustments. I'll test each change and report back."
```

**Expected collaboration**: Iterative tuning where AI proposes weight changes, you test in simulation, refine based on results.

**Example iteration**:

*AI proposes*: "Increase `BaseObstacle.scale` to 0.05 to push robot away from obstacles."

*You test*: Robot maintains better clearance but takes very wide paths.

*Your feedback*: "Better obstacle clearance but paths too conservative—takes 10m detour around 1m obstacle."

*AI refines*: "Try `BaseObstacle.scale: 0.03` and also increase `PathDist.scale` to 48.0 to encourage path following over obstacle avoidance when path is clear."

## Exercise 3: Validate Trajectory Tracking

**Objective**: Confirm controller achieves acceptable tracking error.

**Steps**:

1. Apply tuned DWB configuration
2. Run tracking error monitor
3. Navigate multiple scenarios:
   - Straight line (5m)
   - Gentle curve (90° turn with 2m radius)
   - Sharp curve (90° turn with 1m radius)
   - Goal with specific orientation

4. Record tracking metrics:

| Scenario | Max XTE (m) | Max Heading Error (rad) | Pass/Fail |
|----------|-------------|-------------------------|-----------|
| Straight line | | | |
| Gentle curve | | | |
| Sharp curve | | | |
| Goal orientation | | | |

**Success criteria**:
- Max XTE < 0.3m for all scenarios
- Max heading error < 0.3 rad
- No oscillation at goal

## Troubleshooting Controller Issues

### Issue 1: Robot Accelerates Too Quickly (Jerky Start)

**Symptom**: Robot lurches forward when starting navigation.

**Diagnosis**: `acc_lim_x` too high.

**Fix**: Reduce `acc_lim_x` (try 0.2-0.3 m/s²).

### Issue 2: Robot Can't Follow Curves (High XTE)

**Symptom**: Robot cuts corners or overshoots curves.

**Diagnosis**: `max_vel_theta` too low to turn fast enough, or `PathAlign.scale` too low.

**Fix**:
- Increase `max_vel_theta` (if humanoid can turn faster)
- Increase `PathAlign.scale` (stronger path following)
- Reduce `max_vel_x` during turns (slower = tighter turns)

### Issue 3: Robot Oscillates Near Goal

**Symptom**: Robot wobbles back and forth instead of stopping cleanly.

**Diagnosis**: `Oscillation` critic not effective, or goal tolerance too tight.

**Fix**:
- Increase `Oscillation.scale`
- Increase `xy_goal_tolerance` (more relaxed goal)
- Decrease velocity near goal (`RotateToGoal.slowing_factor`)

### Issue 4: Robot Stops Too Abruptly (Unstable Stop)

**Symptom**: Robot decelerates too fast, causing forward pitch.

**Diagnosis**: `decel_lim_x` magnitude too high.

**Fix**: Reduce `decel_lim_x` magnitude (less negative, e.g., -0.4 instead of -0.5).

### Issue 5: Controller Fails (No Valid Trajectory)

**Symptom**: Controller publishes zero velocity, robot stops unexpectedly.

**Diagnosis**: All sampled trajectories score too high (blocked by obstacles or constraints).

**Fix**:
- Check costmap—is path actually blocked?
- Reduce obstacle critic weight temporarily to diagnose
- Increase `vx_samples` and `vtheta_samples` for more options
- Check velocity limits—are they too restrictive?

## Velocity Profile Visualization

Understanding how velocity changes during navigation helps tune controller:

```python
#!/usr/bin/env python3
"""
Visualize commanded velocity profile during navigation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from collections import deque
import threading

class VelocityProfiler(Node):
    def __init__(self):
        super().__init__('velocity_profiler')

        # Subscription
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )

        # Data storage
        self.max_samples = 1000
        self.times = deque(maxlen=self.max_samples)
        self.vel_x = deque(maxlen=self.max_samples)
        self.vel_theta = deque(maxlen=self.max_samples)
        self.start_time = self.get_clock().now()

        # Plotting thread
        self.plot_thread = threading.Thread(target=self.plot_loop)
        self.plot_thread.daemon = True
        self.plot_thread.start()

        self.get_logger().info('Velocity profiler started')

    def cmd_callback(self, msg):
        """Record velocity command."""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.times.append(elapsed)
        self.vel_x.append(msg.linear.x)
        self.vel_theta.append(msg.angular.z)

    def plot_loop(self):
        """Update plot continuously."""
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

        while True:
            if len(self.times) < 2:
                plt.pause(0.5)
                continue

            ax1.clear()
            ax2.clear()

            times = list(self.times)
            vel_x = list(self.vel_x)
            vel_theta = list(self.vel_theta)

            ax1.plot(times, vel_x, 'b-', label='Linear X')
            ax1.set_ylabel('Linear Velocity (m/s)')
            ax1.set_ylim(-0.1, 0.6)
            ax1.axhline(y=0.4, color='r', linestyle='--', label='Max limit')
            ax1.legend()
            ax1.grid(True)

            ax2.plot(times, vel_theta, 'g-', label='Angular Z')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Angular Velocity (rad/s)')
            ax2.set_ylim(-0.7, 0.7)
            ax2.axhline(y=0.5, color='r', linestyle='--', label='Max limit')
            ax2.axhline(y=-0.5, color='r', linestyle='--')
            ax2.legend()
            ax2.grid(True)

            plt.tight_layout()
            plt.pause(0.1)


def main():
    rclpy.init()
    node = VelocityProfiler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**What to look for**:

- **Smooth ramps**: Velocity should change gradually, not step
- **No overshoots**: Velocity shouldn't exceed limits
- **Coordinated profiles**: Linear and angular velocities should be smooth when turning
- **Clean stops**: Velocity should ramp to zero, not cut abruptly

## Summary: Controller Configuration for Humanoid

**Key configuration decisions**:

1. **Zero lateral velocity**: `max_vel_y: 0.0` enforces bipedal constraint
2. **Conservative velocity**: `max_vel_x: 0.4` matches stable walking speed
3. **Gentle acceleration**: `acc_lim_x: 0.3` prevents balance disruption
4. **Faster deceleration**: `decel_lim_x: -0.5` allows safety stops
5. **Strong path following**: High `PathAlign.scale` and `PathDist.scale`
6. **Oscillation prevention**: `Oscillation` critic prevents goal wobble

**Tracking error targets**:

| Metric | Target | Action if Exceeded |
|--------|--------|-------------------|
| Cross-track error | < 0.3m | Increase PathAlign, reduce max_vel_x |
| Heading error | < 0.3 rad | Increase max_vel_theta, check path curvature |
| Oscillation | None | Increase Oscillation.scale |

**Next lesson**: You'll design behavior trees that orchestrate navigation with recovery behaviors—what happens when the controller gets stuck or path is blocked.

## Checkpoint: Controller Validation

Before proceeding to Lesson 5 (behavior trees), validate:

- [ ] **DWB configured**: Velocity and acceleration limits set for humanoid
- [ ] **Zero lateral velocity**: `max_vel_y: 0.0` confirmed
- [ ] **Tracking error acceptable**: XTE < 0.3m, heading < 0.3 rad
- [ ] **No oscillation**: Robot stops cleanly at goal
- [ ] **Smooth velocity profile**: No jerky acceleration/deceleration
- [ ] **AI collaboration completed**: Three Roles demonstration practiced

**If all checks pass**, your controller is correctly configured for humanoid trajectory tracking.
