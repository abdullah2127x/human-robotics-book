---
title: Dynamic Obstacle Avoidance and Replanning
chapter: 9
lesson: 6
learning_objectives:
  - Configure costmap update frequency for real-time dynamic obstacle detection
  - Implement replanning triggers for obstacle-induced path changes
  - Design safe stop behavior for collision-imminent scenarios
  - Collaborate with AI using Three Roles to tune dynamic avoidance parameters
estimated_time: 150 minutes
skills:
  dynamic-obstacle-handling:
    proficiency_level: B2
generated_by: content-implementer v1.1.0
source_spec: specs/book/part-3/chapter-09-plan.md
created: 2025-12-17
---

# Dynamic Obstacle Avoidance and Replanning

Static obstacles—walls, furniture, permanent structures—are captured in the SLAM map and handled by global path planning. But humanoid robots navigate human-populated environments where obstacles move: people walk, doors open, objects get relocated. Dynamic obstacle handling determines whether your humanoid gracefully navigates around a walking person or collides because it didn't react in time.

This lesson focuses on configuring Nav2 for real-time dynamic obstacle detection and response. You'll tune costmap update frequencies, implement replanning triggers, and design safe-stop behaviors for collision-imminent scenarios.

## Why Dynamic Obstacles Are Challenging

Consider a humanoid walking toward a doorway:

**Static world**: Path planned around furniture, robot follows path, reaches goal. Simple.

**Dynamic world**:
1. Path planned around furniture
2. Person walks across path
3. Robot must detect person (sensor update)
4. Costmap must mark person as obstacle (costmap update)
5. Controller must react (replan or stop)
6. All within ~1 second before collision

**The challenge**: Perception → Decision → Action pipeline must complete faster than collision timeline. For humanoid walking at 0.4 m/s, detecting person 2m away gives 5 seconds to react—comfortable. Detecting at 0.5m gives 1.25 seconds—tight.

### Humanoid-Specific Challenges

**Balance during emergency stop**: Wheeled robots can stop instantly. Humanoids must decelerate gradually or risk falling. This extends stopping distance.

**Perception gaps**: Humanoid depth cameras may have blind spots (below knee level, behind arms). Moving obstacles can appear suddenly.

**Reaction time**: Humanoid gait has momentum. Faster reaction requires sharper deceleration, risking balance.

## Dynamic Obstacle Pipeline

```
Sensor Data (Depth Camera / LiDAR)
         ↓ (sensor rate: 30 Hz)
    Costmap Obstacle Layer
         ↓ (update rate: 5-10 Hz)
    Local Costmap Update
         ↓ (publish rate: 5-10 Hz)
    Controller (DWB)
         ↓ (control rate: 20 Hz)
   ┌─────┴─────┐
   │           │
 Replan    Safe Stop
(if path blocked)  (if collision imminent)
```

### Critical Timing Parameters

| Parameter | Typical Value | Effect if Too Low | Effect if Too High |
|-----------|---------------|-------------------|-------------------|
| Sensor rate | 30 Hz | Miss fast-moving obstacles | Computational load |
| Costmap update | 5-10 Hz | Slow obstacle marking | CPU overhead |
| Controller rate | 20 Hz | Sluggish response | Unnecessary computation |
| Replan rate | 0.5-1 Hz | Slow path adaptation | Excessive replanning |

**Key insight**: The slowest component in the pipeline determines overall reaction time. A 30 Hz sensor with 2 Hz costmap update means obstacles appear in costmap every 0.5 seconds—too slow for fast-moving obstacles.

## Configuring Costmap for Dynamic Obstacles

### Update Frequency Configuration

Local costmap must update frequently for dynamic obstacle response:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # High update frequency for dynamic obstacles
      update_frequency: 10.0    # 10 Hz update rate
      publish_frequency: 5.0    # 5 Hz visualization

      # Obstacle layer configuration
      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true

        # Sensor integration
        observation_sources: depth_camera lidar

        depth_camera:
          topic: /camera/depth/points
          sensor_frame: camera_link
          data_type: "PointCloud2"
          marking: true                   # Mark obstacles
          clearing: true                  # Clear free space
          obstacle_max_range: 4.0         # Dynamic obstacles within 4m
          obstacle_min_range: 0.2         # Ignore very close (sensor noise)
          raytrace_max_range: 5.0         # Clear space to 5m
          raytrace_min_range: 0.0
          max_obstacle_height: 2.0        # Human height
          min_obstacle_height: 0.1        # Ignore floor

        lidar:
          topic: /scan
          sensor_frame: lidar_link
          data_type: "LaserScan"
          marking: true
          clearing: true
          obstacle_max_range: 10.0
          obstacle_min_range: 0.1
          inf_is_valid: false             # Ignore inf readings
```

### Clearing Behavior

**Why clearing matters**: When person walks away, their previous position should become free space. Without clearing, costmap accumulates ghost obstacles.

**Raycasting**: Costmap traces rays from sensor through space. Cells along ray are marked free (cleared) until obstacle is hit.

```yaml
# Clearing configuration
clearing: true              # Enable clearing
raytrace_max_range: 5.0     # Clear space up to 5m from sensor
raytrace_min_range: 0.0     # Start clearing from sensor origin
```

**Problem**: If sensor can't see ground level (depth camera looking forward, not down), low obstacles never get cleared.

**Solution**: Use multiple sensors (depth camera + downward-facing sensor) or accept ground-level blind spot.

### Obstacle Tracking Options

Nav2 provides obstacle tracking for predictive avoidance:

```yaml
# Enable obstacle tracking (experimental)
obstacle_layer:
  track_unknown_space: true
  observation_persistence: 0.0    # How long to keep observations (0 = forever)
```

**Observation persistence**: How long obstacles remain marked after last observation.
- `0.0`: Keep forever (until cleared by raytrace)
- `> 0`: Expire after N seconds

**For dynamic obstacles**: Consider `observation_persistence: 2.0`—if person not seen for 2 seconds, assume they moved (but rely on clearing primarily).

## Replanning Triggers

When should the planner generate a new path?

### Trigger 1: Path Blocked

If local costmap shows obstacle on planned path, replan.

**Configuration** (in behavior tree):
```xml
<RateController hz="0.5">
  <ComputePathToPose goal="{goal}" path="{path}" planner_id="TEB"/>
</RateController>
```

**Rate**: 0.5 Hz means check every 2 seconds. Too slow for fast dynamics.

**Improvement**: Event-driven replanning when costmap changes significantly.

### Trigger 2: Controller Failure

If controller can't find valid trajectory (all trajectories hit obstacles), trigger replan.

```yaml
# Controller failure triggers recovery
controller_server:
  ros__parameters:
    failure_tolerance: 0.3    # Seconds of failure before recovery
```

### Trigger 3: Progress Timeout

If robot makes insufficient progress, replan (maybe current path is suboptimal):

```yaml
progress_checker:
  plugin: "nav2_controller::SimpleProgressChecker"
  required_movement_radius: 0.5   # Must move 0.5m
  movement_time_allowance: 10.0   # Within 10 seconds
```

### Trigger 4: Goal Update

If goal changes, replan immediately.

**Implementation**: BT has `GoalUpdated` condition node that triggers replan.

## Safe Stop Behavior

When collision is imminent and replanning takes too long, robot must stop safely.

### Stopping Distance Calculation

**Physics**: Stopping distance depends on velocity and deceleration.

```
d = v² / (2 × a)

Where:
  d = stopping distance (meters)
  v = current velocity (m/s)
  a = deceleration (m/s²)
```

**For humanoid at 0.4 m/s with 0.5 m/s² deceleration**:
```
d = 0.4² / (2 × 0.5) = 0.16 / 1.0 = 0.16m
```

**Minimum safe detection distance**: 0.16m + safety margin = 0.3m

### Implementing Safe Stop

DWB controller automatically stops when all trajectories score too high (blocked). But we want explicit safe-stop behavior:

```python
#!/usr/bin/env python3
"""
Safe stop node for humanoid collision avoidance.
Monitors obstacle distance and triggers emergency deceleration.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class SafeStopNode(Node):
    def __init__(self):
        super().__init__('safe_stop_node')

        # Parameters
        self.declare_parameter('min_safe_distance', 0.4)  # meters
        self.declare_parameter('emergency_decel', 0.5)    # m/s²
        self.declare_parameter('max_velocity', 0.4)       # m/s

        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.emergency_decel = self.get_parameter('emergency_decel').value
        self.max_velocity = self.get_parameter('max_velocity').value

        # State
        self.current_velocity = 0.0
        self.obstacle_distance = float('inf')
        self.safe_stop_active = False

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.depth_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.depth_callback, 10
        )
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )

        # Publishers
        self.safe_cmd_pub = self.create_publisher(Twist, '/safe_cmd_vel', 10)
        self.stop_active_pub = self.create_publisher(Bool, '/safe_stop_active', 10)

        # Timer for velocity limiting
        self.create_timer(0.05, self.safety_loop)  # 20 Hz

        self.get_logger().info('Safe stop node initialized')
        self.get_logger().info(f'Min safe distance: {self.min_safe_distance}m')

    def scan_callback(self, msg):
        """Process LiDAR scan for obstacle distance."""
        # Find minimum distance in forward cone (±45°)
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Forward cone
        forward_mask = np.abs(angles) < 0.785  # ±45°
        forward_ranges = ranges[forward_mask]

        # Filter invalid readings
        valid_ranges = forward_ranges[(forward_ranges > msg.range_min) &
                                       (forward_ranges < msg.range_max)]

        if len(valid_ranges) > 0:
            self.obstacle_distance = min(self.obstacle_distance, np.min(valid_ranges))

    def depth_callback(self, msg):
        """Process depth camera for obstacle distance."""
        # Extract points in front of robot
        min_dist = float('inf')

        for point in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = point
            # Forward cone: x > 0, |y| < 0.5, z in valid height range
            if x > 0.1 and abs(y) < 0.5 and 0.1 < z < 2.0:
                dist = np.sqrt(x*x + y*y)
                min_dist = min(min_dist, dist)

        self.obstacle_distance = min(self.obstacle_distance, min_dist)

    def cmd_callback(self, msg):
        """Track current commanded velocity."""
        self.current_velocity = msg.linear.x

    def calculate_safe_velocity(self) -> float:
        """Calculate maximum safe velocity given obstacle distance."""
        if self.obstacle_distance >= 10.0:
            return self.max_velocity

        # Required stopping distance at current velocity
        # d = v² / (2a) → v = sqrt(2ad)
        available_distance = self.obstacle_distance - 0.2  # 0.2m buffer

        if available_distance <= 0:
            return 0.0

        safe_velocity = np.sqrt(2 * self.emergency_decel * available_distance)
        return min(safe_velocity, self.max_velocity)

    def safety_loop(self):
        """Main safety loop - limit velocity based on obstacle distance."""
        safe_vel = self.calculate_safe_velocity()

        # Determine if safe stop is active
        was_active = self.safe_stop_active
        self.safe_stop_active = safe_vel < self.current_velocity * 0.9

        if self.safe_stop_active and not was_active:
            self.get_logger().warn(
                f'Safe stop activated: obstacle at {self.obstacle_distance:.2f}m, '
                f'limiting velocity to {safe_vel:.2f} m/s'
            )

        if not self.safe_stop_active and was_active:
            self.get_logger().info('Safe stop deactivated')

        # Publish safe velocity command (velocity limiter)
        safe_cmd = Twist()
        safe_cmd.linear.x = min(self.current_velocity, safe_vel)
        self.safe_cmd_pub.publish(safe_cmd)

        # Publish status
        status = Bool()
        status.data = self.safe_stop_active
        self.stop_active_pub.publish(status)

        # Reset obstacle distance for next cycle
        self.obstacle_distance = float('inf')


def main():
    rclpy.init()
    node = SafeStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Integrating Safe Stop with Nav2

The safe stop node acts as velocity filter:

```
Controller → /cmd_vel → Safe Stop Node → /safe_cmd_vel → Robot
```

**Alternative**: Configure Nav2 controller with obstacle critic weights high enough that it naturally avoids close obstacles.

## Exercise 1: Observe Dynamic Obstacle Response

**Objective**: Understand baseline dynamic obstacle handling.

**Steps**:

1. Launch humanoid navigation in Isaac Sim
2. Start navigation to distant goal (10m away)
3. Spawn moving person in Isaac Sim (crossing robot's path)
4. Observe robot behavior:

| Observation | What Happened | Expected Behavior |
|-------------|---------------|-------------------|
| Person detected in costmap? | | Within 0.5s |
| Path replanned? | | Yes, around person |
| Robot stopped/slowed? | | If person too close |
| Collision occurred? | | No |

**Success criteria**:
- Person appears in local costmap within 0.5 seconds
- Robot adjusts path or stops
- No collision

## Spawning Dynamic Obstacles in Isaac Sim

Create Python script to spawn moving actors:

```python
#!/usr/bin/env python3
"""
Spawn moving obstacles (people) in Isaac Sim for Nav2 testing.
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

class DynamicObstacleSpawner:
    def __init__(self):
        self.world = World.instance()
        self.obstacles = []

    def spawn_walking_person(self, start_pos, end_pos, speed=1.0):
        """
        Spawn person that walks from start to end position.

        Args:
            start_pos: [x, y, z] start position
            end_pos: [x, y, z] end position
            speed: Walking speed in m/s
        """
        # Add person model (use simple cylinder or character mesh)
        person_prim_path = f"/World/DynamicObstacles/Person_{len(self.obstacles)}"

        # Simple cylinder representing person
        from omni.isaac.core.objects import DynamicCylinder
        person = DynamicCylinder(
            prim_path=person_prim_path,
            name=f"Person_{len(self.obstacles)}",
            position=np.array(start_pos),
            radius=0.3,    # Person radius
            height=1.7,    # Person height
            color=np.array([0.8, 0.4, 0.1])  # Orange
        )

        self.world.scene.add(person)
        self.obstacles.append({
            'prim': person,
            'start': np.array(start_pos),
            'end': np.array(end_pos),
            'speed': speed,
            'progress': 0.0
        })

        return person

    def update(self, dt):
        """Update obstacle positions (call each simulation step)."""
        for obs in self.obstacles:
            # Calculate direction
            direction = obs['end'] - obs['start']
            distance = np.linalg.norm(direction)
            if distance < 0.01:
                continue

            direction_normalized = direction / distance

            # Update progress
            obs['progress'] += obs['speed'] * dt

            if obs['progress'] >= distance:
                # Reached end, reverse direction
                obs['start'], obs['end'] = obs['end'].copy(), obs['start'].copy()
                obs['progress'] = 0.0

            # Calculate new position
            new_pos = obs['start'] + direction_normalized * obs['progress']
            obs['prim'].set_world_pose(position=new_pos)


# Usage in Isaac Sim script:
# spawner = DynamicObstacleSpawner()
# spawner.spawn_walking_person([3.0, 2.0, 0.0], [3.0, -2.0, 0.0], speed=0.8)
# In simulation loop: spawner.update(dt)
```

## Exercise 2: Tune Update Rates and Replanning

**Objective**: Configure optimal update frequencies for dynamic obstacle response.

## Try With AI: Three Roles Demonstration

### Role 1: AI as Teacher — Update Rate Impact

You need to understand how update rates affect response time. Ask AI:

```
"I'm tuning Nav2 for dynamic obstacle avoidance. My humanoid walks at 0.4 m/s. A person walking at 1.0 m/s crosses the path.

Explain:
1. How costmap update_frequency affects detection latency
2. How controller frequency affects response time
3. What's the relationship between sensor rate, costmap rate, and controller rate?
4. Calculate minimum safe detection distance given these rates"
```

**Expected learning**: AI explains timing pipeline, calculates latencies, derives minimum safe distance based on total pipeline delay.

### Role 2: AI as Student — Validating Safe Distance

You have requirements for humanoid stopping. Test AI understanding:

```
"My humanoid has these constraints:
- Max velocity: 0.4 m/s
- Safe deceleration: 0.5 m/s² (for balance)
- Depth camera range: 4m forward
- Costmap update: 10 Hz

Calculate:
1. Stopping distance at max velocity
2. Time available to react when person detected at 2m
3. Is 10 Hz costmap update sufficient?

I'll verify your calculations."
```

**Expected interaction**: AI calculates stopping physics, you verify against humanoid balance constraints.

### Role 3: AI as Co-Worker — Tuning Safety Margins

Collaborate to find optimal configuration:

```
"Let's tune dynamic obstacle parameters together. Current config:
- update_frequency: 5.0 Hz
- min_safe_distance: 0.4m
- obstacle_max_range: 4.0m

Problem: Robot sometimes gets too close to walking people before reacting.

Let's iterate:
1. You propose parameter changes
2. I'll test in simulation
3. We refine based on results"
```

**Expected collaboration**: AI proposes increased update frequency, reduced detection range for faster processing, or increased safe distance. You test and report results.

## Exercise 3: Validate Dynamic Avoidance

**Objective**: Confirm robot handles multiple dynamic obstacle scenarios.

**Test scenarios**:

### Scenario A: Person Walking Across Path

1. Robot navigating straight path
2. Person walks perpendicular, crossing robot's path
3. Expected: Robot stops or navigates around

### Scenario B: Person Walking Toward Robot

1. Robot navigating toward person walking toward robot
2. Closing speed: 0.4 + 1.0 = 1.4 m/s
3. Expected: Robot stops with adequate margin

### Scenario C: Multiple Dynamic Obstacles

1. Robot navigating through crowd (3+ people)
2. People walking various directions
3. Expected: Robot finds path through, or stops safely if blocked

### Scenario D: Sudden Appearance

1. Person steps out from behind corner (suddenly visible)
2. Person at 1.5m when detected
3. Expected: Safe stop without collision

**Results table**:

| Scenario | Collision? | Stop Distance | Replan? | Notes |
|----------|------------|---------------|---------|-------|
| A: Cross path | | | | |
| B: Toward robot | | | | |
| C: Multiple | | | | |
| D: Sudden | | | | |

**Success criteria**:
- No collisions in any scenario
- Stop distance > 0.3m from person
- Replanning within 2 seconds when path blocked

## Social Navigation Considerations

Humanoids navigate human spaces. Beyond collision avoidance, consider social behavior:

### Personal Space

People expect robots to respect personal space (0.5-1.2m typically):

```yaml
# Increased inflation for social navigation
inflation_layer:
  inflation_radius: 1.2          # Social distance
  cost_scaling_factor: 2.0       # Gradual cost decay
```

### Passing Distance

When passing people, maintain comfortable distance:

```yaml
# TEB planner: prefer wider margins
min_obstacle_dist: 0.8           # Min 0.8m from people
inflation_dist: 1.0              # Inflation for planning
```

### Movement Predictability

People expect predictable robot movement:

- Avoid sudden direction changes
- Maintain consistent velocity
- Signal intent (if robot has indicators)

```yaml
# DWB controller: penalize sudden changes
Oscillation:
  scale: 10.0                    # Higher penalty for oscillation
```

## Troubleshooting Dynamic Obstacle Issues

### Issue 1: Obstacles Not Appearing in Costmap

**Symptom**: Person visible in Isaac Sim but not in RViz costmap.

**Diagnosis**:
1. Check sensor topic publishing: `ros2 topic hz /camera/depth/points`
2. Check obstacle layer enabled in config
3. Check TF: sensor frame must transform to costmap frame

**Fix**:
- Verify Isaac Sim ROS bridge publishing depth data
- Enable `obstacle_layer` in costmap config
- Publish correct TF transforms

### Issue 2: Costmap Updates Too Slowly

**Symptom**: Obstacle appears in costmap 1-2 seconds after visible in simulation.

**Diagnosis**: `update_frequency` too low.

**Fix**: Increase `update_frequency` (try 10-20 Hz).

### Issue 3: Robot Doesn't Replan Around Obstacles

**Symptom**: Obstacle in costmap, but robot tries to go through it.

**Diagnosis**: Controller thinks it can navigate through, or replan rate too low.

**Fix**:
- Increase obstacle critic weight in DWB
- Decrease replan `RateController` period (higher frequency)
- Check inflation layer (obstacles might not be inflated enough)

### Issue 4: Robot Stops Too Far from Obstacles

**Symptom**: Robot stops 2m from person, very conservative.

**Diagnosis**: `min_safe_distance` or inflation too high.

**Fix**:
- Reduce `min_safe_distance` (if safe given stopping dynamics)
- Reduce `inflation_radius` (if collision-free operation validated)

### Issue 5: Safe Stop Triggers Continuously (Flapping)

**Symptom**: Safe stop activates/deactivates repeatedly.

**Diagnosis**: Obstacle distance near threshold, sensor noise.

**Fix**:
- Add hysteresis to safe stop logic
- Filter obstacle distance (moving average)
- Widen gap between activate/deactivate thresholds

## Summary: Dynamic Obstacle Handling for Humanoid

**Key configuration parameters**:

| Parameter | Recommended Value | Rationale |
|-----------|-------------------|-----------|
| Costmap update_frequency | 10 Hz | Fast enough for 1 m/s obstacles |
| Sensor max range | 4m | Balance range vs processing |
| Min safe distance | 0.4m | Stopping distance + margin |
| Inflation radius | 0.8-1.2m | Social distance |
| Replan rate | 0.5-1 Hz | Balance reactivity vs computation |

**Pipeline timing budget**:

```
Sensor → Costmap → Controller → Robot
  30ms     100ms      50ms      50ms
             Total: ~230ms latency
```

**Safe distance at 0.4 m/s, 230ms latency**: Robot travels 0.09m during latency. Add stopping distance 0.16m. Total: 0.25m minimum + safety margin = 0.4m.

**Next lesson**: You'll integrate Isaac Visual SLAM with Nav2, providing accurate localization for navigation and creating the nav2-humanoid-config skill.

## Checkpoint: Dynamic Obstacle Validation

Before proceeding to Lesson 7 (VSLAM integration), validate:

- [ ] **Costmap updates at 10 Hz**: Verify with `ros2 topic hz /local_costmap/costmap`
- [ ] **Obstacles appear within 0.5s**: Test with Isaac Sim spawned obstacle
- [ ] **Robot avoids walking person**: Cross-path scenario passes
- [ ] **Safe stop works**: Sudden appearance scenario—no collision
- [ ] **Social distance maintained**: Robot passes at ≥ 0.8m from person
- [ ] **No flapping**: Safe stop doesn't oscillate

**If all checks pass**, your dynamic obstacle handling is correctly configured.
