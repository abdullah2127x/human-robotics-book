---
title: "Capstone: Autonomous Humanoid Navigation"
chapter: 9
lesson: 9
learning_objectives:
  - Integrate Isaac Sim, VSLAM, and Nav2 for full autonomous navigation
  - Apply all 6 skills from Part 3 (Chapters 7-9) in unified project
  - Demonstrate specification-first development for complex integration
  - Validate real-time perception→planning→control pipeline
estimated_time: 240 minutes
skills:
  full-stack-integration:
    proficiency_level: C1
  project-orchestration:
    proficiency_level: C1
generated_by: content-implementer v1.1.0
source_spec: specs/book/part-3/chapter-09-plan.md
created: 2025-12-17
---

# Capstone: Autonomous Humanoid Navigation

This capstone integrates everything from Part 3—Isaac Sim photorealistic simulation (Chapter 7), Isaac ROS Visual SLAM (Chapter 8), and Nav2 path planning (Chapter 9)—into a complete autonomous navigation system. Your humanoid robot will navigate through an indoor environment, avoiding obstacles, recovering from failures, and reaching designated goals.

This is Layer 4 of our pedagogical framework: specification-first integration using skills you've created throughout Part 3.

## Part 3 Integration Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    AUTONOMOUS NAVIGATION SYSTEM                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │              Isaac Sim (Chapter 7)                       │    │
│  │  - Photorealistic indoor environment                     │    │
│  │  - Humanoid robot model (URDF)                          │    │
│  │  - RGB-D camera sensor simulation                       │    │
│  │  - Physics-based movement                               │    │
│  │  - ROS 2 Bridge (sensor data, commands)                 │    │
│  │                                                          │    │
│  │  Skills applied:                                         │    │
│  │  ✓ isaac-sim-domain-randomization (lighting variety)    │    │
│  │  ✓ isaac-sim-performance (real-time rendering)          │    │
│  └───────────────────┬─────────────────────────────────────┘    │
│                      │                                           │
│                      │ RGB-D frames, joint states                │
│                      ▼                                           │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │           Isaac ROS Visual SLAM (Chapter 8)              │    │
│  │  - GPU-accelerated visual odometry                       │    │
│  │  - Loop closure for drift correction                     │    │
│  │  - Map generation (occupancy grid)                       │    │
│  │  - TF: map → odom → base_link                           │    │
│  │                                                          │    │
│  │  Skills applied:                                         │    │
│  │  ✓ vslam-debugging (localization validation)            │    │
│  │  ✓ isaac-ros-performance (real-time VSLAM)              │    │
│  └───────────────────┬─────────────────────────────────────┘    │
│                      │                                           │
│                      │ Pose, Map, TF                             │
│                      ▼                                           │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │              Nav2 Navigation (Chapter 9)                 │    │
│  │  - Global costmap from VSLAM map                        │    │
│  │  - Local costmap from depth camera                      │    │
│  │  - TEB planner (smooth humanoid paths)                  │    │
│  │  - DWB controller (balance-safe velocities)             │    │
│  │  - Behavior tree (recovery escalation)                  │    │
│  │                                                          │    │
│  │  Skills applied:                                         │    │
│  │  ✓ nav2-humanoid-config (footprint, velocities)         │    │
│  │  ✓ behavior-tree-design (recovery logic)                │    │
│  └───────────────────┬─────────────────────────────────────┘    │
│                      │                                           │
│                      │ Velocity commands (cmd_vel)               │
│                      ▼                                           │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │              Humanoid Controller                         │    │
│  │  - Velocity → gait translation                          │    │
│  │  - Joint trajectory execution                           │    │
│  │  - Balance maintenance                                   │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Specification-First: Project Requirements

Before implementation, define project requirements clearly:

### Project Specification

```markdown
# Autonomous Humanoid Navigation - Project Specification

## Intent
Demonstrate fully autonomous humanoid navigation in indoor environment using
Isaac Sim for simulation, Isaac ROS Visual SLAM for localization, and Nav2
for path planning and control.

## Success Criteria

### Functional Requirements
- [ ] Humanoid navigates from Start position to Goal position (minimum 10m path)
- [ ] Navigation avoids all static obstacles (walls, furniture)
- [ ] Navigation avoids dynamic obstacles (simulated people)
- [ ] Robot reaches goal within position tolerance (0.25m) and orientation tolerance (0.25 rad)
- [ ] If path blocked, robot replans and finds alternative route
- [ ] If stuck, robot executes recovery behaviors and escapes

### Performance Requirements
- [ ] Real-time operation: Perception→Planning→Control pipeline ≥ 10 Hz
- [ ] Localization accuracy: Pose error < 0.1m during navigation
- [ ] Path smoothness: No sharp corners (> 45° in < 0.5m)
- [ ] Safe velocities: max_vel_x ≤ 0.4 m/s, max_vel_theta ≤ 0.5 rad/s

### Safety Requirements
- [ ] No collisions with static obstacles
- [ ] No collisions with dynamic obstacles (safe stop or avoidance)
- [ ] Recovery behaviors use humanoid-safe speeds
- [ ] Graceful abort if navigation impossible

## Constraints

### Hardware
- Isaac Sim running on GPU-equipped system (RTX 2070+ or equivalent)
- ROS 2 Humble on Ubuntu 22.04
- CUDA 12.x for Isaac ROS acceleration

### Software
- Isaac Sim 2023.1.1 or later
- Isaac ROS packages (isaac_ros_visual_slam, isaac_ros_common)
- Nav2 Humble packages

### Environment
- Indoor environment (office, home, or warehouse)
- Mix of static obstacles (furniture, walls)
- At least 1 dynamic obstacle (simulated person walking)
- Adequate lighting for VSLAM feature tracking

## Non-Goals
- Outdoor navigation (different sensor requirements)
- Manipulation tasks (focus on navigation)
- Multi-robot coordination
- Voice commands or HRI (user interface)
```

## Implementation Guide

### Phase 1: Isaac Sim Environment Setup

**Goal**: Humanoid robot in indoor environment with sensors publishing to ROS 2.

**Apply skills**:
- `isaac-sim-performance`: Ensure real-time rendering
- `isaac-sim-domain-randomization`: Vary lighting for robust VSLAM

**Setup steps**:

1. **Launch Isaac Sim** with humanoid scene from Chapter 7
2. **Verify ROS 2 bridge** publishes:
   - `/camera/rgb/image_raw` (RGB images)
   - `/camera/depth/image_raw` (Depth images)
   - `/camera/depth/points` (Point cloud)
   - `/joint_states` (Robot joint positions)
   - `/tf` (odom → base_link transform from sim)

3. **Add dynamic obstacles**:

```python
# In Isaac Sim Python script
from dynamic_obstacle_spawner import DynamicObstacleSpawner

spawner = DynamicObstacleSpawner()
# Add person walking across typical navigation path
spawner.spawn_walking_person(
    start_pos=[3.0, 2.0, 0.0],
    end_pos=[3.0, -2.0, 0.0],
    speed=0.8
)
```

4. **Validate sensor output**:

```bash
# Check camera topics
ros2 topic hz /camera/depth/points  # Should be ≥ 30 Hz

# Check TF
ros2 run tf2_ros tf2_echo odom base_link  # Should update at ≥ 30 Hz
```

### Phase 2: Isaac Visual SLAM Integration

**Goal**: VSLAM providing localization and map for Nav2.

**Apply skills**:
- `vslam-debugging`: Validate localization quality
- `isaac-ros-performance`: Ensure real-time VSLAM

**Setup steps**:

1. **Launch Isaac Visual SLAM**:

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py \
    enable_localization_n_mapping:=true \
    publish_map_to_odom_tf:=true \
    map_frame:=map \
    odom_frame:=odom \
    base_frame:=base_link
```

2. **Verify VSLAM output**:

```bash
# Check map is being built
ros2 topic echo /map --once  # Should show OccupancyGrid

# Check TF tree is complete
ros2 run tf2_ros tf2_echo map base_link  # Should show valid transform
```

3. **Apply vslam-debugging skill** if localization problems:
   - Check visual features tracked (RViz visualization)
   - Verify loop closures occurring
   - Monitor localization confidence

### Phase 3: Nav2 Configuration

**Goal**: Nav2 configured for humanoid with all skills applied.

**Apply skills**:
- `nav2-humanoid-config`: Complete Nav2 parameter configuration
- `behavior-tree-design`: Recovery behavior tree

**Setup steps**:

1. **Generate configuration using nav2-humanoid-config skill**:

Apply skill questions:
- Stance width: 0.35m
- Stance depth: 0.25m
- Step length: 0.30m
- Step frequency: 1.5 Hz
- Sensor: /camera/depth/points
- Localization: Isaac Visual SLAM

2. **Create master launch file** `autonomous_navigation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    humanoid_config_dir = get_package_share_directory('humanoid_nav2_config')

    # Parameters
    params_file = os.path.join(humanoid_config_dir, 'config', 'nav2_params.yaml')
    bt_file = os.path.join(humanoid_config_dir, 'behavior_trees', 'humanoid_navigate.xml')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Nav2 navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': params_file,
                'default_bt_xml_filename': bt_file,
                'autostart': 'true',
            }.items()
        ),

        # Safe stop node (from Chapter 9, Lesson 6)
        Node(
            package='humanoid_nav2_config',
            executable='safe_stop_node',
            name='safe_stop',
            output='screen',
            parameters=[{
                'min_safe_distance': 0.4,
                'emergency_decel': 0.5,
                'max_velocity': 0.4,
            }]
        ),

        # Tracking error monitor (from Chapter 9, Lesson 4)
        Node(
            package='humanoid_nav2_config',
            executable='tracking_error_monitor',
            name='tracking_monitor',
            output='screen',
        ),
    ])
```

3. **Create nav2_params.yaml** with all configurations:

```yaml
# Autonomous Humanoid Navigation Parameters
# Generated using nav2-humanoid-config skill
# Integrated with Isaac Sim + Isaac Visual SLAM

# ===== ROBOT PARAMETERS =====
# Footprint: 0.40m x 0.29m (with 15% safety margin)
# Max velocity: 0.45 m/s (step_length × frequency)
# Capped at: 0.40 m/s (conservative for stability)

amcl:
  ros__parameters:
    # DISABLED - Using Isaac Visual SLAM instead
    use_sim_time: true

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    # Use humanoid-specific behavior tree
    default_nav_to_pose_bt_xml: $(find humanoid_nav2_config)/behavior_trees/humanoid_navigate.xml

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.0
    min_theta_velocity_threshold: 0.001

    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.4              # Humanoid walking speed
      max_vel_y: 0.0              # NO LATERAL MOVEMENT
      max_vel_theta: 0.5          # Humanoid turning rate
      acc_lim_x: 0.3              # Smooth acceleration
      acc_lim_y: 0.0
      acc_lim_theta: 0.4
      decel_lim_x: -0.5           # Faster deceleration
      decel_lim_theta: -0.6
      vx_samples: 20
      vy_samples: 1               # Only sample zero lateral
      vtheta_samples: 20
      sim_time: 1.7
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathDist.scale: 32.0
      GoalAlign.scale: 24.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      Oscillation.scale: 1.0

planner_server:
  ros__parameters:
    use_sim_time: true
    planner_plugins: ["TEB"]

    TEB:
      plugin: "teb_local_planner/TebLocalPlannerROS"
      max_vel_x: 0.4
      max_vel_y: 0.0              # NO LATERAL
      max_vel_theta: 0.5
      acc_lim_x: 0.3
      acc_lim_theta: 0.4
      min_turning_radius: 0.5
      footprint_model.type: "polygon"
      footprint_model.vertices: [[0.145, 0.20], [0.145, -0.20], [-0.145, -0.20], [-0.145, 0.20]]
      weight_kinematics_nh: 1000.0
      weight_shortest_path: 0.0   # Smoothness over distance

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: map
      robot_base_frame: base_link
      update_frequency: 1.0
      publish_frequency: 1.0
      rolling_window: false
      width: 50
      height: 50
      resolution: 0.05
      footprint: "[[0.145, 0.20], [0.145, -0.20], [-0.145, -0.20], [-0.145, 0.20]]"
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: /map           # Isaac Visual SLAM map
        subscribe_to_updates: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 2.0
        cost_scaling_factor: 3.0

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: odom
      robot_base_frame: base_link
      update_frequency: 10.0      # Fast for dynamic obstacles
      publish_frequency: 5.0
      rolling_window: true
      width: 4
      height: 4
      resolution: 0.025
      footprint: "[[0.145, 0.20], [0.145, -0.20], [-0.145, -0.20], [-0.145, 0.20]]"
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: depth_camera
        depth_camera:
          topic: /camera/depth/points
          sensor_frame: camera_link
          data_type: "PointCloud2"
          marking: true
          clearing: true
          obstacle_max_range: 4.0
          obstacle_min_range: 0.2
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 1.5
        cost_scaling_factor: 5.0
```

### Phase 4: Integration Testing

**Goal**: Validate complete system meets all success criteria.

**Test procedure**:

```bash
# Terminal 1: Launch Isaac Sim
# (Isaac Sim GUI with humanoid scene)

# Terminal 2: Launch Isaac Visual SLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Terminal 3: Launch Nav2
ros2 launch humanoid_nav2_config autonomous_navigation.launch.py

# Terminal 4: Launch RViz
ros2 run rviz2 rviz2 -d ~/humanoid_ws/config/navigation.rviz

# Terminal 5: Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}}}}"
```

### Validation Script

Create comprehensive validation script:

```python
#!/usr/bin/env python3
"""
Capstone validation script for autonomous humanoid navigation.
Tests all success criteria from project specification.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
import tf2_ros
import time
import math

class CapstoneValidator(Node):
    def __init__(self):
        super().__init__('capstone_validator')

        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Metrics storage
        self.metrics = {
            'navigation_success': False,
            'position_error': float('inf'),
            'collisions': 0,
            'max_velocity': 0.0,
            'path_smoothness': True,
            'recovery_triggered': False,
            'real_time_achieved': True,
        }

        # Subscriptions for monitoring
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        # Timing
        self.start_time = None
        self.end_time = None

        self.get_logger().info('Capstone validator initialized')

    def cmd_callback(self, msg):
        """Track commanded velocities."""
        vel = math.sqrt(msg.linear.x**2 + msg.linear.y**2)
        self.metrics['max_velocity'] = max(self.metrics['max_velocity'], vel)

        # Check velocity limits
        if abs(msg.linear.x) > 0.45:
            self.get_logger().warn(f'Velocity exceeded: {msg.linear.x:.2f} m/s')
        if abs(msg.linear.y) > 0.01:
            self.get_logger().error(f'Lateral velocity detected: {msg.linear.y:.2f} m/s')

    def path_callback(self, msg):
        """Analyze path smoothness."""
        if len(msg.poses) < 3:
            return

        # Check for sharp corners
        for i in range(1, len(msg.poses) - 1):
            p0 = msg.poses[i-1].pose.position
            p1 = msg.poses[i].pose.position
            p2 = msg.poses[i+1].pose.position

            # Calculate angle change
            dx1, dy1 = p1.x - p0.x, p1.y - p0.y
            dx2, dy2 = p2.x - p1.x, p2.y - p1.y

            if (dx1**2 + dy1**2) < 0.01 or (dx2**2 + dy2**2) < 0.01:
                continue

            angle1 = math.atan2(dy1, dx1)
            angle2 = math.atan2(dy2, dx2)
            angle_change = abs(angle2 - angle1)
            if angle_change > math.pi:
                angle_change = 2*math.pi - angle_change

            segment_length = math.sqrt(dx1**2 + dy1**2)

            # Sharp corner: > 45° in < 0.5m
            if angle_change > 0.785 and segment_length < 0.5:
                self.metrics['path_smoothness'] = False
                self.get_logger().warn(f'Sharp corner detected: {math.degrees(angle_change):.1f}°')

    def map_callback(self, msg):
        """Verify map is being received."""
        self.get_logger().info(f'Map received: {msg.info.width}x{msg.info.height}')

    def get_robot_pose(self):
        """Get current robot pose from TF."""
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            return trans.transform.translation
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def run_navigation_test(self, goal_x, goal_y, goal_yaw=0.0):
        """Run single navigation test."""

        self.get_logger().info(f'Starting navigation to ({goal_x}, {goal_y})')
        self.start_time = time.time()

        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.orientation.z = math.sin(goal_yaw / 2)
        goal.pose.orientation.w = math.cos(goal_yaw / 2)

        # Wait for server
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation server not available')
            return False

        # Send goal
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=300.0)

        self.end_time = time.time()
        duration = self.end_time - self.start_time

        # Check result
        result = result_future.result()
        success = result.result.result == 0  # NavigationResult.SUCCEEDED

        # Calculate position error
        final_pose = self.get_robot_pose()
        if final_pose:
            dx = final_pose.x - goal_x
            dy = final_pose.y - goal_y
            self.metrics['position_error'] = math.sqrt(dx**2 + dy**2)

        self.metrics['navigation_success'] = success

        self.get_logger().info(f'Navigation {"SUCCEEDED" if success else "FAILED"}')
        self.get_logger().info(f'Duration: {duration:.1f}s')
        self.get_logger().info(f'Position error: {self.metrics["position_error"]:.3f}m')

        return success

    def run_full_validation(self):
        """Run complete validation suite."""

        print('\n' + '='*60)
        print('CAPSTONE VALIDATION: Autonomous Humanoid Navigation')
        print('='*60 + '\n')

        # Test 1: Basic navigation (10m path)
        print('Test 1: Basic Navigation (10m)')
        print('-' * 40)
        test1_pass = self.run_navigation_test(10.0, 0.0)

        # Test 2: Navigation with turn
        print('\nTest 2: Navigation with Turn')
        print('-' * 40)
        test2_pass = self.run_navigation_test(5.0, 5.0, math.pi/2)

        # Test 3: Dynamic obstacle scenario
        print('\nTest 3: Dynamic Obstacle Avoidance')
        print('-' * 40)
        print('(Ensure dynamic obstacle is spawned in Isaac Sim)')
        test3_pass = self.run_navigation_test(3.0, 0.0)

        # Print results
        print('\n' + '='*60)
        print('VALIDATION RESULTS')
        print('='*60)

        results = [
            ('Navigation Success', test1_pass and test2_pass and test3_pass),
            ('Position Error < 0.25m', self.metrics['position_error'] < 0.25),
            ('Max Velocity ≤ 0.4 m/s', self.metrics['max_velocity'] <= 0.45),
            ('Path Smoothness', self.metrics['path_smoothness']),
            ('No Collisions', self.metrics['collisions'] == 0),
        ]

        all_pass = True
        for name, passed in results:
            status = '✓ PASS' if passed else '✗ FAIL'
            print(f'{name}: {status}')
            all_pass = all_pass and passed

        print('-' * 60)
        print(f'OVERALL: {"ALL TESTS PASSED" if all_pass else "SOME TESTS FAILED"}')
        print('='*60 + '\n')

        return all_pass


def main():
    rclpy.init()
    validator = CapstoneValidator()

    try:
        validator.run_full_validation()
    except KeyboardInterrupt:
        pass

    validator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Skills Applied Summary

This capstone applies all 6 skills from Part 3:

| Skill | Chapter | Application |
|-------|---------|-------------|
| isaac-sim-domain-randomization | 7 | Lighting variation for robust VSLAM |
| isaac-sim-performance | 7 | Real-time rendering at ≥ 30 fps |
| vslam-debugging | 8 | Validate localization accuracy |
| isaac-ros-performance | 8 | VSLAM at ≥ 30 Hz |
| nav2-humanoid-config | 9 | Complete Nav2 configuration |
| behavior-tree-design | 9 | Humanoid-safe recovery BT |

## Debugging Guide

### Issue: Robot Doesn't Move

**Check**:
1. Nav2 lifecycle nodes active: `ros2 lifecycle list /controller_server`
2. Goal accepted: Check action server status
3. Path computed: Visualize `/plan` in RViz
4. Velocities published: `ros2 topic echo /cmd_vel`

### Issue: Robot Position Incorrect

**Check**:
1. VSLAM running: `ros2 node list | grep visual_slam`
2. TF valid: `ros2 run tf2_ros tf2_echo map base_link`
3. Map quality: Visualize `/map` in RViz

### Issue: Robot Collides

**Check**:
1. Costmap update rate: `ros2 topic hz /local_costmap/costmap`
2. Obstacle detection: Verify depth camera publishing
3. Inflation radius: May be too small

### Issue: Path Has Sharp Corners

**Check**:
1. Planner: Verify TEB is being used (not Navfn)
2. TEB parameters: `weight_kinematics_nh` should be high

## Deliverables Checklist

By completing this capstone, you should have:

- [ ] **Isaac Sim scene** with humanoid and indoor environment
- [ ] **Dynamic obstacles** spawned and moving
- [ ] **VSLAM integration** providing localization and map
- [ ] **Nav2 configuration** using all humanoid-specific settings
- [ ] **Behavior tree** with humanoid-safe recovery
- [ ] **Validation script** confirming all success criteria
- [ ] **Documentation** of your configuration and results

## Summary: Part 3 Complete

You've completed Part 3: The AI-Robot Brain.

**What you accomplished**:

1. **Chapter 7 (Isaac Sim)**: Photorealistic simulation, synthetic data, ROS 2 integration
2. **Chapter 8 (Isaac ROS VSLAM)**: GPU-accelerated visual odometry and mapping
3. **Chapter 9 (Nav2)**: Path planning, control, behavior trees for bipedal navigation

**Skills created**:
- isaac-sim-domain-randomization
- isaac-sim-performance
- vslam-debugging
- isaac-ros-performance
- nav2-humanoid-config
- behavior-tree-design

**Capstone achievement**: Full autonomous humanoid navigation integrating perception (VSLAM) → planning (Nav2) → control (DWB) → execution (Isaac Sim).

**Next**: Part 4 will explore AI applications—using the navigation foundation to build intelligent behaviors for humanoid robots in real-world scenarios.
