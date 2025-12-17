---
sidebar_position: 6
title: "Lesson 6: Joint Control and Humanoid Movement"
description: "Create reusable skill for commanding humanoid joints via trajectory controllers."
---

# Lesson 6: Joint Control and Humanoid Movement

## Learning Objectives

By completing this lesson, you will:
- Understand ROS 2 trajectory controller interfaces
- Control humanoid joints via joint trajectory commands
- Create reusable patterns in a skill
- Apply Persona + Questions + Principles to skill design
- Build a gazebo-humanoid-control-skill for future use

**Estimated time**: 90 minutes

---

## Why This Lesson Is Different

Lessons 1-5 taught you foundational concepts through manual practice and AI collaboration. You learned individual concepts: physics, spawning, tuning, sensing.

Now you transition to Layer 3 (Intelligence Design). Instead of just using these concepts, you'll encode *patterns from them as reusable skills*.

A skill is documented, tested, and packaged knowledge. Once created, any future project can reuse it.

---

## Understanding Joint Trajectory Controllers

ROS 2 joint trajectory controller is a standard interface for commanding robots.

### Message Definition

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Create trajectory with multiple waypoints
trajectory = JointTrajectory()
trajectory.joint_names = ['hip_joint_l', 'knee_joint_l', 'ankle_joint_l']

# Waypoint 1: Standing position
point1 = JointTrajectoryPoint()
point1.positions = [0.0, 0.0, 0.0]
point1.velocities = [0.0, 0.0, 0.0]
point1.time_from_start.sec = 1

# Waypoint 2: Knee bent
point2 = JointTrajectoryPoint()
point2.positions = [0.0, -0.3, 0.1]
point2.velocities = [0.0, 0.0, 0.0]
point2.time_from_start.sec = 2

trajectory.points = [point1, point2]
```

### Commanding Joints

```python
import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class JointCommanderBasic:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('joint_commander')

        self.trajectory_pub = self.node.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/commands',
            10
        )

        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.current_joint_state = None

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def send_trajectory(self, joint_names, target_positions, duration_sec=1.0):
        """Send trajectory command to specified joints."""
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(joint_names)
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)

        trajectory.points = [point]
        self.trajectory_pub.publish(trajectory)

    def get_current_position(self, joint_name):
        """Get current angle of a joint."""
        if self.current_joint_state is None:
            return None

        try:
            idx = self.current_joint_state.name.index(joint_name)
            return self.current_joint_state.position[idx]
        except (ValueError, IndexError):
            return None

# Usage
commander = JointCommanderBasic()

# Send leg to standing position
commander.send_trajectory(
    joint_names=['hip_joint_l', 'knee_joint_l', 'ankle_joint_l'],
    target_positions=[0.0, 0.0, 0.0],
    duration_sec=2.0
)

import time
time.sleep(2.5)

print(f"Knee position: {commander.get_current_position('knee_joint_l')}")
```

---

## Designing the Skill: Persona + Questions + Principles

Now you'll design a reusable skill by thinking through three aspects:

### Persona: Think Like a Humanoid Roboticist

> "Think like a roboticist standardizing humanoid control interfaces across multiple projects and hardware platforms. Your skill must work for any humanoid (different joint counts, link names, DOF). But it must also handle the specific challenges humanoid control faces: joint limits, safety constraints, coordinated multi-joint movement."

### Questions: Analysis Framework

1. **What control patterns recur?**
   - Single joint setpoint (move one joint to angle)
   - Multi-joint sequences (choreographed movement)
   - Trajectory interpolation (smooth paths between poses)

2. **What constraints exist?**
   - Joint position limits (can't bend knee beyond physical limits)
   - Joint velocity limits (actuators have max speed)
   - Joint acceleration limits (smooth motion vs jerky)

3. **What safety mechanisms matter?**
   - Timeout detection (command didn't execute)
   - Position validation (final position matches command)
   - Emergency stop (halt all motion)
   - Graceful degradation (continue if one joint fails)

4. **What feedback validates success?**
   - Commanded vs actual joint angle
   - Execution time vs expected
   - Torque/force at joints
   - Contact feedback (optional)

### Principles: Decision Frameworks

1. **Separation of Concerns**
   - Joint group control separate from individual joint control
   - Trajectory planning separate from execution
   - Feedback monitoring separate from command generation

2. **Graceful Degradation**
   - Robot continues functioning if some joints fail
   - Non-critical joints can skip execution
   - Core joints (torso, base) take priority

3. **Feedback Validation**
   - Always verify goal achievement before next command
   - Don't assume command succeeded—measure actual position
   - Use tolerances (±0.05 rad) for realistic comparison

4. **Safety by Default**
   - Respect hard joint limits (enforced)
   - Limit command velocity/acceleration (smooth motion)
   - Require explicit overrides for risky moves

---

## The Skill in Practice

Create `.claude/skills/gazebo-humanoid-control/SKILL.md` with complete documentation including:

- **Overview**: What this skill does and why it matters
- **Joint coordinate frames**: How joint angles are interpreted
- **Control patterns**: Single joint, multi-joint, trajectory patterns
- **Safety constraints**: Joint limits, velocity limits
- **Feedback processing**: Reading actual vs commanded states
- **Timeout handling**: What happens if command doesn't complete
- **Error recovery**: How to handle partial failures
- **Common usage scenarios**: Standing, kicking, reaching
- **Error handling**: Common failures and solutions
- **Testing**: Unit and integration tests
- **Implementation tips**: Best practices

---

## Implementing the Skill in Code

Create `gazebo_humanoid_control.py`:

```python
#!/usr/bin/env python3

import rclpy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class HumanoidController:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('humanoid_controller')

        self.trajectory_pub = self.node.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/commands',
            10
        )

        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.current_joint_state = None

        self.joint_limits = {
            'hip_joint_l': (-1.57, 1.57),
            'knee_joint_l': (0.0, 2.618),
            'ankle_joint_l': (-0.785, 0.785),
            'hip_joint_r': (-1.57, 1.57),
            'knee_joint_r': (0.0, 2.618),
            'ankle_joint_r': (-0.785, 0.785),
        }

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def get_current_position(self, joint_name):
        """Get current joint angle."""
        if self.current_joint_state is None:
            return None
        try:
            idx = self.current_joint_state.name.index(joint_name)
            return self.current_joint_state.position[idx]
        except (ValueError, IndexError):
            return None

    def send_trajectory(self, joint_names, target_positions, duration_sec=2.0):
        """Send trajectory command."""
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(joint_names)
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)

        trajectory.points = [point]
        self.trajectory_pub.publish(trajectory)

    def move_joint(self, joint_name, target_angle, duration_sec=2.0):
        """Move single joint to target position."""
        if joint_name not in self.joint_limits:
            return False, f"Unknown joint: {joint_name}"

        min_angle, max_angle = self.joint_limits[joint_name]
        if not (min_angle <= target_angle <= max_angle):
            return False, f"Target {target_angle:.3f} outside limits"

        self.send_trajectory([joint_name], [target_angle], duration_sec)
        time.sleep(duration_sec + 0.5)

        actual = self.get_current_position(joint_name)
        if actual is None:
            return False, "Could not read joint state"

        tolerance = 0.05
        if abs(actual - target_angle) > tolerance:
            return False, f"Position mismatch: wanted {target_angle:.3f}, got {actual:.3f}"

        return True, f"Moved {joint_name} to {target_angle:.3f}"

    def move_joints(self, joint_dict, duration_sec=2.0):
        """Move multiple joints."""
        joint_names = list(joint_dict.keys())
        target_positions = list(joint_dict.values())

        for name, angle in joint_dict.items():
            if name not in self.joint_limits:
                return False, f"Unknown joint: {name}"
            min_angle, max_angle = self.joint_limits[name]
            if not (min_angle <= angle <= max_angle):
                return False, f"Joint {name} target out of range"

        self.send_trajectory(joint_names, target_positions, duration_sec)
        time.sleep(duration_sec + 0.5)

        for name, target in joint_dict.items():
            actual = self.get_current_position(name)
            if actual is None or abs(actual - target) > 0.05:
                return False, f"Failed to reach {name}"

        return True, "All joints reached targets"

# Example usage
if __name__ == '__main__':
    controller = HumanoidController()

    success, msg = controller.move_joints({
        'hip_joint_l': 0.0,
        'knee_joint_l': 0.0,
        'ankle_joint_l': 0.0,
        'hip_joint_r': 0.0,
        'knee_joint_r': 0.0,
        'ankle_joint_r': 0.0,
    }, duration_sec=2.0)

    print(f"Stand: {msg}")
```

---

## Try With AI

**Prompt 1: Skill Extensibility**
```
I've created a humanoid control skill for joint trajectory commands.
What new capabilities should I add to make it more reusable?
Walking? Gestures? Safety features?
```

**Prompt 2: Multi-Robot Applications**
```
Could this skill work for different humanoid robots (different joint
counts, different naming conventions)? How would I make it truly generic?
```

**Prompt 3: Error Handling**
```
What are the most critical error cases in humanoid joint control?
My current skill handles timeouts and limits. What else matters?
```

---

**Next: Proceed to Lesson 7: Debugging and Optimization**

In Lesson 7, you'll create another Layer 3 skill: a debugging and optimization guide for physics simulation problems.
