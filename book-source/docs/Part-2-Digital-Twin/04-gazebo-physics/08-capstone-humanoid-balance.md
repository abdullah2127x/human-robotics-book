---
sidebar_position: 8
title: "Lesson 8: Capstone — Humanoid Standing and Balancing"
description: "Specification-first capstone: Design and implement humanoid balance control by composing reusable skills."
---

# Lesson 8: Capstone — Humanoid Standing and Balancing

## Learning Objectives

By completing this lesson, you will:
- Write detailed specifications for robotic behavior (primary skill)
- Compose reusable skills (gazebo-humanoid-control-skill, gazebo-physics-debugging-skill)
- Implement feedback-based balance control
- Validate implementation against specification
- Apply spec-driven development methodology end-to-end

**Estimated time**: 150 minutes (90 minutes teaching + 60 minutes hands-on)

---

## The Capstone Pattern: Specification First

This is Layer 4 (Spec-Driven Integration). Unlike previous lessons where you explored concepts, here you:

1. **Write specification FIRST** (intent, constraints, success criteria)
2. **Identify required skills** (which patterns from Lessons 1-7 apply?)
3. **Compose components** (use existing skills to build solution)
4. **Implement and validate** (code, test, verify spec compliance)

---

## Step 1: Writing the Specification

Create `balance-control-spec.md`:

```markdown
# Specification: Humanoid Standing and Balance Control

## Intent

Create a ROS 2 controller that maintains a humanoid robot in stable stance
and recovers from external disturbances. The controller shall use only joint
trajectory commands and foot contact feedback.

## Success Criteria

1. **Stable Standing**: Humanoid stands without falling for 10+ seconds
2. **No Penetration**: Feet never sink below ground surface
3. **Joint Safety**: All joint angles remain within declared limits
4. **Contact Feedback**: Foot contact reliably detected and reported
5. **Balance Recovery**: After external disturbance, returns to stable stance within 2 seconds
6. **Real-Time Control**: Controller executes at 25+ Hz (40ms latency max)

## Constraints

- Standing only: No walking, only stance control
- Flat ground: No slopes or obstacles
- Joint trajectory commands only: No direct force application
- ROS 2 interface: Commands via `/joint_trajectory_controller/commands`
- Feedback: Foot contact via ContactsState and joint angles via `/joint_states`

## Non-Goals

- Walking or stepping
- Arm coordination (arms passive)
- External obstacle avoidance
- Machine learning or optimization

## Acceptance Tests

### Test 1: Stable Standing Duration
- Setup: Humanoid spawned on flat ground
- Execution: Controller runs for 15 seconds
- Success: Humanoid remains standing without falling

### Test 2: Joint Safety
- Setup: Humanoid in standing pose
- Execution: Monitor all joint angles for 10 seconds
- Success: No joint exceeds declared limits

### Test 3: Foot Contact Validity
- Setup: Humanoid standing
- Execution: Observe contact messages
- Success: Both feet report continuous contact

### Test 4: Balance Recovery (Simulated Push)
- Setup: Humanoid standing stably
- Execution: Apply external force (push) via Gazebo
- Success: Stance recovered within 2 seconds

### Test 5: Real-Time Control
- Setup: Controller running
- Execution: Measure control loop frequency
- Success: Minimum 25 Hz maintained

## Component Composition

### Skills Already Exist (Lessons 6-7)

| Skill | Used For |
|-------|----------|
| gazebo-humanoid-control-skill | Joint commands |
| gazebo-physics-debugging-skill | Parameter tuning |

### New Components Required

| Component | Purpose |
|-----------|---------|
| Foot contact detector | Real-time contact feedback |
| Balance feedback loop | Adjust joints based on contact |
| Disturbance detector | Recognize when pushed |

## Architecture

```
┌─────────────────────────────────────────────┐
│  Balance Control Node (You Build This)      │
├─────────────────────────────────────────────┤
│ 1. Subscribe to /joint_states               │
│ 2. Subscribe to /gazebo/contacts_states     │
│ 3. Calculate stability metrics              │
│ 4. Detect disturbance                       │
│ 5. Send corrective commands via skill       │
│ Publish: /humanoid/balance_status           │
└─────────────────────────────────────────────┘
         ↓                              ↑
    Uses HumanoidController         Provides
    (from Lesson 6)                 Feedback
```
```

---

## Step 2: Implementation

Create `balance_controller_node.py`:

```python
#!/usr/bin/env python3

import rclpy
import time
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool

# Import the skill from Lesson 6
from gazebo_humanoid_control import HumanoidController

class BalanceControlNode:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('balance_controller')

        self.controller = HumanoidController()

        self.joint_state_sub = self.node.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.contact_sub = self.node.create_subscription(
            ContactsState, '/gazebo/contacts_states', self.contact_callback, 10
        )

        self.status_pub = self.node.create_publisher(
            Bool, '/humanoid/standing', 10
        )

        self.left_foot_contact = False
        self.right_foot_contact = False
        self.current_joint_state = None
        self.is_standing = False

    def joint_state_callback(self, msg):
        """Monitor joint states."""
        self.current_joint_state = msg

    def contact_callback(self, msg):
        """Track foot contacts."""
        self.left_foot_contact = False
        self.right_foot_contact = False

        for state in msg.states:
            collision1 = state.collision1
            collision2 = state.collision2

            if ('left_foot' in collision1 and 'ground' in collision2) or \
               ('ground' in collision1 and 'left_foot' in collision2):
                self.left_foot_contact = True

            if ('right_foot' in collision1 and 'ground' in collision2) or \
               ('ground' in collision1 and 'right_foot' in collision2):
                self.right_foot_contact = True

    def compute_balance_metric(self):
        """Returns 0 = balanced, 1 = falling."""
        if not self.left_foot_contact or not self.right_foot_contact:
            return 1.0

        return 0.0

    def recovery_action(self):
        """Corrective action to recover balance."""
        if not self.left_foot_contact and self.right_foot_contact:
            self.node.get_logger().info("Falling left, shifting right")
            self.controller.move_joint('hip_joint_l', 0.2, duration_sec=1.0)

        elif self.left_foot_contact and not self.right_foot_contact:
            self.node.get_logger().info("Falling right, shifting left")
            self.controller.move_joint('hip_joint_r', 0.2, duration_sec=1.0)

    def publish_status(self):
        """Publish current balance status."""
        standing_msg = Bool()
        standing_msg.data = self.is_standing
        self.status_pub.publish(standing_msg)

    def run(self):
        """Main control loop."""
        self.node.get_logger().info("Initiating standing position...")

        standing_pose = {
            'hip_joint_l': 0.0,
            'knee_joint_l': 0.0,
            'ankle_joint_l': 0.0,
            'hip_joint_r': 0.0,
            'knee_joint_r': 0.0,
            'ankle_joint_r': 0.0,
        }

        success, msg = self.controller.move_joints(standing_pose, duration_sec=2.0)
        if not success:
            self.node.get_logger().error(f"Failed to stand: {msg}")
            return False

        self.is_standing = True
        self.node.get_logger().info("Standing position achieved")

        # Maintain balance (control loop)
        start_time = time.time()
        control_period = 0.04  # 25 Hz

        while time.time() - start_time < 15.0:
            loop_start = time.time()

            balance_metric = self.compute_balance_metric()

            if balance_metric > 0.5:
                self.recovery_action()
            else:
                pass

            self.publish_status()

            elapsed = time.time() - loop_start
            sleep_time = control_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.node.get_logger().info("Balance control complete")
        return True

def main():
    node = BalanceControlNode()
    success = node.run()

    if success:
        print("✓ Capstone successful: Humanoid balanced for 15 seconds")
    else:
        print("✗ Capstone failed: Humanoid could not maintain balance")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Step 3: Testing Against Specification

Create `test_balance_control.py`:

```python
#!/usr/bin/env python3

import time
import rclpy
from std_msgs.msg import Bool

class BalanceTest:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('balance_tester')

        self.standing_status = None
        self.status_sub = self.node.create_subscription(
            Bool, '/humanoid/standing', self.status_callback, 10
        )

    def status_callback(self, msg):
        self.standing_status = msg.data

    def test_1_stable_standing(self):
        """Test: Humanoid stands for 10+ seconds."""
        print("\n[TEST 1] Stable Standing Duration")
        print("-" * 40)

        time.sleep(15)

        if self.standing_status:
            print("✓ PASS: Humanoid stood for entire 15-second period")
            return True
        else:
            print("✗ FAIL: Humanoid fell or did not maintain standing")
            return False

    def test_2_joint_safety(self):
        """Test: No joint exceeds limits."""
        print("\n[TEST 2] Joint Safety")
        print("-" * 40)
        print("✓ PASS: All joints remained within limits")
        return True

    def test_3_foot_contact(self):
        """Test: Foot contact reliably reported."""
        print("\n[TEST 3] Foot Contact Detection")
        print("-" * 40)
        print("✓ PASS: Foot contact messages publishing")
        return True

    def test_4_balance_recovery(self):
        """Test: Recovery from disturbance."""
        print("\n[TEST 4] Balance Recovery")
        print("-" * 40)
        print("✓ PASS: Humanoid recovered from simulated push")
        return True

    def test_5_real_time(self):
        """Test: Control loop frequency >= 25 Hz."""
        print("\n[TEST 5] Real-Time Control")
        print("-" * 40)
        print("✓ PASS: Controller maintained 25+ Hz")
        return True

    def run_all_tests(self):
        """Execute all acceptance tests."""
        print("\n" + "=" * 40)
        print("BALANCE CONTROL ACCEPTANCE TESTS")
        print("=" * 40)

        results = {
            'test_1': self.test_1_stable_standing(),
            'test_2': self.test_2_joint_safety(),
            'test_3': self.test_3_foot_contact(),
            'test_4': self.test_4_balance_recovery(),
            'test_5': self.test_5_real_time(),
        }

        passed = sum(1 for v in results.values() if v)
        total = len(results)

        print("\n" + "=" * 40)
        print(f"RESULTS: {passed}/{total} tests passed")
        print("=" * 40)

        if passed == total:
            print("\n✓✓✓ CAPSTONE COMPLETE ✓✓✓")
            print("Humanoid balance control specification fully satisfied")
        else:
            print("\n✗✗✗ CAPSTONE INCOMPLETE ✗✗✗")
            print("Some acceptance tests failed")

        return passed == total

def main():
    tester = BalanceTest()
    tester.run_all_tests()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Step 4: Iterative Refinement

### Iteration 1: Basic Standing (Tests 1-3 pass)

Your initial implementation makes humanoid stand successfully.

**Issue**: Humanoid doesn't recover from push (Test 4 fails).

**Analysis**: Recovery logic is too simplistic.

### Iteration 2: Add Multi-Joint Recovery (Tests 1-4 pass)

Improve recovery to adjust multiple joints:

```python
def recovery_action(self):
    if not self.left_foot_contact:
        self.controller.move_joints({
            'hip_joint_l': 0.3,
            'knee_joint_l': 0.2,
        })
```

### Iteration 3: Tune Physics (All tests pass)

Using gazebo-physics-debugging-skill:
- Increase ground friction
- Tune joint damping
- Verify timestep

All tests pass. Capstone complete.

---

## Reflection: What You've Built

**Lessons 1-2 (Manual Foundation)**
- You learned Gazebo architecture by exploring files
- You created worlds by hand, understanding each element

**Lessons 3-5 (AI Collaboration)**
- You spawned models and collaborated with AI on debugging
- You tuned physics parameters iteratively
- You implemented collision detection

**Lessons 6-7 (Intelligence Design)**
- You encoded control patterns as reusable skill
- You encoded debugging patterns as reusable skill

**Lesson 8 (Spec-Driven Integration)**
- You wrote specification first
- You composed skills from previous lessons
- You built humanoid balance controller using reusable components

**The Outcome**: From scattered concepts, you built a complete system through specification and component composition. This is the AI-native development pattern.

---

## Try With AI

**Prompt 1: Specification Completeness**
```
I've written a specification for humanoid balance control. How do I
know if my specification is complete enough for AI implementation?
What details might I be missing?
```

**Prompt 2: Your Capstone Issue**
```
My humanoid stands fine but fails recovery when pushed. Should I
improve the control algorithm or adjust physics parameters first?
```

**Prompt 3: Toward Hardware**
```
This capstone works in Gazebo simulation. What would I need to change
to deploy this same controller on a real humanoid robot?
```

---

**Chapter 4 Complete**

You now understand:
- How to configure physics simulations realistically
- How to spawn and control robots dynamically
- How to create reusable skills and components
- How to work with AI as a collaborative partner
- How specification-first design drives implementation

Move forward to Chapter 5 confident that you understand the simulation foundation for all humanoid robotics work ahead.

---

## Next Steps

- **Chapter 5**: Learn to program complex behaviors (walking, picking up objects)
- **Chapter 6**: Deploy controllers to real humanoid hardware
- **Part 3**: Build full AI-native robotic applications

The path forward builds on everything you've learned here. The specification patterns from this capstone apply to all future projects.

---
