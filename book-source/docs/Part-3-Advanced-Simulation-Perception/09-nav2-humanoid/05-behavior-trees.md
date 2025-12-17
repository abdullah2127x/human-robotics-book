---
title: Behavior Tree Design for Navigation Logic
chapter: 9
lesson: 5
learning_objectives:
  - Understand behavior tree fundamentals (sequences, fallbacks, decorators)
  - Design navigation behavior tree with recovery escalation
  - Implement stuck detection and progressive recovery behaviors
  - Apply specification-first workflow to behavior tree development
estimated_time: 180 minutes
skills:
  behavior-tree-design:
    proficiency_level: B2
generated_by: content-implementer v1.1.0
source_spec: specs/book/part-3/chapter-09-plan.md
created: 2025-12-17
---

# Behavior Tree Design for Navigation Logic

Path planners determine *where* to go. Controllers determine *how* to move. But what happens when things go wrong? The robot gets stuck. An obstacle blocks the path. The controller fails to make progress. Behavior trees orchestrate the *what to do next*—the high-level decision logic that makes autonomous navigation robust.

This lesson teaches behavior tree fundamentals and applies them to humanoid navigation. You'll design a behavior tree that detects stuck conditions, applies progressive recovery strategies, and escalates gracefully when recovery fails. The specification-first workflow ensures you define *requirements* before writing XML.

## Why Behavior Trees for Navigation

Consider these navigation failure scenarios:

**Scenario 1: Temporary obstruction**
- Person walks in front of robot, blocking path
- Wait a few seconds → person moves → continue

**Scenario 2: Path blocked**
- Furniture moved, original path now impossible
- Request new path from planner → continue on new route

**Scenario 3: Robot stuck in corner**
- Controller commands result in no motion
- Back up slightly → rotate → try again

**Scenario 4: Unrecoverable situation**
- Robot in dead-end with no exit
- Abort mission → notify operator

A simple "plan → execute" approach fails at Scenario 1 (no retry logic). Hardcoded recovery fails at Scenario 4 (no escalation). Behavior trees provide the flexible, hierarchical decision structure to handle all scenarios.

## Behavior Tree Fundamentals

### Core Concepts

A behavior tree is a directed tree where:
- **Nodes** represent actions or decisions
- **Execution** starts at root, traverses tree
- **Tick** means "evaluate this node"
- **Status** is returned: SUCCESS, FAILURE, or RUNNING

### Node Types

**Action nodes** (leaves): Do something
```
[NavigateToPose] → Sends goal to navigation stack
[Wait] → Waits for specified duration
[BackUp] → Moves robot backward
```

**Condition nodes** (leaves): Check something
```
[IsGoalReached?] → Returns SUCCESS if at goal
[IsBatteryOK?] → Returns SUCCESS if battery > threshold
[IsPathValid?] → Returns SUCCESS if path exists
```

**Control nodes** (internal): Orchestrate children

| Node Type | Symbol | Behavior |
|-----------|--------|----------|
| **Sequence** | → | Run children left-to-right until FAILURE |
| **Fallback** | ? | Run children left-to-right until SUCCESS |
| **Parallel** | ⇉ | Run all children simultaneously |

**Decorator nodes**: Modify single child
```
[Retry(3)] → Retry child up to 3 times
[Timeout(10s)] → Fail child after 10 seconds
[Inverter] → Invert child result (SUCCESS↔FAILURE)
```

### Sequence Node (→)

Runs children in order. Returns FAILURE on first child failure.

```
Sequence
├── CheckBattery
├── ComputePath
└── FollowPath
```

**Behavior**:
1. Tick CheckBattery → if FAILURE, return FAILURE (stop)
2. Tick ComputePath → if FAILURE, return FAILURE (stop)
3. Tick FollowPath → return its status

**Use case**: Sequential steps that all must succeed.

### Fallback Node (?)

Runs children in order. Returns SUCCESS on first child success.

```
Fallback
├── FollowPath
├── ReplanPath
└── AbortMission
```

**Behavior**:
1. Tick FollowPath → if SUCCESS, return SUCCESS (done)
2. Tick ReplanPath → if SUCCESS, return SUCCESS (done)
3. Tick AbortMission → return its status (last resort)

**Use case**: Try alternatives until one works.

### Combining Sequence and Fallback

Real behavior trees combine both:

```
Fallback (try until success)
├── Sequence (normal navigation)
│   ├── ComputePath
│   └── FollowPath
└── Sequence (recovery)
    ├── ClearCostmap
    ├── ComputePath
    └── FollowPath
```

**Behavior**:
1. Try normal navigation
2. If fails, try recovery sequence
3. If recovery fails, fallback fails

## Nav2 Behavior Tree Structure

Nav2 uses BehaviorTree.CPP library. Navigation behavior tree typically follows this pattern:

```
NavigateWithRecovery
├── Sequence (main navigation)
│   ├── RateController (limit replanning rate)
│   │   └── ComputePathToPose
│   └── FollowPath
│
└── RecoveryFallback (if main fails)
    ├── ClearEntireCostmap
    ├── Spin
    ├── Wait
    └── BackUp
```

### Default Nav2 BT

Nav2 provides default behavior tree (`navigate_w_replanning_and_recovery.xml`):

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathRecovery">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPathRecovery">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <ClearEntireCostmap name="ClearingCostmaps" service_name="local_costmap/clear_entirely_costmap"/>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.3" backup_speed="0.025"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

### Key Components

**RecoveryNode**: Wraps navigation in retry logic
- `number_of_retries`: How many recovery attempts before failing

**PipelineSequence**: Runs children, re-ticks running child
- Used for continuous replanning during navigation

**RateController**: Limits child tick rate
- Prevents excessive replanning (1 Hz = replan once per second)

**RoundRobin**: Cycles through children on successive ticks
- First recovery: ClearCostmap
- Second recovery: Spin
- Third recovery: Wait
- Fourth recovery: BackUp
- Then cycles back

## Specification-First Behavior Tree Design

Before writing XML, specify requirements. This is crucial for humanoid navigation where recovery behaviors must respect balance constraints.

### Requirements Specification Template

```markdown
## Behavior Tree Requirements

### Intent
Define navigation behavior with recovery for bipedal humanoid.

### Success Conditions
1. Robot reaches goal pose within tolerance
2. Robot orientation matches goal orientation
3. No collisions during navigation

### Failure Conditions
1. Goal unreachable after all recovery attempts
2. Robot stuck with no valid recovery options
3. Emergency stop triggered

### Recovery Behaviors (Ordered by Escalation)

| Level | Behavior | Trigger | Humanoid Constraints |
|-------|----------|---------|---------------------|
| 1 | Wait | Temporary obstruction | Safe (standing) |
| 2 | Clear costmap | Stale obstacle data | Safe (standing) |
| 3 | Replan | Path blocked | Safe (standing) |
| 4 | Rotate in place | Stuck, need new direction | Slow rotation (balance) |
| 5 | Back up | Stuck in dead-end | Very slow (balance critical) |
| 6 | Abort | All recovery failed | Notify operator |

### Stuck Detection
- Trigger: Robot makes < 0.1m progress in 10 seconds
- Method: Track position, compare to 10s ago

### Timeout Configuration
- Per-recovery timeout: 15 seconds
- Total navigation timeout: 300 seconds (5 minutes)

### Constraints
- No fast rotation (humanoid balance)
- No fast backward movement (humanoid balance)
- Spin recovery: max 0.3 rad/s angular velocity
- BackUp recovery: max 0.05 m/s linear velocity
```

### Exercise 1: Write BT Requirements Specification

**Objective**: Define requirements BEFORE writing XML.

Create `bt_requirements.md` with:

1. **Intent**: What should the behavior tree accomplish?
2. **Success conditions**: When is navigation complete?
3. **Failure conditions**: When should navigation abort?
4. **Recovery behaviors**: What recovery actions, in what order?
5. **Stuck detection**: How to detect robot is stuck?
6. **Humanoid constraints**: What movements are unsafe?

**Success criteria**:
- [ ] Intent is clear and testable
- [ ] Recovery behaviors ordered by escalation
- [ ] Humanoid balance constraints documented
- [ ] Stuck detection threshold specified

## Designing Humanoid-Appropriate Recovery

Humanoid robots require modified recovery behaviors:

### Wait (Safe)

Standard wait is safe for humanoids—robot stands still.

```xml
<Wait wait_duration="5"/>
```

**Humanoid considerations**: None—standing is stable.

### Clear Costmap (Safe)

Clearing stale costmap data is safe—no movement.

```xml
<ClearEntireCostmap service_name="local_costmap/clear_entirely_costmap"/>
```

**Humanoid considerations**: None—computational only.

### Spin (Requires Modification)

Default spin rotates too fast for humanoid balance:

```xml
<!-- DEFAULT (unsafe for humanoid) -->
<Spin spin_dist="1.57" spin_speed="1.0"/>

<!-- HUMANOID-SAFE -->
<Spin spin_dist="0.78" spin_speed="0.3"/>
```

**Changes**:
- `spin_dist`: Reduced from 90° (1.57 rad) to 45° (0.78 rad)
- `spin_speed`: Reduced from 1.0 to 0.3 rad/s

**Rationale**: Humanoids can't rotate quickly without losing balance. Small, slow rotations are safer.

### BackUp (Requires Modification)

Default backup speed is too fast for humanoid:

```xml
<!-- DEFAULT (unsafe for humanoid) -->
<BackUp backup_dist="0.3" backup_speed="0.15"/>

<!-- HUMANOID-SAFE -->
<BackUp backup_dist="0.15" backup_speed="0.05"/>
```

**Changes**:
- `backup_dist`: Reduced from 0.3m to 0.15m
- `backup_speed`: Reduced from 0.15 to 0.05 m/s

**Rationale**: Walking backward is unstable for humanoids. Minimal, very slow movement reduces fall risk.

### Recovery Escalation Order (Humanoid)

Safest-first ordering:

1. **Wait** (5s): Maybe obstacle will move
2. **Wait** (10s): Give more time
3. **Clear costmap**: Remove stale data
4. **Replan**: Find alternative path
5. **Slow spin**: Change direction (small angle)
6. **Slow backup**: Move away from obstacle
7. **Abort**: All recovery failed

## Creating Humanoid Navigation Behavior Tree

### Complete XML Implementation

Create `humanoid_navigate.xml`:

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="HumanoidNavigation">
  <!--
    Humanoid Navigation Behavior Tree
    Designed for bipedal robots with balance constraints
    Recovery behaviors are slower and more conservative than default
  -->

  <BehaviorTree ID="HumanoidNavigation">
    <!-- Outer recovery wrapper: 8 retry attempts with humanoid-safe recoveries -->
    <RecoveryNode number_of_retries="8" name="NavigateRecovery">

      <!-- Main navigation pipeline -->
      <PipelineSequence name="NavigateWithReplanning">

        <!-- Path planning with rate limiting (replan at 0.5 Hz) -->
        <RateController hz="0.5">
          <RecoveryNode number_of_retries="2" name="ComputePathRecovery">
            <!-- Compute path using TEB planner (smooth paths for humanoid) -->
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="TEB"/>
            <!-- If planning fails, clear global costmap and retry -->
            <Sequence name="PlanningRecovery">
              <ClearEntireCostmap name="ClearGlobalForPlanning"
                                  service_name="global_costmap/clear_entirely_costmap"/>
              <Wait wait_duration="1"/>
            </Sequence>
          </RecoveryNode>
        </RateController>

        <!-- Path following with recovery -->
        <RecoveryNode number_of_retries="2" name="FollowPathRecovery">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <!-- If following fails, clear local costmap -->
          <ClearEntireCostmap name="ClearLocalForFollowing"
                              service_name="local_costmap/clear_entirely_costmap"/>
        </RecoveryNode>

      </PipelineSequence>

      <!-- Recovery fallback: executed when main pipeline fails -->
      <ReactiveFallback name="RecoveryFallback">
        <!-- Check if goal was updated (user sent new goal) -->
        <GoalUpdated/>

        <!-- Humanoid-safe recovery sequence -->
        <RoundRobin name="HumanoidRecoveryActions">

          <!-- Level 1: Wait (safest, no movement) -->
          <Sequence name="WaitRecovery">
            <Wait wait_duration="5" name="WaitForObstacle"/>
          </Sequence>

          <!-- Level 2: Clear costmaps (no movement) -->
          <Sequence name="ClearCostmapsRecovery">
            <ClearEntireCostmap name="ClearLocal"
                                service_name="local_costmap/clear_entirely_costmap"/>
            <ClearEntireCostmap name="ClearGlobal"
                                service_name="global_costmap/clear_entirely_costmap"/>
            <Wait wait_duration="2"/>
          </Sequence>

          <!-- Level 3: Longer wait (maybe dynamic obstacle) -->
          <Sequence name="ExtendedWaitRecovery">
            <Wait wait_duration="10" name="ExtendedWait"/>
          </Sequence>

          <!-- Level 4: Slow spin (45 degrees, very slow for humanoid balance) -->
          <Sequence name="SlowSpinRecovery">
            <!-- Spin parameters: 0.78 rad ≈ 45°, 0.3 rad/s = slow for balance -->
            <Spin spin_dist="0.78" spin_speed="0.3" name="HumanoidSpin"/>
            <Wait wait_duration="2"/>
          </Sequence>

          <!-- Level 5: Very slow backup (last resort before abort) -->
          <Sequence name="SlowBackupRecovery">
            <!-- Backup parameters: 0.15m distance, 0.05 m/s = very slow for humanoid -->
            <BackUp backup_dist="0.15" backup_speed="0.05" name="HumanoidBackup"/>
            <Wait wait_duration="3"/>
          </Sequence>

          <!-- Level 6: Spin opposite direction -->
          <Sequence name="OppositeSpinRecovery">
            <Spin spin_dist="-0.78" spin_speed="0.3" name="HumanoidSpinOpposite"/>
            <Wait wait_duration="2"/>
          </Sequence>

          <!-- Level 7: Combined backup + spin -->
          <Sequence name="BackupAndSpinRecovery">
            <BackUp backup_dist="0.1" backup_speed="0.05" name="SmallBackup"/>
            <Wait wait_duration="1"/>
            <Spin spin_dist="1.57" spin_speed="0.3" name="LargerSpin"/>
            <Wait wait_duration="2"/>
          </Sequence>

          <!-- Level 8: Final wait before abort -->
          <Sequence name="FinalWaitRecovery">
            <Wait wait_duration="15" name="FinalWait"/>
          </Sequence>

        </RoundRobin>
      </ReactiveFallback>

    </RecoveryNode>
  </BehaviorTree>
</root>
```

### Key Design Decisions

**1. TEB planner specified**:
```xml
<ComputePathToPose goal="{goal}" path="{path}" planner_id="TEB"/>
```
Uses TEB for smooth, humanoid-appropriate paths.

**2. Lower replan rate**:
```xml
<RateController hz="0.5">
```
0.5 Hz replanning (every 2 seconds) instead of 1 Hz—reduces computation.

**3. Conservative recovery speeds**:
```xml
<Spin spin_dist="0.78" spin_speed="0.3"/>
<BackUp backup_dist="0.15" backup_speed="0.05"/>
```
Humanoid-safe velocities for balance.

**4. Wait-heavy recovery**:
Multiple wait stages before movement—standing is safest.

**5. Escalation structure**:
Safest behaviors (wait) tried before risky (backup).

## Testing Stuck Scenarios

Create test script that intentionally triggers stuck conditions:

```python
#!/usr/bin/env python3
"""
Test humanoid navigation recovery by creating stuck scenarios.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import time

class RecoveryTester(Node):
    def __init__(self):
        super().__init__('recovery_tester')

        # Navigation action client
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # Service clients for scenario setup
        self.spawn_obstacle_client = self.create_client(
            Empty, '/spawn_blocking_obstacle'  # Isaac Sim service
        )
        self.remove_obstacle_client = self.create_client(
            Empty, '/remove_blocking_obstacle'
        )

        self.get_logger().info('Recovery tester initialized')

    def send_goal(self, x: float, y: float, yaw: float = 0.0):
        """Send navigation goal."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        import math
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending goal: ({x}, {y}, {yaw})')

        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation server not available')
            return None

        return self.nav_client.send_goal_async(goal)

    def test_temporary_obstruction(self):
        """
        Test Scenario 1: Temporary obstruction
        - Send goal
        - Spawn obstacle in path
        - Wait for recovery (Wait behavior should handle)
        - Remove obstacle
        - Verify navigation completes
        """
        self.get_logger().info('=== Test: Temporary Obstruction ===')

        # Send goal
        future = self.send_goal(5.0, 0.0)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        # Spawn obstacle after 3 seconds
        time.sleep(3.0)
        self.get_logger().info('Spawning blocking obstacle...')
        # self.spawn_obstacle_client.call_async(Empty.Request())

        # Wait for recovery behaviors (should see Wait in BT)
        time.sleep(10.0)

        # Remove obstacle
        self.get_logger().info('Removing obstacle...')
        # self.remove_obstacle_client.call_async(Empty.Request())

        # Wait for navigation to complete
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        result = result_future.result()
        success = result.result.result == 0  # NavigationResult.SUCCEEDED

        self.get_logger().info(f'Test result: {"PASS" if success else "FAIL"}')
        return success

    def test_path_blocked(self):
        """
        Test Scenario 2: Path blocked permanently
        - Send goal behind permanent obstacle
        - Verify Clear + Replan behaviors triggered
        - Verify new path found around obstacle
        """
        self.get_logger().info('=== Test: Path Blocked ===')

        # Goal behind obstacle (requires replanning)
        future = self.send_goal(5.0, 3.0)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)

        result = result_future.result()
        success = result.result.result == 0

        self.get_logger().info(f'Test result: {"PASS" if success else "FAIL"}')
        return success

    def test_stuck_in_corner(self):
        """
        Test Scenario 3: Robot stuck in corner
        - Navigate into corner
        - Send goal requiring escape
        - Verify Spin + BackUp behaviors triggered
        """
        self.get_logger().info('=== Test: Stuck in Corner ===')

        # First navigate into corner
        future = self.send_goal(1.0, 1.0)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        # Now try to escape (goal requires turning around)
        future = self.send_goal(-2.0, -2.0)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)

        result = result_future.result()
        success = result.result.result == 0

        self.get_logger().info(f'Test result: {"PASS" if success else "FAIL"}')
        return success


def main():
    rclpy.init()
    tester = RecoveryTester()

    # Run tests
    results = {}
    results['temporary_obstruction'] = tester.test_temporary_obstruction()
    results['path_blocked'] = tester.test_path_blocked()
    results['stuck_in_corner'] = tester.test_stuck_in_corner()

    # Print summary
    print('\n' + '='*50)
    print('RECOVERY TEST RESULTS')
    print('='*50)
    for test, passed in results.items():
        status = '✓ PASS' if passed else '✗ FAIL'
        print(f'{test}: {status}')
    print('='*50)

    all_passed = all(results.values())
    print(f'Overall: {"ALL TESTS PASSED" if all_passed else "SOME TESTS FAILED"}')

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercise 2: Implement BT with AI Assistance

**Objective**: Translate requirements specification into working XML.

## Try With AI: Three Roles Demonstration

### Role 1: AI as Teacher — BehaviorTree.CPP Syntax

You need to understand BT XML syntax. Ask AI:

```
"I'm implementing Nav2 behavior tree for humanoid robot. Explain BehaviorTree.CPP syntax for:
1. RecoveryNode (what does number_of_retries mean?)
2. PipelineSequence vs regular Sequence
3. RoundRobin (how does it cycle through children?)
4. How to pass parameters like spin_speed to Spin action"
```

**Expected learning**: AI explains BT node semantics, parameter passing, and Nav2-specific node behavior.

### Role 2: AI as Student — Validating Escalation Logic

You have requirements for recovery escalation. Test AI understanding:

```
"I designed recovery escalation for humanoid navigation:
1. Wait 5s (standing is safe)
2. Clear costmap (no movement)
3. Wait 10s (more time for dynamic obstacles)
4. Slow spin 45° at 0.3 rad/s
5. Slow backup 0.15m at 0.05 m/s
6. Abort

Validate this escalation order. Are there scenarios where different ordering would be better? What if robot is in narrow corridor?"
```

**Expected interaction**: AI evaluates your escalation logic, potentially suggests improvements (e.g., in narrow corridor, spin before wait might clear faster), you decide whether to incorporate feedback.

### Role 3: AI as Co-Worker — Writing BT XML Together

Collaborate with AI to implement the behavior tree:

```
"Let's write the humanoid navigation BT XML together. My requirements:
- TEB planner (smooth paths)
- 0.5 Hz replan rate (not too frequent)
- Recovery: wait → clear → spin → backup → abort
- Humanoid-safe speeds for spin and backup

Start with the main navigation pipeline. I'll review and we'll iterate."
```

**Expected collaboration**: AI drafts XML sections, you review for humanoid safety, refine together.

## Exercise 3: Test Stuck Scenarios

**Objective**: Verify behavior tree handles failure modes correctly.

**Steps**:

1. Load humanoid navigation BT in Nav2
2. Run recovery tester script
3. Observe BT execution in RViz (use BT visualization)
4. Record which recovery behaviors triggered

**BT Visualization in RViz**:

```bash
# Install BT visualization
sudo apt install ros-humble-behaviortree-cpp-v3

# Nav2 publishes BT state on /behavior_tree_log
# Add BehaviorTree display in RViz
```

**Test Matrix**:

| Scenario | Expected Recovery | Actual Recovery | Pass/Fail |
|----------|-------------------|-----------------|-----------|
| Temporary obstruction | Wait | | |
| Stale costmap | Clear costmap | | |
| Path blocked | Replan | | |
| Stuck facing wall | Spin | | |
| Stuck in corner | Spin + Backup | | |
| Unreachable goal | Abort | | |

**Success criteria**:
- [ ] All scenarios trigger expected recovery
- [ ] Recovery behaviors use humanoid-safe speeds
- [ ] Escalation follows designed order
- [ ] Abort triggered only after all recovery exhausted

## Troubleshooting Behavior Tree Issues

### Issue 1: BT Never Triggers Recovery

**Symptom**: Robot gets stuck but stays in navigation mode, no recovery behaviors.

**Diagnosis**: Progress checker or stuck detector not working.

**Fix**:
- Check `progress_checker` configuration in controller
- Verify `required_movement_radius` and `movement_time_allowance`
- Ensure BT has correct `RecoveryNode` wrapping

### Issue 2: Recovery Behaviors Too Aggressive

**Symptom**: Spin or backup executes too fast, humanoid wobbles or falls.

**Diagnosis**: Recovery action parameters not humanoid-safe.

**Fix**:
- Reduce `spin_speed` (try 0.2-0.3 rad/s)
- Reduce `backup_speed` (try 0.03-0.05 m/s)
- Reduce `spin_dist` and `backup_dist`

### Issue 3: Wrong Recovery Order

**Symptom**: Backup executes before Wait, causing unnecessary movement.

**Diagnosis**: RoundRobin order or sequence structure incorrect.

**Fix**: Review RoundRobin children order—first child is first recovery attempt.

### Issue 4: Navigation Aborts Too Quickly

**Symptom**: Robot aborts after 1-2 recovery attempts, should try more.

**Diagnosis**: `number_of_retries` too low.

**Fix**: Increase `number_of_retries` in outer RecoveryNode (try 6-10).

### Issue 5: BT Syntax Error (Won't Load)

**Symptom**: Nav2 fails to start, BT parsing error.

**Diagnosis**: XML malformed or node names incorrect.

**Fix**:
- Validate XML syntax (use online XML validator)
- Check node names match Nav2 BT plugins (case-sensitive)
- Verify all parameters have correct types

## Summary: Behavior Tree Design for Humanoid Navigation

**Key design principles**:

1. **Specification-first**: Define requirements before writing XML
2. **Escalation structure**: Safest behaviors first, risky last
3. **Humanoid-safe speeds**: Slow spin (0.3 rad/s), very slow backup (0.05 m/s)
4. **Wait-heavy**: Standing is stable—wait before moving
5. **Multiple retry levels**: 6-10 recovery attempts before abort

**Recovery behavior summary**:

| Behavior | Humanoid-Safe Parameters | Risk Level |
|----------|--------------------------|------------|
| Wait | Any duration | None (standing) |
| Clear costmap | N/A (no movement) | None |
| Spin | 0.3 rad/s, 45-90° | Low |
| Backup | 0.05 m/s, 0.1-0.15m | Medium |
| Abort | N/A | N/A |

**Next lesson**: You'll handle dynamic obstacles—objects that move while the robot navigates, requiring real-time replanning and safe-stop behaviors.

## Checkpoint: Behavior Tree Validation

Before proceeding to Lesson 6 (dynamic obstacles), validate:

- [ ] **Requirements specified**: BT requirements document created
- [ ] **XML implemented**: Humanoid navigation BT created
- [ ] **Humanoid-safe speeds**: Spin ≤ 0.3 rad/s, backup ≤ 0.05 m/s
- [ ] **Escalation order correct**: Wait → Clear → Spin → Backup → Abort
- [ ] **Stuck scenarios tested**: All test scenarios pass
- [ ] **Recovery behaviors observed**: BT visualization confirms correct execution

**If all checks pass**, your behavior tree is correctly designed for humanoid navigation recovery.
