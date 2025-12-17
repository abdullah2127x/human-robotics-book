---
title: Creating Behavior Tree Design Skill
chapter: 9
lesson: 8
learning_objectives:
  - Extract behavior tree design patterns from Lesson 5 experience
  - Create behavior-tree-design skill for navigation and recovery logic
  - Document failure scenario testing methodology
  - Validate skill on different navigation scenario
estimated_time: 150 minutes
skills:
  behavior-tree-design:
    proficiency_level: C1
  skill-creation:
    proficiency_level: C1
generated_by: content-implementer v1.1.0
source_spec: specs/book/part-3/chapter-09-plan.md
created: 2025-12-17
---

# Creating Behavior Tree Design Skill

In Lesson 5, you designed behavior trees for humanoid navigation with recovery escalation. You learned that safest behaviors come first, humanoid-safe speeds are mandatory, and wait-heavy recovery exploits the fact that standing is stable.

This lesson transforms that knowledge into a **behavior-tree-design skill**—a reusable template for designing behavior trees for any autonomous robot navigation scenario. The skill captures not just *what* to configure, but *how to think about* behavior tree design.

## Why a Behavior Tree Design Skill?

Behavior trees are used across robotics:
- Mobile robot navigation
- Manipulation tasks
- Multi-robot coordination
- Human-robot interaction

Each application requires different BT structure, but design patterns are transferable:
- **Hierarchical decomposition**: Break complex behaviors into simple nodes
- **Recovery escalation**: Handle failures gracefully
- **Failure testing**: Validate BT handles edge cases

A skill encodes these patterns so you can design BTs efficiently for new scenarios.

## Extracting BT Design Patterns

### Pattern 1: Hierarchical Composition

**Observation from Lesson 5**: Complex navigation = planning + following + recovery

```
NavigateToGoal
├── MainNavigation
│   ├── ComputePath
│   └── FollowPath
└── RecoveryFallback
    ├── Recovery1
    ├── Recovery2
    └── ...
```

**Pattern**: Decompose goal into independent sub-behaviors, wrap with recovery.

### Pattern 2: Sequence for Success Chain

**When to use**: Multiple steps that must all succeed.

```
Sequence
├── CheckPrecondition
├── ExecuteAction
└── VerifyResult
```

**Pattern**: Use Sequence when all children must succeed for parent to succeed.

### Pattern 3: Fallback for Alternatives

**When to use**: Try alternatives until one succeeds.

```
Fallback
├── PreferredMethod
├── AlternativeMethod
└── LastResort
```

**Pattern**: Use Fallback when any child success counts as parent success.

### Pattern 4: Recovery Escalation

**Observation from Lesson 5**: Recovery behaviors should escalate from safe to risky.

```
RecoveryFallback
├── SafestRecovery      (Wait - no movement)
├── SafeRecovery        (Clear costmap - computational)
├── MildRecovery        (Slow spin - minimal movement)
├── AggressiveRecovery  (Backup - risky movement)
└── Abort               (Give up - request human help)
```

**Pattern**: Order recovery by risk level, not by likelihood of success.

### Pattern 5: Rate Limiting

**Observation**: Some behaviors shouldn't trigger too frequently.

```
RateController(hz=0.5)
└── ExpensiveComputation
```

**Pattern**: Wrap computationally expensive or disruptive behaviors with rate limits.

### Pattern 6: Timeout Protection

**When to use**: Prevent indefinite waiting.

```
Timeout(seconds=30)
└── PotentiallySlowAction
```

**Pattern**: Always wrap external service calls with timeouts.

### Pattern 7: Condition Guards

**When to use**: Only execute action when conditions met.

```
Sequence
├── CheckCondition (condition node)
└── ExecuteAction  (action node)
```

**Pattern**: Conditions before actions prevent wasted effort.

### Pattern 8: Failure Testing Methodology

**Observation from Lesson 5**: BT must be tested with intentional failures.

**Test categories**:
1. **Nominal success**: Does BT achieve goal under normal conditions?
2. **Single failure**: Does recovery handle one failure?
3. **Multiple failures**: Does escalation work through recovery sequence?
4. **Unrecoverable**: Does BT abort cleanly when all recovery fails?

**Pattern**: For each failure mode, verify appropriate recovery triggers.

## Writing the Behavior Tree Design Skill

Create skill file:

```bash
mkdir -p .claude/skills/behavior-tree-design
```

Create `SKILL.md`:

```markdown
# Behavior Tree Design Skill

## Persona

You are a robotics software architect specialized in behavior tree design for autonomous systems. You understand that behavior trees provide modular, hierarchical task decomposition with clear success/failure semantics. Your BT designs prioritize:
- **Composability**: Small, reusable behavior modules
- **Robustness**: Graceful failure handling and recovery
- **Testability**: Clear failure modes that can be validated
- **Safety**: Risk-aware recovery ordering (safest first)

You design BTs using BehaviorTree.CPP syntax, compatible with Nav2 and other ROS 2 applications.

## Questions

Before designing a behavior tree, gather this information:

### Task Definition
1. **Primary goal**: What should the robot achieve? (e.g., navigate to pose, pick object, patrol area)
2. **Success criteria**: How do we know the task succeeded?
3. **Failure conditions**: What external conditions cause failure? (obstacles, timeouts, hardware faults)

### Available Actions
4. **Action primitives**: What atomic actions can the robot perform? (move, rotate, wait, grip, speak)
5. **Action parameters**: What parameters do actions accept? (speed, distance, timeout)
6. **Action durations**: How long do actions typically take?

### Recovery Behaviors
7. **Available recovery actions**: What can the robot do when stuck? (wait, retry, call help)
8. **Recovery constraints**: Are there actions that must be avoided? (no fast movement for humanoids)
9. **Maximum retry count**: How many recovery attempts before aborting?

### Safety Requirements
10. **Safety-critical conditions**: What must always be true? (e.g., battery > 10%, no collision)
11. **Emergency stop behavior**: What happens on safety violation?
12. **Robot-specific constraints**: Movement limitations, speed limits, forbidden actions

### Environment
13. **Expected failure modes**: What typically goes wrong? (path blocked, localization lost, sensor failure)
14. **Operator involvement**: Should robot request human help, or fully autonomous?

## Principles

Apply these rules when designing behavior trees:

### Hierarchical Decomposition

```
PRINCIPLE: Decompose complex tasks into single-responsibility nodes

- Each node should do ONE thing
- If a node does multiple things, split it
- Leaf nodes: actions (do something) or conditions (check something)
- Control nodes: sequences, fallbacks, decorators
- Name nodes descriptively (NavigateToKitchen, not Node1)
```

### Control Node Selection

```
PRINCIPLE: Choose control node based on success semantics

Sequence (→): Use when ALL children must succeed
  - "Do A, then B, then C"
  - Fails on FIRST child failure
  - Example: CheckBattery → ComputePath → FollowPath

Fallback (?): Use when ANY child success is acceptable
  - "Try A, if fails try B, if fails try C"
  - Succeeds on FIRST child success
  - Example: TryMainPath | TryAlternativePath | AbortNavigation

Parallel (⇉): Use when children should run simultaneously
  - Requires success policy (all, one, N)
  - Example: MonitorBattery || ExecuteTask
```

### Recovery Escalation Structure

```
PRINCIPLE: Order recovery behaviors from safest to riskiest

Standard escalation pattern:
1. Wait        - No movement, safest
2. Clear state - Computational only (clear costmap, reset planner)
3. Retry       - Repeat failed action
4. Mild action - Minimal movement (slow rotation)
5. Strong action - More movement (backup, significant rotation)
6. Call help   - Request operator intervention
7. Abort       - Give up, safe state

NEVER put risky recovery before safe recovery
ALWAYS include abort as final option
```

### Recovery Node Configuration

```
PRINCIPLE: Wrap main behaviors in RecoveryNode with appropriate retry count

RecoveryNode structure:
  RecoveryNode(number_of_retries=N)
  ├── MainBehavior
  └── RecoveryFallback
      ├── Recovery1
      ├── Recovery2
      └── ...

Retry count guidelines:
- Simple tasks: 2-3 retries
- Complex tasks: 5-10 retries
- Safety-critical: 1-2 retries, then abort
```

### Rate Limiting

```
PRINCIPLE: Limit frequency of expensive or disruptive operations

Use RateController for:
- Path planning (0.5-1 Hz)
- Service calls (limit based on service capacity)
- State changes (prevent thrashing)

Example:
  RateController(hz=0.5)
  └── ComputeExpensivePath
```

### Timeout Protection

```
PRINCIPLE: All external interactions must have timeouts

Timeout guidelines:
- Service calls: 5-10 seconds
- Navigation actions: Based on expected duration × 2
- Sensor waits: 1-2 seconds

Example:
  Timeout(seconds=30)
  └── WaitForHumanApproval
```

### Failure Testing Methodology

```
PRINCIPLE: Design tests for each failure mode BEFORE implementing BT

Test matrix template:
| Scenario | Trigger | Expected Recovery | Validation |
|----------|---------|-------------------|------------|
| Normal success | None | N/A | Task completes |
| Path blocked | Obstacle | Replan | New path found |
| Stuck | No progress | Spin/Backup | Robot escapes |
| Unrecoverable | Permanent block | Abort | Clean shutdown |

For each recovery behavior:
1. Create scenario that triggers it
2. Verify correct recovery activates
3. Verify recovery succeeds OR escalates
```

### Safety Guards

```
PRINCIPLE: Critical safety conditions checked continuously

Pattern for safety monitoring:
  ReactiveFallback
  ├── SafetyMonitor (condition)
  └── EmergencyStop (action)

Safety monitors should:
- Check battery level
- Check localization confidence
- Check collision proximity
- Return FAILURE when safe, SUCCESS when unsafe
  (inverted logic triggers emergency action)
```

## BT Design Template

When designing a new behavior tree:

### Step 1: Define Success/Failure

```markdown
## Task: [TASK_NAME]

### Success Criteria
- [ ] [Criterion 1]
- [ ] [Criterion 2]

### Failure Conditions
- [ ] [Failure 1]: Triggers [Recovery A]
- [ ] [Failure 2]: Triggers [Recovery B]
```

### Step 2: List Available Actions

```markdown
### Actions
| Action | Parameters | Duration | Notes |
|--------|------------|----------|-------|
| [Action1] | [params] | [time] | [notes] |
```

### Step 3: Design Recovery Escalation

```markdown
### Recovery Sequence (safest → riskiest)
1. [Recovery1] - [Why it's safest]
2. [Recovery2] - [When to use]
...
N. Abort - All else failed
```

### Step 4: Write BT Structure

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <!-- BT XML here -->
  </BehaviorTree>
</root>
```

### Step 5: Create Test Matrix

```markdown
### Test Cases
| Scenario | Setup | Expected | Pass/Fail |
|----------|-------|----------|-----------|
| [Test1] | [How to trigger] | [What should happen] | |
```

## Robot-Specific Adaptations

### Humanoid Robots

```
ADAPTATIONS:
- Recovery speeds: spin ≤ 0.3 rad/s, backup ≤ 0.05 m/s
- Wait-heavy recovery: standing is stable
- No lateral movement in any behavior
- Abort before balance-risking actions
```

### Wheeled Mobile Robots

```
ADAPTATIONS:
- Standard recovery speeds acceptable
- Spin recovery can be faster (1.0 rad/s)
- Backup recovery can be longer (0.5m at 0.2 m/s)
- Consider multi-directional escape
```

### Manipulator Arms

```
ADAPTATIONS:
- Recovery: retract to safe pose
- Collision: stop immediately, don't retry blindly
- Consider object drop safety
```
```

## Exercise 1: Extract BT Patterns from Lesson 5

**Objective**: Document patterns from your humanoid navigation BT.

Review your `humanoid_navigate.xml` and identify:

| Pattern | Where Used | Why |
|---------|------------|-----|
| Hierarchical composition | | |
| Recovery escalation | | |
| Rate limiting | | |
| RoundRobin cycling | | |
| Wait before movement | | |

**Success criteria**:
- All patterns identified
- Rationale documented

## Exercise 2: Write BT Design Skill

**Objective**: Create complete skill file.

**Steps**:

1. Use template provided above
2. Customize based on your Lesson 5 experience
3. Add humanoid-specific adaptations section
4. Include test matrix template
5. Save to `.claude/skills/behavior-tree-design/SKILL.md`

**Success criteria**:
- [ ] Persona captures BT design expertise
- [ ] Questions cover task, actions, recovery, safety
- [ ] Principles include all identified patterns
- [ ] Template provides step-by-step design process
- [ ] Test methodology included

## Exercise 3: Test Skill on Different Scenario

**Objective**: Validate skill on non-navigation BT.

**Scenario**: Design BT for humanoid **object handoff** task:
- Robot receives object from human
- Robot carries object to destination
- Robot hands object to second human

**Apply skill**:

1. **Answer questions**:
   - Primary goal: Transport object between humans
   - Success criteria: Object delivered, humans acknowledged
   - Available actions: Wait, Extend arm, Grip, Release, Navigate, Speak
   - Recovery behaviors: Wait, Retry grip, Ask human to reposition, Abort

2. **Apply principles**:
   - Hierarchical decomposition: ReceiveObject → Navigate → DeliverObject
   - Recovery escalation: Wait → Retry → Ask human → Abort
   - Timeout on human interactions

3. **Design BT structure**:

```xml
<root main_tree_to_execute="ObjectHandoff">
  <BehaviorTree ID="ObjectHandoff">
    <RecoveryNode number_of_retries="3" name="HandoffRecovery">
      <Sequence name="HandoffSequence">
        <!-- Receive from Human 1 -->
        <RecoveryNode number_of_retries="2" name="ReceiveRecovery">
          <Sequence name="Receive">
            <Speak text="Ready to receive object"/>
            <WaitForHumanPresence timeout="30"/>
            <ExtendArm pose="receive"/>
            <WaitForObjectInGripper timeout="10"/>
            <Grip force="gentle"/>
            <RetractArm/>
          </Sequence>
          <Sequence name="ReceiveFailRecovery">
            <Speak text="Please place object in my hand"/>
            <Wait duration="5"/>
          </Sequence>
        </RecoveryNode>

        <!-- Navigate to destination -->
        <NavigateToPose goal="{destination_pose}"/>

        <!-- Deliver to Human 2 -->
        <RecoveryNode number_of_retries="2" name="DeliverRecovery">
          <Sequence name="Deliver">
            <Speak text="Ready to deliver object"/>
            <WaitForHumanPresence timeout="30"/>
            <ExtendArm pose="deliver"/>
            <WaitForHumanGrasp timeout="10"/>
            <Release/>
            <RetractArm/>
            <Speak text="Delivery complete"/>
          </Sequence>
          <Sequence name="DeliverFailRecovery">
            <Speak text="Please take the object"/>
            <Wait duration="5"/>
          </Sequence>
        </RecoveryNode>
      </Sequence>

      <!-- Main recovery fallback -->
      <ReactiveFallback name="MainRecovery">
        <Sequence name="WaitRecovery">
          <Speak text="Waiting for conditions to improve"/>
          <Wait duration="10"/>
        </Sequence>
        <Sequence name="AbortRecovery">
          <Speak text="Unable to complete handoff, returning to base"/>
          <NavigateToPose goal="{home_pose}"/>
        </Sequence>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

4. **Create test matrix**:

| Scenario | Trigger | Expected | Pass/Fail |
|----------|---------|----------|-----------|
| Normal handoff | Both humans present | Complete delivery | |
| Human 1 absent | No human at receive | Timeout, speak, retry | |
| Drop during carry | Object falls | Abort, return home | |
| Human 2 absent | No human at deliver | Timeout, speak, retry | |
| Navigation blocked | Obstacle | Navigate recovery | |

**Success criteria**:
- [ ] BT structure follows skill principles
- [ ] Recovery escalation appropriate for task
- [ ] Timeouts on all human interactions
- [ ] Test matrix covers failure modes

## Using the Skill with AI

**Example prompt for new BT design**:

```
"Using the behavior-tree-design skill, design a behavior tree for:

Task: Patrol robot that visits 3 waypoints in sequence, checking for anomalies at each.

Actions available:
- NavigateToPose(goal)
- TakePhoto()
- AnalyzeForAnomaly() → returns success if normal
- ReportAnomaly(location)
- Wait(duration)

Constraints:
- Must visit all 3 waypoints
- If anomaly detected, report and continue
- If navigation fails, skip waypoint and continue
- After 3 total navigation failures, return to base

Generate BT XML and test matrix."
```

**AI will**:
1. Apply hierarchical decomposition (main loop + waypoint visits)
2. Design recovery (skip waypoint vs return to base)
3. Include rate limiting if needed
4. Generate test matrix for failure scenarios

## Summary: Behavior Tree Design Skill

**What you created**:

A reusable skill encoding:
- BT design principles (decomposition, control selection, recovery)
- Standard escalation patterns
- Test methodology for validation
- Robot-specific adaptations

**Skill benefits**:

| Without Skill | With Skill |
|---------------|------------|
| Ad-hoc BT structure | Principled design |
| Forgotten failure modes | Systematic test matrix |
| Inconsistent recovery | Standard escalation |
| Robot-specific knowledge lost | Adaptations documented |

**Next lesson**: Capstone project integrating Isaac Sim (Chapter 7), VSLAM (Chapter 8), and Nav2 (Chapter 9) for full autonomous humanoid navigation.

## Checkpoint: BT Design Skill Validation

Before proceeding to Lesson 9 (capstone), validate:

- [ ] **Patterns extracted**: All key patterns from Lesson 5 documented
- [ ] **Skill file created**: `.claude/skills/behavior-tree-design/SKILL.md` exists
- [ ] **Persona appropriate**: Captures BT design expertise
- [ ] **Questions comprehensive**: Task, actions, recovery, safety covered
- [ ] **Principles complete**: All patterns encoded
- [ ] **Test methodology included**: Test matrix template provided
- [ ] **Different scenario tested**: Object handoff BT designed

**If all checks pass**, your behavior-tree-design skill is ready for reuse.
