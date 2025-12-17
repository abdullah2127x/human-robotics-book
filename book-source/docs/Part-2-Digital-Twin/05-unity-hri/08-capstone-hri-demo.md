---
title: "Lesson 8: Complete HRI Demonstration (Capstone)"
chapter: 5
lesson: 8
proficiency_level: B2
learning_objectives:
  - "Write specifications that drive complex system implementation"
  - "Compose multiple components into integrated system"
  - "Orchestrate AI-assisted implementation using specifications"
  - "Validate system against acceptance criteria"
  - "Reflect on design decisions and trade-offs"
estimated_time: "150 minutes"
generated_by: content-implementer v1.0.0
created: 2025-12-16
version: 1.0.0
---

# Lesson 8: Complete HRI Demonstration (Capstone)

## Introduction

This capstone demonstrates the full development cycle: **specification first**, then orchestrate your accumulated knowledge (Lessons 1-7) to implement.

You'll write a specification for a complete HRI scenario, then use that specification to guide implementation. Neither you nor AI has the complete solution—but together, specification-driven development produces a professional system.

This is how production systems get built in the agentic era.

**Estimated time**: 150 minutes (teaching + hands-on implementation)

---

## The Specification-First Approach

**Traditional approach** (code-first):
1. Start building immediately
2. Make decisions ad-hoc
3. Validate at end
4. Surprises arise late (expensive)

**Specification-first approach** (this lesson):
1. Write complete specification FIRST
2. Use spec to guide all decisions
3. Validate continuously
4. Build what's actually needed

---

## Step 1: Write Your Specification

Create `specs/chapter-5-capstone/spec.md`:

```markdown
# HRI Capstone Specification

## Intent

Develop a complete human-robot interaction demonstration in a photorealistic
office environment. A human avatar approaches a humanoid robot. The robot
acknowledges the approach. Both actors are visible, well-lit, and interactive
in real-time. ROS 2 receives all interaction events.

## Success Criteria

- [x] Scene renders at 60+ FPS
- [x] Human avatar visible with natural walking animation
- [x] Humanoid robot visible with proper materials/lighting
- [x] Proximity detection triggers within 1.5-2.0m
- [x] Robot responds with gesture when human approaches
- [x] Interaction completes naturally within 2-3 seconds
- [x] UI displays interaction status to user
- [x] ROS 2 receives interaction events on `/hri/interaction_events`
- [x] Multiple interactions can be triggered sequentially
- [x] No performance degradation during interaction

## Constraints

- Office environment (living room acceptable alternative)
- Real-time rendering (60+ FPS minimum)
- Bidirectional ROS 2 communication
- Humanoid robot model (from Chapter 4)
- Human avatar from Mixamo or equivalent
- No third-party physics engines beyond Unity PhysX

## Non-Goals

- Realistic facial expressions (skeletal animation only)
- Speech recognition or text-to-speech
- Complex manipulation (reaching for objects)
- Multi-human scenarios
- Real robot deployment (simulation only)

## Acceptance Tests

```gherkin
Scenario: Human approaches robot
  Given: Office scene loaded
  When: User presses W to walk human toward robot
  And: Human distance to robot < 1.5m
  Then: Robot animation plays (head turn, gesture)
  And: UI shows "Robot acknowledged"
  And: ROS topic publishes InteractionEvent
  And: Frame rate stays > 60 FPS

Scenario: Complete interaction cycle
  Given: Initial interaction completed
  When: Human walks away (distance > 2m)
  And: Human turns around and approaches again
  Then: Robot responds again (no lingering state)
  And: Interaction triggers reliably every time

Scenario: ROS 2 integration
  Given: ROS-TCP-Connector bridge operational
  When: Human interacts with robot
  Then: ros2 topic echo /hri/interaction_events shows messages
  And: Message contains correct position/timestamp data
  And: Message frequency is event-based (not continuous)
```

## Component Composition

### From Lessons 1-7

**Lesson 1 (Bridge)**:
- ROS-TCP-Connector setup (reuse existing bridge)

**Lesson 2 (URDF Import)**:
- Humanoid robot imported and visible
- Materials applied for visual quality

**Lesson 3 (Environment)**:
- Office scene with three-point lighting
- Professional appearance, 60+ FPS performance

**Lesson 4 (Avatars)**:
- Human avatar with animator state machine
- Walking and idle animations smooth

**Lesson 5 (Interaction)**:
- Proximity detection via trigger colliders
- Event-based interaction triggering
- Animation response to interaction

**Lesson 6 (ROS)**:
- Publishers for interaction events
- Message serialization working correctly

**Lesson 7 (Patterns)**:
- Reusable HRI scene structure
- Prefab template applies

### New for Capstone

**Orchestration**:
- Compose all components into single cohesive scene
- Wire interactions through to ROS publishing
- Test all acceptance criteria

**Optimization**:
- Verify 60+ FPS performance
- Tune lighting/materials if needed
- Profile for bottlenecks

**Documentation**:
- Record design decisions
- Reflect on what worked/didn't
```

---

## Step 2: Analyze Specification and Plan

Read your specification. Ask yourself:

**1. What components exist?**
- Bridge (Lesson 1) ✓
- URDF import (Lesson 2) ✓
- Environment (Lesson 3) ✓
- Avatar system (Lesson 4) ✓
- Interaction logic (Lesson 5) ✓
- ROS integration (Lesson 6) ✓
- Reusable patterns (Lesson 7) ✓

**2. What's missing?**
- Orchestration (combining all components)
- Performance optimization (if needed)
- Final testing and validation

**3. Implementation strategy?**
```
Assembly order:
1. Start with HRI template from Lesson 7
2. Add office environment (Lesson 3 knowledge)
3. Import humanoid robot (Lesson 2)
4. Add human avatar (Lesson 4)
5. Configure interactions (Lesson 5)
6. Enable ROS publishing (Lesson 6)
7. Test acceptance criteria
8. Optimize if needed
```

---

## Step 3: Implementation

### Phase 1: Scene Assembly (30 minutes)

1. **Create capstone scene**:
   ```
   File → New Scene → Save as "HRI_Capstone_Office.unity"
   ```

2. **Use template from Lesson 7**:
   ```
   Right-click Hierarchy → Prefab → HRISceneTemplate
   ```

3. **Add office environment** (Lesson 3 approach):
   - Desk, chair, shelves (simple models)
   - Floor with carpet material
   - Walls with matte color
   - Apply three-point lighting

4. **Import humanoid robot** (Lesson 2 approach):
   - Robotics → Import URDF
   - Apply materials (white torso, gray limbs)
   - Position at center (0, 0, 0)

5. **Add human avatar** (Lesson 4 approach):
   - Drag Mixamo character into scene
   - Position near desk (1, 0, 0)
   - Attach animator controller

6. **Configure interaction zone** (Lesson 5 approach):
   - Add sphere trigger at robot position
   - Radius: 1.5m
   - Attach InteractionZone.cs script

7. **Enable ROS** (Lesson 6 approach):
   - Create ROS Manager GameObject
   - Attach RosPublisher.cs
   - Configure publisher for interaction events

### Phase 2: Testing (30 minutes)

**Test each acceptance criterion**:

```gherkin
✓ Scene renders at 60+ FPS
  → Window → Profiler → Check frame rate during playback

✓ Human avatar visible and animated
  → Press Play, press W, observe walking animation

✓ Robot visible and well-lit
  → Compare against Lesson 2 verification

✓ Proximity detection triggers
  → Walk human toward robot, observe interaction trigger

✓ Robot responds with animation
  → Verify animation plays (head turn, gesture)

✓ UI displays status
  → Check InteractionUI shows "Robot acknowledged"

✓ ROS topics publish
  → ros2 topic echo /hri/interaction_events

✓ Message frequency reasonable
  → ros2 topic hz /hri/interaction_events (should be ~1 msg/interaction)

✓ Interactions sequential
  → Walk away, return, interact again → triggers multiple times

✓ Performance maintained
  → Check FPS during interaction (should stay > 60)
```

### Phase 3: Optimization (20 minutes)

If performance is below target:

**Quick wins**:
1. Enable Static Batching (select environment objects → Inspector → Static)
2. Reduce shadow-casting lights (only key light casts shadows)
3. Use LOD groups on background objects

**Performance profiling**:
```
Window → Analysis → Profiler
Check:
- CPU time (which scripts take longest?)
- GPU time (rendering bottleneck?)
- Memory (any leaks?)
```

Address bottleneck:
- High CPU → Optimize script efficiency (fewer distance checks, etc.)
- High GPU → Reduce mesh complexity or draw calls
- High memory → Check for persistent allocations in Update()

### Phase 4: Final Validation (20 minutes)

**Against specification**:
- [ ] All acceptance tests pass
- [ ] All success criteria met
- [ ] No non-goals violated
- [ ] Constraints satisfied

**Document results**:
```
Capstone Validation Report

✓ Performance: 62 FPS (target 60+)
✓ Interaction latency: 0.3s from approach to response
✓ ROS message rate: 3 events/interaction (approach, acknowledge, disengage)
✓ Animation smoothness: Natural, no jerky transitions
✓ Scene appearance: Professional, credible for research

Issues resolved:
- Initial FPS 45 → Fixed with static batching (62 FPS)
- Initial transition jerky → Fixed with 0.4s transition duration

Ready for publication: YES
```

---

## Hands-On: Build Your Capstone

### Your Tasks

**Task 1**: Write specification (20 minutes)
- Create `specs/chapter-5-capstone/spec.md`
- Define intent, success criteria, constraints, acceptance tests
- Save and review

**Task 2**: Create capstone scene (40 minutes)
- Use HRI template from Lesson 7
- Add office environment
- Import robot and avatar
- Configure interactions and ROS publishing

**Task 3**: Test all acceptance criteria (40 minutes)
- Verify each acceptance test passes
- Document results
- Optimize if needed

**Task 4**: Reflect on design (15 minutes)
- Write 200-300 word reflection:
  - What specification details mattered most?
  - How did reusable skills accelerate development?
  - What would you improve?
  - What new skills would benefit future work?

### Acceptance: Your Capstone Works When

- ✅ Scene loads and renders at 60+ FPS
- ✅ All acceptance tests pass
- ✅ Human approaches robot → robot responds
- ✅ ROS 2 receives interaction events
- ✅ Multiple interactions trigger reliably
- ✅ Code is organized and documented
- ✅ Reflection captures design decisions

---

## Try With AI

**Exploration 1: Specification Refinement**

Ask AI: "Looking at my acceptance tests, what edge cases might I have missed? How would I test them?"

Implement edge case tests:
- Avatar gets stuck (collision issues)
- Multiple interactions overlapping
- Connection lost to ROS 2

**Exploration 2: Production Readiness**

Ask AI: "What would be necessary to move this simulation to deployment on real robots? What changes to the code/architecture?"

Discuss:
- Current simulation assumptions vs reality
- Real-time requirements changes
- Sensor integration

**Exploration 3: Research Application**

Ask AI: "If I were studying human-robot proxemics (personal space) using this system, what metrics would I track? How would I export data for analysis?"

Design study:
- What measurements matter (distance, duration, gaze)?
- How to instrument the system for data collection?
- What ROS topics would researchers subscribe to?

---

## Capstone Complete

**You've built**:
- ✅ ROS 2 ↔ Unity bridge (communication foundation)
- ✅ URDF import and visualization (model transport)
- ✅ Photorealistic environment (credible context)
- ✅ Animated human avatar (interactive partner)
- ✅ Interaction scripting (behavior)
- ✅ ROS 2 integration (robotics connection)
- ✅ Reusable patterns (organizational knowledge)
- ✅ Specification-driven capstone (design methodology)

**You've learned**:
- How AI and humans collaborate to build complex systems
- Why specifications matter in the agentic era
- How to architect interactive systems
- The value of reusable components and patterns
- How to validate systems against clear criteria

---

## Chapter 5 Complete

You've mastered high-fidelity rendering and human-robot interaction in Unity. Your HRI system is production-ready—it could form the basis of actual human-robot interaction research.

From here:
- **Chapter 6**: Add sensor visualization and data display
- **Deployment**: Move HRI scenarios to real robots
- **Research**: Design studies using HRI system for data collection

---

## Try With AI

**Beyond the Book**

Ask AI: "What are the next frontiers in HRI simulation? (hand tracking, gaze, speech, emotion recognition, etc.)"

Start building one advanced feature:
- Eye gaze toward human
- Hand gesture recognition
- Emotional expression on robot face
- Voice interaction

You now have the foundation to extend into any direction.

---

[Back to Chapter 5 Overview](index.md)
