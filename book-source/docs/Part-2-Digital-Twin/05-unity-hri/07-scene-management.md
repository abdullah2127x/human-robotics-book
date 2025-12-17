---
title: "Lesson 7: HRI Scene Management Patterns"
chapter: 5
lesson: 7
proficiency_level: B2
learning_objectives:
  - "Identify and document recurring HRI patterns"
  - "Design reusable scene management framework"
  - "Create templatable prefabs for HRI scenarios"
  - "Document extensibility for new robot types"
estimated_time: "90 minutes"
generated_by: content-implementer v1.0.0
created: 2025-12-16
version: 1.0.0
---

# Lesson 7: HRI Scene Management Patterns

## Introduction

You've built an HRI system: bridge, environment, avatars, interactions, ROS integration. Lessons 1-6 created specific solutions. Now, Lesson 7 encodes these solutions as reusable intelligence.

Think of this as pattern recognition and extraction. What's identical across HRI scenarios? What's customizable? This becomes your reusable framework.

This lesson demonstrates Layer 3: Intelligence Design. You'll document patterns that will accelerate your capstone (Lesson 8) and any future HRI work.

**Estimated time**: 90 minutes
**Concept density**: Pattern documentation (not traditional concepts)

---

## Recurring Patterns in Lessons 1-6

### Pattern 1: Scene Hierarchy Structure

Every HRI scene needs:
```
HRIScene
├── Lighting (three-point lighting setup)
├── Environment (photorealistic setting)
├── Actors
│   ├── Robot (URDF import + materials)
│   └── Human (Animated avatar)
├── Interaction (Detection + Events)
└── ROS Bridge (Publishers + Subscribers)
```

**What's identical**: This structure applies to living room, office, warehouse—any HRI scenario.
**What's customizable**: Furniture, robot model, avatar appearance.

### Pattern 2: Interaction State Machine

Every interaction needs states:
- Idle (nothing happening)
- Detected (human in range)
- Active (responding)
- Completed (back to idle)

**What's identical**: State transitions, timing, events.
**What's customizable**: Specific animations, ROS topic names.

### Pattern 3: ROS Integration Points

Every scenario publishes/subscribes:
- **Publish**: Interaction events (what happened)
- **Subscribe**: Robot state (where is robot, what is it doing)

**What's identical**: Message flow pattern, timestamps, serialization.
**What's customizable**: Message types, topic names, robot-specific state.

---

## Designing the HRI Skill

Create `.claude/skills/unity-hri-interaction/SKILL.md`:

```markdown
# unity-hri-interaction Skill

## Overview
Orchestrates human-robot interaction scenarios in Unity with real-time ROS 2 communication.

## When to Use
- Setting up new HRI research scenarios
- Adding human-robot dynamics to existing simulations
- Prototyping interaction studies before real-world testing

## Prerequisites
- ROS 2 Humble (or compatible)
- Unity 2022.3 LTS
- ROS-TCP-Connector bridge operational
- URDF models available

## Scene Template

Every HRI scene follows this structure:

```
HRIScene
├── [Lighting] Three-point setup
│   ├── KeyLight (intensity 1.2, warm)
│   ├── FillLight (intensity 0.6, cool)
│   └── BackLight (intensity 0.3, white)
├── [Environment] Scenario-specific
│   ├── Walls, floor, ceiling (photorealistic)
│   └── Furniture (imported models or modeled)
├── [Actors]
│   ├── RobotModel (imported URDF, tagged "Robot")
│   └── HumanAvatar (animated, tagged "Human")
├── [Interaction]
│   ├── InteractionZone (sphere trigger, 1.5-2.0m radius)
│   └── EventSystem (publishes to ROS 2)
└── [ROS]
    ├── RosPublisher (publishes InteractionEvent)
    └── RosSubscriber (subscribes to /robot/joint_states)
```

## Implementation Steps

### 1. Create Scene
- File → New Scene → Save as `SceneName.unity`
- Set up lighting using three-point pattern

### 2. Add Environment
- Model or import furniture (grouped under "Environment" folder)
- Apply materials (walls matte, surfaces varied)
- Enable static batching for performance

### 3. Import Humanoid Robot
- Robotics → Import URDF → Select URDF file
- Apply materials, verify lighting
- Add collider for interaction detection

### 4. Add Human Avatar
- Import rigged humanoid (Mixamo or equivalent)
- Create Animator controller with states: Idle, Walking, Gesturing
- Add AvatarController script for input handling

### 5. Create Interaction System
- Add InteractionZone trigger (sphere, 1.5-2.0m radius)
- Attach InteractionZone.cs script
- Connect to RosPublisher for event publishing

### 6. Set Up ROS Communication
- Create ROS Manager GameObject
- Attach RosPublisher.cs for publishing events
- Attach RosSubscriber.cs for robot state updates
- Configure topic names matching your ROS 2 system

## Customization Points

### Add New Robot Type
1. Import URDF (different robot model)
2. Create interaction zone with appropriate radius
3. Configure joint state subscriber for that robot's topic
4. Modify interaction animations for that robot's actuators

### Change Environment
1. Replace furniture models (different scenario)
2. Adjust lighting for environment mood
3. Reposition interaction zones
4. Update materials for environment aesthetics

### Add New Interaction Types
1. Create new animator state for interaction
2. Extend InteractionZone.cs with new event types
3. Publish new message field to ROS 2
4. Subscribe to robot response (if applicable)

## Performance Guidelines

- Target 60+ FPS for interactive scenarios
- Use static batching for environment (~50% performance gain)
- Limit active animators (one human + one robot typically)
- Publish ROS events sparingly (not every frame)
- Use LOD groups for distant objects

## Message Definitions

### InteractionEvent (Published)
```
int64 timestamp_ms
geometry_msgs/Point human_position
geometry_msgs/Point robot_position
string interaction_type
float64 confidence
```

### JointState (Subscribed)
```
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
```

## Testing Checklist

- [ ] Scene loads at 60+ FPS
- [ ] Human avatar walks smoothly
- [ ] Robot visible with proper materials
- [ ] Interaction triggers within 1.5m
- [ ] ROS 2 messages published correctly
- [ ] No message flooding (low frequency)
- [ ] Interaction animations play smoothly
- [ ] Can run multiple interactions sequentially

## Extension Examples

### Multi-Robot Scenario
Create multiple InteractionZones, one per robot
Subscribe to each robot's joint_states topic separately

### Multi-Human Scenario
Support multiple human avatars
Create separate interaction zones for each human-robot pair

### Complex Interaction
Extend interaction state machine for multi-turn exchanges
Implement human gesture recognition
Sequence robot responses based on human input

## Troubleshooting

**Interaction doesn't trigger:**
- Verify InteractionZone radius: ~1.5m
- Check colliders on both robot and human
- Ensure tags ("Robot", "Human") match script expectations

**ROS messages not publishing:**
- Verify bridge connection (Lesson 1 setup)
- Check topic name in script vs ROS 2 system
- Monitor: `ros2 topic list` and `ros2 topic echo`

**Poor performance:**
- Enable static batching
- Reduce mesh complexity (LOD groups)
- Check draw calls: Window → Analysis → Profiler

**Animations look wrong:**
- Increase transition duration (0.2 → 0.4s)
- Adjust animation speed multipliers
- Verify animator parameters match script calls
```

---

## Hands-On: Apply Pattern to New Scenario

### Task 1: Document Your Patterns (15 minutes)

Review Lessons 1-6. Write down:
1. **Identical elements** across scenarios:
   - Lighting setup (always three-point)
   - Scene hierarchy (always same structure)
   - Interaction zones (always sphere trigger)

2. **Customizable elements**:
   - Furniture (living room vs office vs warehouse)
   - Robot appearance (different URDF models)
   - Interaction types (approach, gesture, handover)

### Task 2: Create Reusable Prefab (20 minutes)

Create a prefab template for quick scene setup:

1. Create new scene: `Scenes/HRI_Template.unity`
2. Set up standard lighting (KeyLight, FillLight, BackLight)
3. Create empty Environment folder
4. Create Actors folder with placeholders for robot/human
5. Create Interaction zone with InteractionZone.cs pre-attached
6. Save entire hierarchy as prefab: `Prefabs/HRISceneTemplate`

Now new HRI scenarios can instantiate this prefab as starting point.

### Task 3: Test Reusability (20 minutes)

Create a kitchen scenario using your template:

1. Create new scene
2. Instantiate HRISceneTemplate prefab
3. Add kitchen furniture (table, stove, counters)
4. Adjust lighting for kitchen setting
5. Import different robot URDF
6. Configure interaction for kitchen scenario

Verify:
- ✅ Template reduces setup time (vs building from scratch)
- ✅ Customization is straightforward (just swap furniture)
- ✅ Core systems work identically
- ✅ Performance maintained

### Checkpoint: Reusable Pattern Documented

Verify:
- ✅ Skill.md documents recurring patterns
- ✅ Template prefab created and tested
- ✅ New scenario created quickly using template
- ✅ Customization clear and straightforward

---

## Try With AI

**Exploration 1: Advanced Patterns**

Ask AI: "What patterns from complex HRI systems (multi-agent, hierarchical control, learning-based) could we encode as reusable skills?"

Discuss:
- Behavior trees for complex interactions
- Learning systems that adapt to human feedback
- Multi-agent coordination

**Exploration 2: Scaling Beyond Unity**

Ask AI: "How would the HRI patterns from this skill transfer to other game engines or simulation platforms? What's engine-specific vs universally applicable?"

Think about:
- Which patterns are Unityspecific (animator, prefabs)
- Which are universal (interaction state machines, ROS integration)

---

[Next: Lesson 8 - Complete HRI Demonstration](08-capstone-hri-demo.md)
