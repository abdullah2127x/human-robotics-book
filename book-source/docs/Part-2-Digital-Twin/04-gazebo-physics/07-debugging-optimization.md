---
sidebar_position: 7
title: "Lesson 7: Debugging and Optimization"
description: "Create reusable skill for diagnosing and fixing physics simulation problems."
---

# Lesson 7: Debugging and Optimization

## Learning Objectives

By completing this lesson, you will:
- Categorize physics simulation problems systematically
- Build diagnostic decision trees for common issues
- Use RViz visualization for physics debugging
- Create gazebo-physics-debugging-skill for reuse
- Apply Persona + Questions + Principles to debugging skill

**Estimated time**: 90 minutes

---

## Why Debugging Is a Skill

In Lessons 1-6, you've run into problems: instability, contact failures, sensor issues. Instead of solving each ad-hoc, Lesson 7 encodes a *reusable debugging methodology* as a skill.

When similar problems arise in future projects, you have a diagnostic framework.

---

## Problem Categorization

Gazebo physics problems fall into categories. Each has symptoms, common causes, and fixes.

### Category 1: Penetration Issues

**Symptoms**:
- Objects sink through ground
- Feet penetrate below surface
- Bodies pass through walls

**Diagnostic Workflow**:
```
Object penetrating ground?
├─ First: Increase ground friction (mu=1.0)
│  └─ Still penetrating? → Reduce timestep (0.0005)
│     └─ Still? → Reduce contact penetration tolerance
│        └─ Still? → Check joint geometry/limits
└─ If friction breaks other behavior → Use contact penetration tuning
```

### Category 2: Instability / Oscillation

**Symptoms**:
- Humanoid trembles in stance
- Uncontrolled wobbling
- Continuous micro-motion

**Diagnostic Workflow**:
```
Humanoid oscillating?
├─ Increase damping on torso (1.5-2.0)
│  └─ Still? → Increase on legs (0.8-1.0)
│     └─ Still? → Reduce timestep (0.0005)
│        └─ Still? → Check gravity isn't exaggerated
└─ If too sluggish → Reduce damping incrementally
```

### Category 3: Performance Degradation

**Symptoms**:
- Simulation running slower than real-time
- Lag between command and response
- CPU maxed out

**Diagnostic Workflow**:
```
Simulation too slow?
├─ Check timestep (should be 0.001)
│  └─ If smaller → Increase to 0.001
│     └─ Still slow? → Reduce solver iterations
│        └─ Still? → Check model complexity
└─ Profile: Top CPU consumers usually model complexity
```

### Category 4: Sensor Failures

**Symptoms**:
- Contact sensors publish nothing
- IMU messages missing
- Camera not producing images

**Diagnostic Workflow**:
```
Sensor not publishing?
├─ First: Check topic exists
│  ros2 topic list | grep contact
│  └─ No topic? → Enable plugin in world file
│     └─ Still no? → Restart Gazebo + bridge
└─ Topic exists but no data?
   ├─ Check sensor update_rate > 0
   └─ Check collision happens (visual inspection)
```

---

## Building Decision Trees

Create `.claude/skills/gazebo-physics-debugging/SKILL.md` with complete diagnostic framework including:

- **Quick diagnostic flowchart**: Route user to right problem category
- **Penetration issues**: Visual inspection → friction → timestep → contact tuning → geometry
- **Instability issues**: Damping → friction → timestep → gravity check
- **Performance issues**: Timestep → solver iterations → model complexity → sensor rates
- **Sensor failures**: Topic existence → plugin configuration → sensor definition → connection
- **Debugging tools**: RViz visualization, Gazebo console logging, ROS 2 topic inspection
- **Common fixes checklist**: Ordered list of tuning steps
- **Testing**: Broken world scenarios for practice

---

## Practical Exercise: Diagnose Simulated Problems

Your skill documentation includes "broken" worlds for practice:

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="debug_test">
    <!-- PROBLEM 1: Penetration -->
    <physics type="ode">
      <max_step_size>0.01</max_step_size>
    </physics>

    <!-- PROBLEM 2: High gravity -->
    <gravity>0 0 -20</gravity>

    <!-- PROBLEM 3: No friction -->
    <model name="ground">
      <surface>
        <friction>
          <ode>
            <mu>0.1</mu>
          </ode>
        </friction>
      </surface>
    </model>

    <!-- Student task: Diagnose and fix all 3 problems -->
  </world>
</sdf>
```

Using your skill, students can:
1. Identify what's wrong (penetration, instability, or performance)
2. Apply diagnostic workflow
3. Fix issues using decision tree
4. Verify fixes work

Success: Student can explain what was wrong and how to fix each issue.

---

## Try With AI

**Prompt 1: Systematic Debugging**
```
I'm building a debugging methodology for Gazebo simulations. My
framework has categories: penetration, instability, performance,
sensors. Are there other problem categories I'm missing?
```

**Prompt 2: Your Specific Debug Case**
```
My humanoid simulates okay, but sometimes when it gets knocked over,
it penetrates the ground on recovery. It's intermittent. What causes
intermittent penetration versus consistent penetration?
```

**Prompt 3: Advanced Diagnostics**
```
Beyond visual inspection and log files, what tools can I use to
diagnose physics problems? Are there profilers for Gazebo?
```

---

**Next: Proceed to Lesson 8: Capstone — Humanoid Standing and Balancing**

In Lesson 8 (Layer 4 - Spec-Driven Integration), you'll write a specification for humanoid balance control, then compose the skills from Lessons 6-7 to implement it. This is where everything comes together.
