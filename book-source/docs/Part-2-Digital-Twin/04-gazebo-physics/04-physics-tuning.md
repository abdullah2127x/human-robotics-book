---
sidebar_position: 4
title: "Lesson 4: Physics Parameter Tuning"
description: "Tune physics parameters (timestep, damping, friction) to achieve stable humanoid simulation with AI collaboration."
---

# Lesson 4: Physics Parameter Tuning

## Learning Objectives

By completing this lesson, you will:
- Understand how physics timestep affects simulation accuracy and speed
- Configure damping parameters to control realistic motion
- Tune friction and contact parameters for stable standing
- Work with AI to iteratively refine parameters
- Recognize signs of physics instability and diagnose causes

**Estimated time**: 120 minutes

---

## The Core Physics Tradeoff

Physics simulation is a balancing act:
- **Smaller timestep** = More accurate but slower computation
- **Larger timestep** = Faster but less accurate
- **Higher damping** = Stable but sluggish movement
- **Lower damping** = Reactive but potentially unstable

Your humanoid won't stand realistically without tuning these parameters. Too much damping and it moves like a robot in molasses. Too little and it oscillates wildly.

---

## Timestep Configuration

The physics timestep is how often the simulation engine recalculates physics (integration step).

### Default Configuration

In your world file's `<physics>` element:

```xml
<physics name="default_physics" default="true" type="dart">
  <!-- Simulation advances by this amount each step -->
  <max_step_size>0.001</max_step_size>

  <!-- Run at real-time speed (1.0) or faster (e.g., 10.0 for 10x speed) -->
  <real_time_factor>1.0</real_time_factor>

  <!-- Number of times physics is recalculated per second -->
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

### Understanding These Parameters

**max_step_size** (default: 0.001 = 1 millisecond)
- Smaller values: More accurate but slower
- Typical range: 0.0001 to 0.01 seconds
- For humanoids: 0.001 is standard

**real_time_update_rate** (default: 1000 Hz)
- How many times per second physics recalculates
- Should equal 1 / max_step_size
- If set differently, Gazebo adjusts internally

### Exercise 1: Observe Timestep Effects

Create three worlds with different timesteps and observe falling behavior.

**world_dt_0001.sdf** (very accurate, slow):
```xml
<max_step_size>0.0001</max_step_size>
<real_time_update_rate>10000</real_time_update_rate>
```

**world_dt_001.sdf** (default, balanced):
```xml
<max_step_size>0.001</max_step_size>
<real_time_update_rate>1000</real_time_update_rate>
```

**world_dt_01.sdf** (fast, less accurate):
```xml
<max_step_size>0.01</max_step_size>
<real_time_update_rate>100</real_time_update_rate>
```

Observe:
- [ ] **Accuracy**: Does humanoid fall smoothly or jerkily?
- [ ] **Speed**: How fast is simulation running?
- [ ] **Stability**: Does humanoid settle cleanly or oscillate?

For humanoid standing, 0.001s is usually optimal.

---

## Damping Parameters

Damping reduces motion by dissipating energy—like air resistance or friction within joints.

### Per-Link Damping (in URDF)

Your humanoid URDF can specify damping for each link:

```xml
<link name="torso">
  <dynamics damping="0.1" friction="0.0"/>

  <inertial>
    <mass value="20"/>
    <inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>
```

### Per-Joint Damping (in URDF)

```xml
<joint name="hip_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_leg"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>

  <!-- Damping coefficient: higher = more resistance -->
  <dynamics damping="5.0" friction="0.0"/>
</joint>
```

### Damping Values Guide

| Value | Behavior | Use Case |
|-------|----------|----------|
| 0 | No damping, oscillates freely | Unrealistic |
| 0.1-1 | Light damping, responsive | Walking, running |
| 1-5 | Moderate damping, stable | Standing, reaching |
| 5+ | Heavy damping, slow | Delicate manipulation |

---

## Exercise 2: Test Damping Effects

Modify your humanoid URDF to vary damping:

```xml
<!-- High damping on torso (stable core) -->
<link name="torso">
  <dynamics damping="2.0" friction="0.0"/>
</link>

<!-- Moderate damping on legs -->
<link name="upper_leg_l">
  <dynamics damping="1.0" friction="0.0"/>
</link>

<!-- Low damping on arms -->
<link name="upper_arm_l">
  <dynamics damping="0.5" friction="0.0"/>
</link>
```

Spawn and observe:
- [ ] Does humanoid stand more stably with higher torso damping?
- [ ] Can you move arms if damping is too high?
- [ ] What happens if damping is too low?

---

## Friction and Contact Physics

Friction is the force that prevents objects from sliding.

### Material Friction Configuration

In your world file, define ground friction:

```xml
<model name="ground">
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>100 100 0.1</size>
        </box>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.8</mu>
            <mu2>0.8</mu2>
            <slip1>0</slip1>
            <slip2>0</slip2>
          </ode>
        </friction>
        <contact>
          <collide_bitmask>0xffff</collide_bitmask>
        </contact>
      </surface>
    </collision>
  </link>
</model>
```

### Friction Coefficient Meanings

- **mu = 0.0**: Frictionless (ice)
- **mu = 0.3-0.5**: Smooth surface (polished floor)
- **mu = 0.8-1.0**: Normal surface (concrete)
- **mu > 1.0**: High friction (rubber, carpet)

---

## Exercise 3: Friction Effects on Standing

Create three worlds with different ground friction and spawn humanoid in each:

**high_friction.sdf**: `<mu>1.5</mu>`
**normal_friction.sdf**: `<mu>0.8</mu>`
**low_friction.sdf**: `<mu>0.2</mu>`

Observe:
- [ ] High friction: Humanoid stands stable?
- [ ] Normal friction: What's the baseline?
- [ ] Low friction: Does humanoid slide/slip?

For humanoid standing, mu ≈ 0.8-1.0 is typical.

---

## Collaborative Tuning: Working with AI

You'll now use AI to iteratively find the right parameter values.

### Scenario: Humanoid Wobbles During Stance

**Your observation**: Humanoid trembles rather than standing still.

**Initial attempt** (fails):
```python
# URDF with all damping at 1.0
spawn_humanoid("humanoid.urdf", [0, 0, 1])
# Result: Humanoid wobbles, oscillations visible
```

### Role 1: AI as Teacher

**You ask**:
```
My humanoid is oscillating in stance instead of standing still.
It's like it's constantly micro-correcting position. What causes
oscillation in simulated robots?
```

**AI teaches**:
- "Oscillation happens when damping is too low"
- "Try increasing damping on main body joints"
- "Contact instability can also cause wobble"
- "Pattern: Start with conservative (high) damping, gradually reduce"

**What you learned**: A systematic approach you wouldn't find by trial-and-error.

### Role 2: AI as Student

**You refine**:
```
I increased damping on torso to 2.0 and legs to 1.5.
Now humanoid is stable but moves too slowly.
I need both stability AND responsiveness.
```

**AI adapts**:
- "Good observation. Solution: Different damping for different purposes"
- "High damping for torso (stability): 1.5-2.0"
- "Moderate damping for legs (movement): 0.8-1.0"
- "Low damping for arms (responsiveness): 0.3-0.5"
- "This creates a hierarchy"

**What happened**: AI learned your constraint and adapted.

### Role 3: AI as Co-Worker

**Iteration 1** (you adjust URDF):
```xml
<link name="torso">
  <dynamics damping="1.5" friction="0.0"/>
</link>
```
Result: Still oscillates.

**Iteration 2** (AI suggests):
"Oscillation might be contact-related. Try reducing max_step_size to 0.0005"

```xml
<max_step_size>0.0005</max_step_size>
<real_time_update_rate>2000</real_time_update_rate>
```
Result: More stable but slower.

**Iteration 3** (you balance):
"Try contact penetration tuning instead"
Result: Better stability at normal speed.

**Convergence**: Multi-parameter tuning required, neither solution was optimal alone.

---

## Practical Exercise: Tune Your Humanoid

Create a tuning script that tests different parameter combinations:

```python
#!/usr/bin/env python3

import time

def spawn_with_params(humanoid_urdf, damping_torso, damping_legs, ground_mu):
    """Spawn humanoid with specific parameters and record stability."""

    print(f"Testing: torso_damping={damping_torso}, "
          f"leg_damping={damping_legs}, ground_mu={ground_mu}")

    # Your spawning and measurement code here

    return "stable" or "unstable"

# Test matrix
test_cases = [
    (0.5, 0.3, 0.8),   # Low damping
    (1.0, 0.8, 0.8),   # Moderate damping
    (2.0, 1.5, 0.8),   # High damping
    (1.5, 0.8, 0.5),   # Normal damping, low friction
    (1.5, 0.8, 1.5),   # Normal damping, high friction
]

for torso, legs, mu in test_cases:
    result = spawn_with_params("humanoid.urdf", torso, legs, mu)
    print(f"  Result: {result}\n")
```

Document which combination produces:
- [ ] Stable standing (no oscillation)
- [ ] Responsive to commands (not sluggish)
- [ ] Realistic appearance

---

## Try With AI

**Prompt 1: Physics Tuning Strategy**
```
I'm tuning humanoid physics parameters. Where should I start:
timestep, damping, or friction? What's the systematic approach?
```

**Prompt 2: Your Specific Issue**
```
My humanoid stands okay but wobbles slightly. Timestep is already 0.001.
Should I increase damping or adjust friction? Which is easier to tune?
```

**Prompt 3: Performance vs Accuracy**
```
I have humanoid working well, but my control system needs 50 Hz response
time. My timestep is 0.001. How do I balance simulation accuracy with
controller speed?
```

---

**Next: Proceed to Lesson 5: Collision Detection and Contact Sensing**

In Lesson 5, you'll add sensors to your humanoid—specifically contact sensors on the feet. These provide feedback about whether the humanoid is touching ground, essential for balance control.
