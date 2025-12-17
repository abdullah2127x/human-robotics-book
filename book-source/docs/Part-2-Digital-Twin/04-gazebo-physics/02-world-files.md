---
sidebar_position: 2
title: "Lesson 2: Creating and Modifying World Files"
description: "Create custom SDF world files with physics configuration, gravity, and collision geometry."
---

# Lesson 2: Creating and Modifying World Files

## Learning Objectives

By completing this lesson, you will:
- Create SDF world files from scratch
- Configure physics engines and understand their tradeoffs
- Set gravity and friction parameters
- Place objects using pose elements
- Understand collision geometry vs visual geometry

**Estimated time**: 120 minutes

---

## What You're Building

A world file is a blueprint for simulation. Instead of arranging objects in a 3D editor, you write XML that describes:
- What physics engine to use
- Where objects exist in space
- What forces act on the world (gravity)
- How objects interact (friction, contact)

By writing worlds by hand (not copying templates), you internalize how simulation environment choices affect behavior.

---

## The SDF World Template

Here's a working starting point. Type this into a new file called `my_first_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="training_ground">
    <!-- Physics Configuration -->
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Gravity: Standard Earth gravity pointing downward -->
    <gravity>0 0 -9.81</gravity>

    <!-- Scene Settings: Lighting and Background -->
    <scene>
      <ambient>0.4 0.4 0.4</ambient>
      <background>0.7 0.7 0.7</background>
      <shadows>true</shadows>
    </scene>

    <!-- Sun (Directional Light) -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>5 5 5 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane: Large flat box acting as terrain -->
    <model name="ground_plane">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_collision">
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
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="ground_visual">
          <geometry>
            <box>
              <size>100 100 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

**What each section does**:
- **Physics**: ODE engine with 1ms timesteps (1000 Hz simulation)
- **Gravity**: Vector pointing down (-Z direction) with Earth magnitude
- **Scene**: Ambient lighting and shadow rendering
- **Light**: Sun source for realistic illumination
- **Ground**: Static box forming terrain (won't fall or move)

Test this world:

```bash
gz sim -r my_first_world.sdf
```

You should see a gray ground plane with lighting. Nothing falls because there's nothing except the ground.

---

## Understanding Physics Engines

### The Three Engine Options

| Engine | Speed | Stability | Best For |
|--------|-------|-----------|----------|
| **ODE** | Slow | Moderate | Learning; simple simulations |
| **Bullet** | Fast | Variable | Complex objects; industrial use |
| **DART** | Medium | High | Humanoid robots; precise contact |

### Why DART for Humanoids

DART handles contact forces more precisely than ODE or Bullet. When your humanoid stands and balances, precise contact modeling is critical. ODE sometimes reports false contacts; Bullet sometimes loses contact during micro-impacts. DART tracks contacts robustly.

**Tradeoff**: DART is slightly slower. For fast iteration, ODE is acceptable. For final validation, DART is preferred.

---

## Exercise 1: Create Three Physics Worlds

Create three separate world files, each using a different physics engine. This exposes you to configuration differences.

### World 1: ODE Physics

Create `world_ode.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="ode_world">
    <physics name="ode_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.4 0.4 0.4</ambient>
      <background>0.7 0.7 0.7</background>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>5 5 5 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>100 100 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>100 100 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Falling box to test physics -->
    <model name="falling_box">
      <pose>0 0 2 0 0 0</pose>
      <link name="box_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx>
            <iyy>0.01</iyy>
            <izz>0.01</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### World 2: Bullet Physics

Create `world_bullet.sdf`, identical except change physics type:

```xml
<physics name="bullet_physics" default="true" type="bullet">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

### World 3: DART Physics

Create `world_dart.sdf`, with DART configuration:

```xml
<physics name="dart_physics" default="true" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

### Test Each World

```bash
# Terminal 1: ODE
gz sim -r world_ode.sdf

# Terminal 2 (new): Bullet
gz sim -r world_bullet.sdf

# Terminal 3 (new): DART
gz sim -r world_dart.sdf
```

**Observation task**: Watch the red box fall in each world. Note:
- [ ] **Speed**: How fast does it fall?
- [ ] **Landing**: How does it settle after hitting ground?
- [ ] **Stability**: Does it wobble, or settle smoothly?

Record your observations. These subtle differences matter for humanoid control later.

---

## Gravity and Friction Configuration

### Modifying Gravity

Gravity is a global force affecting all objects. Modify it in your world:

```xml
<!-- Earth-like -->
<gravity>0 0 -9.81</gravity>

<!-- Mars-like (lower gravity) -->
<gravity>0 0 -3.71</gravity>

<!-- 2x Earth gravity (for stability testing) -->
<gravity>0 0 -19.62</gravity>

<!-- Sideways gravity (for testing tilted terrain) -->
<gravity>0 -9.81 0</gravity>
```

**Exercise**: Create `world_low_gravity.sdf` with Mars gravity. Spawn a box and observe how falling behavior changes.

### Friction Parameters

Friction determines how objects slide and grip surfaces. Two parameters: **mu** (primary friction) and **mu2** (secondary friction).

```xml
<!-- High friction: Rubber on rubber -->
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>
      <mu2>1.0</mu2>
    </ode>
  </friction>
</surface>

<!-- Low friction: Ice on ice -->
<surface>
  <friction>
    <ode>
      <mu>0.1</mu>
      <mu2>0.1</mu2>
    </ode>
  </friction>
</surface>

<!-- Realistic concrete -->
<surface>
  <friction>
    <ode>
      <mu>0.8</mu>
      <mu2>0.8</mu2>
    </ode>
  </friction>
</surface>
```

---

## Exercise 2: World with Multiple Objects

Create `training_world.sdf` with diverse geometry:

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="training">
    <physics name="default_physics" default="true" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.4 0.4 0.4</ambient>
      <background>0.7 0.7 0.7</background>
    </scene>

    <light type="directional" name="sun">
      <pose>5 5 5 0 0 0</pose>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground with high friction -->
    <model name="ground">
      <static>true</static>
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
                <mu>0.9</mu>
                <mu2>0.9</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>100 100 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Wall 1: Box wall -->
    <model name="wall_box">
      <static>true</static>
      <pose>0 5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Sphere: Bouncy object -->
    <model name="sphere">
      <pose>0 0 1 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.0008</ixx>
            <iyy>0.0008</iyy>
            <izz>0.0008</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.3</mu>
                <mu2>0.3</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>1 1 0 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Cylinder: Different geometry type -->
    <model name="cylinder">
      <pose>-1 -1 0.2 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx>
            <iyy>0.01</iyy>
            <izz>0.01</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Success Criteria for This Exercise

- [ ] World file loads without errors
- [ ] All objects are visible
- [ ] Objects fall due to gravity
- [ ] Sphere bounces when it falls (contact works)
- [ ] Cylinder can be pushed and slides (friction works)
- [ ] No objects sink through ground

Test it:

```bash
gz sim -r training_world.sdf
```

In the Gazebo UI, you can click objects and drag them to test their behavior.

---

## Understanding Pose: Position and Orientation

Every object has a `<pose>` element specifying where it is in space:

```xml
<pose>x y z roll pitch yaw</pose>
```

**Position** (first 3 values): meters in 3D space
- `x`: Forward/backward (+ toward you)
- `y`: Left/right (+ to the right)
- `z`: Up/down (+ upward)

**Orientation** (last 3 values): rotation angles in radians
- `roll`: Rotation around X axis (tilting forward/back)
- `pitch`: Rotation around Y axis (nodding up/down)
- `yaw`: Rotation around Z axis (turning left/right)

Example:

```xml
<!-- Object at origin, no rotation -->
<pose>0 0 0 0 0 0</pose>

<!-- Object 1m forward, 0.5m up, tilted 45° forward -->
<pose>1 0 0.5 1.5708 0 0</pose>
<!-- Note: 1.5708 radians ≈ 90 degrees -->

<!-- Object rotated 90° around Z axis (turned left) -->
<pose>0 0 0 0 0 1.5708</pose>
```

---

## Collision Geometry vs Visual Geometry

A model has TWO geometries:
1. **Collision geometry**: Used for physics (detecting impacts, calculating forces)
2. **Visual geometry**: Used for rendering (what you see)

Usually they're identical, but not always:

```xml
<model name="example">
  <link name="link">
    <!-- Physics uses this -->
    <collision name="collision">
      <geometry>
        <box>
          <size>0.1 0.1 0.1</size>
        </box>
      </geometry>
    </collision>

    <!-- Graphics renders this -->
    <visual name="visual">
      <geometry>
        <box>
          <size>0.1 0.1 0.1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.9 0.2 0.2 1</ambient>
      </material>
    </visual>
  </link>
</model>
```

**Why separate them?**
- Performance: Collision geometry can be simpler (fewer polygons) than visual
- Accuracy: Collision might be a box while visual is detailed mesh
- Debugging: Visualize collision shapes separately from appearance

---

## Try With AI

**Prompt 1: Physics Tradeoffs**
```
I'm choosing between ODE, Bullet, and DART physics engines for
simulating a humanoid robot. The humanoid needs precise foot contact
detection for balance control. Which engine would you recommend and why?
What are the performance tradeoffs?
```

**Prompt 2: Friction for Movement**
```
A humanoid robot needs to walk on ground with coefficient of friction
0.8. If I made friction too high (1.5), what would happen to walking?
If I made it too low (0.1), what would happen? Why?
```

**Prompt 3: Custom World Design**
```
I'm creating a training ground for a humanoid. What elements should I
include? Ground, walls, obstacles... what makes a good simulation
environment for testing balance?
```

---

**Next: Proceed to Lesson 3: Spawning and Controlling Models**

In Lesson 3, you'll stop manually writing world files and start dynamically spawning models into running simulations using ROS 2 services. You'll encounter failures and work collaboratively to debug them.
