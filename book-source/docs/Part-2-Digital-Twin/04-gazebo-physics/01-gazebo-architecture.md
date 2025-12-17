---
sidebar_position: 1
title: "Lesson 1: Gazebo Architecture and Ecosystem"
description: "Understand Gazebo's client-server architecture, plugin system, and ROS 2 integration."
---

# Lesson 1: Gazebo Architecture and Ecosystem

## Learning Objectives

By completing this lesson, you will:
- Understand how Gazebo separates server computation from client visualization
- Recognize how plugins extend Gazebo's capabilities
- Interpret SDF (Simulation Description Format) files
- Explore the ROS 2 bridge that connects controllers to simulations

**Estimated time**: 90 minutes

---

## The Architecture You'll Build Understanding Of

Gazebo isn't a monolithic program. It's a client-server system where the simulation engine (server) and visualization interface (client) are separate processes. This separation is deliberately designed—your control algorithms communicate with the server, not the visual display. Understanding this architecture prevents confusion when you later build controllers that don't directly interact with what you see on screen.

### Core Components

**Gazebo Server (gzserver)**
- Runs physics simulation
- Processes sensor calculations
- Manages model states
- Publishes simulation data via ROS 2 topics

**Gazebo Client (gzclient)**
- Renders 3D visualization
- Displays simulation from your perspective
- Sends user commands (moving camera, spawning objects)
- Subscribes to server state for visualization

**Plugin System**
- Physics engines (ODE, Bullet, DART)
- Sensor implementations (camera, lidar, contact)
- System components (gravity, wind effects)

### Why This Matters

When your humanoid robot sits and falls over in simulation but works fine on hardware, understanding this architecture helps you diagnose the problem:
- Does the issue exist in the physics simulation (server problem)?
- Or in how you're commanding the robot (controller problem)?
- Or in sensor data processing (plugin problem)?

---

## Exploration: Running Your First Simulation

Before we explain Gazebo, let's experience it. This hands-on exploration builds intuition that explanation reinforces.

### Step 1: Launch a Pre-Built World

Start a basic simulation environment:

```bash
# Terminal 1: Launch Gazebo with a simple world
gz sim -r empty.sdf
```

This command starts both gzserver and gzclient in a single terminal. You'll see a blank 3D scene—the empty world has no objects yet.

**What you're observing**:
- The 3D viewport shows the simulation world
- The world contains only a ground plane (invisible until something collides with it)
- Gravity is active but you can't see it

### Step 2: Inspect a Real World File

Open a text editor and examine Gazebo's included world files:

```bash
# Find Gazebo's installed world files
find /usr/share/gz/gz-sim-7/worlds/ -name "*.sdf" 2>/dev/null | head -5
```

Pick one (e.g., `/usr/share/gz/gz-sim-7/worlds/empty.sdf`) and open it in a text editor:

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="empty">
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <scene>
      <ambient>0.4 0.4 0.4</ambient>
      <background>0.7 0.7 0.7</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>5 5 5 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
  </world>
</sdf>
```

**What you're reading**:
- Line 1: XML declaration (required for all SDF files)
- Line 2: `<sdf>` root element (wraps entire simulation definition)
- Lines 3-8: Physics configuration (type=ODE, timestep 0.001s)
- Lines 10-14: Scene settings (lighting, background color)
- Lines 16-23: Light source definition (sun casting shadows)

Each element—physics, light, geometry—is a plugin that Gazebo loads at startup.

### Step 3: Modify and Observe

Edit the empty.sdf file you found and change gravity:

```xml
<!-- Add this inside <world>, before or after <physics> -->
<gravity>0 0 -15.0</gravity>
```

Launch Gazebo pointing to your modified file:

```bash
gz sim -r /path/to/your/modified/empty.sdf
```

**Observation task**: Gravity has changed from Earth default (9.81 m/s²) to 15 m/s² (stronger). You won't see a difference in the empty world yet, but when objects fall, they accelerate faster.

---

## Architecture Deep Dive

### Client-Server Separation

```
┌─────────────────────────────────────┐
│      Your ROS 2 Controller Node    │
│  (processes sensor data, sends      │
│   commands via topics/services)     │
└──────────────┬──────────────────────┘
               │ ROS 2 Topics/Services
               │
        ┌──────▼──────────────┐
        │   Gazebo Server     │
        │  (gzserver)         │
        │                     │
        │ • Runs physics      │
        │ • Calculates sensors│
        │ • Updates state     │
        │ • Publishes /clock  │
        └──────┬──────────────┘
               │ Transport Protocol
               │ (usually localhost)
        ┌──────▼──────────────┐
        │ Gazebo Client       │
        │ (gzclient)          │
        │                     │
        │ • Renders graphics  │
        │ • Shows visualization
        │ • Handles UI        │
        └─────────────────────┘
```

**Critical insight**: Your controller talks to the server via ROS 2, not to gzclient. Gzclient is purely for visualization. This is why simulation can run headless (without graphics) on a robot or compute cluster—gzserver doesn't need gzclient.

### The Plugin System

Plugins extend Gazebo's capabilities. Three categories matter for humanoid robotics:

**Physics Plugins** (choose one)
- **ODE** (Open Dynamics Engine): Stable, slower. Good for learning.
- **Bullet**: Faster, sometimes unstable with complex structures
- **DART**: Most stable for humanoids, best contact handling

**Sensor Plugins**
- Camera: Simulates vision sensors
- IMU: Simulates accelerometer + gyroscope
- Contact: Detects collisions at specific links
- Lidar: Simulates laser range finder

**System Plugins**
- Wind effects
- Environmental factors
- Custom force application

---

## SDF Format Fundamentals

SDF (Simulation Description Format) is XML for defining simulated worlds. URDF (which you learned in Chapter 3) can be loaded INTO Gazebo, but SDF is Gazebo's native format and supports simulation-specific features URDF doesn't.

### Minimal Valid SDF Document

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="my_world">
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <model name="box">
      <pose>0 0 1 0 0 0</pose>
      <link name="base_link">
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
            <ambient>0.9 0.2 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

**Element Breakdown**:
- `<world>`: Container for entire simulation
- `<physics>`: Simulation engine and timestep configuration
- `<model>`: An object that can move and collide
- `<link>`: The actual physical part (mass, collisions, visuals)
- `<inertial>`: Mass and rotational properties (required for physics)
- `<collision>`: Geometry used for physics calculations
- `<visual>`: Geometry used for rendering (for display)

---

## ROS 2 Integration

Gazebo publishes simulation data via ROS 2 topics. Your controllers subscribe to these topics and publish commands.

### Key ROS 2 Topics from Gazebo

```bash
# When Gazebo runs, these topics exist:
/clock                    # Simulation time (gzserver publishes here)
/model/[name]/pose        # Model position + orientation
/model/[name]/odometry    # Velocity + time derivatives
/joint_states             # All joint angles and velocities
/tf, /tf_static           # Transform frames
/gazebo/model_states      # All models in simulation
/gazebo/link_states       # All links with forces/torques applied
```

### Bridging URDF into Gazebo

Your humanoid URDF from Chapter 3 can be loaded into Gazebo:

```bash
# Method 1: Include URDF in SDF
# In your world.sdf:
<model name="humanoid">
  <pose>0 0 1 0 0 0</pose>
  <uri>file:///path/to/humanoid.urdf</uri>
</model>

# Method 2: Spawn URDF dynamically via service
ros2 service call /spawn_entity gazebo_msgs/SpawnEntity \
  "{name: humanoid, xml: $(cat humanoid.urdf)}"
```

---

## Practical Checkpoint: Examine a Working Setup

Gazebo provides example worlds with humanoids. Let's inspect one:

```bash
# Find Gazebo examples directory
find /usr/share/gz/gz-sim-*/examples -name "*.sdf" 2>/dev/null | head -10
```

Look for a world file with "humanoid" in the name. Open it and answer:

1. **How many models are in this world?** (Count `<model>` tags)
2. **What physics engine is used?** (Check `<physics type="...">`)
3. **How many links does the humanoid have?** (Open referenced URDF and count `<link>` tags)
4. **Are there any sensor plugins?** (Search for `<plugin>`)

Write your answers in a text file—this becomes your mental model of what you just learned.

---

## Try With AI

**Setup**: Open your AI tool (ChatGPT, Claude, or similar) and use these prompts to deepen understanding.

**Prompt 1: Architecture Explanation**
```
I'm learning Gazebo's client-server architecture. Explain why Gazebo
separates the server (physics engine) from the client (graphics).
Why is this separation valuable for robotics simulation?
```

After AI responds, ask yourself: "Did this explain something I didn't understand about gzserver and gzclient?"

**Prompt 2: Real-World Scenario**
```
A robot controller publishes joint commands to a ROS 2 topic. Where
in the Gazebo architecture (client, server, or both) does this command
get processed? Why?
```

Apply this question to a scenario: If your controller sends a command to move the humanoid's arm, trace the path: controller → ROS 2 topic → [where?].

**Prompt 3: Plugin System**
```
Gazebo has modular plugins for physics, sensors, and systems. Why would
a developer choose Bullet physics over ODE? What are the tradeoffs?
```

Consider: When building the humanoid controller later, which physics engine might be most important for balance?

---

**Next: Proceed to Lesson 2: Creating and Modifying World Files**

In Lesson 2, you'll move from understanding Gazebo architecture to building your own worlds. You'll write SDF files from scratch, exploring how physics engine choice, gravity configuration, and model placement affect simulation behavior.
