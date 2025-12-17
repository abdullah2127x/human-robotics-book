---
sidebar_position: 2
title: "Part 2: The Digital Twin"
description: "Build physics-accurate simulations using Gazebo and Unity for robot testing and validation"
---

# Part 2: The Digital Twin (Gazebo & Unity)

**Part Focus:** Physics simulation and environment building

**Part Purpose:** Create virtual proving grounds for robot development - test and iterate in simulation before physical deployment.

---

## Why Simulation?

Physical robots are expensive, fragile, and slow to iterate. Before deploying your humanoid to the real world, you need to:

- **Test safely** - No risk of hardware damage during experiments
- **Iterate quickly** - Modify and retest in minutes, not hours
- **Train at scale** - Generate thousands of scenarios for learning
- **Validate thoroughly** - Ensure behavior is correct before deployment

Simulation provides this virtual laboratory where you can break things, learn, and improve without physical consequences.

---

## What You'll Build

By the end of Part 2, you will have:

1. **Gazebo simulation** with your humanoid standing and balancing
2. **Unity HRI demo** with photorealistic human-robot interaction
3. **Complete sensor suite** (LiDAR, depth camera, IMU) publishing real data

These simulated environments become your testing grounds for Parts 3 and 4.

---

## Learning Outcomes

After completing Part 2, you will be able to:

- **LO-2.1:** Configure Gazebo physics simulations with custom parameters
- **LO-2.2:** Spawn and control humanoid robots in virtual environments
- **LO-2.3:** Set up Unity-ROS 2 bridge for high-fidelity visualization
- **LO-2.4:** Create interactive human-robot scenarios
- **LO-2.5:** Implement simulated sensors (LiDAR, depth camera, IMU)
- **LO-2.6:** Process and visualize sensor data in RViz
- **LO-2.7:** Record and playback sensor data using ROS 2 bags

---

## Chapters in This Part

### Chapter 4: Simulating Physics, Gravity, and Collisions in Gazebo

Bring your humanoid URDF from Part 1 into Gazebo Harmonic. Configure physics engines, tune collision parameters, and control simulated joints.

**Key Concepts:** Gazebo architecture, physics engines, world files, model spawning, collision detection

**Hands-On:** Spawn humanoid in Gazebo, configure physics, implement standing behavior

---

### Chapter 5: High-Fidelity Rendering and Human-Robot Interaction in Unity

Create photorealistic interactive scenarios using Unity. Bridge Unity with ROS 2 for real-time communication and visualization.

**Key Concepts:** Unity-ROS 2 bridge, URDF import, photorealistic rendering, human avatars, interaction scripting

**Hands-On:** Build office environment with human avatar, script proximity-based interaction

---

### Chapter 6: Simulating Sensors - LiDAR, Depth Cameras, and IMUs

Add perception to your humanoid. Simulate realistic sensors and process their data for navigation and awareness.

**Key Concepts:** Sensor plugins, LiDAR point clouds, depth images, IMU data, noise models, RViz visualization

**Hands-On:** Add all three sensor types, visualize in RViz, process sensor data for obstacle detection

---

## Prerequisites

Before starting Part 2, ensure you have:

- [x] Part 1 complete (all 3 chapters)
- [x] Humanoid URDF model from Chapter 3
- [x] ROS 2 Humble installed and working
- [x] Ubuntu 22.04 LTS (or WSL2 on Windows)
- [x] 16GB+ RAM recommended
- [x] NVIDIA GPU recommended (for Unity)

**New Requirements for Part 2:**
- Gazebo Harmonic (installed in Chapter 4)
- Unity 2022.3 LTS (installed in Chapter 5)
- ROS-TCP-Connector Unity package

---

## Cognitive Load: Moderate

Part 2 uses **moderate scaffolding** with guided exercises. You'll apply Part 1 knowledge independently:

| Chapter | Cognitive Load | New Concepts |
|---------|---------------|--------------|
| Chapter 4 | Moderate | 7 simulation concepts |
| Chapter 5 | Moderate | 8 Unity/HRI concepts |
| Chapter 6 | Moderate | 8 sensor concepts |

---

## Connection to Other Parts

Part 2 prepares you for:

- **Part 3 (Isaac):** Gazebo experience transfers to Isaac Sim
- **Part 3 (VSLAM):** Sensor data pipelines feed visual SLAM
- **Part 4 (VLA):** Simulated environments test voice commands

Your digital twin becomes the testing ground for all advanced features.

---

## Ready to Begin?

Start with [Chapter 4: Simulating Physics in Gazebo](./gazebo-physics/) to bring your humanoid URDF to life with realistic physics simulation.
