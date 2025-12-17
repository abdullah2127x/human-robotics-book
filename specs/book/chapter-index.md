# Chapter Index: Physical AI & Humanoid Robotics

**Version:** 1.0.0
**Last Updated:** 2025-12-16

This document is the authoritative reference for all chapters in the book.

---

## Part 1: The Robotic Nervous System (ROS 2)

**Part Focus:** Middleware for robot control
**Cognitive Load:** Light → Moderate
**Scaffolding:** Heavy

| Chapter | Title | Key Concepts | Tier |
|---------|-------|--------------|------|
| 01 | ROS 2 Nodes, Topics, and Services | Nodes, Topics, Publishers, Subscribers, Services, QoS | B1 |
| 02 | Bridging Python Agents to ROS Controllers (rclpy) | rclpy client library, async/await patterns, action clients, message types | B1 |
| 03 | Understanding URDF for Humanoids | URDF syntax, links, joints, visual/collision geometry, inertial properties | B1 |

---

## Part 2: The Digital Twin (Gazebo & Unity)

**Part Focus:** Physics simulation and environment building
**Cognitive Load:** Moderate
**Scaffolding:** Moderate

| Chapter | Title | Key Concepts | Tier |
|---------|-------|--------------|------|
| 04 | Simulating Physics, Gravity, and Collisions in Gazebo | Gazebo architecture, physics engines, world files, model spawning, collision detection | B1-B2 |
| 05 | High-Fidelity Rendering and Human-Robot Interaction in Unity | Unity-ROS 2 bridge, high-fidelity rendering, human avatars, interaction scripting | B2 |
| 06 | Simulating Sensors - LiDAR, Depth Cameras, and IMUs | Sensor plugins, LiDAR point clouds, depth image generation, IMU data, sensor noise models | B2 |

---

## Part 3: The AI-Robot Brain (NVIDIA Isaac)

**Part Focus:** Advanced perception and training
**Cognitive Load:** Moderate → Heavy
**Scaffolding:** Light-Moderate

| Chapter | Title | Key Concepts | Tier |
|---------|-------|--------------|------|
| 07 | NVIDIA Isaac Sim - Photorealistic Simulation and Synthetic Data | Isaac Sim architecture, domain randomization, synthetic data generation, Replicator, Omniverse | B2-C1 |
| 08 | Isaac ROS - Hardware-Accelerated VSLAM and Navigation | Visual SLAM, stereo vision, NVIDIA CUDA acceleration, localization, mapping | B2-C1 |
| 09 | Nav2 - Path Planning for Bipedal Humanoid Movement | Nav2 stack, costmaps, planners, behavior trees, bipedal gait considerations | B2-C1 |

---

## Part 4: Vision-Language-Action (VLA)

**Part Focus:** The convergence of LLMs and Robotics
**Cognitive Load:** Heavy
**Scaffolding:** Light (student-driven)

| Chapter | Title | Key Concepts | Tier |
|---------|-------|--------------|------|
| 10 | Voice-to-Action - Using OpenAI Whisper for Voice Commands | Speech recognition, Whisper API, intent parsing, ROS 2 integration, real-time audio streaming | C1 |
| 11 | Cognitive Planning - LLMs Translating Language to ROS 2 Actions | Task decomposition, action primitives, LLM prompting for robotics, action sequence generation, error recovery | C1-C2 |
| 12 | Capstone Project - The Autonomous Humanoid | Full integration, voice → plan → navigate → perceive → act | C2 |

---

## Chapter Dependencies

```
Chapter 01 (ROS 2 Basics)
    ↓
Chapter 02 (Python + ROS 2)
    ↓
Chapter 03 (URDF)
    ↓
┌───┴───┐
│       │
Chapter 04 (Gazebo)    Chapter 07 (Isaac Sim)
    ↓                       ↓
Chapter 05 (Unity)     Chapter 08 (VSLAM)
    ↓                       ↓
Chapter 06 (Sensors)   Chapter 09 (Nav2)
    │                       │
    └───────┬───────────────┘
            ↓
      Chapter 10 (Voice)
            ↓
      Chapter 11 (LLM Planning)
            ↓
      Chapter 12 (Capstone)
```

---

## Complexity Tier Legend

| Tier | Description | Concept Cap | Scaffolding |
|------|-------------|-------------|-------------|
| B1 | Intermediate Foundation | 5-7 concepts | Heavy guidance |
| B2 | Intermediate Application | 7-10 concepts | Moderate guidance |
| C1 | Advanced Integration | 10-12 concepts | Light guidance |
| C2 | Professional Mastery | No limit | Student-driven |

---

## Hands-On Exercises by Chapter

| Chapter | Primary Exercise |
|---------|-----------------|
| 01 | Create first ROS 2 node, publish/subscribe to topics, call services |
| 02 | Build a Python agent that controls a simulated robot arm |
| 03 | Create URDF model of a humanoid torso and arm |
| 04 | Spawn humanoid in Gazebo world, configure physics, test collisions |
| 05 | Build Unity scene with humanoid robot, implement basic HRI scenario |
| 06 | Add sensors to humanoid, visualize sensor data in RViz, record sensor bags |
| 07 | Generate synthetic dataset for object detection, train model |
| 08 | Implement VSLAM on humanoid, visualize 3D map construction |
| 09 | Configure Nav2 for humanoid, implement obstacle avoidance |
| 10 | Build voice command pipeline, implement wake word + command recognition |
| 11 | Implement LLM planner that converts commands to ROS 2 action sequences |
| 12 | Full capstone: voice → plan → navigate → perceive → act |

---

## Capstone Project Requirements (Chapter 12)

The capstone project integrates all learning:

1. **Voice Input:** Receive voice command via Whisper
2. **Cognitive Planning:** Plan task sequence using LLM
3. **Navigation:** Navigate environment using Nav2 (avoid obstacles)
4. **Perception:** Identify target object using computer vision
5. **Manipulation:** Manipulate object to complete task

**Deliverables:**
- Working simulation
- Documentation
- Demo video

**Success Criteria:**
- End-to-end autonomous task completion from voice command to physical action
