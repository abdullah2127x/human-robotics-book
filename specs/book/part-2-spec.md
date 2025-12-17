# Part 2 Specification: The Digital Twin (Gazebo & Unity)

**Version:** 1.0.0
**Created:** 2025-12-16
**Status:** Active
**Source:** book-spec.md, chapter-index.md

---

## Part Identity

**Part Number:** 2
**Part Title:** The Digital Twin
**Part Subtitle:** Physics Simulation and Environment Building

**Unifying Theme:** Virtual proving grounds for robot development - test and iterate in simulation before physical deployment.

**Part Purpose:** Application - create virtual environments where humanoid robots can be tested, trained, and validated safely and efficiently.

---

## Prerequisites

Students must have completed:
- **Part 1: ROS 2 Foundation** (all 3 chapters)
  - ROS 2 nodes, topics, services
  - Python rclpy development
  - URDF humanoid modeling

**Technical Requirements:**
- Ubuntu 22.04 LTS (or WSL2)
- ROS 2 Humble installed
- Gazebo Harmonic (will be installed in Chapter 4)
- Unity 2022.3 LTS with ROS 2 bridge (will be installed in Chapter 5)
- NVIDIA GPU recommended (for Unity rendering)
- 16GB+ RAM recommended

---

## Part-Level Learning Outcomes

After completing Part 2, students will be able to:

- **LO-2.1:** Configure and customize Gazebo physics simulation environments
- **LO-2.2:** Spawn and control humanoid robots in Gazebo worlds
- **LO-2.3:** Set up Unity-ROS 2 bridge for high-fidelity visualization
- **LO-2.4:** Create interactive human-robot scenarios in Unity
- **LO-2.5:** Implement simulated sensors (LiDAR, depth camera, IMU)
- **LO-2.6:** Process and visualize sensor data in RViz
- **LO-2.7:** Record and playback sensor data using ROS 2 bags

---

## Scaffolding Strategy

**Scaffolding Level:** Moderate
- Less hand-holding than Part 1
- Students expected to apply ROS 2 knowledge independently
- More open-ended exercises with multiple valid solutions
- AI collaboration for complex configurations

**Cognitive Load:** Moderate
- 7-10 concepts per chapter
- Building on Part 1 foundation
- Integration complexity increases

---

## Chapter Specifications

### Chapter 4: Simulating Physics, Gravity, and Collisions in Gazebo

**Chapter Focus:** Gazebo architecture, physics simulation, world building

**Proficiency Tier:** B1-B2 (Intermediate Foundation to Application)

**Core Concepts (7):**
1. Gazebo architecture (server, client, plugins)
2. Physics engine (ODE, Bullet, DART)
3. World files (SDF format)
4. Model spawning (spawn_entity service)
5. Collision detection (contact sensors)
6. Gravity and friction configuration
7. Gazebo-ROS 2 bridge

**Learning Outcomes:**
- **Eval-4.1:** Students configure Gazebo worlds with custom physics parameters
- **Eval-4.2:** Students spawn URDF models into Gazebo simulation
- **Eval-4.3:** Students implement collision detection and response
- **Eval-4.4:** Students control simulated humanoid via ROS 2 topics
- **Eval-4.5:** Students debug physics issues (penetration, instability)

**Hands-On Exercises:**
1. Create custom Gazebo world with ground plane, walls, objects
2. Spawn humanoid URDF from Part 1 into Gazebo
3. Configure physics parameters (gravity, friction, damping)
4. Implement contact sensor and collision callback
5. Control humanoid joints via ROS 2 joint trajectory controller
6. Capstone: Humanoid standing and balancing in simulation

**Estimated Duration:** 7-8 lessons

---

### Chapter 5: High-Fidelity Rendering and Human-Robot Interaction in Unity

**Chapter Focus:** Unity-ROS 2 integration, photorealistic rendering, HRI scenarios

**Proficiency Tier:** B2 (Intermediate Application)

**Core Concepts (8):**
1. Unity-ROS 2 bridge (ROS-TCP-Connector)
2. URDF import to Unity (URDF Importer package)
3. High-fidelity rendering (lighting, materials, shaders)
4. Human avatar animation (Mixamo, animation controllers)
5. Interaction scripting (C# Unity scripts)
6. Scene management (loading, transitions)
7. Real-time visualization (camera views, UI)
8. ROS 2 message publishing/subscribing from Unity

**Learning Outcomes:**
- **Eval-5.1:** Students set up Unity-ROS 2 communication bridge
- **Eval-5.2:** Students import and visualize URDF humanoid in Unity
- **Eval-5.3:** Students create photorealistic indoor environment
- **Eval-5.4:** Students implement human avatar with animations
- **Eval-5.5:** Students script human-robot interaction scenario
- **Eval-5.6:** Students publish Unity events to ROS 2 topics

**Hands-On Exercises:**
1. Install and configure Unity with ROS-TCP-Connector
2. Import humanoid URDF into Unity scene
3. Create indoor environment (living room, office, or warehouse)
4. Add human avatar with walking/gesturing animations
5. Script interaction: Human approaches robot, robot responds
6. Publish interaction events to ROS 2 topics
7. Capstone: Complete HRI demo with Unity visualization + ROS 2 control

**Estimated Duration:** 7-8 lessons

---

### Chapter 6: Simulating Sensors - LiDAR, Depth Cameras, and IMUs

**Chapter Focus:** Sensor simulation, data processing, visualization

**Proficiency Tier:** B2 (Intermediate Application)

**Core Concepts (8):**
1. Sensor plugins (Gazebo sensor system)
2. LiDAR simulation (ray casting, point cloud generation)
3. Depth camera simulation (depth image, RGB-D)
4. IMU simulation (accelerometer, gyroscope, orientation)
5. Sensor noise models (Gaussian noise, drift)
6. ROS 2 sensor message types (PointCloud2, Image, Imu)
7. RViz visualization (point clouds, images, TF)
8. ROS 2 bag recording and playback

**Learning Outcomes:**
- **Eval-6.1:** Students add LiDAR sensor to humanoid URDF
- **Eval-6.2:** Students configure depth camera with correct intrinsics
- **Eval-6.3:** Students add IMU sensor with realistic noise model
- **Eval-6.4:** Students visualize all sensor data in RViz
- **Eval-6.5:** Students process sensor data in Python node
- **Eval-6.6:** Students record and playback sensor data with ROS 2 bags

**Hands-On Exercises:**
1. Add 2D LiDAR to humanoid head (360° scan)
2. Add depth camera to humanoid (RGB-D like RealSense)
3. Add IMU to humanoid torso (orientation tracking)
4. Configure realistic noise models for all sensors
5. Create RViz configuration displaying all sensor data
6. Process LiDAR data to detect obstacles
7. Process depth camera to estimate distance to objects
8. Record sensor data bag, playback and analyze
9. Capstone: Humanoid with full sensor suite navigating environment

**Estimated Duration:** 8-9 lessons

---

## Part 2 Success Criteria

### Technical Success
- [ ] **SC-2.1:** Humanoid URDF from Part 1 loads and simulates in Gazebo
- [ ] **SC-2.2:** Physics simulation stable (no penetration, reasonable behavior)
- [ ] **SC-2.3:** Unity-ROS 2 bridge operational (bidirectional communication)
- [ ] **SC-2.4:** HRI scenario runs with human avatar and robot interaction
- [ ] **SC-2.5:** All three sensor types (LiDAR, depth, IMU) publishing data
- [ ] **SC-2.6:** Sensor data visualized correctly in RViz
- [ ] **SC-2.7:** ROS 2 bag recording and playback functional

### Pedagogical Success
- [ ] All lessons follow 4-layer teaching framework
- [ ] Show-then-explain pattern in every lesson
- [ ] "Try With AI" sections demonstrate AI collaboration
- [ ] Exercises have checkbox success criteria
- [ ] Capstone integrates chapter concepts

---

## Connection to Other Parts

### From Part 1
- URDF humanoid model → spawned in Gazebo (Chapter 4)
- ROS 2 topics/services → control simulated robot (Chapter 4-6)
- Python rclpy skills → sensor data processing (Chapter 6)

### To Part 3
- Gazebo simulation → Isaac Sim provides higher fidelity (Chapter 7)
- Sensor data pipelines → VSLAM input (Chapter 8)
- Simulated environment → Navigation testing (Chapter 9)

### To Part 4
- Sensor perception → object detection for manipulation (Chapter 10-12)
- Simulation environment → voice command testing (Chapter 10)

---

## Risk Mitigations

| Risk | Mitigation |
|------|------------|
| Gazebo installation complexity | Provide Docker container with pre-configured environment |
| Unity learning curve | Focus on minimal C# scripting, use pre-built assets |
| Unity-ROS 2 bridge issues | Pin specific versions, provide troubleshooting guide |
| Sensor data overwhelming | Start with single sensor, add incrementally |
| Performance issues | Provide recommended hardware specs, optimization tips |

---

## File Structure

```
book-source/docs/Part-2-Digital-Twin/
├── index.md (Part 2 introduction)
├── 04-gazebo-physics/
│   ├── index.md (Chapter 4 introduction)
│   ├── 01-gazebo-architecture.md
│   ├── 02-physics-engines.md
│   ├── 03-world-files.md
│   ├── 04-model-spawning.md
│   ├── 05-collision-detection.md
│   ├── 06-joint-control.md
│   ├── 07-gazebo-ros-bridge.md
│   └── 08-capstone-simulation.md
├── 05-unity-hri/
│   ├── index.md (Chapter 5 introduction)
│   ├── 01-unity-ros-bridge.md
│   ├── 02-urdf-import.md
│   ├── 03-environment-design.md
│   ├── 04-human-avatars.md
│   ├── 05-interaction-scripting.md
│   ├── 06-ros-integration.md
│   └── 07-capstone-hri.md
└── 06-sensors-simulation/
    ├── index.md (Chapter 6 introduction)
    ├── 01-sensor-plugins.md
    ├── 02-lidar-simulation.md
    ├── 03-depth-camera.md
    ├── 04-imu-sensor.md
    ├── 05-noise-models.md
    ├── 06-rviz-visualization.md
    ├── 07-sensor-processing.md
    ├── 08-bag-recording.md
    └── 09-capstone-sensor-suite.md
```

---

## Amendment Log

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-16 | Initial Part 2 specification |
