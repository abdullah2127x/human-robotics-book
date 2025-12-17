---
title: "Chapter 6: Simulating Sensors - LiDAR, Depth Cameras, and IMUs"
chapter: 6
part: 2
proficiency_level: B2
learning_objectives:
  - Configure and integrate multiple sensor types (LiDAR, depth cameras, IMU) into Gazebo simulations
  - Generate realistic sensor data with appropriate noise models matching real sensor specifications
  - Process sensor data in ROS 2 pipelines to extract meaningful information (obstacle detection, orientation estimation)
  - Visualize complex sensor data streams in RViz for debugging and analysis
  - Build and validate perception systems suitable for robotics applications
generated_by: content-implementer v1.0.0
created: 2025-12-16
---

# Chapter 6: Simulating Sensors - LiDAR, Depth Cameras, and IMUs

## Overview

When a humanoid robot moves through the world, it perceives its environment through sensors—each capturing different aspects of reality. LiDAR provides 360-degree range measurements. Depth cameras generate 3D point clouds and color images. IMUs track acceleration and rotation. Together, they form a perception system that enables autonomous navigation and interaction.

In this chapter, you'll learn to simulate these sensors in Gazebo, making them output realistic data with appropriate noise models. You'll process that data through ROS 2 pipelines, visualize results in RViz, and build a complete sensor suite integrated with your humanoid.

## What You'll Build

By chapter's end, you'll have:

- **Sensor-equipped humanoid**: URDF with LiDAR on head, depth camera on torso, IMU at center of mass
- **Realistic simulation**: All sensors publishing data with noise models matching real hardware specifications
- **Processing pipeline**: ROS 2 nodes detecting obstacles from sensor data and estimating orientation from IMU measurements
- **Complete visualization**: RViz setup displaying all sensor types simultaneously with layered processing results
- **Data recording**: ROS 2 bags capturing all sensor streams for post-analysis and algorithm validation

## Chapter Structure

| Lesson | Focus | Layer | Time |
|--------|-------|-------|------|
| 1. Sensor Plugin Architecture | Foundation concepts, data flow | Manual | 90 min |
| 2. Adding Sensors to URDF | Modifying robot definition | Manual | 90 min |
| 3. LiDAR Simulation | 2D/3D LiDAR, point clouds | AI Collaboration | 120 min |
| 4. Depth Camera Simulation | RGB-D data, intrinsics | AI Collaboration | 120 min |
| 5. IMU Sensor Simulation | Accelerometer, gyroscope, noise | AI Collaboration | 120 min |
| 6. Sensor Noise Models | Realistic calibration, validation | AI Collaboration | 120 min |
| 7. RViz Visualization | Sensor visualization skill | Intelligence Design | 90 min |
| 8. Sensor Data Processing | Processing skill, algorithms | Intelligence Design | 90 min |
| 9. Capstone: Complete Sensor Suite | Full system integration | Spec-Driven | 150 min |

**Total time**: 13.5–18 hours (content + hands-on)

## Prerequisites

- **Chapter 4 completion**: Gazebo simulation fundamentals, URDF concepts
- **Part 1 completion**: ROS 2 nodes, rclpy, message handling
- **Python**: Comfortable with data structures, numpy, message parsing
- **Linux/Ubuntu**: Familiar with command line, package installation

## Why This Matters

Simulation lets you test perception algorithms before deploying to hardware. But if your simulated sensors don't behave like real sensors—if they're noise-free or use unrealistic parameters—your algorithms will fail when moved to physical robots. This chapter bridges that gap: making simulation realistic enough to validate algorithms that work in the real world.

## Learning Progression

**Layer 1 (Lessons 1–2)**: Build mental models of sensor architecture and URDF integration.
**Layer 2 (Lessons 3–6)**: Collaborate with AI to configure and debug each sensor type.
**Layer 3 (Lessons 7–8)**: Design reusable visualization and processing skills.
**Layer 4 (Lesson 9)**: Specification-first capstone integrating all components.

Each lesson builds on prior work. Complete them sequentially for optimal learning.

## Success Criteria

You'll know you've mastered this chapter when you can:

- ✅ Add any sensor type to humanoid URDF with correct parameters
- ✅ Configure Gazebo plugins to publish realistic sensor data
- ✅ Match simulation noise models to real sensor specifications
- ✅ Write Python ROS 2 nodes processing sensor data
- ✅ Visualize multiple sensor types simultaneously in RViz
- ✅ Design specifications for complete perception systems
- ✅ Record sensor data to ROS 2 bags for offline analysis

---

**Next**: Start with Lesson 1 to learn the sensor plugin architecture that underpins everything in this chapter.
