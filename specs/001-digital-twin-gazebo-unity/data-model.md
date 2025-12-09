# Data Model: Module 2: The Digital Twin (Gazebo & Unity)

**Date**: 2025-12-09
**Feature**: Module 2: The Digital Twin (Gazebo & Unity)

## Entity Overview

This document describes the key conceptual entities for Module 2: The Digital Twin (Gazebo & Unity). These entities represent the core concepts that students will learn to work with during the module.

## Core Entities

### Digital Twin
- **Description**: A physics-enabled simulation environment that mirrors physical robotics systems, enabling safe testing and development
- **Attributes**:
  - Simulation environment (Gazebo-based)
  - Physics parameters (gravity, friction, damping)
  - Visualization layer (Unity integration)
  - ROS 2 interface bridge
- **Relationships**: Contains Robot Kinematics Model, Simulation Environment, and connects to ROS 2 nervous system

### Robot Kinematics Model
- **Description**: A representation of robot structure using URDF/SDF formats defining joints, links, and coordinate frames
- **Attributes**:
  - Links (rigid bodies with geometry and material properties)
  - Joints (connections between links with degrees of freedom)
  - Coordinate frames (for spatial relationships)
  - Kinematic chains (sequences of connected links)
- **Relationships**: Belongs to Digital Twin, contains Simulated Sensors

### Simulated Sensor Configuration
- **Description**: A reusable component that configures virtual sensors to publish data to ROS 2 topics with realistic parameters
- **Attributes**:
  - Sensor type (LiDAR, Depth Camera, IMU, etc.)
  - Noise parameters (standard deviation, bias)
  - Update rates (frequency of data publication)
  - Field of view and range parameters
  - ROS 2 topic configuration
- **Relationships**: Attached to Robot Kinematics Model, publishes to ROS 2 Topics

### Simulation Environment
- **Description**: A physics-enabled space containing robot models, custom assets, and environmental elements for testing
- **Attributes**:
  - Physical properties (gravity, friction coefficients)
  - Collision geometries (meshes, primitives)
  - Environmental assets (furniture, walls, obstacles)
  - Lighting conditions
- **Relationships**: Contains Robot Kinematics Models, implements physics simulation

### Teleoperation Specification
- **Description**: A formal document defining the ROS 2 interface requirements before implementation for controlling simulated robots
- **Attributes**:
  - Input device mapping (joystick, keyboard, etc.)
  - ROS 2 topic definitions (commands, feedback)
  - Safety constraints (speed limits, collision avoidance)
  - Control authority levels
- **Relationships**: Defines interface between virtual input and simulated robot control

## State Transitions

### Robot Kinematics Model States
- **Design Phase**: Model being created with basic structure
- **Verification Phase**: Model being validated in visualizer
- **Simulation Ready**: Model ready for physics simulation
- **Integrated**: Model connected to ROS 2 control system

### Simulation Environment States
- **Static**: Environment created but no robot interaction
- **Dynamic**: Robot interacting with environment
- **Validated**: Environment tested with collision detection
- **Optimized**: Physics parameters tuned for realistic behavior

## Validation Rules

### Robot Kinematics Model
- Must have at least one fixed joint connecting to world/base
- Joint limits must be within physical reality bounds
- Links must have valid geometry definitions
- Coordinate frames must form proper kinematic chain

### Simulated Sensor Configuration
- Update rates must be within realistic ranges for sensor type
- Noise parameters must be configurable and realistic
- ROS 2 topic names must follow standard conventions
- Sensor placement must be physically possible on robot model

### Simulation Environment
- Collision geometries must be properly defined
- Physics parameters must be within realistic ranges
- Environmental assets must not create impossible scenarios
- Gravity and other global parameters must be consistent

## Relationships

### Digital Twin Composition
```
Digital Twin
├── Contains → Robot Kinematics Model (1:*)
├── Contains → Simulation Environment (1:1)
├── Contains → Simulated Sensor Configuration (*:*)
└── Connects → ROS 2 Nervous System (1:1)
```

### Simulation Workflow
```
Robot Kinematics Model → Simulated Sensor Configuration (1:*)
Simulation Environment → Robot Kinematics Model (1:*)
Teleoperation Specification → Simulation Environment (1:1)
```