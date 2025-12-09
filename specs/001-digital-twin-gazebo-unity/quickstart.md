# Quickstart Guide: Module 2: The Digital Twin (Gazebo & Unity)

**Date**: 2025-12-09
**Feature**: Module 2: The Digital Twin (Gazebo & Unity)

## Overview

This quickstart guide provides the essential steps to begin working with Module 2: The Digital Twin (Gazebo & Unity). This module builds upon the ROS 2 nervous system established in Module 1 and introduces students to digital twin concepts, simulation environments, and sensor modeling.

## Prerequisites

Before starting Module 2, ensure you have:

1. **Completed Module 1**: Understanding of ROS 2 fundamentals, nodes, topics, services, and launch files
2. **Development Environment**: Ubuntu 22.04 LTS with ROS 2 Humble Hawksbill installed
3. **Simulation Tools**: Gazebo Garden and basic understanding of URDF/SDF formats
4. **Basic Python**: Understanding of Python 3.10+ and rclpy library usage

## Setup Steps

### 1. Install Gazebo Garden
```bash
# Add ROS 2 repository if not already done
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Install Gazebo Garden
sudo apt update
sudo apt install -y ros-humble-gz-garden
```

### 2. Verify ROS 2 Integration
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Test basic Gazebo launch
gz sim -v 4
```

### 3. Prepare Workspace
```bash
# Create workspace directory for simulation packages
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws

# Build workspace
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## First Exercise: Simple Two-Link Robot

### 1. Create URDF Model
Create a simple two-link robot model in `~/digital_twin_ws/src/robot_description/urdf/simple_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- First joint (revolute) -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- First link -->
  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### 2. Visualize the Robot
```bash
# Launch robot in Gazebo
gz sim -r simple_robot.sdf
```

### 3. Verify in RViz
```bash
# In a new terminal
source /opt/ros/humble/setup.bash
source ~/digital_twin_ws/install/setup.bash
rviz2
```

## Key Concepts

### 1. URDF vs SDF
- **URDF**: Used for robot kinematics and structure (what you learned in Module 1)
- **SDF**: Gazebo's native format for simulation environments and physics properties

### 2. The 4-Layer Framework
1. **Manual Foundation**: Create robot models manually (URDF/SDF)
2. **AI Collaboration**: Use AI to debug simulation environments
3. **Intelligence Design**: Create reusable sensor configuration skills
4. **Spec-Driven Integration**: Integrate with ROS 2 nervous system

### 3. Digital Twin Principles
- Simulation mirrors physical reality
- Safe testing environment for robotics
- Integration with ROS 2 communication patterns

## Next Steps

1. Complete the Robot Kinematics submodule (Layer 1: Manual Foundation)
2. Move to Physics Simulation submodule (Layer 2: AI Collaboration)
3. Progress to Simulated Perception submodule (Layer 3: Intelligence Design)
4. Complete with Spec-Driven Integration (Layer 4: Integration)

## Troubleshooting

### Common Issues
- **Gazebo won't start**: Check ROS 2 environment is sourced
- **URDF not loading**: Verify XML syntax and joint limits
- **RViz not showing robot**: Check robot_state_publisher and joint_state_publisher

### Getting Help
- Review Module 1 ROS 2 concepts if having communication issues
- Check ROS 2 and Gazebo compatibility versions
- Verify all dependencies are installed correctly