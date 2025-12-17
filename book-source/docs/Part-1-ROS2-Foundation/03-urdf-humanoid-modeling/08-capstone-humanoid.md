---
sidebar_position: 8
title: "Lesson 8: Capstone Project — Complete Humanoid Upper Body"
description: "Build a complete humanoid upper body from specification using accumulated knowledge and reusable patterns."
---

# Lesson 8: Capstone Project — Complete Humanoid Upper Body

This is your capstone for Chapter 3. You'll integrate everything you've learned—links, joints, inertia calculations, transform frames, and reusable patterns—to build a complete humanoid upper body.

Unlike previous lessons, you'll start with a **specification first**. The specification defines requirements, constraints, and success criteria. You'll then design and implement the URDF to satisfy the specification.

This is spec-driven development applied to robot modeling.

## Specification: Humanoid Upper Body

### Design Intent

Create a physically realistic humanoid upper body capable of:
- Natural arm movements (reaching, rotating)
- Realistic inertia for physics simulation
- Modular structure reusable for future humanoid variants

### Structural Requirements

**Torso**
- Dimensions: 0.3m wide × 0.2m deep × 0.5m tall
- Mass: 20 kg
- Center frame for upper body
- Must support both arms

**Head**
- Geometry: Sphere radius 0.15m
- Mass: 2 kg
- Attached to torso with fixed joint
- Positioned above torso

**Left Arm & Right Arm (mirrored)**
- **Shoulder**: 2 DOF revolute joints
  - Pitch (up/down): ±90° (±1.57 rad)
  - Roll (forward/back): ±60° (±1.047 rad)
- **Elbow**: 1 DOF revolute
  - Bend/straighten: 0° to 150° (0 to 2.61 rad)
- **Wrist**: 1 DOF revolute
  - Twist/rotate: ±90° (±1.57 rad)
- **Hand**: Sphere endpoint
  - Radius: 0.08m
  - Mass: 1 kg each

### Success Criteria

- [ ] URDF passes `check_urdf` validation
- [ ] All links have visual, collision, and inertial properties
- [ ] All joints have limits and movement axes
- [ ] Model loads in RViz without errors
- [ ] All joints controllable via GUI
- [ ] Joint movements appear natural and realistic
- [ ] Inertia calculated correctly for each segment
- [ ] Total model has ≤ 50 kg mass
- [ ] Model symmetry: left and right arms identical except position

### Dimensional Specification

| Component | Dimension | Mass | Notes |
|-----------|-----------|------|-------|
| Torso | 0.3×0.2×0.5m | 20 kg | Root link |
| Head | Sphere r=0.15m | 2 kg | Fixed to torso |
| Shoulder joint | n/a | 0.5 kg ea | Connection point |
| Upper arm | 0.1×0.1×0.4m | 5 kg | Elbow-to-shoulder |
| Forearm | 0.08×0.08×0.3m | 2 kg | Wrist-to-elbow |
| Hand | Sphere r=0.08m | 1 kg | End effector |
| **Total** | — | **50 kg** | Both arms included |

## Exercise 3.8a: Design Architecture

Before writing URDF, document your design.

**Task**: Create a text document describing:
1. Kinematic chain structure (parent → child relationships)
2. Joint configuration (types, axes, limits)
3. Link dimensions and masses
4. Inertia calculation plan

<details>
<summary>Solution Template</summary>

```
HUMANOID UPPER BODY ARCHITECTURE

Kinematic Chain:
- torso (root, world frame)
  ├─ head_joint (fixed)
  │  └─ head_link
  ├─ left_shoulder_pitch_joint (revolute, ±90°)
  │  └─ left_shoulder_link
  │     ├─ left_shoulder_roll_joint (revolute, ±60°)
  │     │  └─ left_upper_arm_link
  │     │     ├─ left_elbow_joint (revolute, 0-150°)
  │     │     │  └─ left_forearm_link
  │     │     │     ├─ left_wrist_joint (revolute, ±90°)
  │     │     │     │  └─ left_hand_link
  ├─ right_shoulder_pitch_joint (revolute, ±90°)
  │  └─ right_shoulder_link
  │     └─ ... (mirror of left)

Total Links: 11
Total Joints: 10 (1 fixed, 8 revolute per side, 1 per shoulder)

Inertia Plan:
- Torso: Box formula 0.3×0.2×0.5, 20kg
- Head: Sphere formula r=0.15m, 2kg
- Each shoulder: Sphere formula r=0.05m, 0.5kg
- Each upper arm: Box formula 0.1×0.1×0.4, 5kg
- Each forearm: Box formula 0.08×0.08×0.3, 2kg
- Each hand: Sphere formula r=0.08m, 1kg
```

**Success Criteria Checklist:**
- [ ] Architecture document created
- [ ] Kinematic chain clearly documented
- [ ] Joint limits specified in radians
- [ ] All inertia calculation formulas noted

</details>

## Exercise 3.8b: Implement Torso and Head

**Task**: Create URDF with torso and head connected by fixed joint.

<details>
<summary>Solution</summary>

Create `humanoid_upper_body.xacro`:

```xml
<?xml version="1.0"?>
<robot name="humanoid_upper_body" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Macro for box links -->
  <xacro:macro name="box_link" params="name width depth height mass color origin_xyz='0 0 0'">
    <link name="${name}">
      <visual>
        <origin xyz="${origin_xyz}" rpy="0 0 0"/>
        <geometry>
          <box size="${width} ${depth} ${height}"/>
        </geometry>
        <material name="${name}_material">
          <color rgba="${color}"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <box size="${width} ${depth} ${height}"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="${origin_xyz}" rpy="0 0 0"/>
        <mass value="${mass}"/>
        <inertia ixx="${(1/12)*mass*(depth**2 + height**2)}"
                 iyy="${(1/12)*mass*(width**2 + height**2)}"
                 izz="${(1/12)*mass*(width**2 + depth**2)}"
                 ixy="0" ixz="0" iyz="0"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Macro for sphere links -->
  <xacro:macro name="sphere_link" params="name radius mass color">
    <link name="${name}">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <sphere radius="${radius}"/>
        </geometry>
        <material name="${name}_material">
          <color rgba="${color}"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <sphere radius="${radius}"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="${mass}"/>
        <inertia ixx="${(2/5)*mass*radius**2}"
                 iyy="${(2/5)*mass*radius**2}"
                 izz="${(2/5)*mass*radius**2}"
                 ixy="0" ixz="0" iyz="0"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Torso (root) -->
  <xacro:box_link name="torso" width="0.3" depth="0.2" height="0.5" mass="20" color="0.5 0.5 0.5 1"/>

  <!-- Head -->
  <xacro:sphere_link name="head" radius="0.15" mass="2" color="1 0.8 0.6 1"/>

  <!-- Head joint (fixed) -->
  <joint name="head_joint" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
  </joint>

</robot>
```

Test:
```bash
xacro humanoid_upper_body.xacro > humanoid_upper_body.urdf
check_urdf humanoid_upper_body.urdf
```

**Success Criteria Checklist:**
- [ ] URDF valid
- [ ] Torso at origin
- [ ] Head positioned above torso
- [ ] Fixed joint connects them

</details>

## Exercise 3.8c: Implement Left Arm

**Task**: Add complete left arm with shoulder (2 DOF), elbow, wrist, and hand.

**Build step by step:**
1. Left shoulder joint (pitch)
2. Left shoulder link
3. Left shoulder joint (roll)
4. Left upper arm link
5. Left elbow joint
6. Left forearm link
7. Left wrist joint
8. Left hand link

<details>
<summary>Solution (abbreviated)</summary>

Add to your xacro file:

```xml
  <!-- Macro for revolute joints -->
  <xacro:macro name="revolute_joint" params="name parent child axis lower upper effort velocity origin_xyz origin_rpy">
    <joint name="${name}" type="revolute">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <axis xyz="${axis}"/>
      <origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>
      <limit lower="${lower}" upper="${upper}" effort="${effort}" velocity="${velocity}"/>
    </joint>
  </xacro:macro>

  <!-- LEFT ARM -->
  <!-- Shoulder pitch -->
  <xacro:revolute_joint name="left_shoulder_pitch" parent="torso" child="left_shoulder_link"
                        axis="0 1 0" lower="-1.57" upper="1.57" effort="50" velocity="1.0"
                        origin_xyz="0.15 0.1 0.25" origin_rpy="0 0 0"/>

  <xacro:sphere_link name="left_shoulder_link" radius="0.05" mass="0.5" color="0 0 1 1"/>

  <!-- Shoulder roll -->
  <xacro:revolute_joint name="left_shoulder_roll" parent="left_shoulder_link" child="left_upper_arm_link"
                        axis="0 0 1" lower="-1.047" upper="1.047" effort="50" velocity="1.0"
                        origin_xyz="0 0 0" origin_rpy="0 0 0"/>

  <xacro:box_link name="left_upper_arm_link" width="0.1" depth="0.1" height="0.4" mass="5"
                   color="1 0 0 1" origin_xyz="0 0 -0.2"/>

  <!-- Elbow -->
  <xacro:revolute_joint name="left_elbow_joint" parent="left_upper_arm_link" child="left_forearm_link"
                        axis="0 1 0" lower="0" upper="2.61" effort="40" velocity="1.0"
                        origin_xyz="0 0 -0.4" origin_rpy="0 0 0"/>

  <xacro:box_link name="left_forearm_link" width="0.08" depth="0.08" height="0.3" mass="2"
                   color="1 0.5 0 1" origin_xyz="0 0 -0.15"/>

  <!-- Wrist -->
  <xacro:revolute_joint name="left_wrist_joint" parent="left_forearm_link" child="left_hand_link"
                        axis="0 0 1" lower="-1.57" upper="1.57" effort="20" velocity="1.5"
                        origin_xyz="0 0 -0.3" origin_rpy="0 0 0"/>

  <xacro:sphere_link name="left_hand_link" radius="0.08" mass="1" color="1 1 0 1"/>
```

Test that left arm alone works:
```bash
xacro humanoid_upper_body.xacro > humanoid_upper_body.urdf
check_urdf humanoid_upper_body.urdf
ros2 launch ... (view in RViz)
```

**Success Criteria Checklist:**
- [ ] Left arm URDF valid
- [ ] All 4 left arm joints controllable
- [ ] Movements appear natural
- [ ] Inertia values reasonable

</details>

## Exercise 3.8d: Implement Right Arm

**Task**: Mirror the left arm structure for the right arm. Adjust positions (mirror X coordinate).

<details>
<summary>Solution Strategy</summary>

Use xacro's macro system to avoid duplication. Create an arm macro:

```xml
<xacro:macro name="arm" params="side x_offset">
  <!-- Define side-specific variables -->
  <xacro:if value="${side == 'left'}">
    <xacro:property name="color_shoulder" value="0 0 1 1"/>  <!-- Blue -->
    <xacro:property name="arm_x" value="${x_offset}"/>
  </xacro:if>
  <xacro:if value="${side == 'right'}">
    <xacro:property name="color_shoulder" value="0 1 1 1"/>  <!-- Cyan -->
    <xacro:property name="arm_x" value="${-x_offset}"/>
  </xacro:if>

  <!-- Shoulder pitch joint -->
  <xacro:revolute_joint name="${side}_shoulder_pitch" parent="torso" child="${side}_shoulder_link"
                        axis="0 1 0" lower="-1.57" upper="1.57" effort="50" velocity="1.0"
                        origin_xyz="${arm_x} 0.1 0.25" origin_rpy="0 0 0"/>

  <!-- ... rest of arm structure ... -->
</xacro:macro>

<!-- Instantiate both arms -->
<xacro:arm side="left" x_offset="0.15"/>
<xacro:arm side="right" x_offset="0.15"/>
```

Or manually mirror: Replace "left_" with "right_" and change origin_xyz x-values from positive to negative.

**Success Criteria Checklist:**
- [ ] Right arm present
- [ ] Right arm mirrors left (symmetric)
- [ ] All joints controllable
- [ ] Both arms move independently

</details>

## Exercise 3.8e: Add Inertial Properties

**Task**: Verify all links have calculated inertia values.

<details>
<summary>Verification Checklist</summary>

For each link, verify inertia was calculated:

```
Torso (0.3×0.2×0.5, 20kg):
  ixx = 0.483, iyy = 0.567, izz = 0.217

Head (r=0.15m, 2kg):
  i = 0.009

Shoulder (r=0.05m, 0.5kg each × 2):
  i = 0.0001 each

Upper arm (0.1×0.1×0.4, 5kg each × 2):
  ixx = 0.0683, iyy = 0.0683, izz = 0.0042 each

Forearm (0.08×0.08×0.3, 2kg each × 2):
  ixx = 0.0150, iyy = 0.0150, izz = 0.0011 each

Hand (r=0.08m, 1kg each × 2):
  i = 0.0010 each
```

Total mass: 20 + 2 + 0.5×2 + 5×2 + 2×2 + 1×2 = 50 kg ✓

Generate URDF:
```bash
xacro humanoid_upper_body.xacro > humanoid_upper_body.urdf
check_urdf humanoid_upper_body.urdf
```

**Success Criteria Checklist:**
- [ ] All links have inertia defined
- [ ] Values match calculations
- [ ] Total mass = 50 kg
- [ ] URDF valid

</details>

## Exercise 3.8f: Final Validation

**Task**: Load the complete humanoid in RViz and verify all success criteria.

<details>
<summary>Validation Checklist</summary>

Create launch file:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf', 'humanoid_upper_body.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),
    ])
```

Run:
```bash
ros2 launch my_robot launch_humanoid.launch.py
```

In RViz, verify:
- [ ] All 10 joints appear as sliders
- [ ] Torso centered at origin
- [ ] Head above torso
- [ ] Both arms visible and symmetric
- [ ] Moving sliders updates arm positions
- [ ] Movements look natural (not distorted)
- [ ] All links have colors and geometry
- [ ] Model appears as humanoid upper body

Calculate actual mass:
```bash
# Add up all mass values in URDF
grep '<mass value=' humanoid_upper_body.urdf | \
  awk -F'value="' '{print $2}' | \
  awk -F'"' '{sum+=$1} END {print "Total: " sum " kg"}'
```

Should print: Total: 50 kg

**Success Criteria Checklist:**
- [ ] URDF valid (check_urdf passes)
- [ ] All links visible in RViz
- [ ] All joints controllable
- [ ] Model loads without errors
- [ ] Movements appear realistic
- [ ] Inertia calculated for all links
- [ ] Total mass = 50 kg
- [ ] Left/right arms symmetric
- [ ] ALL spec requirements satisfied

</details>

## Complete Humanoid Upper Body URDF

Reference implementation (full xacro file):

```xml
<?xml version="1.0"?>
<robot name="humanoid_upper_body" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Macros for reuse -->
  <xacro:macro name="box_link" params="name width depth height mass color origin_xyz='0 0 0'">
    <link name="${name}">
      <visual>
        <origin xyz="${origin_xyz}" rpy="0 0 0"/>
        <geometry>
          <box size="${width} ${depth} ${height}"/>
        </geometry>
        <material name="${name}_material">
          <color rgba="${color}"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <box size="${width} ${depth} ${height}"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="${origin_xyz}" rpy="0 0 0"/>
        <mass value="${mass}"/>
        <inertia ixx="${(1/12)*mass*(depth**2 + height**2)}"
                 iyy="${(1/12)*mass*(width**2 + height**2)}"
                 izz="${(1/12)*mass*(width**2 + depth**2)}"
                 ixy="0" ixz="0" iyz="0"/>
      </inertial>
    </link>
  </xacro:macro>

  <xacro:macro name="sphere_link" params="name radius mass color">
    <link name="${name}">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <sphere radius="${radius}"/>
        </geometry>
        <material name="${name}_material">
          <color rgba="${color}"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <sphere radius="${radius}"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="${mass}"/>
        <inertia ixx="${(2/5)*mass*radius**2}"
                 iyy="${(2/5)*mass*radius**2}"
                 izz="${(2/5)*mass*radius**2}"
                 ixy="0" ixz="0" iyz="0"/>
      </inertial>
    </link>
  </xacro:macro>

  <xacro:macro name="revolute_joint" params="name parent child axis lower upper effort velocity origin_xyz origin_rpy">
    <joint name="${name}" type="revolute">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <axis xyz="${axis}"/>
      <origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>
      <limit lower="${lower}" upper="${upper}" effort="${effort}" velocity="${velocity}"/>
    </joint>
  </xacro:macro>

  <!-- ROOT: TORSO -->
  <xacro:box_link name="torso" width="0.3" depth="0.2" height="0.5" mass="20" color="0.5 0.5 0.5 1"/>

  <!-- HEAD -->
  <xacro:sphere_link name="head" radius="0.15" mass="2" color="1 0.8 0.6 1"/>
  <joint name="head_joint" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
  </joint>

  <!-- LEFT ARM -->
  <xacro:revolute_joint name="left_shoulder_pitch" parent="torso" child="left_shoulder_link"
                        axis="0 1 0" lower="-1.57" upper="1.57" effort="50" velocity="1.0"
                        origin_xyz="0.15 0.1 0.25" origin_rpy="0 0 0"/>
  <xacro:sphere_link name="left_shoulder_link" radius="0.05" mass="0.5" color="0 0 1 1"/>

  <xacro:revolute_joint name="left_shoulder_roll" parent="left_shoulder_link" child="left_upper_arm_link"
                        axis="0 0 1" lower="-1.047" upper="1.047" effort="50" velocity="1.0"
                        origin_xyz="0 0 0" origin_rpy="0 0 0"/>
  <xacro:box_link name="left_upper_arm_link" width="0.1" depth="0.1" height="0.4" mass="5"
                   color="1 0 0 1" origin_xyz="0 0 -0.2"/>

  <xacro:revolute_joint name="left_elbow_joint" parent="left_upper_arm_link" child="left_forearm_link"
                        axis="0 1 0" lower="0" upper="2.61" effort="40" velocity="1.0"
                        origin_xyz="0 0 -0.4" origin_rpy="0 0 0"/>
  <xacro:box_link name="left_forearm_link" width="0.08" depth="0.08" height="0.3" mass="2"
                   color="1 0.5 0 1" origin_xyz="0 0 -0.15"/>

  <xacro:revolute_joint name="left_wrist_joint" parent="left_forearm_link" child="left_hand_link"
                        axis="0 0 1" lower="-1.57" upper="1.57" effort="20" velocity="1.5"
                        origin_xyz="0 0 -0.3" origin_rpy="0 0 0"/>
  <xacro:sphere_link name="left_hand_link" radius="0.08" mass="1" color="1 1 0 1"/>

  <!-- RIGHT ARM (mirror of left) -->
  <xacro:revolute_joint name="right_shoulder_pitch" parent="torso" child="right_shoulder_link"
                        axis="0 1 0" lower="-1.57" upper="1.57" effort="50" velocity="1.0"
                        origin_xyz="-0.15 0.1 0.25" origin_rpy="0 0 0"/>
  <xacro:sphere_link name="right_shoulder_link" radius="0.05" mass="0.5" color="0 1 1 1"/>

  <xacro:revolute_joint name="right_shoulder_roll" parent="right_shoulder_link" child="right_upper_arm_link"
                        axis="0 0 1" lower="-1.047" upper="1.047" effort="50" velocity="1.0"
                        origin_xyz="0 0 0" origin_rpy="0 0 0"/>
  <xacro:box_link name="right_upper_arm_link" width="0.1" depth="0.1" height="0.4" mass="5"
                   color="0 1 0 1" origin_xyz="0 0 -0.2"/>

  <xacro:revolute_joint name="right_elbow_joint" parent="right_upper_arm_link" child="right_forearm_link"
                        axis="0 1 0" lower="0" upper="2.61" effort="40" velocity="1.0"
                        origin_xyz="0 0 -0.4" origin_rpy="0 0 0"/>
  <xacro:box_link name="right_forearm_link" width="0.08" depth="0.08" height="0.3" mass="2"
                   color="0 1 0 1" origin_xyz="0 0 -0.15"/>

  <xacro:revolute_joint name="right_wrist_joint" parent="right_forearm_link" child="right_hand_link"
                        axis="0 0 1" lower="-1.57" upper="1.57" effort="20" velocity="1.5"
                        origin_xyz="0 0 -0.3" origin_rpy="0 0 0"/>
  <xacro:sphere_link name="right_hand_link" radius="0.08" mass="1" color="0 1 1 1"/>

</robot>
```

## What You've Accomplished

You've built a complete humanoid upper body that demonstrates:

1. **Foundational URDF skills**: Syntax, link definition, collision geometry
2. **Physics modeling skills**: Joint configuration, inertia calculation, iterative refinement
3. **Reusability skills**: Macros, parameterized templates, code reduction
4. **System integration skills**: Specification-first design, acceptance criteria validation, complete model assembly

The humanoid upper body is now ready for:
- Physics simulation in Gazebo
- Motion planning algorithms
- Control system development
- Extension with hands, legs, sensors

## Try With AI

Reflect on your capstone with AI collaboration.

**Part 1: Design Validation**

Ask your AI:
"I built a humanoid upper body. Here's my specification and URDF. Does my implementation match the specification? Any improvements?"

Paste your spec and URDF. Get expert feedback.

**Part 2: Evaluate Suggestions**

Review AI's response:
- Does it identify gaps in your implementation?
- Are suggested improvements feasible?
- Do they align with your design intent?

**Part 3: Constraint Teaching**

Provide feedback:
"The inertia calculations are correct, but I want to reduce the total mass to 40kg. Which links should I reduce?"

AI suggests optimal reductions.

**Part 4: Refinement**

Update your design based on feedback and create a version 2.0 of your humanoid.

**Part 5: Future Planning**

Reflect on what you've learned:
"What would I need to add to make this humanoid complete? What's the next step after this capstone?"

This completes your journey through URDF fundamentals and humanoid upper body design. You're ready for Part 2 of the book: Simulation and Control.
