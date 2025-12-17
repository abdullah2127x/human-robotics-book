---
sidebar_position: 5
title: "Lesson 5: Transform Frames and Kinematic Structure"
description: "Understand how links and joints define coordinate frame relationships and robot kinematics."
---

# Lesson 5: Transform Frames and Kinematic Structure

A robot is more than individual links. The links must be positioned correctly relative to each other. This positioning is defined by transform frames—coordinate systems attached to each link that define where the next link connects.

In this lesson, you'll learn how the URDF structure creates a kinematic chain: a sequence of coordinate frames that position every part of the robot in space.

## What is a Transform Frame?

A transform frame (or "frame") is a coordinate system attached to a link. It answers: "Where is this link in space relative to other links?"

Every link has a frame. When you connect links with joints, you're defining how frames relate to each other.

**Example**: A robot arm has frames:
- World frame (global reference)
- Torso frame (attached to torso)
- Upper arm frame (attached to shoulder joint, moves with upper arm)
- Forearm frame (attached to elbow joint, moves with forearm)
- Hand frame (attached to wrist joint, moves with hand)

Each frame is positioned relative to its parent frame by the connecting joint.

## The Kinematic Chain

The kinematic chain is a sequence of transforms from the root link to each leaf link:

```
world → torso → upper_arm → forearm → hand
```

Each arrow is a joint that transforms coordinates from one frame to the next.

In URDF:
- **torso** is the parent of the shoulder joint (fixed frame)
- **upper_arm** is the child of the shoulder joint (moves with joint)
- **forearm** is the child of the elbow joint (moves with elbow)

## The Origin Element: Positioning Links

The `<origin>` element in a joint specifies where the child link connects relative to the parent link:

```xml
<joint name="shoulder" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0 0 0.25" rpy="0 0 0"/>
  <!-- Child frame is 0.25m above parent frame -->
</joint>
```

The origin has two parts:

**xyz**: Linear position (x, y, z) in meters relative to parent frame
- `xyz="0 0 0.25"` means 0.25m higher on the parent

**rpy**: Rotational orientation (roll, pitch, yaw) in radians
- `rpy="0 0 0"` means no rotation (aligned with parent axes)
- Positive angles use right-hand rule

## Example: Three-Link Arm with Frames

```xml
<?xml version="1.0"?>
<robot name="three_link_arm">

  <!-- Root link at world origin -->
  <link name="base">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- First joint: base → segment1 -->
  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="segment1"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <!-- segment1 frame is 0.1m above base frame -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- First movable segment -->
  <link name="segment1">
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Second joint: segment1 → segment2 -->
  <joint name="joint2" type="revolute">
    <parent link="segment1"/>
    <child link="segment2"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <!-- segment2 frame is 0.2m below segment1 frame -->
    <limit lower="0" upper="2.61" effort="10" velocity="1"/>
  </joint>

  <!-- Second movable segment -->
  <link name="segment2">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.3"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.08 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

</robot>
```

**Frame Structure**:
```
base (world origin)
  ↓ joint1 (+0.1m in Z)
segment1 (0.1m up)
  ↓ joint2 (-0.2m in Z)
segment2 (at -0.1m from base, after joint rotation)
```

## Forward Kinematics

Forward kinematics answers: "Where is the end effector in world coordinates?"

The answer comes from multiplying transforms along the kinematic chain:

```
T_end = T_base→seg1 × T_seg1→seg2
```

When joint1 rotates 45°, the frame segment1 rotates, which moves segment2.

**Why it matters**: You set joint angles. The kinematic chain calculates where every link ends up. This is how simulation knows the robot geometry as joints move.

## Exercise 3.5a: Visualize Transform Tree

Create the three-link URDF above and visualize the transform tree in RViz.

**Task:**
1. Save the three-link URDF
2. Create a launch file with RViz
3. In RViz, enable "TF" display (visualization of transform frames)
4. Rotate joints and observe how frames move together

<details>
<summary>Solution Guide</summary>

Save the URDF above as `three_link_arm.urdf`.

Create launch file:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf', 'three_link_arm.urdf'
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

In RViz:
1. Add display → "TF"
2. Set "Show Axes" to show coordinate frames
3. Move joint sliders and watch frames move

**Success Criteria Checklist:**
- [ ] URDF loads successfully
- [ ] TF tree visible
- [ ] Frames move when joints rotate
- [ ] Parent-child relationships clear

</details>

## Exercise 3.5b: Modify Origins and Observe Effects

Change the origin values and observe how it affects link positioning.

**Task:**
1. Modify joint1 origin from `xyz="0 0 0.1"` to `xyz="0 0 0.2"` (higher)
2. Reload and observe segment1 is now higher
3. Modify joint2 origin rotation from `rpy="0 0 0"` to `rpy="0 0 1.57"` (90° rotation)
4. Reload and observe segment2 rotates relative to segment1

<details>
<summary>Solution</summary>

**Version 1: Change Z position**
```xml
<joint name="joint1" type="revolute">
  <!-- ... other attributes ... -->
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
  <!-- segment1 now 0.2m above base -->
</joint>
```

**Version 2: Add rotation**
```xml
<joint name="joint2" type="revolute">
  <!-- ... other attributes ... -->
  <origin xyz="0 0 -0.2" rpy="0 0 1.57"/>
  <!-- segment2 rotated 90° around Z axis -->
</joint>
```

These changes immediately affect positioning in RViz.

**Success Criteria Checklist:**
- [ ] Origin modifications understood
- [ ] Visual changes appear in RViz
- [ ] Relationship between origin and positioning clear

</details>

## Transform Tree Visualization

RViz displays the transform tree, showing:
- **Blue axis**: X-axis (right)
- **Green axis**: Y-axis (up)
- **Red axis**: Z-axis (forward)

Each frame shows its orientation in space. When joints move, you see how frames rotate and translate together.

This visualization helps debug kinematic issues: If a link appears in the wrong position, the transform tree shows exactly where the error is.

## Checking Your Work

Before moving to Lesson 6, ensure:
- [ ] You understand frame hierarchy (parent → child)
- [ ] You can interpret origin xyz and rpy values
- [ ] You can predict how origin changes affect positioning
- [ ] You can visualize transforms in RViz

## Try With AI

Collaborate with AI on complex kinematic structures.

**Part 1: Understand Rotational Origins**

Ask your AI tool:
"I want joint2 to rotate the segment perpendicular to joint1. What rpy rotation should I use in the origin? Create URDF showing both configurations."

Observe how AI explains rotational transforms.

**Part 2: Evaluate the Response**

Check AI's answer:
- Does the rpy value make sense?
- Do the axes align with expected movements?
- Can you predict the visual result?

**Part 3: Constraint Teaching**

Provide your humanoid requirement:
"I'm building a shoulder joint with 2 DOF. The first rotation is around Y (pitch). The second rotation at the elbow is around Z (roll). What origins should I use?"

**Part 4: Refinement**

AI suggests values. You refine:
"Actually, the forearm needs to rotate around its own axis, not the world axis. How does that change the origin?"

**Part 5: Validation**

Load the refined URDF and verify:
- Do movements match your intent?
- Is the kinematic chain correct?
- Can you explain the transform relationships?

This process develops intuition about transform trees in humanoid structures.
