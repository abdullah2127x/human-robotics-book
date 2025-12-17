---
sidebar_position: 3
title: "Lesson 3: Joints Connecting Links"
description: "Create articulated structures by connecting links with joints. Explore joint types and realistic limits."
---

# Lesson 3: Joints Connecting Links

A single link is static. To build a robot that moves, you need to connect links with joints. Joints define how links move relative to each other. They specify which links connect (parent and child), what direction they move, and what movement limits apply.

In this lesson, you'll build your first articulated structure: two links connected by a joint.

## What is a Joint?

A joint connects two links and constrains their relative motion. It answers these questions:
- Which link does this joint attach to? (parent)
- What moves as a result? (child)
- How does it move? (revolute, prismatic, fixed, continuous)
- What are the movement limits? (minimum and maximum)

Think of a joint like a hinge. A door hinge connects a door (child) to a frame (parent) and allows rotation around one axis.

## Joint Types

URDF supports four main joint types:

### Revolute Joint
Rotates around a fixed axis within limits (most common in robots).

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
</joint>
```

- **lower/upper**: Rotation limits in radians (±1.57 ≈ ±90°)
- **effort**: Maximum torque (Newton-meters)
- **velocity**: Maximum rotation speed (radians/second)

### Prismatic Joint
Slides along an axis (like a piston or linear actuator).

```xml
<joint name="linear_joint" type="prismatic">
  <parent link="base"/>
  <child link="slider"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.5" effort="50" velocity="0.1"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

- **lower/upper**: Linear displacement limits in meters

### Fixed Joint
Rigid connection with no movement (used for structural connections).

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
</joint>
```

No axis, limits, or effort—it's locked in place.

### Continuous Joint
Rotates infinitely without limits (like a wheel).

```xml
<joint name="wheel_joint" type="continuous">
  <parent link="chassis"/>
  <child link="wheel"/>
  <axis xyz="0 1 0"/>
  <limit effort="10" velocity="2"/>
  <origin xyz="0.3 0 0" rpy="0 0 0"/>
</joint>
```

Used for wheels and free-spinning links. No lower/upper limits.

## Understanding Joint Structure

Here's a detailed joint example:

```xml
<joint name="elbow_joint" type="revolute">
  <!-- Name is unique identifier -->

  <parent link="upper_arm"/>
  <!-- Parent link (fixed reference) -->

  <child link="forearm"/>
  <!-- Child link (moves relative to parent) -->

  <axis xyz="0 1 0"/>
  <!-- Rotation axis: [x y z] direction -->
  <!-- This means rotation around Y-axis -->

  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <!-- Where joint attaches relative to parent link frame -->
  <!-- forearm origin is 0.3m below upper_arm -->

  <limit lower="0" upper="2.61" effort="100" velocity="2.0"/>
  <!-- lower: minimum angle (radians), 0 = straight
       upper: maximum angle (radians), 2.61 ≈ 150°
       effort: max torque (N⋅m)
       velocity: max speed (rad/s) -->

  <dynamics damping="0.5" friction="0.5"/>
  <!-- Optional: damping and friction coefficients -->
</joint>
```

## Two-Link Structure Example

Here's a complete URDF with two links connected by a joint:

```xml
<?xml version="1.0"?>
<robot name="two_link_robot">

  <!-- Parent link (fixed) -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="20.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joint connecting torso to upper arm -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <!-- Child link (moves with joint) -->
  <link name="upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="5.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
  </link>

</robot>
```

## Joint Axis Explained

The `<axis>` element specifies the rotation axis as a unit vector [x y z]:

| Axis | Vector | Meaning |
|------|--------|---------|
| X-axis | `xyz="1 0 0"` | Rotation around X (roll) |
| Y-axis | `xyz="0 1 0"` | Rotation around Y (pitch) |
| Z-axis | `xyz="0 0 1"` | Rotation around Z (yaw) |

For a shoulder joint that lifts the arm up/down, use Y-axis: `xyz="0 1 0"`

For a shoulder joint that rotates arm forward/back, use Z-axis: `xyz="0 0 1"`

## Exercise 3.3a: Create Two-Link Structure

Create a URDF with a torso and upper arm connected by a shoulder joint.

**Task:**
1. Create the URDF above (torso + upper arm + shoulder joint)
2. Run `check_urdf` to verify
3. Save as `two_link_arm.urdf`

<details>
<summary>Solution</summary>

Use the complete URDF example above. Key elements to verify:
- Parent link (torso) defined first
- Joint connects torso → upper_arm
- Child link (upper_arm) defined after joint
- All XML tags properly closed
- Joint limits in radians (±1.57 for ±90°)

Run:
```bash
check_urdf two_link_arm.urdf
```

Should output:
```
robot description is OK
```

**Success Criteria Checklist:**
- [ ] URDF has 2 links
- [ ] URDF has 1 joint
- [ ] Joint connects parent → child correctly
- [ ] `check_urdf` passes
- [ ] Limits are physically realistic

</details>

## Exercise 3.3b: Visualize and Manipulate in RViz

Load your two-link URDF in RViz and use the joint_state_publisher to move the joint.

**Task:**
1. Create a launch file for your URDF
2. Add `joint_state_publisher_gui` to allow interactive joint control
3. Run the launch file
4. Use the slider to move the shoulder joint
5. Observe the upper arm moving relative to the torso

<details>
<summary>Solution Guide</summary>

Create launch file `view_two_link.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf', 'two_link_arm.urdf'
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
ros2 launch my_robot view_two_link.launch.py
```

RViz opens with a GUI slider for shoulder_joint. Move the slider and watch the arm move!

**Success Criteria Checklist:**
- [ ] Launch file created
- [ ] RViz opens successfully
- [ ] Joint slider appears in GUI
- [ ] Moving slider moves the arm
- [ ] Movement is within -90° to +90°

</details>

## Exercise 3.3c: Try Different Joint Types

Modify your URDF to use different joint types and observe the behavior change.

**Task:**
1. Copy your URDF and change the shoulder_joint from "revolute" to "prismatic"
2. Reload in RViz and use the slider (now controls linear motion, not rotation)
3. Change back to "revolute" and verify it rotates again
4. Try "fixed" and notice the joint no longer moves

<details>
<summary>Solution</summary>

**Version 1: Prismatic Joint**
Replace the joint definition:
```xml
<joint name="shoulder_joint" type="prismatic">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <axis xyz="0 0 1"/>
  <!-- Z-axis for vertical sliding -->
  <origin xyz="0 0 0.25" rpy="0 0 0"/>
  <limit lower="0" upper="0.3" effort="100" velocity="0.1"/>
  <!-- Linear limits in meters -->
</joint>
```

When you move the slider, the upper arm slides up/down instead of rotating.

**Version 2: Fixed Joint**
```xml
<joint name="shoulder_joint" type="fixed">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0 0 0.25" rpy="0 0 0"/>
  <!-- No axis, limits, or effort for fixed joint -->
</joint>
```

The arm doesn't move at all—it's welded to the torso.

**Success Criteria Checklist:**
- [ ] Prismatic version moves linearly
- [ ] Fixed version has no movement
- [ ] Can switch back to revolute
- [ ] Understand which joint type for each use

</details>

## Realistic Joint Limits

When designing joints, use limits that match physical constraints:

| Joint Type | Example | Realistic Limits |
|-----------|---------|-----------------|
| Shoulder revolute | Rotation up/down | ±90° (-1.57 to 1.57 rad) |
| Elbow revolute | Bending | 0° to 150° (0 to 2.61 rad) |
| Wrist revolute | Twisting | ±90° (-1.57 to 1.57 rad) |
| Waist revolute | Turning | ±180° (-3.14 to 3.14 rad) |
| Slider prismatic | Extension | Depends on mechanism |

Setting realistic limits makes simulation behave like real robots.

## Checking Your Work

Before moving to Lesson 4, ensure:
- [ ] You can create a joint connecting two links
- [ ] You understand parent/child relationships
- [ ] You can set joint limits in radians
- [ ] You can modify and observe joint behavior in RViz

## Try With AI

Now let's collaborate on realistic joint design for humanoid robots.

**Part 1: AI Teaches Joint Types**

Ask your AI tool:
"I'm building a humanoid robot. I need a shoulder joint that can rotate up/down (pitch) and rotate the arm forward/back (roll). What joint types should I use? Create URDF for both."

Observe how AI explains joint types and creates examples.

**Part 2: Evaluate the Response**

Review AI's answer:
- Did AI suggest the correct joint types?
- Are the axes correct for the described movements?
- Do the limits seem physically realistic?

**Part 3: Constraint Teaching**

Provide your actual robot constraints:
"The real humanoid has a shoulder with 2 degrees of freedom. I need:
- Joint 1: Rotates arm up/down (max ±90°)
- Joint 2: Rotates arm forward/back (max ±60°)

The arm weighs 5 kg. What effort values should I use for realistic motion?"

**Part 4: Refinement Through Iteration**

AI suggests effort values. You respond:
"When I move this manually, it requires more torque than your estimate. Increase effort to support the arm weight better."

AI recalculates based on your feedback.

**Part 5: Final Validation**

Compare the refined result to your original request:
- Did iteration improve the design?
- Do you understand why effort values matter?
- Can you predict if the joint will behave realistically?

This collaborative process teaches you to work with AI on articulation design.
