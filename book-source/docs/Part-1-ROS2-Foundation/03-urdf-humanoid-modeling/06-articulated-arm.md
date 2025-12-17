---
sidebar_position: 6
title: "Lesson 6: Building Articulated Arm"
description: "Integrate all URDF concepts to create a complete articulated arm with multiple joints and realistic inertia."
---

# Lesson 6: Building Articulated Arm

Now you'll bring together everything you've learned: links, joints, geometric properties, and inertia calculations. You'll build a complete articulated arm with a shoulder (2 DOF), elbow (1 DOF), and wrist (1 DOF)—four joints total moving a four-segment arm.

This is the capstone of manual URDF construction before we introduce reusable patterns in Lesson 7.

## Arm Architecture

Here's the structure you'll build:

```
torso (root)
  ↓ shoulder_pitch (rotates up/down)
shoulder_link
  ↓ shoulder_roll (rotates forward/back)
upper_arm_link
  ↓ elbow_joint (bends at elbow)
forearm_link
  ↓ wrist_joint (twists at wrist)
hand_link
```

**Degrees of Freedom**: 4 joints = 4 DOF (realistic for basic humanoid arm)

| Joint | Movement | Limits |
|-------|----------|--------|
| Shoulder Pitch | Rotate up/down | ±90° |
| Shoulder Roll | Rotate fwd/back | ±60° |
| Elbow | Bend/straighten | 0° to 150° |
| Wrist | Twist/rotate | ±90° |

## Complete Articulated Arm URDF

```xml
<?xml version="1.0"?>
<robot name="articulated_arm">

  <!-- Root: Torso (fixed in world) -->
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

  <!-- Joint 1: Shoulder Pitch (up/down) -->
  <joint name="shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="shoulder_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 0.1 0.25" rpy="0 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
  </joint>

  <!-- Shoulder link (small connection) -->
  <link name="shoulder_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint 2: Shoulder Roll (forward/back) -->
  <joint name="shoulder_roll" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <axis xyz="0 0 1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <limit lower="-1.047" upper="1.047" effort="50" velocity="1.0"/>
    <!-- ±60° (±1.047 rad) -->
  </joint>

  <!-- Upper arm (long segment from shoulder to elbow) -->
  <link name="upper_arm_link">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.4"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <mass value="5.0"/>
      <!-- Box 0.1×0.1×0.4 with 5kg -->
      <inertia ixx="0.0683" ixy="0" ixz="0" iyy="0.0683" iyz="0" izz="0.0042"/>
    </inertial>
  </link>

  <!-- Joint 3: Elbow (bending) -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <limit lower="0" upper="2.61" effort="40" velocity="1.0"/>
    <!-- 0° to 150° (0 to 2.61 rad) -->
  </joint>

  <!-- Forearm (shorter segment from elbow to wrist) -->
  <link name="forearm_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.3"/>
      </geometry>
      <material name="orange">
        <color rgba="1 0.5 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="2.0"/>
      <!-- Box 0.08×0.08×0.3 with 2kg -->
      <inertia ixx="0.0150" ixy="0" ixz="0" iyy="0.0150" iyz="0" izz="0.0011"/>
    </inertial>
  </link>

  <!-- Joint 4: Wrist (twisting) -->
  <joint name="wrist_joint" type="revolute">
    <parent link="forearm_link"/>
    <child link="hand_link"/>
    <axis xyz="0 0 1"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1.5"/>
    <!-- ±90° (±1.57 rad) -->
  </joint>

  <!-- Hand (end effector) -->
  <link name="hand_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <!-- Sphere 0.08m radius, 1kg -->
      <inertia ixx="0.0010" ixy="0" ixz="0" iyy="0.0010" iyz="0" izz="0.0010"/>
    </inertial>
  </link>

</robot>
```

## Exercise 3.6a: Create Torso Link

**Task**: Start with just the torso (root link) and verify it loads.

<details>
<summary>Solution</summary>

Create file `arm_part1_torso.urdf`:

```xml
<?xml version="1.0"?>
<robot name="articulated_arm_part1">

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

</robot>
```

Verify:
```bash
check_urdf arm_part1_torso.urdf
```

**Success Criteria Checklist:**
- [ ] URDF valid
- [ ] Torso appears in RViz
- [ ] Gray box visible

</details>

## Exercise 3.6b: Add Shoulder Joints

**Task**: Add shoulder_link, shoulder_pitch, and shoulder_roll joints.

<details>
<summary>Solution</summary>

Extend your URDF with:

```xml
<!-- (keep torso from Exercise 3.6a) -->

<joint name="shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="shoulder_link"/>
  <axis xyz="0 1 0"/>
  <origin xyz="0 0.1 0.25" rpy="0 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
</joint>

<link name="shoulder_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <sphere radius="0.05"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <sphere radius="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="0.5"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="shoulder_roll" type="revolute">
  <parent link="shoulder_link"/>
  <child link="upper_arm_link"/>
  <axis xyz="0 0 1"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <limit lower="-1.047" upper="1.047" effort="50" velocity="1.0"/>
</joint>

<!-- Continue with upper_arm_link... -->
```

**Success Criteria Checklist:**
- [ ] Both shoulder joints present
- [ ] Joints connect correctly (parent → child)
- [ ] RViz shows 2 joint sliders
- [ ] Arm moves when sliders adjusted

</details>

## Exercise 3.6c: Add Elbow and Wrist

**Task**: Complete the arm by adding elbow_joint, forearm_link, wrist_joint, and hand_link.

<details>
<summary>Solution</summary>

Use the complete URDF example at the beginning of this lesson. The key is maintaining the chain:

```
torso → shoulder_pitch → shoulder_link → shoulder_roll →
upper_arm_link → elbow_joint → forearm_link → wrist_joint → hand_link
```

Verify with:
```bash
check_urdf complete_arm.urdf
ros2 launch ... (with RViz)
```

You should see 4 joint sliders in RViz.

**Success Criteria Checklist:**
- [ ] All 4 joints present
- [ ] All 5 links present
- [ ] URDF valid
- [ ] RViz shows complete arm
- [ ] All 4 joints controllable

</details>

## Exercise 3.6d: Calculate and Apply Inertia

**Task**: For each link, calculate inertia based on geometry and update URDF values.

**Given proportions:**
- Torso: 0.3×0.2×0.5m, 20kg
- Upper arm: 0.1×0.1×0.4m, 5kg
- Forearm: 0.08×0.08×0.3m, 2kg
- Hand: 0.08m sphere, 1kg
- Shoulder/wrist spheres: 0.05m sphere, 0.5kg each

<details>
<summary>Solution Calculations</summary>

**Torso** (box 0.3×0.2×0.5, 20kg):
```
ixx = (1/12)*20*(0.2²+0.5²) = (1/12)*20*0.29 = 0.483
iyy = (1/12)*20*(0.3²+0.5²) = (1/12)*20*0.34 = 0.567
izz = (1/12)*20*(0.3²+0.2²) = (1/12)*20*0.13 = 0.217
```

**Upper arm** (box 0.1×0.1×0.4, 5kg):
```
ixx = (1/12)*5*(0.1²+0.4²) = 0.0683
iyy = (1/12)*5*(0.1²+0.4²) = 0.0683
izz = (1/12)*5*(0.1²+0.1²) = 0.0042
```

**Forearm** (box 0.08×0.08×0.3, 2kg):
```
ixx = (1/12)*2*(0.08²+0.3²) = 0.0150
iyy = (1/12)*2*(0.08²+0.3²) = 0.0150
izz = (1/12)*2*(0.08²+0.08²) = 0.0011
```

**Hand** (sphere 0.08m, 1kg):
```
i = (2/5)*1*0.08² = 0.0010
All three: 0.0010
```

**Shoulder/Wrist spheres** (sphere 0.05m, 0.5kg):
```
i = (2/5)*0.5*0.05² = 0.0001
All three: 0.0001
```

Use these values in the URDF above.

**Success Criteria Checklist:**
- [ ] All inertia values calculated
- [ ] Values applied to URDF
- [ ] URDF still validates
- [ ] Simulation feels realistic

</details>

## Complete Arm Behavior

When you load the complete arm in RViz:
- **Shoulder pitch**: Rotates arm up/down
- **Shoulder roll**: Rotates arm forward/back
- **Elbow**: Bends the forearm
- **Wrist**: Twists the hand

With realistic inertia values and joint limits, the arm behaves like a real robotic limb.

## Checking Your Work

Before moving to Lesson 7, ensure:
- [ ] You can build a complete arm URDF
- [ ] All joints work and have realistic limits
- [ ] Inertia values are calculated correctly
- [ ] The arm is ready for physics simulation

## Try With AI

Collaborate on extending the arm design.

**Part 1: Gripper Addition**

Ask your AI:
"I want to add a gripper (two fingers) to the hand. Create URDF for:
- Left finger: box 0.02×0.02×0.1m
- Right finger: mirror of left
- Both attached to hand_link with fixed joints
Calculate inertia for 0.2kg each"

**Part 2: Evaluate Response**

Check the output:
- Are fingers positioned correctly?
- Do they appear symmetric?
- Are inertia values reasonable?

**Part 3: Constraint Teaching**

Refine with your requirement:
"The fingers need to be closer together (currently 0.1m apart). Space them 0.04m apart instead."

**Part 4: Refinement**

AI updates positions. Load and verify in RViz.

**Part 5: Validation**

With gripper added, you now have a more complete arm:
- Torso (root)
- Shoulder (2 DOF)
- Elbow (1 DOF)
- Wrist (1 DOF)
- Gripper (fixed, 2 fingers)

This forms the foundation for humanoid upper body design in Lesson 8.
