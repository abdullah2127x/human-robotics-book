---
sidebar_position: 7
title: "Lesson 7: Reusable URDF Macros and Patterns"
description: "Extract common patterns into reusable xacro macros to reduce URDF complexity and improve maintainability."
---

# Lesson 7: Reusable URDF Macros and Patterns

The arm you built in Lesson 6 is functional, but it has repetition: similar joint structures (shoulder, elbow, wrist), similar link definitions (boxes with inertia calculations). In a humanoid robot with two arms, you'd duplicate all this code.

Xacro (XML Macros) solves this by letting you parameterize URDF, creating reusable templates. This is your first step toward designing reusable robot descriptions that you can apply across projects.

## What is Xacro?

Xacro is a preprocessor that expands macros into URDF. It lets you:
- **Define parameters**: Variables that replace repeated values
- **Create macros**: Templates for common structures
- **Use conditionals**: Build different models from one file
- **Include files**: Organize large models into modules

Here's the workflow:

```
my_robot.xacro (with macros)
    ↓ (xacro processes it)
    ↓ (substitutes parameters, expands macros)
    ↓
my_robot.urdf (standard URDF)
```

## Simple Macro: Parameterized Box Link

Instead of writing a box link repeatedly, define it once as a macro:

```xml
<?xml version="1.0"?>
<robot name="macro_example" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Macro definition -->
  <xacro:macro name="box_link" params="name width depth height mass color">
    <link name="${name}">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
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
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="${mass}"/>
        <!-- Simplified inertia; calculate properly in real use -->
        <inertia ixx="${(1/12)*mass*(depth**2 + height**2)}"
                 iyy="${(1/12)*mass*(width**2 + height**2)}"
                 izz="${(1/12)*mass*(width**2 + depth**2)}"
                 ixy="0" ixz="0" iyz="0"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:box_link name="torso" width="0.3" depth="0.2" height="0.5" mass="20" color="0.5 0.5 0.5 1"/>
  <xacro:box_link name="upper_arm" width="0.1" depth="0.1" height="0.4" mass="5" color="1 0 0 1"/>

</robot>
```

**Key elements**:
- `<xacro:macro name="...">`: Defines a reusable template
- `params="..."`: Parameters passed to the macro
- `${name}`, `${width}`, etc.: Variable substitution
- `<xacro:box_link ...>`: Invoke the macro with specific values

## Macro for Revolute Joints

Similarly, define a revolute joint macro:

```xml
<xacro:macro name="revolute_joint" params="name parent child axis lower upper effort velocity origin_xyz origin_rpy">
  <joint name="${name}" type="revolute">
    <parent link="${parent}"/>
    <child link="${child}"/>
    <axis xyz="${axis}"/>
    <origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>
    <limit lower="${lower}" upper="${upper}" effort="${effort}" velocity="${velocity}"/>
  </joint>
</xacro:macro>
```

Use it:
```xml
<xacro:revolute_joint name="shoulder_pitch" parent="torso" child="shoulder_link"
                      axis="0 1 0" lower="-1.57" upper="1.57" effort="50" velocity="1.0"
                      origin_xyz="0 0.1 0.25" origin_rpy="0 0 0"/>
```

## Complete Arm Using Macros

Here's the Lesson 6 arm rewritten with macros:

```xml
<?xml version="1.0"?>
<robot name="articulated_arm_macros" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Macros -->
  <xacro:macro name="box_link" params="name width depth height mass color">
    <link name="${name}">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
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
        <origin xyz="0 0 0" rpy="0 0 0"/>
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

  <!-- Links using macros -->
  <xacro:box_link name="torso" width="0.3" depth="0.2" height="0.5" mass="20" color="0.5 0.5 0.5 1"/>
  <xacro:sphere_link name="shoulder_link" radius="0.05" mass="0.5" color="0 0 1 1"/>
  <xacro:box_link name="upper_arm_link" width="0.1" depth="0.1" height="0.4" mass="5" color="1 0 0 1"/>
  <xacro:box_link name="forearm_link" width="0.08" depth="0.08" height="0.3" mass="2" color="1 0.5 0 1"/>
  <xacro:sphere_link name="hand_link" radius="0.08" mass="1" color="1 1 0 1"/>

  <!-- Joints using macros -->
  <xacro:revolute_joint name="shoulder_pitch" parent="torso" child="shoulder_link"
                        axis="0 1 0" lower="-1.57" upper="1.57" effort="50" velocity="1.0"
                        origin_xyz="0 0.1 0.25" origin_rpy="0 0 0"/>

  <xacro:revolute_joint name="shoulder_roll" parent="shoulder_link" child="upper_arm_link"
                        axis="0 0 1" lower="-1.047" upper="1.047" effort="50" velocity="1.0"
                        origin_xyz="0 0 0" origin_rpy="0 0 0"/>

  <xacro:revolute_joint name="elbow_joint" parent="upper_arm_link" child="forearm_link"
                        axis="0 1 0" lower="0" upper="2.61" effort="40" velocity="1.0"
                        origin_xyz="0 0 -0.4" origin_rpy="0 0 0"/>

  <xacro:revolute_joint name="wrist_joint" parent="forearm_link" child="hand_link"
                        axis="0 0 1" lower="-1.57" upper="1.57" effort="20" velocity="1.5"
                        origin_xyz="0 0 -0.3" origin_rpy="0 0 0"/>

</robot>
```

Compare to Lesson 6:
- **Original URDF**: ~120 lines
- **Macro version**: ~80 lines (33% shorter)
- **More important**: Patterns are reusable

## Xacro Processing

To convert xacro to URDF:

```bash
xacro arm.xacro > arm.urdf
```

Or in Python:
```python
from xacro import process_file
urdf_content = process_file('arm.xacro')
```

Then use the generated URDF with ROS tools.

## Exercise 3.7a: Create Link Macro

**Task**: Define a parameterized link macro for boxes.

<details>
<summary>Solution</summary>

Create `link_macro.xacro`:

```xml
<?xml version="1.0"?>
<robot name="link_macro_test" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="box_link" params="name width depth height mass color">
    <link name="${name}">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
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
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="${mass}"/>
        <inertia ixx="${(1/12)*mass*(depth**2 + height**2)}"
                 iyy="${(1/12)*mass*(width**2 + height**2)}"
                 izz="${(1/12)*mass*(width**2 + depth**2)}"
                 ixy="0" ixz="0" iyz="0"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Test the macro -->
  <xacro:box_link name="segment1" width="1.0" depth="0.1" height="0.1" mass="2" color="1 0 0 1"/>
  <xacro:box_link name="segment2" width="0.8" depth="0.15" height="0.15" mass="3" color="0 1 0 1"/>

</robot>
```

Process it:
```bash
xacro link_macro.xacro > test_output.urdf
check_urdf test_output.urdf
```

Should expand to 2 links with calculated inertia.

**Success Criteria Checklist:**
- [ ] Macro defined with parameters
- [ ] Can be invoked multiple times
- [ ] Xacro processes without errors
- [ ] Generated URDF is valid

</details>

## Exercise 3.7b: Create Joint Macro

**Task**: Define a parameterized joint macro.

<details>
<summary>Solution</summary>

Add to your xacro file:

```xml
<xacro:macro name="revolute_joint" params="name parent child axis lower upper effort velocity origin_xyz origin_rpy">
  <joint name="${name}" type="revolute">
    <parent link="${parent}"/>
    <child link="${child}"/>
    <axis xyz="${axis}"/>
    <origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>
    <limit lower="${lower}" upper="${upper}" effort="${effort}" velocity="${velocity}"/>
  </joint>
</xacro:macro>
```

Use it:
```xml
<xacro:revolute_joint name="joint1" parent="segment1" child="segment2"
                      axis="0 1 0" lower="-1.57" upper="1.57" effort="10" velocity="1.0"
                      origin_xyz="0 0 0" origin_rpy="0 0 0"/>
```

**Success Criteria Checklist:**
- [ ] Joint macro defined
- [ ] Can accept all parameters
- [ ] Expands correctly in URDF

</details>

## Exercise 3.7c: Rebuild Arm with Macros

**Task**: Rewrite the Lesson 6 arm using macros. Verify it produces identical behavior.

**Goal**: ~30% reduction in file size while maintaining functionality.

<details>
<summary>Solution</summary>

Use the complete macro-based arm URDF from the "Complete Arm Using Macros" section above.

Process it:
```bash
xacro arm_macros.xacro > arm_macros.urdf
check_urdf arm_macros.urdf
```

Load in RViz and verify:
- All 4 joints present
- All 5 links visible
- Behavior identical to non-macro version

Compare file sizes:
```bash
wc -l arm.urdf arm_macros.xacro
# Original should be ~120 lines, macro version ~80 lines
```

**Success Criteria Checklist:**
- [ ] Xacro file valid
- [ ] Generates valid URDF
- [ ] File size reduced
- [ ] Behavior identical to Lesson 6

</details>

## Why Macros Matter

Macros solve the **reusability problem**:

**Without macros**: Building a humanoid with 2 identical arms means duplicating 100+ lines of code. Changes to one arm require updating both.

**With macros**: Define arm once, instantiate twice with different parameters:

```xml
<!-- Humanoid torso with two identical arms -->
<xacro:box_link name="torso" width="0.3" depth="0.2" height="0.5" mass="20" color="0.5 0.5 0.5 1"/>

<!-- Left arm -->
<xacro:revolute_joint name="left_shoulder_pitch" parent="torso" child="left_shoulder_link"
                      axis="0 1 0" lower="-1.57" upper="1.57" effort="50" velocity="1.0"
                      origin_xyz="0.15 0 0.25" origin_rpy="0 0 0"/>
<!-- ... more left arm joints ... -->

<!-- Right arm (mirror position) -->
<xacro:revolute_joint name="right_shoulder_pitch" parent="torso" child="right_shoulder_link"
                      axis="0 1 0" lower="-1.57" upper="1.57" effort="50" velocity="1.0"
                      origin_xyz="-0.15 0 0.25" origin_rpy="0 0 0"/>
<!-- ... more right arm joints ... -->
```

## Checking Your Work

Before moving to Lesson 8, ensure:
- [ ] You understand xacro macro syntax
- [ ] You can create parameterized macros
- [ ] You can process xacro into URDF
- [ ] Macro-based design produces identical functionality
- [ ] File size is reduced through reuse

## Try With AI

Collaborate on advanced macro design.

**Part 1: Cylinder Macro**

Ask your AI:
"Create a xacro macro for cylinder links that includes inertia calculation. Include parameters for radius, height, mass, and color. Show how to use it for arm segments."

**Part 2: Evaluate the Response**

Check if:
- Macro parameters are well-named
- Inertia calculation is correct
- Macro is easy to invoke

**Part 3: Constraint Teaching**

Provide your use case:
"I need cylinder segments with different radii (0.1m, 0.08m, 0.05m) but same length (0.4m). How do I parameterize this efficiently?"

**Part 4: Refinement**

AI suggests using lists or loops. You adapt for your humanoid design.

**Part 5: Validation**

Load the refined macro design and verify:
- Can you create multiple segments easily?
- Are they properly connected?
- Can you maintain two mirrored arms with minimal duplication?

This collaborative design process teaches you to think about reusability at the macro level—the foundation for Lesson 8's capstone design.
