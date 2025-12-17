---
sidebar_position: 1
title: "Lesson 1: URDF Fundamentals and Single Link"
description: "Master URDF XML syntax and create your first rigid body with visual geometry."
---

# Lesson 1: URDF Fundamentals and Single Link

The Unified Robot Description Format (URDF) is how we describe robots to ROS2. It's an XML-based format that defines the physical structure of a robot—the rigid bodies (links) and how they connect (joints). Before building articulated systems, you need to understand how to define a single link with visual properties.

In this lesson, you'll write your first URDF file from scratch, verify it with ROS2 tools, and visualize it in RViz.

## What is URDF?

URDF stands for Unified Robot Description Format. It's a standard XML format used by ROS to describe:
- **Links**: Rigid bodies that represent physical segments (links, cylinders, boxes)
- **Joints**: Connections between links with movement constraints
- **Geometry**: Visual representation and collision shapes
- **Properties**: Mass, inertia tensors, and frame transformations

Think of URDF as a blueprint. Just as an architect defines a building's structure with blueprints, you define a robot's structure with URDF.

## URDF XML Structure Basics

Every URDF file follows this structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links and joints go here -->
</robot>
```

The `<robot>` element is the root container. Inside it, you define `<link>` and `<joint>` elements that make up your robot.

## Creating Your First Link

A link is the simplest building block. It represents a rigid body. Here's a complete URDF with a single link:

```xml
<?xml version="1.0"?>
<robot name="single_link_robot">

  <link name="base">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1.0 1.0 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

</robot>
```

Let's break this down:

**`<link name="base">`**: Defines a link named "base". This will be your reference frame.

**`<visual>`**: Describes how the link looks in visualization tools like RViz.

**`<origin>`**: Specifies the position (xyz) and rotation (rpy = roll, pitch, yaw) of the geometry relative to the link's frame. All zeros means the geometry is at the origin with no rotation.

**`<geometry>`**: Defines the shape. In this case, a box with dimensions 1.0m × 1.0m × 0.5m (width × depth × height).

**`<material>`**: Optional but helpful. Gives the geometry a color for visualization. The RGBA values are (red, green, blue, alpha). `0 0 1 1` is opaque blue.

## Understanding Geometry Types

URDF supports several geometry types:

**Box**: Defined by width, depth, and height.
```xml
<geometry>
  <box size="1.0 1.0 0.5"/>
</geometry>
```

**Cylinder**: Defined by radius and height.
```xml
<geometry>
  <cylinder radius="0.1" length="0.5"/>
</geometry>
```

**Sphere**: Defined by radius only.
```xml
<geometry>
  <sphere radius="0.15"/>
</geometry>
```

**Mesh**: References an external mesh file (for complex shapes).
```xml
<geometry>
  <mesh filename="package://my_robot/meshes/arm.dae"/>
</geometry>
```

For now, we'll focus on boxes, cylinders, and spheres—these are sufficient for humanoid modeling.

## Exercise 3.1a: Create a Minimal URDF

Create a new file called `single_box.urdf` with a single blue box link.

**Task:**
1. Create a new file with the URDF from above
2. Save it as `single_box.urdf`
3. Open a terminal and run: `check_urdf single_box.urdf`
4. The tool should print: "robot description is OK"

<details>
<summary>Solution</summary>

Create this file as `single_box.urdf`:

```xml
<?xml version="1.0"?>
<robot name="single_link_robot">

  <link name="base">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1.0 1.0 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

</robot>
```

Run the validator:
```bash
check_urdf single_box.urdf
```

**Output:**
```
robot description is OK
```

**Success Criteria Checklist:**
- [ ] File created and named correctly
- [ ] URDF syntax is valid (check_urdf passes)
- [ ] Link named "base"
- [ ] Geometry is a box with correct dimensions

</details>

## Exercise 3.1b: Visualize in RViz

Now let's see your robot in RViz. You'll need a launch file to load the URDF and start RViz.

**Task:**
1. Create a launch file that loads your URDF
2. Run the launch file
3. Observe the blue box in RViz

<details>
<summary>Solution Guide</summary>

Create a launch file called `view_robot.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf', 'single_box.urdf'
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
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),
    ])
```

Run with:
```bash
ros2 launch my_robot view_robot.launch.py
```

RViz will open and display your blue box at the origin.

**Success Criteria Checklist:**
- [ ] Launch file created
- [ ] RViz opens successfully
- [ ] Blue box visible in RViz
- [ ] Box appears at origin (0, 0, 0)

</details>

## Exercise 3.1c: Modify Geometry Dimensions

Change your URDF to create different shapes and observe how the visualization changes.

**Task:**
1. Modify the box dimensions to `2.0 0.5 0.5` (a long, thin stick)
2. Save the file
3. Reload in RViz and observe the change
4. Try changing to a cylinder: `<cylinder radius="0.2" length="1.0"/>`
5. Reload again and observe the new shape

<details>
<summary>Solution</summary>

**Version 1: Elongated box**
```xml
<geometry>
  <box size="2.0 0.5 0.5"/>
</geometry>
```

**Version 2: Cylinder**
```xml
<geometry>
  <cylinder radius="0.2" length="1.0"/>
</geometry>
```

When you reload RViz, the visualization updates in real-time (or refresh by selecting the robot_state_publisher node and updating the parameter).

**Success Criteria Checklist:**
- [ ] Modified box dimensions
- [ ] URDF still validates
- [ ] Visual change visible in RViz
- [ ] Tried cylinder geometry successfully

</details>

## XML Syntax Rules (Important!)

When writing URDF, follow these XML rules or you'll get validation errors:

1. **Proper nesting**: Every opening tag must have a closing tag
   ```xml
   <link name="base">
     <!-- content -->
   </link>  <!-- Correct closing tag -->
   ```

2. **Attributes in quotes**: All attribute values must be quoted
   ```xml
   <box size="1.0 1.0 0.5"/>  <!-- Correct -->
   <box size=1.0 1.0 0.5/>    <!-- WRONG: missing quotes -->
   ```

3. **Self-closing tags**: For elements with no children, use `/>` at the end
   ```xml
   <geometry>
     <box size="1.0 1.0 0.5"/>  <!-- Self-closing -->
   </geometry>
   ```

4. **Whitespace doesn't matter**: You can indent for readability
   ```xml
   <!-- Both are valid -->
   <link name="base"><visual><geometry><box size="1.0 1.0 0.5"/></geometry></visual></link>

   <link name="base">
     <visual>
       <geometry>
         <box size="1.0 1.0 0.5"/>
       </geometry>
     </visual>
   </link>
   ```

## Common URDF Attributes

Here are attributes you'll use frequently:

| Attribute | Location | Meaning | Example |
|-----------|----------|---------|---------|
| `name` | `<robot>`, `<link>`, `<joint>` | Unique identifier | `name="base"` |
| `xyz` | `<origin>` | X, Y, Z position in meters | `xyz="0 0 0.5"` |
| `rpy` | `<origin>` | Roll, pitch, yaw in radians | `rpy="0 0 1.57"` |
| `size` | `<box>` | Width, depth, height | `size="1.0 0.5 0.2"` |
| `radius` | `<cylinder>`, `<sphere>` | Radius in meters | `radius="0.1"` |
| `length` | `<cylinder>` | Height in meters | `length="0.5"` |
| `rgba` | `<color>` | Red, green, blue, alpha | `rgba="1 0 0 1"` |

## Checking Your Work

Before proceeding to the next lesson, make sure:
- [ ] You can create a valid URDF file
- [ ] `check_urdf` validates your file
- [ ] You can visualize it in RViz
- [ ] You can modify geometry and see the change

## Try With AI

Now that you understand URDF structure, collaborate with AI to explore variations.

**Part 1: Experiment with Different Shapes**

Ask your AI tool:
"I have a basic URDF with a box geometry. Create variations with cylinder and sphere geometries. Show me the URDF for each."

Review each response. Notice how AI follows the same XML structure with different geometry elements.

**Part 2: Critical Evaluation**

After reviewing AI's responses, ask yourself:
- Do the geometries match the XML structure I learned?
- What's the difference between `<box size="..."/>` and `<cylinder radius="..." length="..."/>`?
- Could I identify a syntax error in AI's code?

**Part 3: Constraint Teaching**

Tell your AI the specific constraint you're working with:

"I'm building a humanoid robot. The torso should be a box 0.3m wide, 0.2m deep, and 0.5m tall. Create the URDF for just the torso link with appropriate color."

**Part 4: Validation**

Ask your AI to verify:
"Does this URDF follow the structure I showed you? Are all the XML tags properly nested?"

**Part 5: Final Reflection**

Compare what you created manually to what AI generated:
- Which approach was faster?
- Did AI make any syntax errors?
- How confident are you evaluating AI's URDF code?
