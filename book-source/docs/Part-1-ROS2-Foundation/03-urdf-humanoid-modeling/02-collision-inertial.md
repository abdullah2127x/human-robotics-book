---
sidebar_position: 2
title: "Lesson 2: Collision Geometry and Link Properties"
description: "Add collision shapes and inertial properties to create physically realistic links."
---

# Lesson 2: Collision Geometry and Link Properties

In Lesson 1, you created a link with visual geometry—how it *looks* in RViz. But simulation needs more information. When robots interact with their environment, they need collision geometry for detecting collisions. When they move, they need inertial properties (mass, center of mass, rotational inertia) for physics calculations.

This lesson teaches you to add these properties to make your links physically realistic.

## Why Three Geometries?

Every link can have three different geometries:

**Visual**: How the link looks in visualization tools (RViz)
- Can be detailed and beautiful
- Used only for display
- Doesn't affect physics

**Collision**: Simplified shape for collision detection
- Often simpler than visual (faster computation)
- Used by physics engine to detect when objects collide
- Can be different from visual geometry

**Inertial**: Mass distribution for physics simulation
- Defines where mass is concentrated
- Determines how link responds to forces
- Usually centered at center of mass

Here's why they're different: A robot arm might have a detailed 3D mesh for visualization (looks nice in RViz), but a simpler cylinder for collision (fast collision checking), and its mass concentrated at the joint (inertial property).

## Example: Link with All Three Properties

```xml
<?xml version="1.0"?>
<robot name="complete_link_robot">

  <link name="arm_segment">
    <!-- Visual: How it looks -->
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1.0 0.1 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <!-- Collision: For collision detection -->
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.9 0.08 0.08"/>
      </geometry>
    </collision>

    <!-- Inertial: Mass and inertia for physics -->
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
  </link>

</robot>
```

Notice:
- **Visual** and **Collision** can have different dimensions
- **Collision** is often slightly smaller (physical geometry might extend beyond collision volume)
- **Inertial** has mass and an inertia tensor (3×3 matrix, simplified to 6 values for a symmetric matrix)

## Understanding Inertial Properties

The `<inertial>` element contains three pieces of information:

**Mass (value)**: Total weight in kilograms
```xml
<mass value="2.0"/>  <!-- 2 kilogram link -->
```

**Center of Mass (origin)**: Where the mass is concentrated, relative to link frame
```xml
<origin xyz="0 0 0" rpy="0 0 0"/>  <!-- Centered at origin -->
```

**Inertia Tensor**: How mass is distributed, affecting rotational dynamics

The inertia tensor is a 3×3 symmetric matrix:
```
[ixx  ixy  ixz]
[ixy  iyy  iyz]
[ixz  iyz  izz]
```

For links with simple shapes centered at origin, you only need diagonal elements:
```xml
<inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
```

**ixx, iyy, izz**: Moment of inertia around x, y, z axes (in kg⋅m²)
**ixy, ixz, iyz**: Off-diagonal elements (usually 0 for aligned geometries)

## Inertia Formulas for Common Shapes

Here are formulas for calculating inertia. We'll dive deeper in Lesson 4, but you need to know these basics now.

### Box (solid rectangular)
- Dimensions: width (w), depth (d), height (h)
- Formula:
  ```
  ixx = (1/12) * m * (d² + h²)
  iyy = (1/12) * m * (w² + h²)
  izz = (1/12) * m * (w² + d²)
  ```

**Example**: Box 1.0m × 0.1m × 0.1m, mass 2.0 kg
```
ixx = (1/12) * 2.0 * (0.1² + 0.1²) = 0.00333
iyy = (1/12) * 2.0 * (1.0² + 0.1²) = 0.1683
izz = (1/12) * 2.0 * (1.0² + 0.1²) = 0.1683
```

### Cylinder (solid)
- Dimensions: radius (r), height (h)
- Formula:
  ```
  ixx = (1/12) * m * h² + (1/4) * m * r²
  iyy = (1/12) * m * h² + (1/4) * m * r²
  izz = (1/2) * m * r²
  ```

### Sphere (solid)
- Dimensions: radius (r)
- Formula:
  ```
  ixx = iyy = izz = (2/5) * m * r²
  ```

## Exercise 3.2a: Add Collision Geometry

Start with your URDF from Lesson 1. Add a collision element that's slightly smaller than the visual.

**Task:**
1. Add a `<collision>` element to your "base" link
2. Use the same box shape, but slightly smaller (e.g., 10% reduction)
3. Run `check_urdf` to verify
4. Visualize in RViz (collision geometry isn't shown by default, but the URDF is valid)

<details>
<summary>Solution</summary>

Modify your URDF to include collision:

```xml
<?xml version="1.0"?>
<robot name="single_link_robot">

  <link name="base">
    <!-- Visual: How it looks -->
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1.0 1.0 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>

    <!-- Collision: For physics simulation -->
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.9 0.9 0.45"/>
      </geometry>
    </collision>
  </link>

</robot>
```

Run:
```bash
check_urdf your_file.urdf
```

**Success Criteria Checklist:**
- [ ] `<collision>` element added
- [ ] Has `<geometry>` with box shape
- [ ] Dimensions slightly smaller than visual
- [ ] `check_urdf` passes
- [ ] URDF file still valid

</details>

## Exercise 3.2b: Add Inertial Properties

Now add the inertial element with mass and inertia tensor.

**Task:**
1. Calculate inertia for your box (use formulas above)
2. Add `<inertial>` element with mass 10 kg
3. Include calculated inertia values
4. Verify with `check_urdf`

<details>
<summary>Solution</summary>

For a 1.0m × 1.0m × 0.5m box with 10 kg mass:

Calculate inertia:
```
ixx = (1/12) * 10 * (1.0² + 0.5²) = (1/12) * 10 * 1.25 = 1.0417
iyy = (1/12) * 10 * (1.0² + 0.5²) = 1.0417  (same, symmetric)
izz = (1/12) * 10 * (1.0² + 1.0²) = (1/12) * 10 * 2.0 = 1.6667
```

Add to your URDF:

```xml
<?xml version="1.0"?>
<robot name="single_link_robot">

  <link name="base">
    <!-- Visual -->
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1.0 1.0 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>

    <!-- Collision -->
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.9 0.9 0.45"/>
      </geometry>
    </collision>

    <!-- Inertial -->
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="1.0417" ixy="0" ixz="0" iyy="1.0417" iyz="0" izz="1.6667"/>
    </inertial>
  </link>

</robot>
```

Run:
```bash
check_urdf your_file.urdf
```

**Success Criteria Checklist:**
- [ ] `<inertial>` element present
- [ ] Mass value set to 10.0
- [ ] Inertia values calculated correctly
- [ ] `check_urdf` passes
- [ ] All three geometries now present (visual, collision, inertial)

</details>

## Exercise 3.2c: Modify Mass and Observe Impact

Change the mass and observe how inertia values change.

**Task:**
1. Change mass from 10 kg to 50 kg
2. Recalculate inertia values (multiply by 5)
3. Update URDF and verify
4. Explain: How does mass affect inertia?

<details>
<summary>Solution and Explanation</summary>

For 50 kg (5× the original):

```
ixx = 1.0417 * 5 = 5.2083
iyy = 1.0417 * 5 = 5.2083
izz = 1.6667 * 5 = 8.3333
```

Update URDF:
```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="50.0"/>
  <inertia ixx="5.2083" ixy="0" ixz="0" iyy="5.2083" iyz="0" izz="8.3333"/>
</inertial>
```

**Explanation:**
Inertia is proportional to mass. If you double the mass (keeping geometry same), all inertia values double. This makes physical sense: a heavier object is harder to rotate because it has more inertia.

In simulation:
- Heavier objects fall faster due to gravity
- Heavier objects are harder to move (require more force)
- Heavier objects are harder to rotate (require more torque)

</details>

## Collision Geometry Considerations

When designing collision geometry:

1. **Simpler is usually faster**: Collision detection is computationally expensive. Simpler shapes (box, cylinder) are faster than complex meshes.

2. **Slightly smaller is safer**: If collision geometry is slightly smaller than visual, objects won't appear to overlap in simulation when they touch in visualization.

3. **Match physical constraints**: Collision geometry should represent where objects can't pass through.

4. **Different from visual is OK**: You might have a detailed visual mesh but a simple collision cylinder for performance.

## Common Inertia Mistakes

1. **Forgetting to calculate**: Many robots don't move realistically because inertia is wrong.

2. **Using wrong formula**: Make sure you're using the right shape formula (box vs cylinder vs sphere).

3. **Off-diagonal elements**: For most cases, ixy, ixz, iyz should be 0. Only use if mass isn't aligned with axes.

4. **Unrealistic values**: If inertia values are very small or very large compared to mass, double-check calculations.

## Checking Your Work

Before proceeding to Lesson 3, verify:
- [ ] You can add collision geometry to a link
- [ ] You can calculate inertia for a box
- [ ] URDF validates with all three properties
- [ ] You understand why three geometries exist

## Try With AI

Now you can collaborate with AI on more complex link designs.

**Part 1: Design a Cylinder Link**

Ask your AI tool:
"I need a cylindrical arm segment. Create URDF with:
- Visual: cylinder radius 0.1m, height 0.5m, red color
- Collision: slightly smaller cylinder
- Inertial: mass 5 kg
- Calculate the inertia tensor for this cylinder"

Review the response. Check:
- Are the formulas correct?
- Is the inertia calculation accurate?

**Part 2: Evaluate the Response**

Ask yourself:
- Did AI use the correct inertia formula for a cylinder?
- Are the inertia values reasonable?
- Could there be an error in the calculation?

**Part 3: Constraint Teaching**

Provide your actual requirements:
"This cylinder is for a humanoid robot arm. The real segment needs to support 20 kg at the end. Should I adjust the mass or inertia values?"

**Part 4: Refinement**

Ask AI to recalculate with your constraint:
"Recalculate the inertia for 10 kg (not 5 kg) to support the load better."

**Part 5: Validation Check**

Compare your understanding to AI's response:
- Can you explain why heavier segments provide better load support?
- Do you understand the relationship between mass and inertia?
- Can you identify if AI made calculation errors?

This collaborative process builds your ability to work with AI on physics-based modeling.
