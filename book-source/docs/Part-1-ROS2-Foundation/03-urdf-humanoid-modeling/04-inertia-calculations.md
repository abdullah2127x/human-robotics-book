---
sidebar_position: 4
title: "Lesson 4: Calculating Inertial Properties"
description: "Master inertia formulas for common shapes and apply calculations to realistic robot segments."
---

# Lesson 4: Calculating Inertial Properties

In Lesson 2, you learned that every link needs inertial properties (mass and inertia tensor) for realistic physics simulation. But where do those numbers come from? This lesson teaches you to calculate them.

Incorrect inertia values produce unrealistic simulation behavior: links fall too fast or too slow, joints feel weightless or impossibly heavy. Getting inertia right is essential for believable robot motion.

## The Inertia Tensor

The inertia tensor is a 3×3 matrix that describes how mass is distributed:

```
[ixx  ixy  ixz]
[ixy  iyy  iyz]
[ixz  iyz  izz]
```

For standard geometric shapes with mass centered at the origin and aligned with coordinate axes, the tensor simplifies:

```
[ixx   0    0 ]
[ 0   iyy   0 ]
[ 0    0   izz]
```

You only need to calculate three values: **ixx**, **iyy**, **izz**

The off-diagonal elements (ixy, ixz, iyz) are usually zero unless mass is distributed asymmetrically.

## Formulas for Common Shapes

### Box (Solid Rectangular Prism)

For a box with width **w**, depth **d**, height **h**, and mass **m**:

```
ixx = (1/12) * m * (d² + h²)
iyy = (1/12) * m * (w² + h²)
izz = (1/12) * m * (w² + d²)
```

**Intuition**: ixx increases with the square of dimensions perpendicular to the X-axis (d and h). Larger objects have more inertia around each axis.

**Example**: Box 1.0m × 0.1m × 0.1m, mass 2.0 kg

```
ixx = (1/12) * 2.0 * (0.1² + 0.1²)
    = (1/12) * 2.0 * (0.01 + 0.01)
    = (1/12) * 2.0 * 0.02
    = 0.00333 kg⋅m²

iyy = (1/12) * 2.0 * (1.0² + 0.1²)
    = (1/12) * 2.0 * 1.01
    = 0.1683 kg⋅m²

izz = (1/12) * 2.0 * (1.0² + 0.1²)
    = (1/12) * 2.0 * 1.01
    = 0.1683 kg⋅m²
```

Notice: ixx is much smaller because the box is thin in those directions. Rotating around the long axis requires less torque; rotating around short axes requires more torque.

### Cylinder (Solid)

For a cylinder with radius **r**, height **h**, and mass **m**:

```
ixx = (1/12) * m * h² + (1/4) * m * r²
iyy = (1/12) * m * h² + (1/4) * m * r²
izz = (1/2) * m * r²
```

**Note**: ixx and iyy are equal (rotational symmetry around Z-axis).

**Example**: Cylinder radius 0.1m, height 0.5m, mass 5.0 kg

```
ixx = (1/12) * 5.0 * 0.5² + (1/4) * 5.0 * 0.1²
    = (1/12) * 5.0 * 0.25 + (1/4) * 5.0 * 0.01
    = 0.1042 + 0.0125
    = 0.1167 kg⋅m²

iyy = 0.1167 kg⋅m² (same)

izz = (1/2) * 5.0 * 0.1²
    = (1/2) * 5.0 * 0.01
    = 0.025 kg⋅m²
```

Notice: izz is much smaller because the cylinder is thin in that dimension.

### Sphere (Solid)

For a sphere with radius **r** and mass **m**:

```
ixx = iyy = izz = (2/5) * m * r²
```

All three values are equal due to spherical symmetry.

**Example**: Sphere radius 0.15m, mass 2.0 kg

```
i = (2/5) * 2.0 * 0.15²
  = (2/5) * 2.0 * 0.0225
  = 0.018 kg⋅m²
```

All three: ixx = iyy = izz = 0.018

## Exercise 3.4a: Calculate Box Inertia

**Task**: Calculate inertia for a box 1.0m × 0.5m × 0.3m with 10 kg mass.

<details>
<summary>Solution</summary>

Given:
- Width (w) = 1.0m
- Depth (d) = 0.5m
- Height (h) = 0.3m
- Mass (m) = 10 kg

Calculate:
```
ixx = (1/12) * 10 * (0.5² + 0.3²)
    = (1/12) * 10 * (0.25 + 0.09)
    = (1/12) * 10 * 0.34
    = 0.2833 kg⋅m²

iyy = (1/12) * 10 * (1.0² + 0.3²)
    = (1/12) * 10 * (1.0 + 0.09)
    = (1/12) * 10 * 1.09
    = 0.9083 kg⋅m²

izz = (1/12) * 10 * (1.0² + 0.5²)
    = (1/12) * 10 * (1.0 + 0.25)
    = (1/12) * 10 * 1.25
    = 1.0417 kg⋅m²
```

**Verification**: All positive, reasonable values ✓

**Success Criteria Checklist:**
- [ ] All three components calculated
- [ ] Formula correctly applied
- [ ] Values are positive
- [ ] Values scale with mass

</details>

## Exercise 3.4b: Calculate Cylinder Inertia

**Task**: Calculate inertia for a cylinder radius 0.1m, height 0.5m, mass 5.0 kg.

<details>
<summary>Solution</summary>

Given:
- Radius (r) = 0.1m
- Height (h) = 0.5m
- Mass (m) = 5.0 kg

Calculate:
```
ixx = (1/12) * 5.0 * 0.5² + (1/4) * 5.0 * 0.1²
    = (1/12) * 5.0 * 0.25 + (1/4) * 5.0 * 0.01
    = 0.1042 + 0.0125
    = 0.1167 kg⋅m²

iyy = 0.1167 kg⋅m² (same due to symmetry)

izz = (1/2) * 5.0 * 0.1²
    = (1/2) * 5.0 * 0.01
    = 0.025 kg⋅m²
```

**Pattern**: izz is much smaller because the cylinder is thin along its axis.

**Success Criteria Checklist:**
- [ ] ixx ≈ iyy (symmetry)
- [ ] izz < ixx (thin axis)
- [ ] Values positive and reasonable

</details>

## Exercise 3.4c: Calculate Sphere Inertia

**Task**: Calculate inertia for a sphere radius 0.15m, mass 3.0 kg.

<details>
<summary>Solution</summary>

Given:
- Radius (r) = 0.15m
- Mass (m) = 3.0 kg

Calculate:
```
i = (2/5) * 3.0 * 0.15²
  = (2/5) * 3.0 * 0.0225
  = 0.027 kg⋅m²
```

All three:
```
ixx = iyy = izz = 0.027 kg⋅m²
```

**Pattern**: All equal due to spherical symmetry.

**Success Criteria Checklist:**
- [ ] All three values equal
- [ ] Correct formula applied
- [ ] Value positive

</details>

## Exercise 3.4d: Apply Calculations to URDF

**Task**: Create a URDF with a cylinder link. Use calculated inertia values from Exercise 3.4b.

**Given**:
- Cylinder radius 0.1m, height 0.5m, mass 5.0 kg
- Color it red for visibility
- Add collision geometry (slightly smaller)

<details>
<summary>Solution</summary>

```xml
<?xml version="1.0"?>
<robot name="cylinder_link_robot">

  <link name="arm_segment">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.5"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.095" length="0.48"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="5.0"/>
      <inertia ixx="0.1167" ixy="0" ixz="0" iyy="0.1167" iyz="0" izz="0.025"/>
    </inertial>
  </link>

</robot>
```

Verify:
```bash
check_urdf your_file.urdf
```

**Success Criteria Checklist:**
- [ ] URDF valid
- [ ] Inertia values from calculation
- [ ] Mass matches
- [ ] Collision slightly smaller than visual

</details>

## Common Calculation Mistakes

1. **Forgetting to square dimensions**: Inertia formulas have d², h², r². Missing the squares gives wrong orders of magnitude.

2. **Using diameter instead of radius**: Cylinder formulas use **r** (radius), not **d** (diameter).

3. **Swapping width/depth/height**: Box inertia depends on which dimension is which. Ixx uses dimensions perpendicular to X-axis.

4. **Wrong units**: Ensure mass is in kg and dimensions in meters. Inertia will be in kg⋅m².

5. **Assuming off-diagonal zeros**: If your geometry isn't centered at origin or aligned with axes, ixy, ixz, iyz may not be zero. For now, assume they're zero.

## Validation Technique

A quick sanity check: Calculate total moment of inertia:

```
I_total = ixx + iyy + izz
```

Compare to point-mass approximation:
```
I_point = m * d²
```

Where d is distance from origin. For a distributed mass, I_total should be less than I_point.

**Example**: 5 kg cylinder with radius 0.1m

```
I_total = 0.1167 + 0.1167 + 0.025 = 0.2584 kg⋅m²
I_point = 5.0 * 0.1² = 0.05 kg⋅m²  (if all at edge)
```

Our total is reasonable since mass is distributed, not at edge. ✓

## Checking Your Work

Before moving to Lesson 5, ensure:
- [ ] You can calculate inertia for box, cylinder, sphere
- [ ] You apply the correct formula for each shape
- [ ] You can identify calculation errors
- [ ] You understand the relationship between geometry and inertia

## Try With AI

Collaborate with AI on inertia validation and humanoid design.

**Part 1: AI Teaches Common Mistakes**

Ask your AI tool:
"I calculated inertia for a cylinder but the values seem wrong. Here's my calculation: [show your work]. Can you review and identify any mistakes?"

Observe how AI catches calculation errors.

**Part 2: Evaluate the Feedback**

Review AI's response:
- Did it identify the mistake?
- Is the correction accurate?
- Do you understand the error now?

**Part 3: Constraint Teaching**

Provide your design constraint:
"I'm building an arm segment that needs to support 20 kg at the end. My cylinder segment is 0.1m radius, 0.5m long. Is 5 kg enough mass for realistic physics, or should I increase it?"

**Part 4: Recalculation with Feedback**

AI suggests a new mass. You respond:
"That feels heavy. Let's try 7 kg instead. Recalculate the inertia."

AI recalculates. You now have values tailored to your design.

**Part 5: Final Validation**

Compare original to refined calculation:
- Did iteration improve the design?
- Do the final inertia values match your constraint?
- Can you explain why heavier segments feel different in simulation?

This process builds confidence in inertia calculations for realistic humanoid segments.
