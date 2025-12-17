---
title: "Lesson 2: URDF Import and Basic Visualization"
chapter: 5
lesson: 2
proficiency_level: B2
learning_objectives:
  - "Import URDF model into Unity using URDF Importer"
  - "Understand model hierarchy and parent-child relationships"
  - "Apply materials and basic shaders to imported meshes"
  - "Position camera for effective model viewing"
  - "Verify model structure matches URDF definition"
estimated_time: "90 minutes"
generated_by: content-implementer v1.0.0
created: 2025-12-16
version: 1.0.0
---

# Lesson 2: URDF Import and Basic Visualization

## Introduction

From Chapter 4, you have a URDF definition of a humanoid robot. This XML file precisely describes the robot's structure: links (solid parts), joints (connections), and geometry (meshes).

Now you'll translate that abstract description into visual objects in Unity. This lesson teaches you to import URDF files and understand the model hierarchy that emerges.

Think of URDF as a blueprint, and this lesson converts that blueprint into a 3D object you can see, manipulate, and eventually animate.

**Estimated time**: 90 minutes
**Concept density**: 5 new concepts (within B2 limit)

---

## The URDF to Unity Translation Process

When you import a URDF, the importer creates:

```
URDF File (XML)
    ├── Links (rigid bodies)
    │   ├── base_link
    │   ├── torso
    │   ├── upper_arm_left
    │   ├── forearm_left
    │   ├── hand_left
    │   └── ... (continues for right side and legs)
    ├── Joints (connections between links)
    │   ├── torso_joint
    │   ├── shoulder_left_joint
    │   ├── elbow_left_joint
    │   └── ... (each joint specifies parent and child links)
    └── Collisions and Visuals (mesh files referenced)

        ↓ URDF Importer ↓

Unity Prefab (Hierarchy)
    ├── Humanoid (root GameObject)
    │   ├── BaseLink (parent)
    │   │   ├── Torso (child, connected by joint)
    │   │   │   ├── UpperArmLeft (child)
    │   │   │   │   ├── ForearmLeft (child)
    │   │   │   │   │   └── HandLeft (child)
    │   │   │   │       [mirror structure for right side]
    │   │   │   └── LeftThigh (child)
    │   │   │       └── LeftCalf (child)
    │   │   │           └── LeftFoot (child)
    │   │   │       [mirror structure for right side]
```

Each link becomes a GameObject. Each joint becomes a configuration between parent and child.

---

## Step 1: Prepare Your URDF File

From Chapter 4, you should have `humanoid.urdf`. If not, create a minimal humanoid:

`humanoid.urdf`:
```xml
<?xml version="1.0" ?>
<robot name="humanoid">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Torso link -->
  <link name="torso">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.2"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.3"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Right arm -->
  <link name="right_shoulder">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <!-- Joints connect links -->
  <joint name="base_torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <joint name="torso_right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_shoulder"/>
    <origin xyz="0.1 -0.1 0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>
```

Save as `Assets/ROS/humanoid.urdf`

---

## Step 2: Install URDF Importer

In Unity Package Manager:

1. **Window → Package Manager**
2. **+  → Add package from git URL**
3. Paste: `https://github.com/Unity-Robotics/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`

Wait for installation. You should see:
```
Robotics URDF Importer (by Unity Technologies)
```

---

## Step 3: Import the URDF

1. In Project, right-click `Assets/` → **Robotics → Import URDF**
2. Navigate to `Assets/ROS/humanoid.urdf`
3. Configure import settings:

   **Import Settings Dialog**:
   ```
   ☑ Import mesh as convex colliders
   ☑ Import mesh as game objects
   ☑ Import materials from mesh
   [X] Axis Conversion: ROS Z-up to Unity Y-up
   ```

4. Click **Import**

The importer creates:
- `humanoid.prefab` — Reusable prefab in Assets/
- Hierarchy in current scene showing full URDF structure

---

## Concept 1: Model Hierarchy in the Scene

After import, examine your Hierarchy panel:

```
Humanoid (root)
├── BaseLink (mesh renderer + collider)
│   ├── Torso (mesh renderer + collider)
│   │   ├── RightShoulder (mesh renderer + collider)
│   │   ├── LeftShoulder (mesh renderer + collider)
│   │   ├── RightThigh (mesh renderer + collider)
│   │   └── LeftThigh (mesh renderer + collider)
├── Joints
│   ├── BaseTorsoJoint (Joint component)
│   ├── TorsoRightShoulderJoint (Joint component)
│   └── TorsoLeftShoulderJoint (Joint component)
```

Each link is a GameObject with:
- **Mesh Filter** — The 3D shape
- **Mesh Renderer** — Makes it visible
- **Collider** — Physics/interaction boundary

---

## Concept 2: Understanding Joint Articulation

Select any joint in Hierarchy. In Inspector, note:

```
Joint (Component)
├── Connected Body (points to child link's Rigidbody)
├── Anchor (connection point in parent)
├── Axis (rotation direction: X, Y, or Z)
├── Drive limits (min/max angles)
└── Spring/Damper (physics simulation parameters)
```

This configuration allows:
- **Fixed joints** — Locked in place (base ↔ torso often fixed)
- **Revolute joints** — Rotation on one axis (shoulder, elbow, hip, knee)
- **Prismatic joints** — Linear translation (less common in humanoids)

For example, the shoulder joint rotates around the X-axis:
```
Axis: (1, 0, 0)  means rotation around X
Limit: 0 to π radians (0° to 180°)
```

---

## Concept 3: Materials and Visual Appearance

The imported model currently uses default materials. Let's make it look better.

### Assigning Colors

Create a material for each body part:

1. Right-click `Assets/` → **Create → Material**
2. Name it `mat_torso`
3. In Inspector:
   ```
   Shader: Standard
   Albedo: White (or any color)
   Metallic: 0.5
   Smoothness: 0.7
   ```

4. Drag `mat_torso` onto the Torso GameObject in the scene
   - The mesh should immediately change color

Repeat for other body parts:
- `mat_base_link` — Dark gray or black
- `mat_right_shoulder` — Light blue
- `mat_left_shoulder` — Light red
- `mat_right_thigh` — Dark blue
- `mat_left_thigh` — Dark red

Now your humanoid has visual distinction between parts.

### Understanding Shaders

The **Standard shader** provides:
- **Albedo** — Base color
- **Metallic** — How much like metal (0=matte, 1=mirror)
- **Smoothness** — Surface finish (0=rough, 1=glossy)
- **Normal maps** — Surface detail (optional)
- **Emission** — Self-glowing color (optional)

For HRI visualization, Standard shader works well. More complex shaders (PBR, glass, transparent) come later.

---

## Concept 4: Camera Positioning

Set up a camera to view the humanoid well.

### Manual Camera Setup

1. Select **Main Camera** in Hierarchy
2. Position it to see the full model:
   ```
   Position: (1.5, 1.0, 1.5)
   Rotation: (20, -45, 0)
   ```

3. Adjust Field of View:
   ```
   Field of View: 45° (default, good for close-up)
   ```

4. Test by pressing **Play** — you should see the colored humanoid model

### Scripted Camera Positioning

For reproducible viewing, create `Assets/Scripts/CameraController.cs`:

```csharp
using UnityEngine;

public class CameraController : MonoBehaviour
{
    [SerializeField] private Transform targetModel;
    [SerializeField] private float orbitDistance = 2.0f;
    [SerializeField] private float orbitHeight = 1.0f;

    void Update()
    {
        if (targetModel == null) return;

        // Orbit around model
        float angle = Time.time * 20f * Mathf.Deg2Rad; // rotate 20°/sec
        Vector3 offset = new Vector3(
            Mathf.Cos(angle) * orbitDistance,
            orbitHeight,
            Mathf.Sin(angle) * orbitDistance
        );

        transform.position = targetModel.position + offset;
        transform.LookAt(targetModel.position + Vector3.up * 0.5f);
    }

    public void ResetToDefault()
    {
        transform.position = new Vector3(1.5f, 1.0f, 1.5f);
        transform.rotation = Quaternion.Euler(20, -45, 0);
    }
}
```

Attach to Main Camera:
1. Drag `Main Camera` to Inspector → Camera Controller script
2. Assign Camera field: Target Model = Humanoid in Hierarchy
3. Press Play — camera orbits around the model

---

## Concept 5: Runtime Joint Inspection

Verify joints are properly configured. Create `Assets/Scripts/JointInspector.cs`:

```csharp
using UnityEngine;

public class JointInspector : MonoBehaviour
{
    private bool displayJointInfo = false;

    void OnGUI()
    {
        if (GUILayout.Button("Toggle Joint Info"))
        {
            displayJointInfo = !displayJointInfo;
        }

        if (displayJointInfo)
        {
            DisplayAllJoints();
        }
    }

    void DisplayAllJoints()
    {
        int y = 30;
        var joints = FindObjectsOfType<Joint>();

        foreach (var joint in joints)
        {
            GUI.Label(new Rect(10, y, 400, 25),
                $"Joint: {joint.name} | Body: {joint.connectedBody?.name ?? "None"}");
            y += 25;

            if (joint is ConfigurableJoint cj)
            {
                GUI.Label(new Rect(20, y, 400, 25),
                    $"  Axis: X={cj.axis.x:F1} Y={cj.axis.y:F1} Z={cj.axis.z:F1}");
                y += 25;

                if (cj.angularXMotion == ConfigurableJointMotion.Limited)
                {
                    GUI.Label(new Rect(20, y, 400, 25),
                        $"  X Rotation Limit: {cj.angularXLimit.limit}°");
                    y += 25;
                }
            }
        }
    }
}
```

Attach to Humanoid. Press Play → Button "Toggle Joint Info" shows all joints.

---

## Hands-On: Complete URDF Import

### Task 1: Import Your URDF (15 minutes)

1. Copy your humanoid URDF (from Chapter 4 or use example above) to `Assets/ROS/`
2. Right-click Assets → **Robotics → Import URDF**
3. Select your URDF file
4. Ensure these settings are checked:
   - ✅ Import as game objects
   - ✅ Import materials
   - ✅ Axis conversion (ROS Z-up → Unity Y-up)

### Task 2: Color the Model (15 minutes)

1. Create 5 materials in `Assets/Materials/`:
   - `mat_base` (dark gray)
   - `mat_torso` (white)
   - `mat_left_arm` (light blue)
   - `mat_right_arm` (light blue)
   - `mat_legs` (dark gray)

2. Assign materials to body parts in Hierarchy:
   - Drag material onto each GameObject
   - Verify colors are visible in Scene view

### Task 3: Position Camera and Verify (20 minutes)

1. Attach `CameraController.cs` to Main Camera
2. Assign Humanoid model as target
3. Press Play
4. Verify:
   - ✅ Model is visible and colored
   - ✅ Camera orbits around model
   - ✅ All body parts clearly visible

### Task 4: Inspect Joint Structure (20 minutes)

1. Attach `JointInspector.cs` to Humanoid
2. Press Play
3. Click "Toggle Joint Info" button
4. Verify:
   - ✅ All joints listed with correct names
   - ✅ Parent-child relationships match URDF
   - ✅ Joint axes correctly show rotation limits

### Checkpoint: Model Successfully Imported

Verify these conditions:
- ✅ Humanoid model visible in Scene
- ✅ All body parts colored distinctly
- ✅ Model hierarchy matches URDF structure
- ✅ Camera positioned for good viewing angle
- ✅ Joint information displays correctly

Take a screenshot of your colored, visible humanoid for your records.

---

## Troubleshooting

### Problem: "URDF Importer not found"

**Solution**: Verify installation in Package Manager. If missing, re-import:
```
Window → Package Manager → + → Add from git URL
https://github.com/Unity-Robotics/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

### Problem: Model appears but is invisible

**Causes**:
- No materials assigned (uses wireframe rendering)
- Material is fully transparent
- Model is outside camera frustum

**Solution**:
1. Select model in Hierarchy
2. Check Mesh Renderer component (should be enabled)
3. Verify material has Albedo color assigned
4. Adjust camera position: `transform.position = Vector3.zero;`

### Problem: Model appears too small or too large

**Cause**: Scale difference between URDF and Unity (URDF meters vs Unity units)

**Solution**:
1. Select root Humanoid in Hierarchy
2. Inspector → Transform → Scale
3. Adjust: If model is 100m tall, set Scale to (0.01, 0.01, 0.01)

### Problem: Joints not showing in inspector

**Cause**: Model has no Joint components (may have been excluded during import)

**Solution**:
1. Select child link (e.g., Torso)
2. Click **Add Component → Configurable Joint**
3. Set Connected Body to parent link's Rigidbody
4. Manually configure axis and limits from URDF

---

## Try With AI

**Exploration 1: URDF vs Visualization Trade-offs**

Ask AI: "What's the relationship between URDF definition accuracy and visual realism in game engines? What details matter for simulation vs rendering?"

Discuss:
- Collision meshes (simple) vs visual meshes (detailed)
- Physics accuracy (affects motion) vs visual polish (affects perception)
- Performance implications of high-detail models

**Exploration 2: Multi-Robot Import**

Ask AI: "How would you import multiple different robot URDFs into a single Unity scene? What naming or organization structure would prevent conflicts?"

Try: Import a second simpler model (e.g., create a box robot):
```xml
<robot name="box_robot">
  <link name="body">
    <visual><geometry><box size="0.5 0.5 0.5"/></geometry></visual>
  </link>
</robot>
```

Import both humanoid and box_robot into same scene.

---

[Next: Lesson 3 - Environment Design and Lighting](03-environment-design.md)
