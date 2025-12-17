# Lesson 3: Importing URDF Humanoids into Isaac Sim

You've built humanoid models in URDF (Part 1) and simulated them in Gazebo (Part 2). Now you'll import those same models into Isaac Sim—but the conversion reveals important differences between Gazebo's ODE physics and Isaac's PhysX 5.

This lesson demonstrates collaboration with AI to troubleshoot import issues, configure joint parameters, and validate physics behavior.

## Understanding URDF → USD Conversion

Isaac Sim doesn't directly simulate URDF files. Instead, it **converts** URDF to USD format using the URDF importer extension.

**What gets converted**:
- Links → USD Mesh prims with rigid body physics
- Joints → USD Joint prims with PhysX articulation
- Visuals/collisions → Separate geometry for rendering vs physics
- Materials → USD shaders (basic conversion, often needs refinement)

**What requires manual configuration**:
- Joint drive parameters (stiffness, damping)
- Articulation root settings
- Material properties (friction, restitution)
- Collision mesh resolution

## Prerequisites: Locate Your URDF

From Part 1, you should have a humanoid URDF. If not, use this simple example:

**Create test humanoid** (`~/test_humanoid/robot.urdf`):
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link (torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.6"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.8" ixy="0" ixz="0" iyy="0.8" iyz="0" izz="0.4"/>
    </inertial>
  </link>

  <!-- Left leg (thigh) -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.08"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.08"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <inertia ixx="0.06" ixy="0" ixz="0" iyy="0.06" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Hip joint (torso to left thigh) -->
  <joint name="left_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="-0.1 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="150" lower="-1.57" upper="1.57" velocity="10.0"/>
  </joint>

  <!-- Right leg (mirror of left) -->
  <link name="right_thigh">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.08"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.08"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <inertia ixx="0.06" ixy="0" ixz="0" iyy="0.06" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_hip" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="0.1 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="150" lower="-1.57" upper="1.57" velocity="10.0"/>
  </joint>
</robot>
```

**Save this as** `~/test_humanoid/robot.urdf` if you don't have a URDF yet.

## Initial Import Attempt (Expect Issues)

Let's import the URDF and observe what happens.

**Step 1: Open URDF Importer**

In Isaac Sim:
1. **Isaac Utils → URDF Importer**
2. URDF import window opens

**Step 2: Configure Import Settings**

**Import Settings**:
- **URDF Path**: Browse to `/home/username/test_humanoid/robot.urdf`
- **Import Location**: `/World/humanoid`
- **Fix Base Link**: ☐ Unchecked (humanoid is free-floating)
- **Joint Drive Mode**: Position (default)
- **Import Inertia Tensor**: ☑ Checked

**Click "Import"**

**Expected outcome**: Humanoid appears in viewport, but when you click Play... **it collapses or falls through the ground**.

**Why?** PhysX 5 articulation requires explicit configuration that URDF doesn't provide.

## Issue 1: Humanoid Collapses on Spawn

**Observed behavior**: Click Play → Humanoid's legs collapse, body falls.

**Root cause**: URDF joint limits convert to USD, but **joint drives** (stiffness/damping) aren't specified in URDF. PhysX defaults to zero stiffness = no resistance to motion.

### Discovering the Solution

At this point, you might try adjusting settings randomly. Instead, let's collaborate with AI to understand the problem.

**Your request**:
```
I imported a URDF humanoid into Isaac Sim. In Gazebo, it stands stable, but in Isaac Sim, the legs collapse when I click Play. The joints seem to have no resistance. What's the difference between Gazebo and Isaac Sim joint control?
```

**AI's response** (example):
```
In Gazebo (ODE), joints have implicit damping and friction that provide resistance. In Isaac Sim (PhysX 5), articulations require explicit joint drives.

For a standing humanoid, you need to configure:
1. Joint drive stiffness (analogous to spring constant)
2. Joint drive damping (energy dissipation)
3. Target position (desired joint angle)

Without these, PhysX treats joints as freely rotating (zero torque).

Try adding joint drives with these parameters:
- Stiffness: 1000-5000 N⋅m/rad (for leg joints)
- Damping: 100-500 N⋅m⋅s/rad
- Target: 0.0 (neutral pose)
```

**What emerged**: Through AI's explanation, you discovered that PhysX requires explicit drive parameters that Gazebo inferred automatically. This is a fundamental architectural difference.

### Configuring Joint Drives

**Step 3: Add Joint Drives Manually**

In Isaac Sim:
1. Select **left_hip** joint in Stage panel
2. Property panel → **Raw USD Properties**
3. Find **drive** section
4. Add these properties:

**Joint drive parameters**:
```python
# Open Script Editor, run this to configure all joints
import omni.isaac.core.utils.prims as prim_utils
from pxr import UsdPhysics

# Get humanoid prim
humanoid_path = "/World/humanoid"

# Configure each joint
joints = ["left_hip", "right_hip"]
for joint_name in joints:
    joint_path = f"{humanoid_path}/{joint_name}"
    joint_prim = prim_utils.get_prim_at_path(joint_path)

    # Get PhysX joint drive API
    drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "angular")

    if not drive_api:
        # Create drive API if it doesn't exist
        drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")

    # Set drive parameters
    drive_api.GetStiffnessAttr().Set(2000.0)  # N⋅m/rad
    drive_api.GetDampingAttr().Set(200.0)     # N⋅m⋅s/rad
    drive_api.GetTargetPositionAttr().Set(0.0) # Neutral pose

print("Joint drives configured!")
```

**Run the script**, then **click Play**.

**Expected outcome**: Humanoid now stands! Legs resist collapse due to stiffness.

## Issue 2: Tuning Joint Parameters

**Observed behavior**: Humanoid stands but legs tremble (high-frequency oscillation).

**Your observation**:
```
The humanoid stands now, but the legs shake. I set stiffness to 2000 and damping to 200. How do I tune these parameters to reduce oscillation?
```

**AI's guidance**:
```
High stiffness with low damping causes oscillation. For stable standing, try:
1. Reduce stiffness: 1000-1500 (still enough to resist gravity)
2. Increase damping: 300-500 (more energy dissipation)
3. Increase PhysX solver iterations (for stability)

Start with stiffness=1200, damping=400. If still oscillating, increase damping first (cheaper than stiffness).
```

**Refinement**: You test these values:
```python
# Adjusted parameters
drive_api.GetStiffnessAttr().Set(1200.0)
drive_api.GetDampingAttr().Set(400.0)
```

**Outcome**: Oscillation reduced. Through iterative feedback, you and AI converged on stable parameters.

**Parameter tuning insight**: High stiffness simulates strong motors (fast response, but requires high damping to prevent oscillation). Lower stiffness with moderate damping simulates compliant actuators (slower response, more stable).

## Issue 3: Material Properties Missing

**Observed behavior**: Humanoid stands, but feet slide on ground (no friction).

**Your specification**:
```
The humanoid's feet are sliding on the ground plane like it's ice. I need realistic friction for foot contact. URDF doesn't specify friction coefficients—how do I add these in Isaac Sim?
```

**AI's adaptation**:
```
Isaac Sim requires explicit collision material properties. Add:
1. Static friction coefficient (prevents sliding when stationary)
2. Dynamic friction coefficient (resists sliding when moving)
3. Restitution (bounciness, 0=no bounce)

For realistic foot-ground contact:
- Static friction: 0.8-1.0
- Dynamic friction: 0.6-0.8
- Restitution: 0.0-0.1 (feet shouldn't bounce)
```

**You apply friction**:
```python
from pxr import PhysxSchema

# Apply friction to feet links
feet_links = ["left_thigh", "right_thigh"]  # Simplified; full model would have feet
for link_name in feet_links:
    link_path = f"{humanoid_path}/{link_name}"
    link_prim = prim_utils.get_prim_at_path(link_path)

    # Apply material API
    material_api = PhysxSchema.PhysxMaterialAPI.Apply(link_prim)
    material_api.CreateStaticFrictionAttr().Set(0.9)
    material_api.CreateDynamicFrictionAttr().Set(0.7)
    material_api.CreateRestitutionAttr().Set(0.05)

print("Friction materials applied!")
```

**Outcome**: Feet now grip the ground. No more sliding during standing.

## Issue 4: Collision Mesh Resolution

**Observed behavior**: Humanoid stands stable but torso penetrates through thin obstacles (like shelves).

**The dialogue**:

**Your constraint**:
```
The humanoid's torso is a simple box collision mesh (from URDF). It's too coarse and sometimes passes through thin geometry. Should I increase collision mesh resolution or simplify the environment?
```

**AI's response**:
```
Two options:
1. Convex decomposition: Break collision mesh into multiple convex pieces (more accurate, higher computation)
2. Simplified environment: Use thicker obstacles (avoid thin geometry)

For synthetic data generation (your use case), option 2 is better:
- Thin shelves aren't common in training data scenarios
- Simplified collisions = faster simulation (important for dataset generation)

If you need accurate collision for manipulation tasks, use convex decomposition.
```

**Your decision**: Simplify environment (warehouse has thick shelves anyway). AI adapted recommendation to your product constraints (dataset generation vs manipulation).

**What emerged**: The solution you converged on (simpler environment) wasn't your initial thought (fix collision mesh) or AI's first suggestion (convex decomposition). Collaboration revealed the constraint-driven choice.

## Validation Workflow

**Always validate URDF imports systematically**:

### Validation Checklist

- [ ] **Visual appearance**: Does robot look correct? (All links visible, materials applied)
- [ ] **Joint hierarchy**: Stage panel shows correct parent-child relationships
- [ ] **Physics stability**: Robot stands without collapsing (with joint drives configured)
- [ ] **Joint range of motion**: Manual joint control moves within URDF limits
- [ ] **Collision detection**: Robot doesn't penetrate floor or obstacles
- [ ] **Mass properties**: Center of mass is reasonable (not causing toppling)

**Test joint control**:
```python
from omni.isaac.core import World
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import Articulation

# Initialize world
world = World()
world.reset()

# Get articulation
humanoid = Articulation("/World/humanoid")
humanoid.initialize()

# Test joint control
action = ArticulationAction(joint_positions=[0.5, -0.5])  # Left hip 0.5 rad, right hip -0.5 rad
humanoid.apply_action(action)

# Step simulation
for _ in range(100):
    world.step(render=True)

print(f"Joint positions: {humanoid.get_joint_positions()}")
```

**Expected outcome**: Humanoid legs move to specified angles smoothly.

## Comparison: Gazebo vs Isaac Sim URDF Import

| Aspect | Gazebo (ODE) | Isaac Sim (PhysX 5) |
|--------|--------------|---------------------|
| **Joint drives** | Implicit damping/friction | Explicit stiffness/damping required |
| **Stability** | Often stable out-of-the-box | Requires parameter tuning |
| **Material properties** | Basic friction in URDF | Must add PhysX material API |
| **Collision mesh** | Single mesh per link | Can use convex decomposition |
| **Performance** | CPU-limited (~10 articulated robots) | GPU-accelerated (~100+ robots) |

**Takeaway**: Isaac Sim requires more explicit configuration but enables GPU-scale simulation.

## Exercise: Import Your Humanoid from Part 1

**Task**: Import the humanoid you built in Part 1 into Isaac Sim.

**Steps**:
1. Locate your URDF from Part 1 (or use example above)
2. Import using Isaac Utils → URDF Importer
3. Configure joint drives for all revolute joints
4. Add friction materials to foot links
5. Validate using checklist above
6. Record any issues you encounter and how you resolved them

**Success criteria**:
- [ ] Humanoid stands stable for 10+ simulated seconds
- [ ] All joints move within URDF-specified limits
- [ ] Feet don't slide on ground plane
- [ ] No collision penetration with test obstacles

## Try With AI

**Setup**: Open ChatGPT or Claude alongside Isaac Sim.

**Part 1: Initial Troubleshooting**
Ask AI:
```
I imported a URDF humanoid into Isaac Sim. It has 12 revolute joints (6 per leg). When I click Play, the robot collapses. In the Property panel, I see joint drive stiffness is 0. What values should I use for humanoid leg joints to support body weight?
```

Review AI's suggestions. Do they match what you learned in this lesson?

**Part 2: Parameter Tuning Dialogue**
After configuring initial parameters, ask:
```
I set leg joint stiffness to 3000 and damping to 100. The humanoid stands but oscillates at high frequency. Should I increase or decrease stiffness? What about damping?
```

Test AI's recommendation. Does the oscillation reduce?

**Part 3: Constraint Teaching**
Based on your specific humanoid design, tell AI:
```
My humanoid has compliant ankle joints that should behave like springs (stiffness 500 N⋅m/rad). But the hips need to be stiff (stiffness 3000 N⋅m/rad) to maintain posture. How do I set per-joint drive parameters in Isaac Sim's Python API?
```

Implement AI's code snippet. Did it handle per-joint configuration correctly?

**Part 4: Convergence Check**
Compare your final working configuration to your initial attempt:
- What improved through iteration with AI?
- What did you specify based on your humanoid design?
- What did AI suggest that you hadn't considered?
- Is the result better than if you'd worked alone? Why?

**Safety note**: When running Python scripts in Isaac Sim, start with a single joint to test before applying to all joints. Incorrect parameters can cause unstable physics (robot explodes or freezes simulation).
