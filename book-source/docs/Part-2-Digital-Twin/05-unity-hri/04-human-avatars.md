---
title: "Lesson 4: Human Avatar Animation and Character Control"
chapter: 5
lesson: 4
proficiency_level: B2
learning_objectives:
  - "Import humanoid avatars with rigged animations"
  - "Create animator state machines for natural movement"
  - "Use animation parameters to control state transitions"
  - "Implement blend trees for smooth motion"
  - "Collaborate with AI on animation timing"
estimated_time: "120 minutes"
generated_by: content-implementer v1.0.0
created: 2025-12-16
version: 1.0.0
---

# Lesson 4: Human Avatar Animation and Character Control

## Introduction

The robot is visible. Now add a human to the scene—someone who will approach the robot and interact. Unlike the robot (which is rigid until later), the human needs natural motion: walking, standing, gesturing.

This lesson teaches you animation systems: how to blend multiple animations and control them through states. You'll work with AI to iteratively improve animation quality from "technically working" to "naturally lifelike."

**Estimated time**: 120 minutes
**Concept density**: 5 new concepts (within B2 limit)

---

## Concept 1: Avatar Selection and Import

Choose a humanoid avatar from free sources:

**Option A: Mixamo** (Easiest)
- Free rigged humanoid characters
- Hundreds of animations included
- Direct FBX download
- Humanoid rig already configured

1. Visit mixamo.com
2. Download character (e.g., "Xbot") as FBX
3. Save to `Assets/Characters/`

**Option B: Unity Asset Store**
- Search "humanoid character"
- Many free rigged options
- Download directly to project

**Option C: Manual Rigging** (Advanced)
- Import character model (OBJ, FBX without rig)
- Use Blender to add humanoid skeleton
- Export as FBX

Use **Option A (Mixamo)** for this lesson. Import into your scene:

1. Drag downloaded character into Hierarchy
2. Position near desk: (1, 0, 0)
3. Scale to match humanoid height (~1.7m human scale)

Verify:
- ✅ Avatar visible in scene
- ✅ Positioned appropriately
- ✅ Not overlapping with robot

---

## Concept 2: Animation Controller and State Machines

An **Animation Controller** manages which animation plays based on conditions.

**State Machine Diagram**:
```
        ┌─────────────┐
        │    Idle     │← Start here
        └──────┬──────┘
               │ IsWalking = true
               ↓
        ┌─────────────┐
        │   Walking   │
        └──────┬──────┘
               │ IsGesturing = true
               ↓
        ┌─────────────┐
        │  Gesturing  │
        └──────┬──────┘
               │ IsGesturing = false
               ↓ IsWalking = false
        ┌─────────────┐
        │    Idle     │
        └─────────────┘
```

**Create Animator Controller**:

1. Right-click `Assets/Animators/` → Create → Animator Controller
2. Name: `HumanoidAnimator`
3. Double-click to open Animator window
4. Right-click in editor → Create State → Empty
5. Create states:
   - `Idle` (default, right-click → Set as Layer Default State)
   - `Walking`
   - `Gesturing`
   - `Running` (optional)

6. Create transitions (right-click state → Make Transition):
   - Idle → Walking (when IsWalking = true)
   - Walking → Idle (when IsWalking = false)
   - Idle → Gesturing (when IsGesturing = true)
   - Gesturing → Idle (when IsGesturing = false)

---

## Concept 3: Animation Parameters and Conditions

Parameters control transitions:

**Types of Parameters**:
- **Bool**: On/off (IsWalking, IsGesturing)
- **Int**: Discrete values (ActionID = 0,1,2,...)
- **Float**: Continuous values (Speed = 0.0 to 1.0)

**Create Parameters** in Animator window:
1. Animator window → Parameters tab
2. Click `+` → Add Bool "IsWalking"
3. Click `+` → Add Bool "IsGesturing"
4. Click `+` → Add Float "Speed"

**Configure Transitions** with parameter conditions:

For `Idle → Walking` transition:
- Conditions: `IsWalking == true`
- Transition Duration: 0.2 seconds (smooth blend)

For `Walking → Idle` transition:
- Conditions: `IsWalking == false`
- Exit Time: 0 (immediate)

**Control from Script**:

```csharp
using UnityEngine;

public class AvatarAnimator : MonoBehaviour
{
    private Animator animator;

    void Start()
    {
        animator = GetComponent<Animator>();
    }

    void Update()
    {
        // Check input (simplified example)
        bool isWalking = Input.GetKey(KeyCode.W);
        bool isGesturing = Input.GetKey(KeyCode.G);

        animator.SetBool("IsWalking", isWalking);
        animator.SetBool("IsGesturing", isGesturing);

        // Control speed during walking
        if (isWalking)
        {
            float speed = Input.GetKey(KeyCode.LeftShift) ? 1.0f : 0.5f;
            animator.SetFloat("Speed", speed);
        }
    }
}
```

---

## Concept 4: Blend Trees

**Blend Trees** smoothly interpolate between animations based on a parameter.

**Example: Walk/Run Speed**

Instead of discrete "Walking" and "Running" states, use one state with a blend tree:

```
Movement Blend Tree (1D)
├── Idle (Speed = 0.0)
├── Walk (Speed = 0.5)
└── Run (Speed = 1.0)

When Speed parameter changes, Unity smoothly blends animations.
```

**Create Blend Tree**:

1. In Animator, create new state: `Movement`
2. Right-click state → Create New Blend Tree
3. Double-click blend tree to edit
4. Set parameter: `Speed` (float)
5. Add child motions:
   - Point 0: Idle animation, parameter value 0.0
   - Point 0.5: Walk animation, parameter value 0.5
   - Point 1.0: Run animation, parameter value 1.0

6. Set up transitions:
   - Idle → Movement (when Speed > 0)
   - Movement → Idle (when Speed = 0)

---

## Concept 5: Animation Quality Through Iteration

Animations feel "wrong" initially. Here's the iterative refinement:

**Iteration 1: Basic Setup**
- Import avatar, assign Idle animation
- Result: Avatar stands motionless
- Issue: No motion, doesn't feel alive

**Iteration 2: Add Walking**
- Create Walk animation state
- Assign Mixamo walk animation
- Result: Avatar walks when commanded
- Issue: Transitions are jerky, looks robotic

**Iteration 3: Smooth Transitions**
- Increase transition duration from 0.1 to 0.3 seconds
- Add blend tree for speed variation
- Result: Walking looks smoother, more natural
- Issue: Animation speed doesn't match movement speed

**Iteration 4: Synchronize Animation to Movement**
- Adjust animation speed multiplier based on velocity
- Tweak transition exit times
- Verify gesture animations have proper timing
- Result: Natural, lifelike movement

**Convergence**: Through iteration, jerky robotic motion becomes fluid human motion.

---

## Hands-On: Animate Your Human Avatar

### Step 1: Import Avatar and Animations (15 minutes)

1. Download Xbot from Mixamo (or use your chosen avatar)
2. Drag FBX into `Assets/Characters/`
3. In Project, select FBX → Inspector:
   ```
   Model tab:
   ☑ Humanoid
   ☑ Optimize Game Objects
   Avatar: (Auto-generated)
   ```
4. Download 3 animations from Mixamo:
   - Idle (select Xbot → Download)
   - Walking forward
   - Waving gesture
5. Repeat steps 2-3 for each animation FBX

### Step 2: Create Animator Controller (15 minutes)

1. Create folder: `Assets/Animators/`
2. Right-click → Create → Animator Controller → `HumanoidAnimator`
3. Double-click to open Animator window
4. Create states: `Idle`, `Walking`, `Waving`
5. Create transitions with parameters:

   **Parameter 1: IsWalking (Bool)**
   - Idle → Walking (condition: IsWalking == true)
   - Walking → Idle (condition: IsWalking == false)

   **Parameter 2: IsWaving (Bool)**
   - Idle → Waving (condition: IsWaving == true)
   - Waving → Idle (condition: IsWaving == false)

6. Assign animations:
   - Click Idle state → Drag idle animation FBX into Motion field
   - Click Walking state → Drag walking animation FBX
   - Click Waving state → Drag waving animation FBX

### Step 3: Attach Controller to Avatar (10 minutes)

1. Select avatar in Hierarchy
2. Add component: Animator
3. Drag `HumanoidAnimator` into Controller field
4. Verify Motion Matching: Humanoid (avatar must use humanoid rig)

### Step 4: Control via Script (20 minutes)

Create `Assets/Scripts/AvatarController.cs`:

```csharp
using UnityEngine;

public class AvatarController : MonoBehaviour
{
    private Animator animator;
    private Rigidbody rb;
    [SerializeField] private float moveSpeed = 2.0f;
    [SerializeField] private Transform targetPosition;

    void Start()
    {
        animator = GetComponent<Animator>();
        rb = GetComponent<Rigidbody>();

        // Add Rigidbody if missing (for physics)
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
            rb.useGravity = true;
            rb.isKinematic = false;
        }
    }

    void Update()
    {
        HandleInput();
        HandleMovement();
    }

    void HandleInput()
    {
        bool isWalking = Input.GetKey(KeyCode.W);
        bool isWaving = Input.GetKey(KeyCode.G);

        animator.SetBool("IsWalking", isWalking);
        animator.SetBool("IsWaving", isWaving);
    }

    void HandleMovement()
    {
        if (animator.GetBool("IsWalking"))
        {
            Vector3 moveDirection = transform.forward * moveSpeed * Time.deltaTime;
            transform.Translate(moveDirection, Space.World);
        }
    }

    public void MoveTowards(Vector3 position)
    {
        // For later use: automated walking to position
        targetPosition = null; // placeholder
    }
}
```

Attach to avatar.

### Step 5: Test Animation States (20 minutes)

1. Drag avatar into scene
2. Position near robot
3. Press Play
4. Test controls:
   - Press W → Avatar walks
   - Release W → Avatar returns to idle
   - Press G → Avatar waves
   - Release G → Avatar stops waving
5. Verify transitions are smooth (not jerky)
6. Observe animation quality

### Step 6: Iterative Refinement (20 minutes)

If animations feel wrong:

**Problem: Jerky transitions**
- Solution: Increase transition duration from 0.2 to 0.4 seconds
- In Animator, click transition → Inspector → Transition Duration

**Problem: Animation plays too slow**
- Solution: Increase animation speed multiplier
- In Animator, select state → Motion tab → Speed: 1.2

**Problem: Avatar sliding (feet don't move with animation)**
- Solution: Ensure "Root Motion" enabled
  - In avatar animator: Check "Apply Root Motion"

**Problem: Avatar doesn't return to idle after animation**
- Solution: Verify transition back to Idle has no condition (always triggers)
- In Animator, click Waving → Idle transition → No conditions

### Checkpoint: Smooth Avatar Animation

Verify:
- ✅ Avatar walks smoothly when W pressed
- ✅ Avatar idles when walking stops
- ✅ Avatar waves when G pressed
- ✅ Transitions are fluid (0.3+ seconds, no jerking)
- ✅ Avatar positioned in office scene

---

## Try With AI

**Exploration 1: Animation Blending**

Ask AI: "I want walking speed to vary based on distance to destination. How would you implement a speed parameter that smoothly transitions walk → run?"

Implement:
1. Add `Speed` float parameter
2. Create blend tree: Idle (0) → Walk (0.5) → Run (1.0)
3. Script: Calculate speed based on distance, update parameter

Test: Avatar walks toward target, accelerates to running as it approaches.

**Exploration 2: Natural Gesture Integration**

Ask AI: "What gesture animations would make a robot interaction study feel more natural? How would you sequence gestures with interaction timing?"

Try: Create gesture sequence:
- Waving (attention getter)
- Then standing (attentive listening)
- Then nodding (acknowledgment)

---

[Next: Lesson 5 - Interaction Scripting](05-interaction-scripting.md)
