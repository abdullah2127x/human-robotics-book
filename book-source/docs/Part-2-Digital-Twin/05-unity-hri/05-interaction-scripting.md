---
title: "Lesson 5: Scripting Human-Robot Interaction"
chapter: 5
lesson: 5
proficiency_level: B2
learning_objectives:
  - "Implement proximity detection for human-robot interaction"
  - "Create event systems for triggering responses"
  - "Use raycasting for line-of-sight detection"
  - "Build UI feedback for interaction status"
  - "Collaborate with AI on interaction timing and feel"
estimated_time: "120 minutes"
generated_by: content-implementer v1.0.0
created: 2025-12-16
version: 1.0.0
---

# Lesson 5: Scripting Human-Robot Interaction

## Introduction

The robot stands. The human walks. But nothing happens between them. This lesson brings them to life through code.

When the human approaches the robot, something should happen: The robot acknowledges, perhaps by turning its head, raising an arm, or displaying a message. You'll script this using proximity detection and event systems.

This is where timing matters enormously. AI can suggest interaction patterns, you refine them based on how they *feel*, and through iteration, a natural interaction emerges.

**Estimated time**: 120 minutes
**Concept density**: 5 new concepts (within B2 limit)

---

## The Interaction Flow

Here's the intended sequence:

```
Human walks forward (W key)
    ↓ (distance to robot < 1.5m)
Proximity detector triggers
    ↓
Interaction event fires
    ↓
Robot responds (turns head, gestures)
    ↓
UI shows "Interaction Complete"
    ↓
Human continues walking past
    ↓ (distance > 2.0m)
Interaction ends, state resets
```

---

## Concept 1: Proximity Detection

Detect when avatar gets close to robot. Three approaches:

### Approach A: Distance Calculation (Simple, less efficient)

```csharp
Vector3 humanPos = humanAvatar.position;
Vector3 robotPos = robot.position;
float distance = Vector3.Distance(humanPos, robotPos);

if (distance < 1.5f)
{
    TriggerInteraction();
}
```

**Pros**: Easy to understand
**Cons**: Runs every frame, slight performance cost

### Approach B: Physics.OverlapSphere (Efficient, recommended)

```csharp
Collider[] colliders = Physics.OverlapSphere(robot.position, 1.5f);
foreach (var col in colliders)
{
    if (col.CompareTag("Human"))
    {
        TriggerInteraction();
        break;
    }
}
```

**Pros**: Efficient, built-in physics optimization
**Cons**: Slightly more complex

### Approach C: OnTriggerEnter/Exit (Event-based, best for this use case)

```csharp
void OnTriggerEnter(Collider other)
{
    if (other.CompareTag("Human"))
    {
        TriggerInteraction();
    }
}

void OnTriggerExit(Collider other)
{
    if (other.CompareTag("Human"))
    {
        EndInteraction();
    }
}
```

**Pros**: Event-driven, very efficient
**Cons**: Requires trigger colliders

**For this lesson, use Approach C** (OnTriggerEnter/Exit). It's efficient and provides clear event boundaries.

---

## Concept 2: Event System and Callbacks

When proximity triggers, fire an event. Other scripts listen:

```csharp
// Define event
public event System.Action<GameObject> OnHumanApproach;

// In proximity detector
void OnTriggerEnter(Collider other)
{
    if (other.CompareTag("Human"))
    {
        OnHumanApproach?.Invoke(other.gameObject);
    }
}

// In robot controller (listening)
void Start()
{
    proximityDetector.OnHumanApproach += RespondToHuman;
}

void RespondToHuman(GameObject human)
{
    // Robot does something
    animator.SetBool("IsAcknowledging", true);
}
```

This pattern decouples detection from response. Multiple systems can listen to the same event.

---

## Concept 3: Raycasting for Line-of-Sight

Proximity alone isn't enough. The robot should only respond if it can "see" the human.

Raycasting shoots a line from robot's eyes toward human:

```csharp
Vector3 robotEyePosition = robot.GetChild("Head").position;
Vector3 directionToHuman = (human.position - robotEyePosition).normalized;

RaycastHit hit;
bool canSeeHuman = Physics.Raycast(
    robotEyePosition,
    directionToHuman,
    out hit,
    distanceToHuman
);

if (canSeeHuman && hit.collider.CompareTag("Human"))
{
    // Robot sees human, can respond
    Debug.DrawLine(robotEyePosition, human.position, Color.green);
}
else
{
    // Line of sight blocked, robot doesn't notice
    Debug.DrawLine(robotEyePosition, human.position, Color.red);
}
```

---

## Concept 4: UI Feedback System

Show users interaction status on screen.

Using TextMeshPro (built-in to Unity):

```csharp
using TMPro;
using UnityEngine;

public class InteractionUI : MonoBehaviour
{
    [SerializeField] private TextMeshProUGUI statusText;
    [SerializeField] private CanvasGroup canvasGroup;

    public void ShowInteractionStatus(string message)
    {
        statusText.text = message;
        canvasGroup.alpha = 1f;  // visible
    }

    public void HideInteractionStatus()
    {
        canvasGroup.alpha = 0f;  // invisible
    }

    public void UpdateStatus(string message)
    {
        statusText.text = message;
    }
}
```

**Setup in Scene**:
1. Create → UI Panel → Text (TextMeshPro)
2. Position at top-left of screen
3. Attach InteractionUI script
4. Drag Text into statusText field

---

## Concept 5: Timing and Natural Feeling

The same interaction plays completely different depending on timing:

**Bad Timing** (instant response):
- Human detects robot at 0ms
- Robot responds at 0ms
- Feels: Robotic, unnatural, too fast

**Good Timing** (slight delay, synchronized animation):
- Human detects robot at 0ms
- Robot waits 0.3s (processing time)
- Robot responds at 0.3s, while human is still in range
- Feels: Natural, intentional, responsive

```csharp
void RespondToHuman()
{
    // Simulate "thinking" delay
    StartCoroutine(DelayedResponse(0.3f));
}

IEnumerator DelayedResponse(float delay)
{
    yield return new WaitForSeconds(delay);

    // Now respond
    animator.SetTrigger("Acknowledge");
    ui.ShowInteractionStatus("Robot acknowledged!");

    // Keep status visible for duration of animation
    yield return new WaitForSeconds(1.5f);

    ui.HideInteractionStatus();
}
```

---

## Hands-On: Implement Interaction

### Step 1: Set Up Interaction Trigger (15 minutes)

Create `Assets/Scripts/InteractionZone.cs`:

```csharp
using UnityEngine;
using System.Collections;

public class InteractionZone : MonoBehaviour
{
    [SerializeField] private Animator robotAnimator;
    [SerializeField] private float interactionDistance = 1.5f;
    private bool interactionActive = false;

    void Start()
    {
        // Add trigger collider
        SphereCollider collider = gameObject.AddComponent<SphereCollider>();
        collider.radius = interactionDistance;
        collider.isTrigger = true;

        gameObject.tag = "InteractionZone";
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Human") && !interactionActive)
        {
            Debug.Log($"Human entered interaction zone: {other.name}");
            TriggerInteraction(other.gameObject);
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Human"))
        {
            Debug.Log("Human left interaction zone");
            EndInteraction();
        }
    }

    void TriggerInteraction(GameObject human)
    {
        if (interactionActive) return;

        interactionActive = true;
        StartCoroutine(PerformInteraction(human));
    }

    IEnumerator PerformInteraction(GameObject human)
    {
        // Simulate processing delay
        yield return new WaitForSeconds(0.3f);

        // Robot responds
        robotAnimator.SetBool("IsAcknowledging", true);
        Debug.Log("Robot acknowledged human!");

        // Hold acknowledgment for 1.5 seconds
        yield return new WaitForSeconds(1.5f);

        robotAnimator.SetBool("IsAcknowledging", false);
        interactionActive = false;
    }

    void EndInteraction()
    {
        StopAllCoroutines();
        robotAnimator.SetBool("IsAcknowledging", false);
        interactionActive = false;
    }
}
```

Attach to robot:
1. Select Humanoid robot in Hierarchy
2. Add Component → InteractionZone
3. Drag robot's Animator into robotAnimator field

### Step 2: Create UI Display (15 minutes)

Create `Assets/Scripts/InteractionUI.cs`:

```csharp
using UnityEngine;
using TMPro;
using System.Collections;

public class InteractionUI : MonoBehaviour
{
    [SerializeField] private TextMeshProUGUI statusText;
    private CanvasGroup canvasGroup;

    void Start()
    {
        canvasGroup = GetComponent<CanvasGroup>();
        if (canvasGroup == null)
        {
            canvasGroup = gameObject.AddComponent<CanvasGroup>();
        }

        HideStatus();
    }

    public void ShowStatus(string message, float duration = 2f)
    {
        statusText.text = message;
        canvasGroup.alpha = 1f;

        StartCoroutine(AutoHide(duration));
    }

    public void HideStatus()
    {
        canvasGroup.alpha = 0f;
    }

    IEnumerator AutoHide(float delay)
    {
        yield return new WaitForSeconds(delay);
        HideStatus();
    }
}
```

Setup in scene:
1. Create → UI → Canvas
2. Create → UI → Text (TextMeshPro) as child of Canvas
3. Position text at top-center
4. Rename: "InteractionStatus"
5. Attach InteractionUI script
6. Drag Text into statusText field

### Step 3: Human Avatar Setup (10 minutes)

Tag the avatar so interaction scripts recognize it:
1. Select human avatar in Hierarchy
2. Inspector → Tag → + New Tag
3. Create tag: "Human"
4. Apply tag to avatar

Add Collider to avatar:
1. Select avatar
2. Add Component → Capsule Collider
3. Adjust height/center to match avatar

### Step 4: Robot Animation Response (15 minutes)

Add acknowledgment animation to robot:
1. Select robot animator (HumanoidAnimator)
2. Create new state: "Acknowledging"
3. Assign acknowledgment animation (e.g., head turn, wave)
4. Create transitions:
   - Idle → Acknowledging (when IsAcknowledging == true)
   - Acknowledging → Idle (when IsAcknowledging == false)

### Step 5: Test Interaction (20 minutes)

1. Press Play
2. Use W key to walk human toward robot
3. When human reaches robot (< 1.5m):
   - Robot should turn/gesture
   - UI should display acknowledgment
   - Interaction should last ~1.5 seconds
   - Then return to idle

4. Walk human away
5. Observe interaction ends cleanly

### Step 6: Iterative Refinement (20 minutes)

**If interaction feels wrong**:

**Too fast** (robot responds instantly):
- Increase delay in PerformInteraction: `yield return new WaitForSeconds(0.5f);`

**Too slow** (human leaves before response):
- Decrease delay or acknowledgment duration
- Adjust interactionDistance (currently 1.5f)

**Glitchy** (multiple triggers):
- Verify `interactionActive` flag prevents re-triggering
- Check OnTriggerExit properly resets state

**Animation doesn't play**:
- Verify animator has "IsAcknowledging" bool parameter
- Check animation assigned to "Acknowledging" state
- Test manually: Select animator → Play button

### Checkpoint: Natural-Feeling Interaction

Verify:
- ✅ Human approaches robot
- ✅ Robot responds within ~0.5 seconds
- ✅ Acknowledgment animation plays smoothly
- ✅ UI shows interaction status
- ✅ Interaction completes naturally
- ✅ Can re-trigger by approaching again

---

## Try With AI

**Exploration 1: Conversation-Like Exchange**

Ask AI: "How would you design a multi-turn interaction where the human and robot exchange gestures? What state machine would support back-and-forth communication?"

Implement:
- Human approaches (first interaction)
- Robot acknowledges
- Human waves (second trigger)
- Robot waves back
- Sequence completes

**Exploration 2: Proximity-Based Behavior**

Ask AI: "What if robot behavior changed based on how close the human gets? (Far = notice, Medium = orient, Close = acknowledge, Very Close = back away)"

Implement:
- 3m: Robot notices (looks in direction)
- 2m: Robot orients toward human
- 1.5m: Robot acknowledges
- 0.8m: Robot backs up (respects personal space)

---

[Next: Lesson 6 - ROS 2 Integration](06-ros-integration.md)
