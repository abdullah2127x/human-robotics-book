---
title: "Lesson 6: ROS 2 Integration and Message Publishing"
chapter: 5
lesson: 6
proficiency_level: B2
learning_objectives:
  - "Define custom ROS 2 message types for HRI events"
  - "Implement publishers and subscribers in Unity"
  - "Serialize Unity data to ROS 2 message format"
  - "Handle message callbacks and asynchronous updates"
  - "Debug ROS 2 communication from Unity"
estimated_time: "120 minutes"
generated_by: content-implementer v1.0.0
created: 2025-12-16
version: 1.0.0
---

# Lesson 6: ROS 2 Integration and Message Publishing

## Introduction

Your interaction system works in Unity. Now connect it to ROS 2 so the robot control system knows when humans approach.

This lesson bridges simulation (Unity) and robotics control (ROS 2). When the human interacts with the humanoid in Unity, you'll publish a message to ROS 2. The actual robot could subscribe and respond in the real world.

This is where iteration and AI collaboration matter most—message definitions, timing, and synchronization require real-time feedback to get right.

**Estimated time**: 120 minutes
**Concept density**: 4 new concepts (within B2 limit)

---

## Concept 1: Custom Message Definition

ROS 2 messages have strict contracts. Define exactly what an interaction event contains.

Create `src/hri_msgs/msg/InteractionEvent.msg`:

```
int64 timestamp_ms
geometry_msgs/Point human_position
geometry_msgs/Point robot_position
string interaction_type
float64 confidence
```

This defines:
- **timestamp_ms** — When did interaction occur (milliseconds)
- **human_position** — Where was human (x, y, z)
- **robot_position** — Where was robot
- **interaction_type** — "approached", "waving", "completed"
- **confidence** — How sure are we (0-1)

Build ROS 2 message (in ROS 2 workspace):

```bash
cd ~/ros2_ws
colcon build --packages-select hri_msgs
```

---

## Concept 2: Publisher Implementation in Unity

Create `Assets/Scripts/RosPublisher.cs`:

```csharp
using UnityEngine;
using ROS2;
using geometry_msgs.msg;

public class RosPublisher : MonoBehaviour
{
    private IPublisher<InteractionEvent> publisher;
    private ROS2Node ros2Node;

    void Start()
    {
        ros2Node = ROS2UnityComponent.Instance.ROS2Node;

        // Create publisher on /hri/interaction_events
        publisher = ros2Node.CreatePublisher<InteractionEvent>(
            "/hri/interaction_events"
        );

        Debug.Log("[ROS Publisher] Ready to publish interaction events");
    }

    public void PublishInteractionEvent(
        Vector3 humanPos,
        Vector3 robotPos,
        string interactionType,
        float confidence = 1.0f)
    {
        var message = new InteractionEvent();

        message.timestamp_ms = (long)(Time.realtimeSinceStartup * 1000);
        message.human_position = ConvertVector3ToPoint(humanPos);
        message.robot_position = ConvertVector3ToPoint(robotPos);
        message.interaction_type = interactionType;
        message.confidence = confidence;

        publisher.Publish(message);

        Debug.Log($"[ROS] Published: {interactionType} at {humanPos}");
    }

    private Point ConvertVector3ToPoint(Vector3 v)
    {
        return new Point { x = v.x, y = v.y, z = v.z };
    }
}
```

**Key points**:
- Publisher created on topic `/hri/interaction_events`
- Message serialized before publishing
- Vector3 → Point conversion (Unity units to ROS message format)
- Only publishes on actual events (not every frame—efficient)

---

## Concept 3: Subscriber Implementation

Subscribe to robot state from ROS 2:

Create `Assets/Scripts/RosSubscriber.cs`:

```csharp
using UnityEngine;
using ROS2;
using sensor_msgs.msg;

public class RosSubscriber : MonoBehaviour
{
    private ISubscription<JointState> subscription;
    private JointState lastRobotState;

    void Start()
    {
        var ros2Node = ROS2UnityComponent.Instance.ROS2Node;

        // Subscribe to robot joint states
        subscription = ros2Node.CreateSubscription<JointState>(
            "/robot/joint_states",
            OnJointStateReceived
        );

        Debug.Log("[ROS Subscriber] Listening to /robot/joint_states");
    }

    void OnJointStateReceived(JointState message)
    {
        lastRobotState = message;

        // Update robot model based on received joint angles
        UpdateRobotPose(message);
    }

    void UpdateRobotPose(JointState state)
    {
        // Example: Apply joint angles to robot animator
        // In production, use proper IK (Inverse Kinematics) solvers
        Debug.Log($"[ROS] Received {state.position.Length} joint angles");

        // For now, just log receipt
        foreach (var angle in state.position)
        {
            Debug.Log($"  Joint angle: {angle}");
        }
    }
}
```

---

## Concept 4: Message Serialization and Deserialization

Serialization converts C# objects to bytes for transmission:

```csharp
// Publishing (C# → ROS bytes)
InteractionEvent evt = new InteractionEvent {
    timestamp_ms = 1000,
    interaction_type = "approached"
};
publisher.Publish(evt);  // Serialization automatic

// Subscribing (ROS bytes → C#)
void OnMessageReceived(JointState message)
{
    // Automatic deserialization
    foreach (var position in message.position)
    {
        // Use deserialized data
    }
}
```

ROS-TCP-Connector handles serialization automatically using message contracts.

---

## Hands-On: Connect Interaction to ROS 2

### Step 1: Connect Publisher to Interaction (20 minutes)

Modify `InteractionZone.cs` to publish events:

```csharp
using UnityEngine;
using System.Collections;

public class InteractionZone : MonoBehaviour
{
    [SerializeField] private Animator robotAnimator;
    [SerializeField] private RosPublisher rosPublisher;
    [SerializeField] private Transform robotTransform;
    private bool interactionActive = false;

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Human") && !interactionActive)
        {
            GameObject human = other.gameObject;

            // Publish to ROS 2
            if (rosPublisher != null)
            {
                rosPublisher.PublishInteractionEvent(
                    humanPos: human.transform.position,
                    robotPos: robotTransform.position,
                    interactionType: "approached",
                    confidence: 1.0f
                );
            }

            TriggerInteraction(human);
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Human"))
        {
            GameObject human = other.gameObject;

            // Publish exit event
            if (rosPublisher != null)
            {
                rosPublisher.PublishInteractionEvent(
                    humanPos: human.transform.position,
                    robotPos: robotTransform.position,
                    interactionType: "disengaged",
                    confidence: 1.0f
                );
            }

            EndInteraction();
        }
    }

    IEnumerator PerformInteraction(GameObject human)
    {
        yield return new WaitForSeconds(0.3f);

        robotAnimator.SetBool("IsAcknowledging", true);

        // Publish acknowledgment event
        if (rosPublisher != null)
        {
            rosPublisher.PublishInteractionEvent(
                humanPos: human.transform.position,
                robotPos: robotTransform.position,
                interactionType: "acknowledged",
                confidence: 1.0f
            );
        }

        yield return new WaitForSeconds(1.5f);

        robotAnimator.SetBool("IsAcknowledging", false);
        interactionActive = false;
    }

    void TriggerInteraction(GameObject human)
    {
        if (interactionActive) return;
        interactionActive = true;
        StartCoroutine(PerformInteraction(human));
    }

    void EndInteraction()
    {
        StopAllCoroutines();
        robotAnimator.SetBool("IsAcknowledging", false);
        interactionActive = false;
    }
}
```

### Step 2: Set Up Publisher in Scene (10 minutes)

1. Create empty GameObject: "ROS Manager"
2. Attach `RosPublisher.cs`
3. Select robot InteractionZone
4. Drag ROS Manager's RosPublisher script into InteractionZone's rosPublisher field
5. Drag robot transform into robotTransform field

### Step 3: Monitor ROS 2 Topics (15 minutes)

In ROS 2 terminal, monitor published events:

```bash
# In Ubuntu/Linux terminal
ros2 topic echo /hri/interaction_events

# Press Play in Unity, walk avatar toward robot
# Should see messages like:
# timestamp_ms: 1234567
# human_position:
#   x: 0.5
#   y: 0.0
#   z: 1.2
# robot_position:
#   x: 0.0
#   y: 0.0
#   z: 0.0
# interaction_type: approached
# confidence: 1.0
```

### Step 4: Verify Message Frequency (10 minutes)

Check messages aren't being published excessively:

```bash
# Show message frequency
ros2 topic hz /hri/interaction_events

# Should show ~1 message per interaction (low frequency)
# NOT multiple messages per second (would indicate publishing every frame)
```

If publishing too frequently:
- Ensure interaction events only fire on trigger enter/exit
- Not in Update() loop
- Use `interactionActive` flag to prevent duplicates

### Step 5: Test Full Cycle (20 minutes)

1. Start ROS 2 bridge (Lesson 1 setup)
2. Press Play in Unity
3. Walk human toward robot
4. Verify:
   - ✅ Interaction triggers in Unity
   - ✅ Message appears in `ros2 topic echo`
   - ✅ Message contains correct position data
   - ✅ Message frequency is low (not spamming)
5. Walk away, verify "disengaged" message appears
6. Repeat interaction, verify it works reliably

### Checkpoint: ROS 2 Publishing Works

Verify:
- ✅ InteractionEvent messages published on correct topic
- ✅ Message contains valid position and timestamp data
- ✅ ROS 2 system receives messages reliably
- ✅ Message frequency is appropriate (event-based, not continuous)

---

## Debugging Checklist

**Problem: "Publisher not initialized"**
- Solution: Verify ROS-TCP-Connector installed and initialized before creating publisher
- Check: `ROS2UnityComponent.Instance` returns valid node

**Problem: Messages never arrive in ROS 2**
- Solution: Verify topic name matches: `/hri/interaction_events`
- Check: `ros2 topic list` shows the topic
- Verify: Message type matches message definition

**Problem: Position data incorrect**
- Solution: Check Vector3 → Point conversion
- Verify: Human and robot positions make sense (not NaN or infinity)
- Test: Print positions to console before publishing

**Problem: Multiple messages per single interaction**
- Solution: Add `Debug.Log()` in publish method, count logs
- Verify: `interactionActive` flag prevents re-entry
- Check: OnTriggerEnter fires only once per entry

---

## Try With AI

**Exploration 1: Message Definition Improvements**

Ask AI: "What additional fields in an InteractionEvent message would be valuable for a real-world HRI study? (confidence, gesture type, duration, etc.)"

Add fields to message definition and update publishing code.

**Exploration 2: Synchronization Across Network**

Ask AI: "When Unity and ROS 2 are on different machines with network latency, how do we keep them synchronized? What timestamp and buffering strategies exist?"

Discuss:
- Synchronized clocks (NTP)
- Circular buffers for out-of-order messages
- Acknowledgment protocols

---

[Next: Lesson 7 - HRI Scene Management Patterns](07-scene-management.md)
