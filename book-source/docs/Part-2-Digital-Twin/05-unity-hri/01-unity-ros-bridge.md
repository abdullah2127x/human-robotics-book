---
title: "Lesson 1: Unity-ROS 2 Bridge Setup"
chapter: 5
lesson: 1
proficiency_level: B2
learning_objectives:
  - "Install and configure ROS-TCP-Connector in Unity 2022.3 LTS"
  - "Establish bidirectional communication between Unity and ROS 2"
  - "Understand message serialization and bridge architecture"
  - "Validate communication with monitoring tools"
estimated_time: "90 minutes"
generated_by: content-implementer v1.0.0
created: 2025-12-16
version: 1.0.0
---

# Lesson 1: Unity-ROS 2 Bridge Setup

## Introduction

Before visualizing robots, you must establish communication between Unity (your visualization client) and ROS 2 (your robotics control system). This lesson creates the foundation: a reliable bridge that passes data between worlds.

Think of this bridge as a postal service between two systems. Messages travel in both directions. The ROS 2 side publishes sensor data and commands. Unity subscribes to these messages and responds with visualization updates. When humans interact with the Unity scene, those events publish back to ROS 2.

Setting this up correctly now prevents frustrating debugging later.

**Estimated time**: 90 minutes
**Concept density**: 5 new concepts (well within B2 limit)

---

## The Architecture Before We Build

When you complete this lesson, your system will look like this:

```
┌─────────────────────────────────────────┐
│         ROS 2 System (Linux)            │
│  ┌─────────────────────────────────┐    │
│  │   ROS Master (roscore equivalent)    │
│  │   IP: 192.168.1.100, Port: 9090    │
│  └─────────────────────────────────┘    │
│                  ↕                       │
│   Publishers (publish data)              │
│   Subscribers (receive commands)         │
└─────────────────────────────────────────┘
          ↓ TCP Connection ↓
    ROS-TCP-Connector Bridge
    (Translates between protocols)
          ↓ TCP Connection ↓
┌─────────────────────────────────────────┐
│    Unity Client (Windows/Mac/Linux)     │
│  ┌─────────────────────────────────┐    │
│  │   ROS-TCP-Connector Package     │    │
│  │   IP: 192.168.1.50, Port: 5005  │    │
│  └─────────────────────────────────┘    │
│                                         │
│   Receives messages from ROS 2          │
│   Sends interaction events to ROS 2     │
└─────────────────────────────────────────┘
```

The bridge translates ROS 2's native protocol (DDS) into TCP/IP messages that Unity can process.

---

## Concept 1: Project Structure and GameObjects

Create a new Unity 2022.3 LTS project with standard 3D scene setup.

Open Unity Hub → New Project → 3D (Standard) → Unity 2022.3 LTS

Once Unity loads, examine the default scene:

**Hierarchy Panel** (left side):
```
Sample Scene
├── Main Camera
│   └── Default camera for rendering
├── Directional Light
│   └── Default sun light
└── Ground plane (optional)
```

**Key folders in Assets**:
- `Assets/Scripts/` — Your C# code goes here
- `Assets/Scenes/` — Scenes stored here
- `Assets/Prefabs/` — Reusable components (we'll use this later)
- `Assets/Resources/` — Runtime-loaded assets

Create these folders now (right-click Assets → Create → Folder):

```bash
Assets/Scripts/
Assets/ROS/
Assets/ROS/Messages/
Assets/Data/
```

The `Assets/ROS/` folder will store ROS-TCP-Connector and message definitions.

---

## Concept 2: ROS-TCP-Connector Package

ROS-TCP-Connector is the bridge. It translates between:
- **ROS 2 side**: DDS protocol (Data Distribution Service)
- **Unity side**: TCP/IP socket communication

### Installation

In Unity, open **Window → TextMesh Pro → Import TMP Essential Resources** (required by some packages).

Then install ROS-TCP-Connector:
1. **Window → Package Manager**
2. **Add package from git URL**
3. Paste: `https://github.com/Unity-Technologies/ROS2-For-Unity.git?path=/unity_ws/src/ros2_csharp_src`

Wait for installation to complete. This adds:
- ROS message definitions (standard types)
- TCP bridge communication layer
- Example scenes (we'll reference these)

### Verify Installation

After installation, you should see in Package Manager:
```
ROS2 For Unity (by Unity Technologies)
  - ROS2 For Unity
  - ROS2 For Unity Examples
  - ROS2 For Unity Glue Lib
```

---

## Concept 3: Network Configuration and Endpoints

ROS 2 and Unity must find each other on the network. This requires configuration:

### On ROS 2 Machine (Ubuntu/Linux):

Set ROS master endpoint:
```bash
# Terminal on ROS 2 machine
export ROS_DOMAIN_ID=42
ros2 run ros2_control_node tcp_bridge_node --ros-args \
  -p server_ip_address:=0.0.0.0 \
  -p port:=9090
```

This starts the ROS bridge:
- **IP**: 0.0.0.0 (listens on all interfaces)
- **Port**: 9090 (standard bridge port)

### On Unity Machine (Windows/Mac/Linux):

Create a new C# script: `Assets/Scripts/RosConnector.cs`

```csharp
using UnityEngine;
using ROS2;

public class RosConnector : MonoBehaviour
{
    [SerializeField] private string rosHostIP = "192.168.1.100";
    [SerializeField] private int rosPort = 9090;
    [SerializeField] private string unityClientIP = "192.168.1.50";

    private ROS2UnityComponent ros2Component;

    void Start()
    {
        // Get ROS2 component (added to scene or managed by package)
        ros2Component = GetComponent<ROS2UnityComponent>();

        // Configure connection parameters
        Debug.Log($"[ROS Bridge] Connecting to ROS 2 at {rosHostIP}:{rosPort}");
        Debug.Log($"[ROS Bridge] Unity client identified as {unityClientIP}");
    }

    void OnGUI()
    {
        if (GUILayout.Button("Check Bridge Status"))
        {
            CheckBridgeStatus();
        }
    }

    void CheckBridgeStatus()
    {
        if (ros2Component != null && ros2Component.IsInitialized)
        {
            Debug.Log("[ROS Bridge] Status: CONNECTED");
        }
        else
        {
            Debug.Log("[ROS Bridge] Status: DISCONNECTED");
        }
    }
}
```

**Configuration breakdown**:
- `rosHostIP`: Machine running ROS 2 (e.g., `192.168.1.100`)
- `rosPort`: ROS bridge port (default 9090)
- `unityClientIP`: Your development machine (e.g., `192.168.1.50`)

Find your machine IPs:
- **Windows**: `ipconfig` in Command Prompt
- **Mac/Linux**: `ifconfig` in Terminal

---

## Concept 4: Message Serialization (ROS 2 ↔ C#)

ROS 2 messages have strict type contracts. When Unity sends/receives messages, both sides must agree on format.

### Example: Standard String Message

ROS 2 message definition (`.msg` file):
```
# std_msgs/String.msg
string data
```

C# equivalent (auto-generated by ROS2-For-Unity):
```csharp
namespace std_msgs.msg
{
    public class String
    {
        public string data = "";
    }
}
```

### Custom Message Example: Position Data

Create `Assets/ROS/Messages/PositionMsg.msg`:
```
geometry_msgs/Point position
int32 confidence
```

In C#, this becomes:
```csharp
using geometry_msgs.msg;

public class PositionMsg
{
    public Point position = new Point();
    public int confidence = 0;
}
```

ROS-TCP-Connector automatically serializes/deserializes these messages.

---

## Concept 5: Connection Validation

Before proceeding to visualization, verify the bridge works bidirectionally.

### Testing: ROS 2 → Unity (Receiving)

Create `Assets/Scripts/MessageReceiver.cs`:

```csharp
using UnityEngine;
using ROS2;
using std_msgs.msg;

public class MessageReceiver : MonoBehaviour
{
    private ISubscription<String> subscription;
    private string lastMessage = "No message yet";

    void Start()
    {
        var ros2Node = ROS2.ROS2UnityComponent.Instance.ROS2Node;

        // Subscribe to /test_topic
        subscription = ros2Node.CreateSubscription<String>(
            "/test_topic",
            OnMessageReceived
        );

        Debug.Log("[Receiver] Subscribed to /test_topic");
    }

    void OnMessageReceived(String message)
    {
        lastMessage = message.data;
        Debug.Log($"[Receiver] Got message: {lastMessage}");
    }

    void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 400, 30), $"Latest message: {lastMessage}");
    }
}
```

Attach to a GameObject in the scene (Main Camera or create empty).

### Testing: Unity → ROS 2 (Publishing)

Create `Assets/Scripts/MessagePublisher.cs`:

```csharp
using UnityEngine;
using ROS2;
using std_msgs.msg;

public class MessagePublisher : MonoBehaviour
{
    private IPublisher<String> publisher;
    private int messageCount = 0;

    void Start()
    {
        var ros2Node = ROS2.ROS2UnityComponent.Instance.ROS2Node;

        // Create publisher on /unity_topic
        publisher = ros2Node.CreatePublisher<String>("/unity_topic");

        Debug.Log("[Publisher] Ready to publish on /unity_topic");

        // Publish a test message every 3 seconds
        InvokeRepeating("PublishMessage", 1.0f, 3.0f);
    }

    void PublishMessage()
    {
        var message = new String();
        message.data = $"Unity message #{messageCount++} at {System.DateTime.Now:HH:mm:ss.fff}";

        publisher.Publish(message);
        Debug.Log($"[Publisher] Sent: {message.data}");
    }

    void OnDestroy()
    {
        CancelInvoke("PublishMessage");
    }
}
```

---

## Hands-On: Setting Up Bidirectional Communication

### Step 1: ROS 2 Terminal (Ubuntu/Linux Machine)

Open a terminal and launch the ROS 2 TCP bridge:

```bash
# Start ROS 2 bridge (if not already running)
export ROS_DOMAIN_ID=42
ros2 run ros2_control_node tcp_bridge_node

# In another terminal, publish test messages:
ros2 topic pub /test_topic std_msgs/String "data: 'Hello from ROS 2'"
```

### Step 2: Unity Setup

1. Create an empty GameObject: **Right-click Hierarchy → Create Empty**
2. Name it `BridgeManager`
3. Add both scripts as components:
   - Attach `MessageReceiver.cs`
   - Attach `MessagePublisher.cs`
4. In Project Settings → ROS2 → Configure:
   - ROS Master IP: `192.168.1.100` (your ROS 2 machine)
   - Port: `9090`

### Step 3: Verify Connection

Press **Play** in Unity. Watch the Console:

**Expected output**:
```
[ROS Bridge] Connecting to ROS 2 at 192.168.1.100:9090
[ROS Bridge] Status: CONNECTED
[Receiver] Subscribed to /test_topic
[Receiver] Got message: Hello from ROS 2
[Publisher] Sent: Unity message #0 at 14:32:15.123
[Publisher] Sent: Unity message #1 at 14:32:18.456
```

### Checkpoint: Bidirectional Communication

Verify:
- ✅ Console shows "CONNECTED" (not "DISCONNECTED")
- ✅ Receiver logs incoming ROS 2 messages
- ✅ Publisher logs outgoing messages

In ROS 2 terminal, verify messages arriving:
```bash
ros2 topic echo /unity_topic
```

Should show:
```
data: 'Unity message #0 at 14:32:15.123'
data: 'Unity message #1 at 14:32:18.456'
```

If you see messages flowing both directions, **the bridge is operational**.

---

## Debugging Connection Issues

### Problem: "Connection refused" in Unity Console

**Causes**:
- ROS 2 machine not running tcp_bridge_node
- Firewall blocking port 9090
- Wrong IP address configured

**Solution**:
1. Verify bridge is running: `ros2 topic list` (should work on ROS 2 machine)
2. Check firewall: `sudo ufw allow 9090/tcp`
3. Verify IP with `ifconfig` on ROS 2 machine
4. Update Unity ROS settings with correct IP

### Problem: Publisher publishes but ROS 2 never receives

**Causes**:
- Topic name mismatch (`/unity_topic` vs `/unityTopic`)
- Message type mismatch (publishing String but ROS 2 expects different type)

**Solution**:
1. Check published topics: `ros2 topic list` (should show `/unity_topic`)
2. Inspect messages: `ros2 topic echo /unity_topic`
3. Verify message types: `ros2 topic info /unity_topic --verbose`

### Problem: No console output in Unity

**Causes**:
- ROS2UnityComponent not initialized
- Scripts attached to GameObject but not active
- Domain ID mismatch (ROS_DOMAIN_ID environment variable)

**Solution**:
1. Ensure `BridgeManager` GameObject is active in scene
2. Check Console for errors (red text)
3. Set same `ROS_DOMAIN_ID=42` on both ROS 2 and Unity
4. Restart both systems after ID change

---

## Network Topology Best Practices

**Development Setup** (this lesson):
```
ROS 2 Machine (Ubuntu)  ←→  TCP Bridge (port 9090)  ←→  Unity (Windows)
```

**Requirements**:
- Both machines on same network (or VPN tunnel)
- Firewall configured to allow port 9090
- ROS_DOMAIN_ID same on both sides

**Why not direct connection?**
- ROS 2 uses DDS (complex protocol)
- Unity doesn't have native DDS support
- TCP bridge translates between protocols
- This is the production pattern for any ROS 2 + game engine integration

---

## Try With AI

**Exploration 1: Understanding Bridge Failures**

Ask AI: "What are the most common ROS 2 to Unity bridge connection problems, and how would you diagnose each one?"

Then for each problem AI mentions:
- Simulate it (break the connection intentionally)
- Observe the error message
- Practice recovery

**Exploration 2: Scaling to Multiple Robots**

Ask AI: "If we wanted to control multiple robots from a single Unity scene, what changes to the bridge architecture would be necessary?"

Discuss:
- Multiple ROS 2 instances (one per robot)
- Namespace separation (robot1/topic vs robot2/topic)
- Single vs multiple subscriptions in Unity

Try implementing a second robot namespace:
```csharp
// Subscribe to two robots' topics
var sub1 = ros2Node.CreateSubscription<String>("/robot1/status", ...);
var sub2 = ros2Node.CreateSubscription<String>("/robot2/status", ...);
```

Monitor both topics: `ros2 topic echo /robot1/status` and `ros2 topic echo /robot2/status`

---

[Next: Lesson 2 - URDF Import and Visualization](02-urdf-import.md)
