---
sidebar_position: 7
---

# Lesson 7: ROS 2 Integration

**Layer 2: AI Collaboration** | **Estimated Time: 120 minutes**

---

## Learning Objectives

By the end of this lesson, you will be able to:

- [ ] Define custom ROS 2 message types for voice intents
- [ ] Build a voice command node using rclpy
- [ ] Publish intents to ROS 2 topics
- [ ] Implement lifecycle node patterns
- [ ] Connect voice intents to Nav2 navigation

---

## Custom ROS 2 Message Types

Define a message type for voice intents.

### VoiceIntent.msg

Create `msg/VoiceIntent.msg`:

```
# Voice Intent Message
# Represents a parsed voice command for robot execution

string action           # navigate, pick, place, query, control
string target           # kitchen, ball, location, stop
string[] parameter_keys # color, size, location
string[] parameter_values
float32 confidence      # 0.0 to 1.0
string raw_text         # Original transcription
builtin_interfaces/Time timestamp
string error            # Error message if any
```

### Building the Message

In `CMakeLists.txt`:
```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VoiceIntent.msg"
  DEPENDENCIES builtin_interfaces
)
```

In `package.xml`:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

---

## Voice Node Architecture

The voice command node orchestrates the full pipeline.

```
┌─────────────────────────────────────────────────┐
│              VoiceCommandNode                    │
│                                                 │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐      │
│  │  Audio   │─▶│ Whisper  │─▶│  Intent  │      │
│  │ Capture  │  │Transcribe│  │  Parser  │      │
│  └──────────┘  └──────────┘  └──────────┘      │
│                                    │            │
│                                    ▼            │
│  ┌──────────────────────────────────────┐      │
│  │         Publisher                     │      │
│  │  /voice_intent (VoiceIntent)         │      │
│  │  /voice_diagnostics (DiagnosticArray)│      │
│  └──────────────────────────────────────┘      │
└─────────────────────────────────────────────────┘
```

### Basic Implementation

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from voice_interfaces.msg import VoiceIntent
from std_msgs.msg import String
import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel

class VoiceCommandNode(Node):
    """ROS 2 node for voice command processing."""

    def __init__(self):
        super().__init__('voice_command_node')

        # Parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('whisper_model', 'base')
        self.declare_parameter('confidence_threshold', 0.6)

        self.sample_rate = self.get_parameter('sample_rate').value
        self.model_size = self.get_parameter('whisper_model').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        # Publishers
        self.intent_pub = self.create_publisher(
            VoiceIntent,
            '/voice_intent',
            10
        )

        self.diagnostics_pub = self.create_publisher(
            String,
            '/voice_diagnostics',
            10
        )

        # Initialize components
        self.get_logger().info('Loading Whisper model...')
        self.whisper = self._load_whisper()

        self.parser = HybridParser()

        # Start audio capture
        self.audio_buffer = []
        self.is_listening = True

        self.get_logger().info('Voice command node ready!')

    def _load_whisper(self):
        """Load Whisper model with fallback."""
        try:
            return WhisperModel(self.model_size, device="cuda")
        except:
            self.get_logger().warn('GPU not available, using CPU')
            return WhisperModel(self.model_size, device="cpu")

    def process_audio(self, audio: np.ndarray):
        """Process audio through the full pipeline."""
        # Transcribe
        segments, _ = self.whisper.transcribe(audio, language="en")
        text = " ".join(s.text.strip() for s in segments)

        if not text:
            return

        self.get_logger().info(f'Transcribed: "{text}"')

        # Parse intent
        intent = self.parser.parse(text)

        # Publish if confident enough
        if intent.confidence >= self.confidence_threshold:
            self.publish_intent(intent)
        else:
            self.get_logger().warn(
                f'Low confidence ({intent.confidence:.2f}): {text}'
            )

    def publish_intent(self, intent):
        """Publish intent to ROS 2 topic."""
        msg = VoiceIntent()
        msg.action = intent.action
        msg.target = intent.target or ''
        msg.parameter_keys = list(intent.parameters.keys())
        msg.parameter_values = [str(v) for v in intent.parameters.values()]
        msg.confidence = intent.confidence
        msg.raw_text = intent.raw_text
        msg.timestamp = self.get_clock().now().to_msg()
        msg.error = intent.error or ''

        self.intent_pub.publish(msg)
        self.get_logger().info(
            f'Published: {intent.action} -> {intent.target}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Topic Design

### /voice_intent

Primary topic for parsed intents:

```bash
ros2 topic echo /voice_intent
# Output:
# action: navigate
# target: kitchen
# parameter_keys: []
# parameter_values: []
# confidence: 0.95
# raw_text: go to the kitchen
# timestamp: ...
# error: ''
```

### /voice_diagnostics

Diagnostic information for monitoring:

```python
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

def publish_diagnostics(self):
    """Publish diagnostic information."""
    msg = DiagnosticArray()
    msg.header.stamp = self.get_clock().now().to_msg()

    status = DiagnosticStatus()
    status.name = 'voice_command_node'
    status.level = DiagnosticStatus.OK
    status.message = 'Operating normally'
    status.values = [
        KeyValue(key='model', value=self.model_size),
        KeyValue(key='sample_rate', value=str(self.sample_rate)),
        KeyValue(key='is_listening', value=str(self.is_listening)),
    ]

    msg.status.append(status)
    self.diagnostics_pub.publish(msg)
```

---

## Lifecycle Node Patterns

Use lifecycle nodes for managed startup/shutdown.

```python
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn

class VoiceCommandLifecycleNode(LifecycleNode):
    """Lifecycle-managed voice command node."""

    def __init__(self):
        super().__init__('voice_command_node')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure node (load models, create publishers)."""
        self.get_logger().info('Configuring...')

        # Load Whisper model
        self.whisper = self._load_whisper()

        # Create publishers (inactive until activated)
        self.intent_pub = self.create_lifecycle_publisher(
            VoiceIntent, '/voice_intent', 10
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate node (start listening)."""
        self.get_logger().info('Activating...')
        self.start_audio_capture()
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate node (stop listening)."""
        self.get_logger().info('Deactivating...')
        self.stop_audio_capture()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Cleanup resources."""
        self.get_logger().info('Cleaning up...')
        self.whisper = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown node."""
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS
```

---

## Nav2 Integration

Convert voice intents to Nav2 navigation goals.

```python
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class Nav2Bridge:
    """Bridge between voice intents and Nav2."""

    # Predefined waypoints (in map frame)
    WAYPOINTS = {
        'kitchen': {'x': 5.0, 'y': 2.0, 'theta': 0.0},
        'bedroom': {'x': -3.0, 'y': 4.0, 'theta': 1.57},
        'living room': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
    }

    def __init__(self, node):
        self.node = node
        self.nav_client = ActionClient(
            node,
            NavigateToPose,
            'navigate_to_pose'
        )

    def handle_navigate_intent(self, intent):
        """Convert navigate intent to Nav2 goal."""
        location = intent.target.lower()

        if location not in self.WAYPOINTS:
            self.node.get_logger().error(f'Unknown location: {location}')
            return False

        waypoint = self.WAYPOINTS[location]

        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint['x']
        goal.pose.pose.position.y = waypoint['y']

        # Convert theta to quaternion
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, waypoint['theta'])
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        # Send goal
        self.node.get_logger().info(f'Navigating to {location}...')
        self.nav_client.send_goal_async(goal)

        return True
```

---

## Practice Exercise

### Goal
Build a complete voice command node and test with Nav2.

### Steps

1. **Create ROS 2 package**:
```bash
ros2 pkg create voice_command --build-type ament_python
```

2. **Define VoiceIntent message**

3. **Implement voice_command_node.py**

4. **Test with topic echo**:
```bash
ros2 topic echo /voice_intent
```

5. **Integrate with Nav2** (if simulation available)

### Success Criteria

- [ ] VoiceIntent message defined and building
- [ ] Node publishes intents to /voice_intent
- [ ] Parameters configurable at runtime
- [ ] Lifecycle transitions work correctly
- [ ] Nav2 bridge converts intents to goals
- [ ] Full pipeline: speak -> transcribe -> parse -> publish -> navigate

---

## Lesson Checkpoint

Before proceeding, verify you can answer:

1. **Why use custom message types?**
   > Structured data for intents with all fields needed by subscribers

2. **What are lifecycle node benefits?**
   > Managed startup/shutdown, resource cleanup, state transitions

3. **How do you test the voice node?**
   > ros2 topic echo /voice_intent to see published messages

4. **How does Nav2 integration work?**
   > Convert intent target to waypoint coordinates, send as NavigateToPose goal

---

## Summary

In this lesson, you learned:

- Custom ROS 2 message types for voice intents
- Voice command node architecture
- Topic design (/voice_intent, /voice_diagnostics)
- Lifecycle node patterns for managed operation
- Parameter server integration
- Nav2 bridge for navigation intents

**Next**: [Lesson 8: Voice-Intent-Parsing Skill](./lesson-08-voice-intent-parsing-skill.md) - Create a reusable skill.
