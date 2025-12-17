---
sidebar_position: 9
---

# Lesson 9: Capstone - Voice-Controlled Humanoid Navigation

**Layer 4: Spec-Driven Integration** | **Estimated Time: 180 minutes**

---

## Learning Objectives

By the end of this capstone, you will have:

- [ ] Written a specification for voice-controlled navigation
- [ ] Integrated all Chapter 10 components into a working system
- [ ] Connected to Nav2 from Part 3
- [ ] Achieved under 2 second end-to-end latency
- [ ] Handled errors gracefully
- [ ] Demonstrated voice-controlled humanoid navigation

---

## Capstone Specification

**Write your specification BEFORE implementation.**

### Intent

Implement a voice-controlled humanoid navigation system. You speak navigation commands ("Go to the kitchen"), the system transcribes with Whisper, parses to intent, publishes to ROS 2, and the humanoid navigates using Nav2.

### Constraints

| Constraint | Target |
|------------|--------|
| End-to-end latency | under 2 seconds from speech end to navigation start |
| Transcription accuracy | over 90% WER in quiet environment |
| Intent parsing accuracy | over 90% on navigation commands |
| Error handling | Graceful degradation, no crashes |
| Integration | Works with Nav2 stack from Part 3 |

### Success Criteria

- [ ] Voice command "Go to the kitchen" triggers navigation to kitchen waypoint
- [ ] Voice command "Stop" halts navigation immediately
- [ ] System asks for clarification on ambiguous commands ("Go there")
- [ ] Full pipeline latency under 2 seconds (measured)
- [ ] Error handling works (API failure -> graceful degradation)
- [ ] Integration with Isaac Sim or Gazebo humanoid from Parts 2-3

---

## Component Composition

### From Chapter 10 Lessons

| Component | Source | Purpose |
|-----------|--------|---------|
| Audio Capture | Lesson 3 | Real-time microphone input |
| VAD | Lesson 4 | Command segmentation |
| Whisper | Lesson 2 | Speech-to-text |
| Intent Parser | Lesson 6 | Text-to-intent |
| ROS 2 Node | Lesson 7 | Intent publishing |
| Parsing Skill | Lesson 8 | Design patterns |

### From Part 3

| Component | Source | Purpose |
|-----------|--------|---------|
| Nav2 Stack | Chapter 9 | Navigation execution |
| Waypoints | Chapter 9 | Location coordinates |
| Behavior Trees | Chapter 9 | Navigation behaviors |

### From Part 1

| Component | Source | Purpose |
|-----------|--------|---------|
| rclpy | Chapter 2 | ROS 2 Python client |
| Async patterns | Chapter 2 | Non-blocking operations |

---

## Integration Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Voice-Controlled Navigation                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐    │
│  │   Mic    │──▶│   VAD    │──▶│ Whisper  │──▶│  Parser  │    │
│  │ Capture  │   │ Segment  │   │Transcribe│   │  Intent  │    │
│  └──────────┘   └──────────┘   └──────────┘   └──────────┘    │
│       │              │              │              │           │
│       │              │              │              ▼           │
│       │              │              │       ┌──────────┐       │
│       │              │              │       │  ROS 2   │       │
│       │              │              │       │Publisher │       │
│       │              │              │       └────┬─────┘       │
│       │              │              │            │             │
├───────┼──────────────┼──────────────┼────────────┼─────────────┤
│       │              │              │            │             │
│       │              │              │            ▼             │
│  ┌────────────────────────────────────────────────────┐       │
│  │                     Nav2 Stack                      │       │
│  │  ┌──────────┐   ┌──────────┐   ┌──────────┐       │       │
│  │  │ Waypoint │──▶│ Planner  │──▶│Controller│       │       │
│  │  │  Manager │   │          │   │          │       │       │
│  │  └──────────┘   └──────────┘   └──────────┘       │       │
│  └────────────────────────────────────────────────────┘       │
│                            │                                   │
│                            ▼                                   │
│                    ┌──────────────┐                           │
│                    │   Humanoid   │                           │
│                    │   (Gazebo/   │                           │
│                    │   Isaac Sim) │                           │
│                    └──────────────┘                           │
└─────────────────────────────────────────────────────────────────┘
```

---

## Integration Workflow

### Step 1: Write Your Spec (FIRST!)

Before writing any code, create `capstone_spec.md`:

```markdown
# Voice Navigation Capstone Spec

## Supported Commands
- "Go to [location]" - Navigate to predefined waypoint
- "Stop" - Halt current navigation
- "Where are you?" - Report current location

## Locations
- kitchen: (5.0, 2.0)
- bedroom: (-3.0, 4.0)
- living room: (0.0, 0.0)

## Error Handling
- Unknown location: "I don't know where [X] is"
- Ambiguous: "Where would you like me to go?"
- Low confidence: "Did you say [X]?"

## Latency Budget
- Audio capture: under 100ms
- VAD + segmentation: under 200ms
- Whisper transcription: under 800ms
- Intent parsing: under 100ms
- Nav2 goal send: under 200ms
- TOTAL: under 1400ms (buffer to 2000ms)
```

### Step 2: Assemble Components

```python
#!/usr/bin/env python3
"""
Capstone: Voice-Controlled Humanoid Navigation
"""

import rclpy
from rclpy.node import Node
import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel

from voice_interfaces.msg import VoiceIntent
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class VoiceNavigationNode(Node):
    """Complete voice-controlled navigation system."""

    WAYPOINTS = {
        'kitchen': {'x': 5.0, 'y': 2.0, 'theta': 0.0},
        'bedroom': {'x': -3.0, 'y': 4.0, 'theta': 1.57},
        'living room': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
    }

    def __init__(self):
        super().__init__('voice_navigation_node')

        # Components from Chapter 10
        self.whisper = WhisperModel("base", device="cpu")
        self.parser = HybridParser()
        self.vad = CommandSegmenter()

        # Nav2 action client
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # Publishers
        self.intent_pub = self.create_publisher(
            VoiceIntent, '/voice_intent', 10
        )

        self.get_logger().info('Voice Navigation Ready!')
        self.start_listening()

    def process_command(self, audio: np.ndarray):
        """Full pipeline: audio -> navigation."""
        import time

        start_time = time.time()

        # Transcribe
        segments, _ = self.whisper.transcribe(audio, language="en")
        text = " ".join(s.text.strip() for s in segments)
        transcribe_time = time.time() - start_time

        if not text:
            return

        self.get_logger().info(f'Heard: "{text}"')

        # Parse
        parse_start = time.time()
        intent = self.parser.parse(text)
        parse_time = time.time() - parse_start

        # Publish intent
        self.publish_intent(intent)

        # Execute navigation
        if intent.action == "navigate" and intent.target:
            self.navigate_to(intent.target)
        elif intent.action == "control" and intent.target == "stop":
            self.stop_navigation()
        elif intent.needs_clarification():
            self.request_clarification(intent)

        total_time = time.time() - start_time
        self.get_logger().info(
            f'Latency: transcribe={transcribe_time:.2f}s, '
            f'parse={parse_time:.3f}s, total={total_time:.2f}s'
        )

    def navigate_to(self, location: str):
        """Send navigation goal to Nav2."""
        location = location.lower()

        if location not in self.WAYPOINTS:
            self.get_logger().warn(f'Unknown location: {location}')
            return

        waypoint = self.WAYPOINTS[location]

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint['x']
        goal.pose.pose.position.y = waypoint['y']

        self.get_logger().info(f'Navigating to {location}...')
        self.nav_client.send_goal_async(goal)

    def stop_navigation(self):
        """Cancel current navigation."""
        self.get_logger().info('Stopping navigation')
        # Cancel all goals
        self.nav_client._cancel_goal_async()
```

### Step 3: Test Iteratively

```bash
# Terminal 1: Start simulation
ros2 launch humanoid_gazebo simulation.launch.py

# Terminal 2: Start Nav2
ros2 launch nav2_bringup navigation_launch.py

# Terminal 3: Start voice navigation
ros2 run voice_command voice_navigation_node

# Terminal 4: Monitor
ros2 topic echo /voice_intent
```

### Step 4: Measure and Optimize

```python
def measure_latency(self):
    """Measure end-to-end latency."""
    import time

    # Record timestamps at each stage
    timestamps = {
        'audio_start': time.time(),
        'vad_complete': None,
        'transcribe_complete': None,
        'parse_complete': None,
        'goal_sent': None,
    }

    # ... run pipeline ...

    # Calculate stage latencies
    stages = [
        ('VAD', timestamps['vad_complete'] - timestamps['audio_start']),
        ('Transcribe', timestamps['transcribe_complete'] - timestamps['vad_complete']),
        ('Parse', timestamps['parse_complete'] - timestamps['transcribe_complete']),
        ('Goal', timestamps['goal_sent'] - timestamps['parse_complete']),
    ]

    total = timestamps['goal_sent'] - timestamps['audio_start']
    self.get_logger().info(f'Total latency: {total:.2f}s')

    for name, lat in stages:
        self.get_logger().info(f'  {name}: {lat:.3f}s')

    return total < 2.0  # Target: under 2 seconds
```

---

## Success Validation

### Validation Checklist

Run through each criterion:

- [ ] **Navigation Command**: Say "Go to the kitchen"
  - Robot starts moving toward kitchen waypoint
  - Latency under 2 seconds

- [ ] **Stop Command**: Say "Stop"
  - Robot halts immediately
  - Navigation canceled

- [ ] **Ambiguous Command**: Say "Go there"
  - System asks "Where would you like me to go?"
  - Does not crash or execute wrong action

- [ ] **Error Handling**: Disconnect Whisper API (if using)
  - Falls back gracefully
  - Reports error clearly

- [ ] **Latency Measurement**: Record 10 commands
  - Average latency under 2 seconds
  - All commands under 3 seconds

### Final Demo

**5-Minute Voice-Controlled Navigation Demo**:

1. Start simulation and navigation stack
2. Demonstrate 3 successful navigation commands
3. Demonstrate stop command
4. Demonstrate ambiguous command handling
5. Show latency measurements

---

## Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| High latency | Use smaller Whisper model, increase buffer |
| Navigation fails | Check waypoint coordinates, Nav2 map |
| VAD false positives | Increase threshold, use Silero VAD |
| Intent parsing errors | Add more patterns, tune confidence |

---

## Summary

In this capstone, you:

1. **Wrote specification first** - Defined requirements before code
2. **Integrated all components** - Audio, VAD, Whisper, Parser, ROS 2
3. **Connected to Nav2** - Voice commands trigger real navigation
4. **Met performance targets** - under 2 second latency, over 90% accuracy
5. **Handled errors gracefully** - No crashes, helpful responses

**Congratulations!** You've built a voice-controlled humanoid robot.

---

## What's Next?

- **Chapter 11**: LLM-Powered Task Planning
  - Use the intent parsing skill for more complex reasoning
  - Break down high-level commands into multi-step plans

- **Chapter 12**: Autonomous Humanoid Capstone
  - Full VLA integration
  - End-to-end autonomous operation
