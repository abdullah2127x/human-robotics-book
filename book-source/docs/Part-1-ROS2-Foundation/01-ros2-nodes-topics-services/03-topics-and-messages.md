---
sidebar_position: 3
title: "Lesson 3: Debugging with ROS 2 CLI Tools"
description: "Learn to inspect running ROS 2 systems using command-line tools"
---

# Lesson 3: Debugging with ROS 2 CLI Tools

**Learning Outcome:** Use ROS 2 command-line tools to inspect and debug running nodes and topics.

**Proficiency Level:** B1

**Estimated Time:** 45 minutes

**New Concepts:** 2
- ROS 2 CLI tools for system introspection
- Systematic debugging workflow

---

## Working Example: Diagnosing a Broken System

First, let's create a publisher with a bug (on purpose):

**Broken Publisher** (`broken_publisher.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BrokenPublisher(Node):
    def __init__(self):
        super().__init__('broken_publisher')
        # BUG: Publishing to wrong topic name
        self.publisher_ = self.create_publisher(String, 'temperature_wrong', 10)
        self.get_logger().info('Publisher created (on wrong topic!)')
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'temperature: 25.0°C'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing temperature')


def main(args=None):
    rclpy.init(args=args)
    node = BrokenPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Working Subscriber** (`working_subscriber.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WorkingSubscriber(Node):
    def __init__(self):
        super().__init__('working_subscriber')
        self.subscription = self.create_subscription(
            String,
            'temperature',  # Correct topic name
            self.listener_callback,
            10)
        self.get_logger().info('Subscribed to "temperature"')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = WorkingSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Run both nodes:**
```bash
# Terminal 1
python3 broken_publisher.py

# Terminal 2
python3 working_subscriber.py
```

The subscriber logs nothing! The publisher says it's publishing, but subscriber receives nothing. What's wrong?

---

## Debugging with ROS 2 CLI Tools

Instead of guessing, use ROS 2 tools to inspect the system:

### Tool 1: List All Topics

```bash
# Terminal 3
ros2 topic list
```

**Output:**
```
/rosout
/temperature_wrong
```

Aha! The publisher is sending to `/temperature_wrong`, but the subscriber listens to `/temperature`. The topic names don't match!

### Tool 2: Check Topic Details

```bash
ros2 topic info /temperature
```

**Output:**
```
Type: std_msgs/msg/String
Publisher count: 0
Subscriber count: 1
```

The topic `/temperature` exists with 1 subscriber, but NO publishers. This confirms the topic name mismatch.

```bash
ros2 topic info /temperature_wrong
```

**Output:**
```
Type: std_msgs/msg/String
Publisher count: 1
Subscriber count: 0
```

The topic `/temperature_wrong` has 1 publisher but NO subscribers.

### Tool 3: See Live Messages

```bash
ros2 topic echo /temperature_wrong
```

**Output:**
```
data: temperature: 25.0°C
---
data: temperature: 25.0°C
---
```

Messages ARE being published to `/temperature_wrong`! Now we know the problem: topic name mismatch.

### Tool 4: Measure Message Frequency

```bash
ros2 topic hz /temperature_wrong
```

**Output:**
```
average rate: 1.000
    min: 1.000s max: 1.000s std dev: 0.00000s count: 10
```

The publisher sends 1 message per second as expected.

### Tool 5: List All Nodes

```bash
ros2 node list
```

**Output:**
```
/broken_publisher
/rosout
/working_subscriber
```

Both nodes are running. We can see which nodes exist in the system.

### Tool 6: Get Node Details

```bash
ros2 node info /broken_publisher
```

**Output:**
```
/broken_publisher
  Subscribers:
  Publishers:
    /rosout: std_msgs/msg/Log
    /temperature_wrong: std_msgs/msg/String
  ...
```

The publisher publishes to `/temperature_wrong` (confirming our finding).

---

## Systematic Debugging Workflow

When something isn't working, follow this sequence:

1. **List topics:** `ros2 topic list`
   - Are the expected topics there?
   - Are there unexpected topics?

2. **Check topic info:** `ros2 topic info [topic_name]`
   - Do publishers and subscribers match what you expect?
   - Is something publishing when nothing should be?

3. **Watch live messages:** `ros2 topic echo [topic_name]`
   - Are messages actually flowing?
   - Do messages have the right data?

4. **Measure frequency:** `ros2 topic hz [topic_name]`
   - Are messages arriving at the expected rate?
   - Is the system too slow?

5. **Check nodes:** `ros2 node list` and `ros2 node info [node_name]`
   - Are all nodes running?
   - What publishers/subscribers does each have?

---

## Hands-On Practice

### Exercise 3.1: Diagnose the Broken System

Use the broken and working nodes from above. Without looking at the code, use CLI tools to:
1. Identify that the publisher and subscriber are on different topics
2. List which topics have publishers and subscribers
3. View the actual messages being published

**Success Criteria:**
- [ ] Use `ros2 topic list` successfully
- [ ] Use `ros2 topic info` successfully
- [ ] Use `ros2 topic echo` successfully
- [ ] Identify the mismatch

### Exercise 3.2: Verify Message Flow

Create two working nodes (temperature_publisher.py and temperature_subscriber.py from Lesson 2).

Use CLI tools to verify:
1. Both topics exist and match
2. Each topic has the correct publisher and subscriber count
3. Messages are flowing at the correct frequency

**Success Criteria:**
- [ ] Publisher and subscriber on same topic
- [ ] `ros2 topic info` shows 1 publisher and 1 subscriber
- [ ] `ros2 topic hz` shows ~1 Hz frequency

---

## Try With AI

Use AI to develop debugging intuition:

**Step 1: Ask about common problems**
Prompt your AI:
```
In ROS 2 systems, what are common reasons why:
1. A subscriber doesn't receive messages from a publisher?
2. A node crashes immediately after starting?
3. Messages arrive very slowly (high latency)?

For each problem, what ROS 2 CLI tool would help diagnose it?
```

**Step 2: Review the AI's suggestions**
Ask yourself:
- Which tool checks topic connectivity?
- Which tool checks message flow?
- Which tool measures system performance?

**Step 3: Test real scenarios**
Create intentional problems and use CLI tools to diagnose:
- Wrong topic names (like the broken_publisher above)
- Missing nodes
- Slow message rates

**Step 4: Document findings**
Create a table:
| Problem | CLI Tool | What It Reveals |
|---------|----------|-----------------|
| Topic name mismatch | ros2 topic info | Publisher/subscriber counts |
| ... | ... | ... |

**Expected Outcome:** Ability to use CLI tools systematically to diagnose any ROS 2 communication problem.
