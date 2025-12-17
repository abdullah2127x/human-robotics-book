---
sidebar_position: 5
title: "Lesson 5: Quality of Service (QoS)"
description: "Configure QoS policies to manage reliability and performance tradeoffs"
---

# Lesson 5: Quality of Service (QoS)

**Learning Outcome:** Configure Quality of Service policies and observe how reliability affects message delivery.

**Proficiency Level:** B1

**Estimated Time:** 40 minutes

**New Concepts:** 1
- QoS policies (reliability, history, durability) controlling message delivery

---

## Working Example: Observing Message Loss

First, let's see QoS impact in action. Create a **fast publisher** that sends 100 messages per second:

**Fast Publisher** (`fast_publisher.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time


class FastPublisher(Node):
    def __init__(self):
        super().__init__('fast_publisher')
        self.publisher_ = self.create_publisher(Int32, 'fast_topic', 10)
        self.get_logger().info('Fast publisher started (100 msg/sec)')
        self.counter = 0
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz

    def timer_callback(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher_.publish(msg)
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = FastPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Default Subscriber** (`default_subscriber.py`) - uses default BEST_EFFORT QoS:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class DefaultSubscriber(Node):
    def __init__(self):
        super().__init__('default_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'fast_topic',
            self.listener_callback,
            10)  # Default QoS (BEST_EFFORT)
        self.count = 0
        self.get_logger().info('Subscribing with default QoS')

    def listener_callback(self, msg):
        self.count += 1
        if self.count % 10 == 0:  # Log every 10 messages
            self.get_logger().info(f'Received message #{msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = DefaultSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Run both:**
```bash
# Terminal 1
python3 fast_publisher.py

# Terminal 2
python3 default_subscriber.py
```

**Observation:** Messages skip numbers! Publisher sends 1, 2, 3, 4... but subscriber receives 1, 5, 12, 18... Messages are being dropped.

---

## Understanding QoS Policies

Quality of Service (QoS) defines how aggressively ROS 2 tries to deliver messages. There are three main policies:

### Policy 1: Reliability

**BEST_EFFORT** (default):
- Publisher sends, subscriber gets if it's listening
- Fast but may drop messages
- Good for: Sensor streams (ok to miss a temperature reading)

**RELIABLE**:
- Publisher waits for acknowledgment
- Slower but guaranteed delivery
- Good for: Critical commands (must not miss emergency stop)

### Policy 2: Durability

**VOLATILE** (default):
- Messages only for currently-subscribed listeners
- Old messages discarded

**TRANSIENT_LOCAL**:
- New subscribers get recent messages
- Good for: Startup recovery (late subscriber gets state)

### Policy 3: History

**KEEP_LAST N**:
- Remember only last N messages
- Efficient memory use

**KEEP_ALL**:
- Remember every message
- Risk of memory overflow

---

## Working Example: Reliable QoS

**Reliable Publisher** (`reliable_publisher.py`):

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int32


class ReliablePublisher(Node):
    def __init__(self):
        super().__init__('reliable_publisher')

        # Create custom QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10)

        self.publisher_ = self.create_publisher(
            Int32,
            'reliable_topic',
            qos_profile)
        self.get_logger().info('Reliable publisher started')
        self.counter = 0
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher_.publish(msg)
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = ReliablePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Reliable Subscriber** (`reliable_subscriber.py`):

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int32


class ReliableSubscriber(Node):
    def __init__(self):
        super().__init__('reliable_subscriber')

        # Create matching QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10)

        self.subscription = self.create_subscription(
            Int32,
            'reliable_topic',
            self.listener_callback,
            qos_profile)
        self.count = 0
        self.get_logger().info('Subscribing with RELIABLE QoS')

    def listener_callback(self, msg):
        self.count += 1
        if self.count % 10 == 0:
            self.get_logger().info(f'Received message #{msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ReliableSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Run both:**
```bash
# Terminal 1
python3 reliable_publisher.py

# Terminal 2
python3 reliable_subscriber.py
```

**Observation:** Messages arrive in order without skipping! 1, 2, 3, 4, 5... All messages delivered reliably.

---

## QoS Matching Requirement

**Important:** Publisher and subscriber QoS must be compatible. If they don't match, ROS 2 issues warnings:

```
[WARN] QoS mismatch between publisher and subscriber
```

The publisher sets the guarantee it can make. The subscriber must request a guarantee at or below what the publisher offers:
- RELIABLE publisher can talk to RELIABLE subscribers
- RELIABLE publisher can talk to BEST_EFFORT subscribers (subscriber gets less reliability)
- BEST_EFFORT publisher CANNOT talk to RELIABLE subscribers (can't guarantee reliability)

---

## Hands-On Practice

### Exercise 5.1: Observe Message Loss

Run the fast_publisher with default_subscriber (from above). Count how many messages are skipped.

**Task:** In 10 seconds of publishing, how many messages are dropped?

**Success Criteria:**
- [ ] Fast publisher runs at 100 Hz
- [ ] Default subscriber shows message gaps
- [ ] Document the loss percentage

### Exercise 5.2: Configure Custom QoS

Create a file called `custom_qos_subscriber.py` that:
- Subscribes with RELIABLE QoS policy
- Runs with the fast_publisher
- Receives all messages without drops

**Success Criteria:**
- [ ] QoS profile created with RELIABLE policy
- [ ] Subscriber receives all messages in sequence
- [ ] No gaps in message numbers

### Exercise 5.3: QoS Decision Matrix

For each robot component, decide the appropriate QoS:

| Component | Topic | Reliability | Reason |
|-----------|-------|-------------|--------|
| Temperature sensor | /sensor/temperature | BEST_EFFORT | ??? |
| Emergency stop | /cmd/emergency | RELIABLE | ??? |
| Motor feedback | /motor/state | ??? | ??? |

Fill in the missing rows and justify each choice.

**Success Criteria:**
- [ ] All components assigned appropriate QoS
- [ ] Clear reasoning for each choice
- [ ] Consideration of consequences of message loss

---

## Common QoS Patterns

**Sensor Data:**
```python
QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
```
Fast, occasional message loss acceptable.

**Critical Commands:**
```python
QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)
```
Guaranteed delivery, latency less critical.

**Startup Recovery:**
```python
QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=5)
```
Late subscribers get recent state.

---

## Try With AI

Explore QoS tradeoffs in robotics:

**Step 1: Ask about real-world scenarios**
Prompt your AI:
```
In a robot system, consider these scenarios:

1. Camera frame streaming (30 fps)
2. Navigation commands to robot
3. Emergency stop signal
4. Robot battery percentage update

For each, should QoS be BEST_EFFORT or RELIABLE? Why?
What happens if you choose wrong?
```

**Step 2: Think about consequences**
Ask yourself:
- Which components can tolerate message loss?
- Which must never lose a message?
- What's the latency tolerance for each?

**Step 3: Design a robot system's QoS**
Create a document listing:
- 5 robot topics/services
- QoS policy for each
- Justification for the choice

**Step 4: Consider performance**
Prompt your AI:
```
If we set all topics to RELIABLE QoS, what happens to:
1. Message latency?
2. Network bandwidth?
3. System responsiveness?

Is there a middle ground?
```

**Expected Outcome:** Understanding that QoS is a tradeoff between safety (reliability) and performance (speed).
