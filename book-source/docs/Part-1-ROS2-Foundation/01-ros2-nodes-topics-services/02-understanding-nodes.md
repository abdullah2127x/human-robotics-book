---
sidebar_position: 2
title: "Lesson 2: Topics and Pub/Sub Communication"
description: "Connect nodes together using publisher-subscriber pattern through topics"
---

# Lesson 2: Topics and Pub/Sub Communication

**Learning Outcome:** Implement a publisher node that sends data and a subscriber node that receives it.

**Proficiency Level:** B1

**Estimated Time:** 50 minutes

**New Concepts:** 3
- Topic: named bus for many-to-many communication
- Publisher: node that sends messages to a topic
- Subscriber: node that receives messages from a topic

---

## Working Example: Temperature Sensor System

Let's see a complete publisher-subscriber system in action. This simulates a temperature sensor sending readings.

**Publisher Node** (`temperature_publisher.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(String, 'temperature', 10)
        self.get_logger().info('Publisher created on topic "temperature"')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.temperature = 20.0

    def timer_callback(self):
        msg = String()
        msg.data = f'temperature: {self.temperature:.1f}°C'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.temperature += 0.5  # Simulate warming


def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Subscriber Node** (`temperature_subscriber.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TemperatureSubscriber(Node):
    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(
            String,
            'temperature',
            self.listener_callback,
            10)
        self.get_logger().info('Subscribed to topic "temperature"')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**To run both nodes:**

```bash
# Terminal 1 (Publisher)
python3 temperature_publisher.py

# Terminal 2 (Subscriber)
python3 temperature_subscriber.py
```

**Expected Output (Terminal 1):**
```
[INFO] [temperature_publisher]: Publisher created on topic "temperature"
[INFO] [temperature_publisher]: Publishing: temperature: 20.0°C
[INFO] [temperature_publisher]: Publishing: temperature: 20.5°C
[INFO] [temperature_publisher]: Publishing: temperature: 21.0°C
...
```

**Expected Output (Terminal 2):**
```
[INFO] [temperature_subscriber]: Subscribed to topic "temperature"
[INFO] [temperature_subscriber]: Received: temperature: 20.0°C
[INFO] [temperature_subscriber]: Received: temperature: 20.5°C
[INFO] [temperature_subscriber]: Received: temperature: 21.0°C
...
```

The subscriber receives all messages the publisher sends!

---

## Understanding Topics and Communication

A **topic** is like a radio frequency. Multiple publishers can broadcast to it, and multiple subscribers can listen to it. They don't need to know about each other directly.

### The Pub/Sub Pattern

```
Publisher Node          Topic "temperature"          Subscriber Node
    |                          |                           |
    |--- publishes message --> |--- delivers message ----> |
    |                          |                           |
    |--- publishes message --> |--- delivers message ----> |
    |                          |                           |
```

### Key Characteristics

**Decoupled:** The publisher doesn't know about subscribers. The subscriber doesn't know about publishers. They only know the topic name.

**Many-to-Many:** Multiple publishers can send to the same topic. Multiple subscribers can listen to the same topic.

**Asynchronous:** The publisher doesn't wait for subscribers to receive. It just publishes and continues.

### Breaking Down the Publisher Code

**Create a Publisher:**
```python
self.publisher_ = self.create_publisher(String, 'temperature', 10)
```
- `String` is the message type (from `std_msgs.msg`)
- `'temperature'` is the topic name
- `10` is the queue size (buffer for messages if subscriber is slow)

**Publish a Message:**
```python
msg = String()
msg.data = f'temperature: {self.temperature:.1f}°C'
self.publisher_.publish(msg)
```
Create a message, fill it with data, and publish to the topic.

### Breaking Down the Subscriber Code

**Create a Subscriber:**
```python
self.subscription = self.create_subscription(
    String,
    'temperature',
    self.listener_callback,
    10)
```
- `String` is the message type to receive
- `'temperature'` is the topic to listen to
- `self.listener_callback` is the function called when a message arrives
- `10` is the queue size

**Handle Incoming Messages:**
```python
def listener_callback(self, msg):
    self.get_logger().info(f'Received: {msg.data}')
```
This function automatically executes when a message arrives on the topic.

---

## Hands-On Practice

### Exercise 2.1: Create a Battery Publisher

Create a new file called `battery_publisher.py`. Write a publisher that:
- Creates a node named 'battery_publisher'
- Publishes to a topic called 'battery'
- Sends messages like "battery: 100%", "battery: 95%", etc.
- Decreases battery by 1% every message
- Publishes every 0.5 seconds

**Success Criteria:**
- [ ] Node runs without errors
- [ ] Messages appear in logs showing battery percentage
- [ ] Battery decreases each message
- [ ] Publishes at 0.5 second interval

<details>
<summary>Solution</summary>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher_ = self.create_publisher(String, 'battery', 10)
        self.get_logger().info('Battery publisher started')
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.battery = 100

    def timer_callback(self):
        msg = String()
        msg.data = f'battery: {self.battery}%'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.battery -= 1
        if self.battery < 0:
            self.battery = 100


def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</details>

### Exercise 2.2: Create a Battery Subscriber

Create a new file called `battery_subscriber.py` that:
- Subscribes to the 'battery' topic
- Receives messages from the battery_publisher
- Logs received messages with "Battery Status: [message]"

Run both the publisher and subscriber in separate terminals. Verify that the subscriber receives all messages.

**Success Criteria:**
- [ ] Subscriber connects to topic without errors
- [ ] Receives messages from publisher
- [ ] Both nodes communicate successfully

<details>
<summary>Solution</summary>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BatterySubscriber(Node):
    def __init__(self):
        super().__init__('battery_subscriber')
        self.subscription = self.create_subscription(
            String,
            'battery',
            self.listener_callback,
            10)
        self.get_logger().info('Battery subscriber listening...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Battery Status: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = BatterySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</details>

### Exercise 2.3: Multiple Subscribers

Run the battery_publisher from Exercise 2.1, but start TWO subscribers in different terminals. Observe that both receive the same messages.

This demonstrates the many-to-many capability of topics.

**Success Criteria:**
- [ ] Both subscribers receive messages from single publisher
- [ ] Messages arrive at both subscribers simultaneously
- [ ] Each subscriber logs independently

---

## Try With AI

Use AI to explore how the publisher-subscriber pattern handles real-world scenarios:

**Step 1: Ask about message loss**
Prompt your AI:
```
In the temperature publisher-subscriber example, what happens if:
1. The subscriber starts AFTER the publisher has been running?
2. The subscriber crashes and restarts?
3. Multiple subscribers connect to the same topic?

How does ROS 2 handle these scenarios?
```

**Step 2: Review the response**
Ask yourself:
- Which scenarios could result in lost messages?
- What's the difference between buffering (the queue size parameter) and guaranteed delivery?

**Step 3: Test your understanding**
Modify one of the examples:
- Start the publisher
- Wait 5 seconds
- Start the subscriber
- Observe what messages it receives

**Step 4: Reflect**
- Did the subscriber get all messages, or only new ones?
- Why is this behavior useful in robotics?

**Expected Outcome:** Understanding that topic-based communication is asynchronous and that message delivery depends on timing.
