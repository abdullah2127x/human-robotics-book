---
sidebar_position: 6
title: "Lesson 6: Reusable Node Patterns"
description: "Design parameterized publisher and subscriber templates for code reuse"
---

# Lesson 6: Reusable Node Patterns

**Learning Outcome:** Extract common patterns from previous lessons into reusable templates that work with different topics and callbacks.

**Proficiency Level:** B1

**Estimated Time:** 50 minutes

**New Concepts:** 0 (synthesis of previous lessons)

---

## Why Reusable Patterns Matter

In Lessons 1-5, you created many similar publishers and subscribers:
- temperature_publisher
- battery_publisher
- fast_publisher
- And their corresponding subscribers

Notice the repetition: They all follow the same structure. We can extract this into reusable templates.

**Before (repetitive):**
```
temperature_publisher.py (200 lines)
battery_publisher.py (200 lines)
fast_publisher.py (200 lines)
...
```

**After (template-based):**
```
generic_publisher.py (50 lines, reusable)
temperature_publisher_v2.py (10 lines, instantiate template)
battery_publisher_v2.py (10 lines, instantiate template)
...
```

---

## Working Example: Generic Publisher Template

**Generic Publisher Template** (`generic_publisher.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GenericPublisher(Node):
    """Reusable publisher template.

    Parameters:
    - topic_name: Topic to publish to
    - publish_frequency: How often to call message generator (Hz)
    - message_generator: Function that returns message content (str)
    """

    def __init__(self, topic_name, publish_frequency, message_generator):
        # Generate unique node name from topic
        node_name = f'{topic_name}_publisher'
        super().__init__(node_name)

        # Store parameters
        self.message_generator = message_generator

        # Create publisher
        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.get_logger().info(f'Publisher created on topic "{topic_name}"')

        # Create timer at specified frequency
        period = 1.0 / publish_frequency
        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = self.message_generator()  # Call generator function
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def run_publisher(topic_name, frequency, message_fn):
    """Helper function to run a publisher."""
    rclpy.init()
    node = GenericPublisher(topic_name, frequency, message_fn)
    rclpy.spin(node)
    rclpy.shutdown()
```

Now, to create **Temperature Publisher V2** using the template:

```python
# temperature_publisher_v2.py
import rclpy
from generic_publisher import GenericPublisher


class TemperaturePublisher(GenericPublisher):
    def __init__(self):
        self.temp = 20.0
        super().__init__(
            topic_name='temperature',
            publish_frequency=1.0,
            message_generator=self.generate_temperature)

    def generate_temperature(self):
        msg = f'temperature: {self.temp:.1f}°C'
        self.temp += 0.5
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Compare the two:
- **temperature_publisher.py (before):** 30+ lines
- **temperature_publisher_v2.py (after):** 20 lines, reuses generic_publisher.py

To add a **Battery Publisher**, just instantiate the template again:

```python
# battery_publisher_v2.py
import rclpy
from generic_publisher import GenericPublisher


class BatteryPublisher(GenericPublisher):
    def __init__(self):
        self.battery = 100
        super().__init__(
            topic_name='battery',
            publish_frequency=2.0,  # Different frequency!
            message_generator=self.generate_battery)

    def generate_battery(self):
        msg = f'battery: {self.battery}%'
        self.battery -= 1
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Same template, different configuration!

---

## Generic Subscriber Template

**Generic Subscriber Template** (`generic_subscriber.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GenericSubscriber(Node):
    """Reusable subscriber template.

    Parameters:
    - topic_name: Topic to subscribe to
    - callback: Function to call when message arrives
    """

    def __init__(self, topic_name, callback):
        # Generate unique node name from topic
        node_name = f'{topic_name}_subscriber'
        super().__init__(node_name)

        # Store callback
        self.message_callback = callback

        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10)
        self.get_logger().info(f'Subscribed to topic "{topic_name}"')

    def listener_callback(self, msg):
        self.message_callback(msg.data)  # Call user callback


def run_subscriber(topic_name, callback_fn):
    """Helper function to run a subscriber."""
    rclpy.init()
    node = GenericSubscriber(topic_name, callback_fn)
    rclpy.spin(node)
    rclpy.shutdown()
```

**Temperature Subscriber V2** using the template:

```python
# temperature_subscriber_v2.py
import rclpy
from generic_subscriber import GenericSubscriber


class TemperatureSubscriber(GenericSubscriber):
    def __init__(self):
        super().__init__(
            topic_name='temperature',
            callback=self.process_temperature)

    def process_temperature(self, data):
        self.get_logger().info(f'Temp reading: {data}')


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Pattern: Choosing Pub/Sub vs Service

Now that you've implemented both patterns, let's formalize when to use each:

**Use Pub/Sub (Topics) when:**
- Data flows **one direction** (sensor → processor)
- **Multiple consumers** may be interested (many subscribers)
- **Asynchronous** is okay (don't need immediate response)
- **Continuous** or **frequent** updates (stream of data)

Examples:
- Sensor data (temperature, position, velocity)
- Real-time video feed
- System status broadcasts

**Use Service (Request-Response) when:**
- **Query pattern** (ask and wait for answer)
- **Synchronous** response required (must wait)
- **Single responder** (one server)
- **Infrequent** calls (not a stream)

Examples:
- "What's the current battery level?"
- "Move arm to pose X"
- "Get system configuration"

### Decision Table

| Scenario | Pattern | Reason |
|----------|---------|--------|
| Robot streams video frames | Topic | Continuous, many consumers, async ok |
| Client queries robot state | Service | Query pattern, sync, single responder |
| Motor publishes current load | Topic | Continuous stream, multiple processors |
| Navigation system requests path | Service | Query, sync needed |
| Robot publishes warnings | Topic | Broadcast to all listeners |

---

## Hands-On Practice

### Exercise 6.1: Create Publisher Template

Design a reusable publisher template based on Lessons 1-5. Requirements:
- Accept topic name as parameter
- Accept publish frequency as parameter
- Accept message generator function as parameter
- Work with any message type

Test by creating publishers for:
1. Temperature (1 Hz)
2. Battery (2 Hz)
3. Position (5 Hz)

All using the same template code.

**Success Criteria:**
- [ ] Template created and documented
- [ ] Works for multiple topics
- [ ] All publishers run without errors

### Exercise 6.2: Create Subscriber Template

Design a reusable subscriber template. Requirements:
- Accept topic name as parameter
- Accept callback function as parameter
- Work with any message type

Test by creating subscribers for the three publishers from Exercise 6.1.

**Success Criteria:**
- [ ] Template created and documented
- [ ] Works for multiple topics
- [ ] All subscribers receive messages correctly

### Exercise 6.3: Communication Pattern Guide

Create a decision guide documenting:
- When to use pub/sub (with 3 examples)
- When to use services (with 3 examples)
- How to tell which one to use

Format as a table with:
- Scenario description
- Pattern choice
- Justification

**Success Criteria:**
- [ ] At least 6 scenarios covered
- [ ] Clear reasoning for each
- [ ] Someone else could use this guide to choose patterns

---

## Benefits of Reusable Patterns

1. **Less Code:** Write once, use many times
2. **Consistency:** All publishers work the same way
3. **Maintainability:** Fix a bug in template, all instances fixed
4. **Extensibility:** Add new publishers without rewriting core
5. **Learning:** Understanding one pattern applies to all instances

---

## Try With AI

Use AI to explore pattern design:

**Step 1: Ask about template design**
Prompt your AI:
```
Design a reusable ROS 2 publisher template that works with:
- Different topics
- Different message types (String, Int32, Float64)
- Different publish frequencies

What would the interface look like? How would you pass
the message type and message generator function?
```

**Step 2: Review and discuss**
Ask yourself:
- What parameters must the template accept?
- What can stay fixed in the template?
- How does the template balance flexibility and simplicity?

**Step 3: Compare pub/sub and service patterns**
Prompt your AI:
```
I have 10 robot operations:
1. Sensor reading (temperature)
2. Motor speed query
3. Position update
4. Emergency stop
5. Configuration request
6. Velocity stream
7. Battery query
8. Lidar point cloud
9. Path request
10. State broadcast

For each, should I use Pub/Sub or Service? Why?
```

**Step 4: Design a system**
Create a document showing:
- A robot system with 10+ topics/services
- Which use pub/sub pattern
- Which use service pattern
- Reasoning for each choice

**Expected Outcome:** Understanding that good patterns reduce code, improve consistency, and make systems easier to extend.
