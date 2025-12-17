---
sidebar_position: 1
title: "Lesson 1: Introduction to ROS 2 Nodes"
description: "Create and run your first ROS 2 node that prints messages independently"
---

# Lesson 1: Introduction to ROS 2 Nodes

**Learning Outcome:** Create and run a ROS 2 node that performs computation and prints messages at regular intervals.

**Proficiency Level:** B1 (Intermediate Foundation)

**Estimated Time:** 45 minutes

**New Concepts:** 3
- Node: independent process in ROS 2
- Node initialization and lifecycle (init, spin, shutdown)
- Timer callbacks for periodic execution

---

## Working Example: Hello Robot Node

Before we explain what a node is, let's see one in action. Type this code into a file called `minimal_node.py`:

```python
import rclpy
from rclpy.node import Node


class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node started')
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Hello Robot')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**To run this node:**

```bash
# Terminal 1
python3 minimal_node.py
```

**Expected Output:**
```
[INFO] [minimal_node]: Node started
[INFO] [minimal_node]: Hello Robot
[INFO] [minimal_node]: Hello Robot
[INFO] [minimal_node]: Hello Robot
...
```

The node prints "Hello Robot" every 1 second. Press Ctrl+C to stop it.

---

## Understanding the Node

A **node** is an independent ROS 2 process that performs computation. Just as a human brain has neurons that specialize in different tasks, a robot has nodes that specialize:

- One node reads sensor data
- Another node processes that data
- A third node sends commands to actuators
- They all work together through ROS 2 communication

### Analyzing the Code

Let's walk through each part of the minimal node:

**Part 1: Initialization**
```python
import rclpy
from rclpy.node import Node
```
These imports give us access to ROS 2 (rclpy = ROS Client Library for Python).

**Part 2: Node Class**
```python
class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
```
We create a node by subclassing the ROS 2 Node class. The name 'minimal_node' identifies this node in the ROS 2 system.

**Part 3: Logger**
```python
self.get_logger().info('Node started')
```
Instead of using `print()`, ROS 2 nodes use `get_logger()` for structured logging. This makes logs easier to filter and organize in large robot systems.

**Part 4: Timer**
```python
self.timer = self.create_timer(1.0, self.timer_callback)
```
This creates a timer that calls `timer_callback()` every 1.0 seconds. The first argument is the period in seconds.

**Part 5: Timer Callback**
```python
def timer_callback(self):
    self.get_logger().info('Hello Robot')
```
This function runs automatically every 1 second (because the timer triggered it). It logs a message.

**Part 6: Main Function and Spin**
```python
def main(args=None):
    rclpy.init(args=args)  # Start ROS 2
    node = MinimalNode()    # Create our node
    rclpy.spin(node)        # Keep node alive, process callbacks
    rclpy.shutdown()        # Cleanup when Ctrl+C is pressed
```

The `spin()` function is crucial—it keeps the node alive and processes callbacks. Without it, the node would exit immediately after creation.

### Node Lifecycle

```
rclpy.init()
    ↓
Node created
    ↓
rclpy.spin() ← node stays alive here, callbacks execute
    ↓
Ctrl+C pressed
    ↓
rclpy.shutdown()
    ↓
Node exits
```

---

## Hands-On Practice

### Exercise 1.1: Create Your Own Node

Create a new file called `hello_name_node.py`. Write a node that:
- Creates a node with name 'hello_name_node'
- Logs "Hello [your name]" every 2 seconds
- (Hint: use 2.0 as the timer period instead of 1.0)

**Success Criteria:**
- [ ] File runs without errors
- [ ] Message appears in terminal every 2 seconds
- [ ] You can stop it with Ctrl+C

<details>
<summary>Solution</summary>

```python
import rclpy
from rclpy.node import Node


class HelloNameNode(Node):
    def __init__(self):
        super().__init__('hello_name_node')
        self.get_logger().info('Hello Name Node started')
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Hello [your name]')


def main(args=None):
    rclpy.init(args=args)
    node = HelloNameNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</details>

### Exercise 1.2: Modify Node Behavior

Take the original `minimal_node.py` and modify it to:
- Change "Hello Robot" to "ROS 2 is awesome"
- Change timer period from 1.0 to 0.5 (messages appear twice per second)

Run it and observe the difference.

**Success Criteria:**
- [ ] Modified code runs without errors
- [ ] New message appears in logs
- [ ] Messages appear faster (every 0.5 seconds)

---

## Try With AI

Use an AI tool to help you understand how to modify the node behavior:

**Step 1: Ask AI a clarifying question**
Prompt your AI:
```
I have a ROS 2 node that prints "Hello Robot" every 1 second.
How would I modify it to print "Robot Status: Active" instead,
and have it execute every 0.25 seconds?
```

**Step 2: Review the AI's response**
Ask yourself:
- Does the suggested approach match what you learned about timers?
- Which parts of the code need to change?
- Which parts stay the same?

**Step 3: Apply the modification**
Implement the suggested changes yourself and test the node.

**Step 4: Compare with original**
- Does the timer period affect how frequently the message appears?
- Does changing the logger message break anything?

**Expected Outcome:** A working node that prints "Robot Status: Active" four times per second.
