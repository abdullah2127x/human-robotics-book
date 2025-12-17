---
sidebar_position: 2
title: "Lesson 2: Async/Await and Callbacks"
description: "Master non-blocking callbacks using async/await to handle concurrent events in your ROS 2 nodes."
---

# Lesson 2: Async/Await and Callbacks

Imagine a robot with a 100 Hz timer (fires 100 times per second) that also subscribes to sensor data. If the sensor callback takes 1 second to process, the timer can't fire—the robot becomes unresponsive. This lesson teaches you to handle multiple events concurrently using Python's async/await pattern.

## Understanding Blocking vs Non-Blocking

**Blocking execution** happens when one operation stops everything else:

```python
def sensor_callback(msg):
    time.sleep(1)  # Blocks for 1 second
    print("Processing done")

# Timer: 100 Hz (should fire 100 times/sec)
# With blocking: Only ~1 callback per second (blocked by sleep)
```

**Non-blocking execution** allows operations to yield control:

```python
async def sensor_callback(msg):
    await asyncio.sleep(1)  # Yields control, doesn't block
    print("Processing done")

# Timer: 100 Hz (fires 100 times/sec)
# With non-blocking: Timer fires every 10ms even during 1-second callback
```

The difference: `time.sleep()` freezes the entire thread. `await asyncio.sleep()` yields to the executor, allowing other callbacks to run.

## How ROS 2 Callbacks Work

A **callback** is a function that ROS 2 automatically calls when an event happens:

```python
# Event: Message arrives
# ROS 2 calls: my_callback(msg)
node.create_subscription(String, 'my_topic', my_callback)
```

You don't call the callback—ROS 2 does. You just register it and ROS 2 invokes it when the event occurs.

## Single-Threaded Blocking Behavior

Here's a node with a timer and subscriber running on the default single-threaded executor:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class BlockingNode(Node):
    def __init__(self):
        super().__init__('blocking_node')

        # Timer fires every 0.1 seconds (10 Hz)
        self.create_timer(0.1, self.timer_callback)

        # Subscriber gets messages and calls slow callback
        self.create_subscription(String, 'input', self.slow_callback)
        self.get_logger().info("Node started")

    def timer_callback(self):
        self.get_logger().info(f"Timer fired at {time.time():.2f}")

    def slow_callback(self, msg):
        self.get_logger().info(f"Got message: {msg.data}")
        time.sleep(1)  # Block for 1 second
        self.get_logger().info("Finished processing")

def main(args=None):
    rclpy.init(args=args)
    node = BlockingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**What happens:**
1. Timer fires (logs "Timer fired")
2. Sensor message arrives → slow_callback runs
3. **slow_callback blocks for 1 second** (time.sleep)
4. Timer can't fire during that 1 second (blocked waiting)
5. After 1 second, slow_callback finishes
6. Timer can fire again

**Output:**
```
Timer fired at 100.00
Got message: hello
[1 second passes, timer blocked]
Finished processing
Timer fired at 101.01
```

Notice the timer skipped ~9 firings while the callback was blocking.

## Non-Blocking Async/Await Pattern

Here's the same node using async callbacks:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import time

class AsyncNode(Node):
    def __init__(self):
        super().__init__('async_node')

        # Timer fires every 0.1 seconds (10 Hz)
        self.create_timer(0.1, self.timer_callback)

        # Subscriber calls async callback
        self.create_subscription(String, 'input', self.async_slow_callback)
        self.get_logger().info("Node started")

    def timer_callback(self):
        self.get_logger().info(f"Timer fired at {time.time():.2f}")

    async def async_slow_callback(self, msg):
        self.get_logger().info(f"Got message: {msg.data}")
        await asyncio.sleep(1)  # Yield, don't block
        self.get_logger().info("Finished processing")

def main(args=None):
    rclpy.init(args=args)
    node = AsyncNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key change:** `def` → `async def` and `time.sleep()` → `await asyncio.sleep()`

**What happens:**
1. Timer fires (logs "Timer fired")
2. Sensor message arrives → async callback starts
3. **async callback hits await asyncio.sleep(1)** → yields control
4. Timer can fire during that 1 second (not blocked)
5. After 1 second, callback resumes and finishes
6. All ~10 timer firings happen during the 1-second callback

**Output:**
```
Timer fired at 100.00
Got message: hello
Timer fired at 100.10
Timer fired at 100.20
Timer fired at 100.30
[...more firings...]
Finished processing
Timer fired at 101.00
```

Notice the timer fires ~10 times while the callback is executing (non-blocking).

## Understanding async/await Syntax

**async def:** Declares a coroutine (a function that can be paused/resumed):

```python
async def my_callback(self, msg):
    # Inside here, you can use 'await'
    pass
```

**await:** Pauses execution and yields to the event loop:

```python
await asyncio.sleep(1)  # Pause for 1 second, let others run
```

**Inside the callback, you still have access to self:**

```python
async def async_callback(self, msg):
    self.get_logger().info(f"Processing: {msg.data}")
    await asyncio.sleep(0.5)
    self.my_variable = "updated"  # Still can modify node state
```

## Callbacks Interleaving

When you have multiple async callbacks, they interleave execution:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio

class InterleavedNode(Node):
    def __init__(self):
        super().__init__('interleaved_node')

        # Timer: fires every 0.5 seconds
        self.create_timer(0.5, self.timer_callback)

        # Subscriber: async callback
        self.create_subscription(String, 'input', self.async_callback)
        self.counter = 0

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f"Timer #{self.counter}")

    async def async_callback(self, msg):
        self.get_logger().info(f"Callback started: {msg.data}")
        await asyncio.sleep(1)
        self.get_logger().info(f"Callback done: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = InterleavedNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Execution trace:**
```
Timer #1
Callback started: msg1
Timer #2  [Timer fires while callback waits]
Timer #3  [Timer fires again during callback]
Callback done: msg1
Timer #4
```

The key insight: `await asyncio.sleep()` pauses the callback, allowing the timer to fire.

## Exercise 2.2a: Create a Blocking Node

Create a file named `blocking_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class BlockingNode(Node):
    def __init__(self):
        super().__init__('blocking_node')

        # Timer: 10 Hz (fires every 0.1 seconds)
        self.create_timer(0.1, self.timer_callback)

        # Subscriber: slow callback
        self.create_subscription(String, 'slow_topic', self.slow_callback)

        self.get_logger().info("Blocking node started")

    def timer_callback(self):
        self.get_logger().info("Timer fired")

    def slow_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        time.sleep(1)  # Block for 1 second
        self.get_logger().info("Processing complete")

def main(args=None):
    rclpy.init(args=args)
    node = BlockingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Publish messages and observe: **Timer stops firing while the callback blocks.**

```bash
# Terminal 1: Run the node
ros2 run my_package blocking_node

# Terminal 2: Publish messages
ros2 topic pub /slow_topic std_msgs/String "data: 'hello'"
```

**Success criteria:**
- [ ] Node runs without error
- [ ] Timer fires at 0.1s intervals initially
- [ ] When message arrives, timer stops firing for ~1 second (blocking visible)
- [ ] After callback completes, timer resumes

---

## Exercise 2.2b: Convert to Non-Blocking Async

Modify the node to use `async def`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio

class AsyncNode(Node):
    def __init__(self):
        super().__init__('async_node')

        # Timer: 10 Hz (fires every 0.1 seconds)
        self.create_timer(0.1, self.timer_callback)

        # Subscriber: async callback
        self.create_subscription(String, 'slow_topic', self.async_callback)

        self.get_logger().info("Async node started")

    def timer_callback(self):
        self.get_logger().info("Timer fired")

    async def async_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        await asyncio.sleep(1)  # Non-blocking wait
        self.get_logger().info("Processing complete")

def main(args=None):
    rclpy.init(args=args)
    node = AsyncNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Success criteria:**
- [ ] Node runs without error
- [ ] Timer fires continuously at 0.1s intervals
- [ ] When message arrives, timer continues firing (~10 times during 1-second callback)
- [ ] Callback completes after 1 second

**Compare output:** Blocking node skips timer firings. Async node fires timer throughout callback execution.

---

## Exercise 2.2c: Multiple Concurrent Callbacks

Create a node with both a fast timer and a slow subscriber callback running concurrently:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import time

class ConcurrentNode(Node):
    def __init__(self):
        super().__init__('concurrent_node')

        # Fast timer: 5 Hz (0.2s interval)
        self.create_timer(0.2, self.fast_timer)

        # Slow subscriber: takes 1 second to process
        self.create_subscription(String, 'input', self.slow_callback)

        self.timer_count = 0

    def fast_timer(self):
        self.timer_count += 1
        self.get_logger().info(f"Timer #{self.timer_count}")

    async def slow_callback(self, msg):
        self.get_logger().info(f"Callback START: {msg.data}")
        await asyncio.sleep(1)
        self.get_logger().info(f"Callback END: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ConcurrentNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Test it:

```bash
# Terminal 1: Run node
ros2 run my_package concurrent_node

# Terminal 2: Publish multiple messages
ros2 topic pub /input std_msgs/String "data: 'msg1'" -r 2  # Publish at 2 Hz
```

**Expected output:**
```
Timer #1
Timer #2
Callback START: msg1
Timer #3  [Timer fires during callback]
Timer #4
Timer #5
Callback END: msg1
Timer #6
Callback START: msg2
```

**Success criteria:**
- [ ] Timers continue firing while callback is executing
- [ ] Callback yields control (doesn't block timer)
- [ ] Multiple callbacks can overlap without blocking each other

---

## Try With AI

**Setup:** Open your AI tool and reference the blocking vs async nodes you created.

**Prompt 1: Understand the Pattern**

Ask AI: "I have two versions of the same node—one with `def slow_callback` using `time.sleep()`, and one with `async def slow_callback` using `await asyncio.sleep()`. What's the fundamental difference in how they execute, and why would I choose async for a ROS 2 node?"

**Prompt 2: Identify Blocking Code**

Show AI code that has blocking operations and ask: "This node has a callback that does file I/O and network requests. Which parts would block a ROS 2 node, and how would you convert them to non-blocking?"

**Prompt 3: Design Concurrency**

Ask AI: "I have a robot with: (1) 50 Hz sensor reading, (2) 10 Hz decision-making (takes 0.5s), and (3) 100 Hz motor control. How would async callbacks help ensure all three execute concurrently?"

**Expected Outcomes:**

Your AI should clarify:
- Async callbacks yield control, allowing other callbacks to execute
- Blocking operations (sleep, file I/O) prevent other callbacks from running
- ROS 2 nodes need non-blocking callbacks to handle multiple concurrent events
- `await` is the key to writing responsive ROS 2 code

**Self-Reflection:**

Before moving forward, verify:
- [ ] You understand blocking callbacks prevent other callbacks from running
- [ ] You can convert `time.sleep()` to `await asyncio.sleep()`
- [ ] You understand that `async def` enables yielding control
- [ ] You can visualize how multiple callbacks interleave with async
