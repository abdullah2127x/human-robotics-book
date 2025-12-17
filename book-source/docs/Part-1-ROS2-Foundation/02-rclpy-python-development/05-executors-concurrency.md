---
sidebar_position: 5
title: "Lesson 5: Executors and Concurrency"
description: "Choose between single-threaded and multi-threaded executors based on your node's callback performance requirements."
---

# Lesson 5: Executors and Concurrency

You have a ROS 2 node with multiple callbacks: a fast sensor reader (50 Hz), a slow decision-maker (takes 0.5 seconds), and a motor controller (100 Hz). If any of these blocks the others, your robot becomes unresponsive. The solution is understanding ROS 2 **executors**—components that decide how callbacks run relative to each other.

## Single-Threaded vs Multi-Threaded Execution

**SingleThreadedExecutor** (default):
```python
executor = SingleThreadedExecutor()
executor.add_node(node)
executor.spin()
```

Processes callbacks one at a time:
```
Time  0ms: Sensor callback runs
Time  10ms: (waiting for sensor callback to finish)
Time  50ms: Sensor callback finishes
Time  50ms: Decision callback starts
Time 550ms: Decision callback finishes
Time 550ms: Motor callback runs
```

One slow callback blocks everything else.

**MultiThreadedExecutor**:
```python
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
```

Processes callbacks in parallel:
```
Time  0ms: Sensor callback (thread 1)
Time  0ms: Motor callback (thread 2) [parallel, not blocked]
Time 10ms: Decision callback (thread 3)
Time 20ms: Sensor callback again (thread 4)
Time 500ms: Motor continues...
Time 550ms: Decision callback finishes
```

Multiple callbacks execute simultaneously.

## Understanding Blocking Behavior

Single-threaded executor with a slow callback:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class BlockingNode(Node):
    def __init__(self):
        super().__init__('blocking_node')

        # Fast timer: 10 Hz (every 100ms)
        self.create_timer(0.1, self.fast_timer)

        # Slow subscriber: takes 5 seconds
        self.create_subscription(String, 'slow_topic', self.slow_callback)

        self.timer_count = 0

    def fast_timer(self):
        self.timer_count += 1
        self.get_logger().info(f"Timer #{self.timer_count}")

    def slow_callback(self, msg):
        self.get_logger().info("Slow callback started")
        time.sleep(5)  # Blocks for 5 seconds
        self.get_logger().info("Slow callback done")

def main(args=None):
    rclpy.init(args=args)
    node = BlockingNode()

    # Default is SingleThreadedExecutor
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

With `SingleThreadedExecutor`:
```
Timer #1
Timer #2
[Message arrives]
Slow callback started
[5 second wait - timer blocked]
Timer #3  [Only fires once during 5 second callback]
Slow callback done
Timer #4
```

The timer fires ~10 times during 5 seconds with non-blocking code, but only once if blocked.

## Switching to MultiThreadedExecutor

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import time

class ResponsiveNode(Node):
    def __init__(self):
        super().__init__('responsive_node')

        # Fast timer: 10 Hz
        self.create_timer(0.1, self.fast_timer)

        # Slow subscriber
        self.create_subscription(String, 'slow_topic', self.slow_callback)

        self.timer_count = 0

    def fast_timer(self):
        self.timer_count += 1
        self.get_logger().info(f"Timer #{self.timer_count}")

    def slow_callback(self, msg):
        self.get_logger().info("Slow callback started")
        time.sleep(5)
        self.get_logger().info("Slow callback done")

def main(args=None):
    rclpy.init(args=args)
    node = ResponsiveNode()

    # Use MultiThreadedExecutor instead
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

With `MultiThreadedExecutor`:
```
Timer #1
Timer #2
[Message arrives]
Slow callback started (thread 1)
Timer #3               (thread 2 - parallel!)
Timer #4               (timer continues firing)
Timer #5
...
Timer #52
Slow callback done
```

The timer fires ~50 times during the 5-second callback (not blocked).

## The Tradeoff: Race Conditions

Parallel callbacks sound great, but they introduce **race conditions** when multiple threads modify shared state:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32

class UnsafeCounter(Node):
    def __init__(self):
        super().__init__('unsafe_counter')

        self.counter = 0  # Shared state

        # Two subscribers, both increment counter
        self.create_subscription(Int32, 'increment_a', self.callback_a)
        self.create_subscription(Int32, 'increment_b', self.callback_b)

        self.create_timer(1.0, self.print_counter)

    def callback_a(self, msg):
        # Thread 1: Read counter, modify, write back
        current = self.counter
        # Context switch: Thread 2 runs
        self.counter = current + 1

    def callback_b(self, msg):
        # Thread 2: Read counter, modify, write back
        current = self.counter
        # Context switch: Thread 1 resumes
        self.counter = current + 1

    def print_counter(self):
        self.get_logger().info(f"Counter: {self.counter}")

def main(args=None):
    rclpy.init(args=args)
    node = UnsafeCounter()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

If you send 100 messages (50 to each topic), the counter should be 100. But with parallel threads:

```
Expected counter: 100
Actual counter: 63  [Lost updates due to race condition]
```

What happened:
```
Thread 1: Read counter=0
Thread 2: Read counter=0  [Both read 0!]
Thread 1: Write counter=1
Thread 2: Write counter=1  [Overwrites Thread 1's write]
Result: Counter is 1, should be 2
```

## Protecting Shared State with Locks

Use `threading.Lock` to protect critical sections:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32
import threading

class SafeCounter(Node):
    def __init__(self):
        super().__init__('safe_counter')

        self.counter = 0
        self.counter_lock = threading.Lock()  # Protect counter

        self.create_subscription(Int32, 'increment_a', self.callback_a)
        self.create_subscription(Int32, 'increment_b', self.callback_b)

        self.create_timer(1.0, self.print_counter)

    def callback_a(self, msg):
        with self.counter_lock:  # Acquire lock
            current = self.counter
            self.counter = current + 1
        # Release lock automatically

    def callback_b(self, msg):
        with self.counter_lock:  # Acquire lock
            current = self.counter
            self.counter = current + 1
        # Release lock

    def print_counter(self):
        with self.counter_lock:
            count = self.counter
        self.get_logger().info(f"Counter: {count}")

def main(args=None):
    rclpy.init(args=args)
    node = SafeCounter()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**How locks work:**
1. `with self.counter_lock:` acquires the lock
2. Only one thread can hold the lock at a time
3. Other threads wait until the lock is released
4. Critical section (reading and writing counter) is atomic
5. Exiting the `with` block releases the lock

Now the counter is always correct, even with parallel threads.

## Choosing Between Executors

**Use SingleThreadedExecutor when:**
- Callbacks don't block (using async/await)
- You have few callbacks
- Simple, predictable execution order desired
- Thread safety is not a concern (no locks needed)

**Use MultiThreadedExecutor when:**
- Some callbacks are slow (blocking operations)
- You need responsive node behavior
- Multiple callbacks must run concurrently
- You're willing to handle thread safety with locks

## Executor Selection Strategy

For a robot with:
- Sensor reader (50 Hz, fast)
- Decision-maker (1 Hz, takes 0.5s)
- Motor controller (100 Hz, fast)

**Scenario 1:** Using SingleThreadedExecutor
- Decision maker blocks sensor and motor callbacks for 0.5 seconds
- Motor commands are delayed
- Robot becomes jerky

**Scenario 2:** Using MultiThreadedExecutor
- Sensor, decision, and motor run in parallel
- Motor gets responsive commands
- Need locks if shared state exists

For responsive robot control, MultiThreadedExecutor is better.

## Thread Safety Best Practices

**1. Minimize critical sections**

```python
# Bad: Long lock holding
with self.state_lock:
    data = self.state
    self.process_slow(data)  # Don't do slow work inside lock!
    self.state = updated

# Good: Lock only where needed
with self.state_lock:
    data = self.state
    updated = self.process_slow(data)  # Do this outside
with self.state_lock:
    self.state = updated
```

**2. Avoid nested locks**

```python
# Bad: Nested locks can deadlock
with self.lock_a:
    with self.lock_b:  # What if another callback has lock_b, lock_a?
        pass
```

**3. Use ROS 2's thread-safe containers when available**

Some data structures provide built-in thread safety. Check ROS 2 documentation.

## Exercise 5.1: Observe Blocking Behavior

Create `blocking_demo.py` with the BlockingNode class above.

```bash
ros2 run my_package blocking_demo
```

In another terminal, publish messages:

```bash
ros2 topic pub /slow_topic std_msgs/String "data: 'test'" -r 0.2  # Once per 5 seconds
```

**Expected output:**
```
Timer #1
Timer #2
Timer #3
[Message arrives]
Slow callback started
[5 second pause - timer blocked]
Timer #4
Slow callback done
Timer #5
```

**Success criteria:**
- [ ] Timer fires regularly initially
- [ ] When slow callback starts, timer stops firing
- [ ] Timer resumes after callback finishes
- [ ] Blocking behavior is visible

---

## Exercise 5.2: Switch to MultiThreadedExecutor

Modify `blocking_demo.py` to use `MultiThreadedExecutor(num_threads=2)`:

```bash
ros2 run my_package blocking_demo
ros2 topic pub /slow_topic std_msgs/String "data: 'test'" -r 0.2
```

**Expected output:**
```
Timer #1
Timer #2
[Message arrives]
Slow callback started
Timer #3  [Timer continues!]
Timer #4
Timer #5
...
Slow callback done
```

**Success criteria:**
- [ ] Timer continues firing during slow callback
- [ ] Parallel execution visible in output
- [ ] Node is more responsive

---

## Exercise 5.3: Demonstrate Race Condition

Create `unsafe_counter.py` with the UnsafeCounter class above.

```bash
# Terminal 1: Run node
ros2 run my_package unsafe_counter

# Terminal 2: Send 100 increments rapidly
for i in {1..50}; do ros2 topic pub /increment_a std_msgs/Int32 "data: 1" -r 100; done &
for i in {1..50}; do ros2 topic pub /increment_b std_msgs/Int32 "data: 1" -r 100; done
```

**Expected output:**
```
Counter: 47  [Should be 100, but race condition lost increments]
Counter: 63
Counter: 78
```

**Success criteria:**
- [ ] Counter is less than 100 (race condition visible)
- [ ] Different runs show different final values
- [ ] You understand why: parallel access to shared state

---

## Exercise 5.4: Fix with Locks

Create `safe_counter.py` with the SafeCounter class above (adds `threading.Lock`).

```bash
# Terminal 1: Run node
ros2 run my_package safe_counter

# Terminal 2: Send 100 increments
for i in {1..50}; do ros2 topic pub /increment_a std_msgs/Int32 "data: 1"; done &
for i in {1..50}; do ros2 topic pub /increment_b std_msgs/Int32 "data: 1"; done
```

**Expected output:**
```
Counter: 100  [Correct! Lock prevented race condition]
```

**Success criteria:**
- [ ] Counter reaches exactly 100
- [ ] Multiple runs show consistent results
- [ ] Lock protected shared state effectively

---

## Exercise 5.5: Executor Choice Decision

Describe a robot arm scenario:
- **Vision system** (30 Hz, takes 100ms to process each frame)
- **Joint controller** (100 Hz feedback)
- **Gripper command** (10 Hz, occasional commands)

Questions to answer:

1. **With SingleThreadedExecutor:**
   - [ ] Can the joint controller respond to 100 Hz commands when vision blocks?
   - [ ] What's the consequence for robot control?

2. **With MultiThreadedExecutor:**
   - [ ] Can all components run in parallel?
   - [ ] What shared state might need locks?
   - [ ] Would performance improve?

Ask your AI: "Given these three callbacks, which executor would you choose and why? What thread safety concerns exist?"

**Success criteria:**
- [ ] You justify executor choice with concrete reasoning
- [ ] You identify which data needs protection
- [ ] You understand tradeoffs between responsiveness and complexity

---

## Try With AI

**Setup:** Reference your blocking and async node implementations.

**Prompt 1: Executor Selection**

Ask AI: "I have a ROS 2 node with: sensor callback (1ms), slow analysis callback (2 seconds), and motor command callback (50Hz). The motor commands need to be responsive even during analysis. Should I use SingleThreadedExecutor or MultiThreadedExecutor? What's the tradeoff?"

**Prompt 2: Race Condition Identification**

Show AI a node with multiple callbacks that modify a shared robot state variable. Ask: "This node publishes robot state that's modified by three different callbacks running in parallel. What race conditions could occur? How would you protect the state?"

**Prompt 3: Lock Design**

Ask AI: "I have a complex node with multiple shared variables: robot position, velocity, gripper state. Should I use one lock for all state, or separate locks per variable? What are the tradeoffs?"

**Expected Outcomes:**

Your AI should help clarify:
- SingleThreadedExecutor is simple but slow callbacks block everything
- MultiThreadedExecutor enables responsive parallel execution
- Parallel execution requires thread-safe protection of shared state
- Locks prevent race conditions but must be used carefully

**Self-Reflection:**

Before moving to Lesson 6, verify:
- [ ] You understand SingleThreadedExecutor processes callbacks sequentially
- [ ] You understand MultiThreadedExecutor processes callbacks in parallel
- [ ] You can identify when each executor is appropriate
- [ ] You understand race conditions and can use locks to prevent them
- [ ] You recognize that more concurrency requires more thread-safety care
