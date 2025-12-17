---
sidebar_position: 6
title: "Lesson 6: Building Reusable ROS 2 Python Patterns"
description: "Encapsulate patterns from Lessons 1-5 into reusable components and decision guides."
---

# Lesson 6: Building Reusable ROS 2 Python Patterns

You've learned how to create packages, write async callbacks, implement actions, define messages, and choose executors. Now you'll extract the common patterns from Lessons 1-5 and transform them into reusable templates. This is how experienced ROS 2 developers build quickly—by applying proven patterns rather than starting from scratch each time.

## Pattern Extraction: From Specific to General

**Lesson 3 specifics:**
```python
# Move arm action with specific fields
goal = MoveArm.Goal()
goal.target_x = 1.0
goal.target_y = 2.0
goal.target_z = 0.5
```

**Pattern generalization:**
```python
# Generic action client that works with ANY action type
class ActionClientTemplate:
    def __init__(self, action_type, action_name):
        self.client = ActionClient(self, action_type, action_name)

    async def send_goal(self, goal_params):
        # Works with any action type
        goal = self.action_type.Goal()
        for param, value in goal_params.items():
            setattr(goal, param, value)
        # ... rest of generic flow
```

## Reusable Action Client Template

Here's a generic action client that works with any action type:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import asyncio
from typing import Dict, Any, Type

class GenericActionClient(Node):
    """Template for action clients.

    Usage:
        client = GenericActionClient(
            action_type=MoveArm,
            action_name='move_arm'
        )
        result = await client.send_goal(
            goal_params={'target_x': 1.0, 'target_y': 2.0, 'target_z': 0.5},
            timeout_sec=10.0
        )
    """

    def __init__(self, node_name: str, action_type: Type, action_name: str):
        super().__init__(node_name)
        self.action_type = action_type
        self.action_client = ActionClient(self, action_type, action_name)
        self.feedback_callback = None

    def set_feedback_callback(self, callback):
        """Register a callback for feedback updates."""
        self.feedback_callback = callback

    async def send_goal(self, goal_params: Dict[str, Any],
                       timeout_sec: float = 10.0) -> bool:
        """Send goal and wait for result.

        Args:
            goal_params: Dict of goal field names to values
            timeout_sec: Max seconds to wait for result

        Returns:
            True if successful, False otherwise
        """
        try:
            # Wait for server
            if not self.action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"Server not ready after 5s")
                return False

            # Create goal with provided parameters
            goal = self.action_type.Goal()
            for field_name, value in goal_params.items():
                if not hasattr(goal, field_name):
                    self.get_logger().error(f"Unknown goal field: {field_name}")
                    return False
                setattr(goal, field_name, value)

            self.get_logger().info(f"Sending goal with params: {goal_params}")

            # Send goal
            goal_handle = await self.action_client.send_goal_async(
                goal,
                feedback_callback=self.feedback_callback
            )

            # Wait for result with timeout
            result = await asyncio.wait_for(
                goal_handle.get_result_async(),
                timeout=timeout_sec
            )

            self.get_logger().info("Goal completed successfully")
            return True

        except asyncio.TimeoutError:
            self.get_logger().error(f"Goal timed out after {timeout_sec}s")
            return False
        except Exception as e:
            self.get_logger().error(f"Error sending goal: {e}")
            return False
```

**Usage with MoveArm action:**

```python
async def main():
    rclpy.init()
    client = GenericActionClient('move_client', MoveArm, 'move_arm')

    # Set feedback callback
    client.set_feedback_callback(
        lambda msg: print(f"Feedback: {msg.feedback.progress_percent}%")
    )

    # Send different goals using same template
    await client.send_goal({
        'target_x': 1.0,
        'target_y': 2.0,
        'target_z': 0.5
    })

    rclpy.shutdown()
```

## Reusable Async Node Base Class

Lesson 2 showed async callbacks. Here's a base class that provides the async infrastructure:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import asyncio
from typing import Callable, Dict

class AsyncNodeBase(Node):
    """Base class for nodes with async callbacks.

    Provides:
    - Automatic async callback management
    - MultiThreadedExecutor setup
    - Common initialization pattern

    Usage:
        class MyAsyncNode(AsyncNodeBase):
            def __init__(self):
                super().__init__('my_node')
                self.register_async_timer(0.1, self.on_timer)
                self.register_async_subscriber(
                    String, 'my_topic', self.on_message
                )

            async def on_timer(self):
                self.get_logger().info("Timer fired")

            async def on_message(self, msg):
                self.get_logger().info(f"Got: {msg.data}")
    """

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.async_callbacks = []

    def register_async_timer(self, period: float, callback: Callable):
        """Register an async callback for a timer.

        Args:
            period: Timer period in seconds
            callback: Async function to call
        """
        def sync_wrapper():
            # Schedule async callback
            task = asyncio.ensure_future(callback())

        self.create_timer(period, sync_wrapper)

    def register_async_subscriber(self, msg_type, topic: str,
                                 callback: Callable):
        """Register an async callback for a subscriber.

        Args:
            msg_type: ROS 2 message type
            topic: Topic name
            callback: Async function(msg)
        """
        def sync_wrapper(msg):
            asyncio.ensure_future(callback(msg))

        self.create_subscription(msg_type, topic, sync_wrapper, 10)

def run_async_node(node_class, num_threads: int = 2):
    """Run an async node with MultiThreadedExecutor.

    Args:
        node_class: Class to instantiate
        num_threads: Number of executor threads
    """
    rclpy.init()
    node = node_class()
    executor = MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
```

**Usage:**

```python
class RobotControlNode(AsyncNodeBase):
    def __init__(self):
        super().__init__('robot_controller')
        self.register_async_timer(0.1, self.read_sensors)
        self.register_async_subscriber(
            String, 'commands', self.execute_command
        )

    async def read_sensors(self):
        # Non-blocking sensor reading
        await asyncio.sleep(0.01)
        self.get_logger().info("Sensors read")

    async def execute_command(self, msg):
        # Non-blocking command execution
        self.get_logger().info(f"Executing: {msg.data}")
        await asyncio.sleep(0.5)

if __name__ == '__main__':
    run_async_node(RobotControlNode)
```

## Communication Pattern Decision Guide

You've learned three communication patterns: Messages (pub/sub), Services, and Actions. Here's a guide to choose the right pattern:

```
Decision Tree: Which ROS 2 Communication Pattern?

1. Is data sent continuously (streaming)?
   YES → Use Topic + Message (pub/sub)
         Examples:
         - Sensor data (100x/second)
         - Camera frames (30x/second)
         - Robot state updates (10x/second)

   NO → Continue to step 2

2. Is the operation fast (< 1 second)?
   YES → Use Service (request/response)
         Examples:
         - Query battery level
         - Reset calibration
         - Enable/disable feature

   NO → Continue to step 3

3. Does the operation have feedback?
   YES → Use Action (goal/feedback/result)
         Examples:
         - Arm movement (5-10 seconds with progress)
         - Navigation to waypoint (needs progress)
         - Complex manipulation (multi-step with feedback)

   NO → Use Service (request/response)
```

## Real-World Pattern Applications

**Pattern 1: Sensor Stream + Decision + Command**

```
Sensor Data (Topic) → ROS 2 node → Decision → Command (Action/Topic)

Example:
Temperature sensor (Topic) → Node reads, decides if cooling needed → Send motor command
```

**Pattern 2: Responding to Events**

```
Request (Service) → Processing → Response

Example:
/set_mode service: Client requests "autonomous" mode → Node sets up → Returns success
```

**Pattern 3: Long-Running Tasks**

```
Goal (Action) → Progress (Feedback) → Result

Example:
/pickup_object action: Client sends object ID → Server picks up (feedback: grasping, lifting) → Returns success
```

## Exercise 6.1: Create Generic Action Client

Create a file `generic_action_client.py` with the `GenericActionClient` class above.

Test with a simple action:

```python
# Test with MoveArm action
from my_package.action import MoveArm

async def test():
    rclpy.init()
    client = GenericActionClient('test_client', MoveArm, 'move_arm')

    success = await client.send_goal({
        'target_x': 1.0,
        'target_y': 2.0,
        'target_z': 0.5
    })

    print(f"Result: {success}")
    rclpy.shutdown()

if __name__ == '__main__':
    import asyncio
    asyncio.run(test())
```

**Success criteria:**
- [ ] Generic client template works with MoveArm action
- [ ] Goal parameters can be passed as dict
- [ ] Timeout handling works
- [ ] Error handling is informative

---

## Exercise 6.2: Apply Template to Multiple Actions

Create a second action type (e.g., `OpenGripper`):

```
# OpenGripper.action
float32 force_percent

---

int32 progress_percent

---

bool success
string message
```

Use the same `GenericActionClient` with both actions:

```python
# Works with MoveArm
result_move = await client.send_goal({'target_x': 1.0, ...})

# Works with OpenGripper (same template)
result_grip = await client.send_goal({'force_percent': 50.0})
```

**Success criteria:**
- [ ] Same template works for different action types
- [ ] No code modification needed between action types
- [ ] Generic client truly generalizes the pattern

---

## Exercise 6.3: Create Async Node Base Class

Create `async_node_base.py` with the `AsyncNodeBase` class above.

Implement a concrete node:

```python
class SensorReaderNode(AsyncNodeBase):
    def __init__(self):
        super().__init__('sensor_reader')
        self.register_async_timer(0.5, self.read_sensors)

    async def read_sensors(self):
        self.get_logger().info("Reading sensors...")
        await asyncio.sleep(1)  # Simulate I/O
        self.get_logger().info("Done")

if __name__ == '__main__':
    run_async_node(SensorReaderNode)
```

**Success criteria:**
- [ ] Base class simplifies async callback setup
- [ ] Subclass only needs to define async methods
- [ ] MultiThreadedExecutor is automatically configured
- [ ] Less boilerplate than manual async setup

---

## Exercise 6.4: Create Communication Pattern Guide

Create a markdown file `communication_patterns.md` with decision tree and examples:

For each pattern, document:
1. **When to use** (fast/slow, streaming/request, feedback needs)
2. **Pros** (responsiveness, simplicity, etc.)
3. **Cons** (overhead, complexity, etc.)
4. **Examples** (3-5 real robot scenarios)

Example entry:

```markdown
## Service (Request/Response)

**When to use:** Fast operations (< 1 second) without feedback

**Pros:**
- Synchronous: Client waits for response
- Simple: Request/response only
- Good for configuration changes

**Cons:**
- Client blocks while waiting
- No feedback during execution

**Examples:**
- Query robot battery level
- Reset calibration
- Set operation mode
- Check if movement is complete
```

Test by applying it to different scenarios:

- "I need to stream motor temperatures 100 times/second" → Topic
- "I need to ask the robot if it's ready" → Service
- "I need to move the arm with progress feedback" → Action

**Success criteria:**
- [ ] Decision guide is clear and usable
- [ ] Examples cover realistic robot scenarios
- [ ] Someone unfamiliar with ROS 2 could follow it

---

## Try With AI

**Setup:** Reference the generic action client and async node base class.

**Prompt 1: Template Design**

Ask AI: "I've created a generic action client that works with any action type by accepting goal parameters as a dict. What are the tradeoffs of this approach vs. creating specialized clients for each action type? When would you use generics vs. specifics?"

**Prompt 2: Base Class Extraction**

Show AI your async node base class and ask: "I want this base class to be used by many different ROS 2 nodes. What methods or features would make it more flexible? What edge cases should it handle?"

**Prompt 3: Pattern Reusability**

Ask AI: "I have three different robot projects (manipulator, mobile robot, humanoid). What communication patterns do they all share? How would I design reusable templates that work across all three?"

**Expected Outcomes:**

Your AI should help clarify:
- Templates reduce code duplication across similar workflows
- Generics trade specificity for reusability
- Well-designed base classes enable rapid development
- Patterns compound across projects (growing efficiency)

**Self-Reflection:**

Before moving to the capstone, verify:
- [ ] You understand pattern extraction (specific → general)
- [ ] You can design a generic template for common workflows
- [ ] You recognize when to create base classes vs. utilities
- [ ] You can decide what goes in a template vs. concrete code
