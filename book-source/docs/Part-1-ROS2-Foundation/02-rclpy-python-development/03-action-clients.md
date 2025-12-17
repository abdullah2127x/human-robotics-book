---
sidebar_position: 3
title: "Lesson 3: Action Clients for Long-Running Tasks"
description: "Master ROS 2 actions for long-running tasks with feedback, moving beyond services to handle complex goal-driven workflows."
---

# Lesson 3: Action Clients for Long-Running Tasks

Imagine commanding a robot arm to move to a position. The movement takes 10 seconds. A ROS 2 **service** would block for 10 seconds waiting for a response—the entire node freezes. A ROS 2 **action** handles this differently: the client sends a goal, receives periodic feedback (progress updates), and eventually gets a result when complete. No blocking.

## Services vs Actions: When to Use Each

**Services** work for fast request-response:

```python
# Fast operation (< 1 second)
result = service_client.call(request)
# Waits for result, then continues
```

**Actions** work for long-running tasks with feedback:

```python
# Long operation (5+ seconds, needs progress feedback)
goal_handle = action_client.send_goal(goal)
# Client doesn't wait; continues immediately

# Receives feedback callbacks
# Eventually receives final result
```

## Anatomy of an Action

An action has three components:

**Goal:** What the server should do
```
Move arm to position: x=1.0, y=2.0, z=0.5
```

**Feedback:** Progress during execution
```
Progress: 25% complete
Progress: 50% complete
Progress: 75% complete
```

**Result:** What happened when done
```
Success: Reached target position
Or: Failure: Collision detected
```

## Defining an Action Type

Action definitions go in a `.action` file. Here's a simple action for moving a robotic arm:

```
# MoveArm.action

# GOAL: What does the client ask the server to do?
float32 target_x
float32 target_y
float32 target_z

---

# FEEDBACK: What does the server report periodically?
int32 progress_percent
string status

---

# RESULT: What does the server report at completion?
bool success
string message
```

The three dashes (`---`) separate the three sections.

## Creating Action Files in Your Package

Place the `.action` file in a `action/` subdirectory:

```
my_package/
├── action/
│   └── MoveArm.action
├── my_package/
│   ├── arm_server.py
│   └── arm_client.py
├── package.xml
├── setup.py
└── setup.cfg
```

Update `package.xml` to declare the action:

```xml
<buildtool_depend>ament_cmake_python</buildtool_depend>

<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>

<depend>rclpy</depend>
<depend>action_msgs</depend>
```

And update `setup.py` to generate the action:

```python
from setuptools import setup, find_packages
import glob
import os

package_name = 'my_package'

# Find .action files
action_files = glob.glob(os.path.join('action', '*.action'))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    description='Action-based arm control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arm_server=my_package.arm_server:main',
            'arm_client=my_package.arm_client:main',
        ],
    },
)
```

After building with `colcon build`, ROS 2 generates Python classes:
- `my_package.action.MoveArm` (the action type)
- `my_package.action.MoveArm.Goal`
- `my_package.action.MoveArm.Feedback`
- `my_package.action.MoveArm.Result`

## Basic Action Server

An action server executes goals and sends feedback:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import asyncio

from my_package.action import MoveArm

class ArmServer(Node):
    def __init__(self):
        super().__init__('arm_server')

        self.action_server = ActionServer(
            self, MoveArm, 'move_arm', self.execute_goal
        )
        self.get_logger().info("Arm server started")

    async def execute_goal(self, goal_handle):
        self.get_logger().info(
            f"Executing goal: move to ({goal_handle.request.target_x}, "
            f"{goal_handle.request.target_y}, {goal_handle.request.target_z})"
        )

        # Simulate movement over 5 seconds
        for i in range(1, 6):
            # Send feedback
            feedback = MoveArm.Feedback()
            feedback.progress_percent = i * 20
            feedback.status = f"Moving... {i*20}% complete"
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(feedback.status)

            # Simulate processing time
            await asyncio.sleep(1)

        # Send result
        result = MoveArm.Result()
        result.success = True
        result.message = "Reached target position"

        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    server = ArmServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key methods:**
- `goal_handle.publish_feedback(feedback)` - Send progress update
- `goal_handle.succeed()` - Mark goal as successful
- `goal_handle.abort()` - Mark goal as failed

## Basic Action Client

An action client sends goals and receives feedback:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import asyncio

from my_package.action import MoveArm

class ArmClient(Node):
    def __init__(self):
        super().__init__('arm_client')
        self.action_client = ActionClient(self, MoveArm, 'move_arm')

    async def send_goal(self):
        # Wait for server to be ready
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            return

        # Create goal
        goal = MoveArm.Goal()
        goal.target_x = 1.0
        goal.target_y = 2.0
        goal.target_z = 0.5

        self.get_logger().info("Sending goal to move arm")

        # Send goal (returns immediately, doesn't wait)
        goal_handle = await self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )

        # Wait for result
        result = await goal_handle.get_result_async()

        self.get_logger().info(
            f"Goal completed: {result.result.message}"
        )

    def feedback_callback(self, feedback):
        self.get_logger().info(
            f"Feedback: {feedback.feedback.status}"
        )

async def main(args=None):
    rclpy.init(args=args)
    client = ArmClient()

    # Run client logic
    await client.send_goal()

    rclpy.shutdown()

if __name__ == '__main__':
    import asyncio
    asyncio.run(main())
```

**Client workflow:**
1. `wait_for_server()` - Check server is running
2. `send_goal_async()` - Send goal and provide feedback callback
3. `get_result_async()` - Wait for final result

The client doesn't block waiting for goal execution. It receives feedback via callbacks.

## Handling Feedback During Execution

When the server publishes feedback, the client's feedback callback is invoked:

```python
def feedback_callback(self, feedback_msg):
    # This is called whenever server publishes feedback
    feedback = feedback_msg.feedback  # Extract actual feedback
    self.get_logger().info(f"Progress: {feedback.progress_percent}%")
    self.get_logger().info(f"Status: {feedback.status}")
```

This is where **AI as Teacher** helps: the client suggests capturing feedback to show progress, something you might not consider initially.

## Robust Action Client with Error Handling

Production clients need error handling:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import asyncio

from my_package.action import MoveArm

class RobustArmClient(Node):
    def __init__(self):
        super().__init__('robust_client')
        self.action_client = ActionClient(self, MoveArm, 'move_arm')

    async def send_goal_with_timeout(self, target_x, target_y, target_z):
        try:
            # Wait for server (5 second timeout)
            if not self.action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Server not ready after 5 seconds")
                return False

            # Create goal
            goal = MoveArm.Goal()
            goal.target_x = target_x
            goal.target_y = target_y
            goal.target_z = target_z

            # Send goal with feedback
            goal_handle = await self.action_client.send_goal_async(
                goal,
                feedback_callback=self.feedback_callback
            )

            # Wait for result with timeout
            result = await asyncio.wait_for(
                goal_handle.get_result_async(),
                timeout=15.0  # 15 second timeout
            )

            if result.result.success:
                self.get_logger().info(f"Success: {result.result.message}")
                return True
            else:
                self.get_logger().error(f"Failed: {result.result.message}")
                return False

        except asyncio.TimeoutError:
            self.get_logger().error("Goal execution timed out")
            return False
        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")
            return False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Progress: {feedback.progress_percent}%")

async def main(args=None):
    rclpy.init(args=args)
    client = RobustArmClient()

    success = await client.send_goal_with_timeout(1.0, 2.0, 0.5)
    if success:
        client.get_logger().info("Movement completed successfully")

    rclpy.shutdown()

if __name__ == '__main__':
    import asyncio
    asyncio.run(main())
```

**Error handling:**
- `wait_for_server()` - Detects if server isn't running
- `asyncio.wait_for()` - Detects if goal takes too long
- Try/except - Catches communication errors

## Exercise 3.1: Create Action Definition

Create `action/MoveArm.action`:

```
float32 target_x
float32 target_y
float32 target_z

---

int32 progress_percent
string status

---

bool success
string message
```

Build the package:

```bash
colcon build --packages-select my_package
source install/setup.bash
```

**Success criteria:**
- [ ] File created at `action/MoveArm.action`
- [ ] colcon build succeeds
- [ ] `from my_package.action import MoveArm` works in Python

---

## Exercise 3.2: Implement Action Server

Create `my_package/arm_server.py` with the ArmServer class above.

```bash
ros2 run my_package arm_server
```

**Success criteria:**
- [ ] Server runs without error
- [ ] "Arm server started" message appears
- [ ] Server doesn't crash when receiving goals (test in next exercise)

---

## Exercise 3.3: Implement Action Client

Create `my_package/arm_client.py` with the ArmClient class above.

Test client and server together:

```bash
# Terminal 1: Start server
ros2 run my_package arm_server

# Terminal 2: Run client
ros2 run my_package arm_client
```

**Success criteria:**
- [ ] Client connects to server
- [ ] Goal is sent
- [ ] Feedback messages appear (progress updates)
- [ ] Result is received and logged

---

## Exercise 3.4: Add Error Handling

Extend the client with the RobustArmClient implementation.

Test timeout handling:

```bash
# Kill server while client is waiting
# Client should gracefully handle timeout
```

**Success criteria:**
- [ ] Timeout is detected and logged
- [ ] Client exits cleanly (no crash)
- [ ] Error message is informative

---

## Try With AI

**Setup:** Reference your action server and client implementation.

**Prompt 1: Understand the Workflow**

Ask AI: "I have an action server that takes 10 seconds to execute a goal, and it sends feedback every second. I have an action client that sends a goal and waits for the result. Walk me through what happens step-by-step—when does the client block, when does it receive feedback, what happens if the server crashes?"

**Prompt 2: Identify Feedback Handling**

Show AI your basic client (without feedback callback) and ask: "My client receives feedback, but I'm not capturing it. How would you modify this to log all feedback updates? Why is that useful?"

**Prompt 3: Design Robust Error Handling**

Ask AI: "I want to make my client production-ready. What error cases should I handle? The server might not be running, the goal might take longer than expected, the network might disconnect—how would you add safeguards?"

**Expected Outcomes:**

Your AI should help clarify:
- Actions are for long-running tasks (services are for fast operations)
- Feedback keeps the client informed during execution
- Robust clients handle timeouts and errors gracefully
- Client never blocks waiting for goal execution (async is key)

**Self-Reflection:**

Before moving to Lesson 4, verify:
- [ ] You understand when to use actions vs services
- [ ] You can create and build a .action file
- [ ] You can implement a basic action server
- [ ] You can implement a basic action client
- [ ] You understand feedback flow and error handling
