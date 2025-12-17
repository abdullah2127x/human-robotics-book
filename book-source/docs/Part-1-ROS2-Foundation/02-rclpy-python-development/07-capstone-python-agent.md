---
sidebar_position: 7
title: "Lesson 7: Capstone Project — Python Agent Controlling Robot"
description: "Build an autonomous Python agent from specification that integrates all Chapter 2 concepts into a working multi-node system."
---

# Lesson 7: Capstone Project — Python Agent Controlling Robot

This capstone brings everything together. You'll read a specification, design a system architecture, implement multiple coordinating nodes, and validate that your system meets the spec. This is where Lessons 1-6 become a real, working robot agent.

## System Specification

Read this specification completely before writing any code.

```
Robot Arm Controller Agent

INTENT
Build an autonomous Python agent that monitors a robot arm's state
and makes decisions about when to command movements.

CONSTRAINTS
- Sensor topic: /arm/sensors (custom RobotSensor message)
  * temperature_celsius: float32
  * position_x, position_y, position_z: float32 (current arm position)

- Movement action: /arm/move (MoveArm action)
  * Goal: target_x, target_y, target_z (float32)
  * Feedback: progress_percent (int32)
  * Result: success (bool)

- Status publisher: /arm/status (custom ArmStatus message)
  * current_x, current_y, current_z: float32
  * is_moving: bool
  * last_command: string
  * temperature_celsius: float32

- Decision logic: Move if temperature < 50°C, else wait for cooling

- All callbacks must be non-blocking (async/await)

- Use MultiThreadedExecutor for responsiveness

SUCCESS CRITERIA
✓ Agent subscribes to /arm/sensors topic
✓ Agent publishes to /arm/status topic (every 1 second)
✓ Agent sends movement goals when temperature permits
✓ Agent monitors feedback from arm movements
✓ All nodes coordinate without crashes
✓ No blocking, fully async implementation
```

## System Architecture

Before coding, visualize the system:

```
┌─────────────────┐
│ Sensor Simulator│ (publishes sensor data)
│   10 Hz         │
└────────┬────────┘
         │ /arm/sensors (RobotSensor)
         │
    ┌────▼──────────────────┐
    │                       │
    │  Agent Node           │
    │  (decision logic)      │
    │  - subscribes sensors  │
    │  - publishes status    │
    │  - sends move goals    │
    │                       │
    └────┬──────────┬───────┘
         │          │
         │ /arm/status (ArmStatus)
         │
    ┌────▼──────────────────┐
    │  Monitoring Console   │
    │  (displays status)    │
    └──────────────────────┘

    ┌───────────────────────┐
    │  Arm Action Server    │
    │  /arm/move            │
    │  (simulates movement) │
    │  5 sec execution      │
    └──────────────────────┘
    (Agent sends goals, server responds with feedback)
```

## Component 1: Custom Message Types

Create the messages needed:

**msg/RobotSensor.msg:**
```
float32 temperature_celsius
float32 position_x
float32 position_y
float32 position_z
```

**msg/ArmStatus.msg:**
```
float32 current_x
float32 current_y
float32 current_z
bool is_moving
string last_command
float32 temperature_celsius
```

**action/MoveArm.action:**
```
float32 target_x
float32 target_y
float32 target_z

---

int32 progress_percent

---

bool success
string message
```

## Component 2: Sensor Simulator Node

Publishes simulated sensor data:

```python
# sensor_simulator.py
import rclpy
from rclpy.node import Node
from my_package.msg import RobotSensor
import random

class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')

        self.publisher = self.create_publisher(
            RobotSensor, '/arm/sensors', 10
        )

        self.create_timer(0.1, self.publish_sensor_data)  # 10 Hz

        # Simulated state
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.5
        self.temperature = 45.0

        self.get_logger().info("Sensor simulator started")

    def publish_sensor_data(self):
        msg = RobotSensor()
        msg.position_x = self.position_x
        msg.position_y = self.position_y
        msg.position_z = self.position_z
        msg.temperature_celsius = self.temperature

        self.publisher.publish(msg)

        # Simulate temperature fluctuation
        self.temperature += random.uniform(-0.5, 0.5)
        self.temperature = max(40.0, min(55.0, self.temperature))

def main(args=None):
    rclpy.init(args=args)
    node = SensorSimulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Component 3: Arm Action Server

Simulates arm movement with feedback:

```python
# arm_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import asyncio

from my_package.action import MoveArm

class ArmServer(Node):
    def __init__(self):
        super().__init__('arm_server')

        self.action_server = ActionServer(
            self, MoveArm, '/arm/move', self.execute_goal
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.5

        self.get_logger().info("Arm server started")

    async def execute_goal(self, goal_handle):
        self.get_logger().info(
            f"Goal: move to ({goal_handle.request.target_x}, "
            f"{goal_handle.request.target_y}, {goal_handle.request.target_z})"
        )

        # Simulate 5-second movement
        for i in range(1, 6):
            feedback = MoveArm.Feedback()
            feedback.progress_percent = i * 20
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(f"Moving... {i*20}% complete")
            await asyncio.sleep(1)

        # Update position
        self.current_x = goal_handle.request.target_x
        self.current_y = goal_handle.request.target_y
        self.current_z = goal_handle.request.target_z

        result = MoveArm.Result()
        result.success = True
        result.message = "Movement complete"

        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    server = ArmServer()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server)
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Component 4: Decision-Making Agent Node

The core of the system—makes autonomous decisions:

```python
# agent_node.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import asyncio

from my_package.msg import RobotSensor, ArmStatus
from my_package.action import MoveArm

class ArmAgent(Node):
    def __init__(self):
        super().__init__('arm_agent')

        # Subscribe to sensors
        self.sensor_sub = self.create_subscription(
            RobotSensor, '/arm/sensors', self.sensor_callback, 10
        )

        # Publish status
        self.status_pub = self.create_publisher(
            ArmStatus, '/arm/status', 10
        )

        # Action client for arm movement
        self.move_client = ActionClient(self, MoveArm, '/arm/move')

        # Status update timer
        self.create_timer(1.0, self.publish_status)

        # Internal state
        self.last_temperature = 50.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.5
        self.is_moving = False
        self.last_command = "none"

        self.get_logger().info("Agent started")

    async def sensor_callback(self, msg):
        """Handle sensor updates - make decisions."""
        self.current_x = msg.position_x
        self.current_y = msg.position_y
        self.current_z = msg.position_z
        self.last_temperature = msg.temperature_celsius

        # Decision: Move if cool enough
        if (msg.temperature_celsius < 50.0 and
            not self.is_moving):
            self.get_logger().info(
                f"Temperature OK ({msg.temperature_celsius:.1f}C). "
                f"Sending movement goal."
            )

            # Non-blocking: spawn movement task
            asyncio.ensure_future(self.move_arm())

    async def move_arm(self):
        """Send movement goal (non-blocking)."""
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Arm server not available")
            return

        goal = MoveArm.Goal()
        goal.target_x = 1.0
        goal.target_y = 1.0
        goal.target_z = 1.0

        self.is_moving = True
        self.last_command = "move_to_1_1_1"

        goal_handle = await self.move_client.send_goal_async(goal)
        result = await goal_handle.get_result_async()

        if result.result.success:
            self.get_logger().info("Movement successful")
        else:
            self.get_logger().warn("Movement failed")

        self.is_moving = False

    def publish_status(self):
        """Publish current status (1 Hz)."""
        msg = ArmStatus()
        msg.current_x = self.current_x
        msg.current_y = self.current_y
        msg.current_z = self.current_z
        msg.is_moving = self.is_moving
        msg.last_command = self.last_command
        msg.temperature_celsius = self.last_temperature

        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    agent = ArmAgent()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(agent)
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Component 5: Monitoring Console

Optional display node to show system status:

```python
# monitor.py
import rclpy
from rclpy.node import Node
from my_package.msg import ArmStatus

class Monitor(Node):
    def __init__(self):
        super().__init__('monitor')

        self.create_subscription(
            ArmStatus, '/arm/status', self.status_callback, 10
        )

        self.get_logger().info("Monitor started")

    def status_callback(self, msg):
        moving = "MOVING" if msg.is_moving else "IDLE"
        self.get_logger().info(
            f"Status: {moving} | "
            f"Pos: ({msg.current_x:.1f}, {msg.current_y:.1f}, {msg.current_z:.1f}) | "
            f"Temp: {msg.temperature_celsius:.1f}C | "
            f"Last: {msg.last_command}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Monitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 7.1: System Setup

Create the message and action files:

- [ ] Create `msg/RobotSensor.msg`
- [ ] Create `msg/ArmStatus.msg`
- [ ] Create `action/MoveArm.action`

Build the package:

```bash
colcon build --packages-select my_package
source install/setup.bash
```

Verify generation:

```bash
ros2 interface show my_package/RobotSensor
ros2 interface show my_package/ArmStatus
ros2 interface show my_package/MoveArm
```

**Success criteria:**
- [ ] All message files created
- [ ] All action files created
- [ ] colcon build succeeds
- [ ] `ros2 interface show` displays all types correctly

---

## Exercise 7.2: Implement All Components

Create the Python files:

- [ ] `sensor_simulator.py` - Publishes sensor data
- [ ] `arm_server.py` - Action server for arm movement
- [ ] `agent_node.py` - Decision-making agent
- [ ] `monitor.py` - Status monitoring

Update `setup.py` entry_points:

```python
'console_scripts': [
    'sensor_sim=my_package.sensor_simulator:main',
    'arm_server=my_package.arm_server:main',
    'agent=my_package.agent_node:main',
    'monitor=my_package.monitor:main',
],
```

Build:

```bash
colcon build --packages-select my_package
```

**Success criteria:**
- [ ] All Python files created with correct syntax
- [ ] All entry points defined in setup.py
- [ ] colcon build succeeds

---

## Exercise 7.3: System Integration Test

Run all components together (you'll need 4 terminals):

**Terminal 1: Start sensor simulator**
```bash
ros2 run my_package sensor_sim
```

**Terminal 2: Start arm server**
```bash
ros2 run my_package arm_server
```

**Terminal 3: Start agent**
```bash
ros2 run my_package agent
```

**Terminal 4: Monitor status**
```bash
ros2 run my_package monitor
```

Expected output (agent terminal):
```
Agent started
Temperature OK (45.5C). Sending movement goal.
[After 5 seconds]
Movement successful
```

Expected output (monitor terminal):
```
Status: IDLE | Pos: (0.0, 0.0, 0.5) | Temp: 45.5C | Last: none
Status: MOVING | Pos: (0.0, 0.0, 0.5) | Temp: 45.3C | Last: move_to_1_1_1
[5 seconds later]
Status: IDLE | Pos: (1.0, 1.0, 1.0) | Temp: 45.0C | Last: move_to_1_1_1
```

**Success criteria:**
- [ ] All nodes start without errors
- [ ] Sensor simulator publishes data
- [ ] Agent receives sensor data and makes decisions
- [ ] Arm server responds to movement goals
- [ ] Monitor displays status updates
- [ ] No crashes during execution

---

## Exercise 7.4: Specification Validation

Check your system against the specification:

**Specification Requirements:**

- [ ] Agent subscribes to /arm/sensors (RobotSensor message)
- [ ] Agent publishes to /arm/status (ArmStatus message)
- [ ] Agent sends movement goals via /arm/move action
- [ ] Agent monitors feedback from movements
- [ ] All callbacks are non-blocking (async/await used)
- [ ] MultiThreadedExecutor used for concurrency
- [ ] Status published every 1 second
- [ ] Movement triggered when temperature < 50°C
- [ ] System runs without crashes

Create a validation checklist and mark each item as verified:

```
Specification Compliance Checklist

Core Functionality:
- [ ] Agent reads sensor data at 10 Hz
- [ ] Agent publishes status at 1 Hz
- [ ] Agent commands movements when cool
- [ ] Arm movement takes ~5 seconds
- [ ] Feedback logged during movement

Non-Blocking Implementation:
- [ ] All node callbacks are async (no blocking sleeps)
- [ ] MultiThreadedExecutor used (checked code)
- [ ] Concurrent callbacks visible (monitor shows no delays)

Message Types:
- [ ] RobotSensor has all required fields
- [ ] ArmStatus has all required fields
- [ ] MoveArm action has Goal/Feedback/Result

System Integration:
- [ ] All 4 nodes run together
- [ ] No communication errors
- [ ] No race conditions detected
- [ ] Graceful shutdown possible
```

**Success criteria:**
- [ ] All specification requirements met
- [ ] System validated against requirements
- [ ] Evidence collected (logs, screenshots)

---

## Exercise 7.5: Design Documentation

Document your design decisions:

1. **Architecture diagram** (draw or describe)
   - How do components communicate?
   - What messages/actions flow between nodes?

2. **Decision logic justification**
   - Why move when temperature < 50°C?
   - Why use async/await for agent callbacks?
   - Why MultiThreadedExecutor instead of single-threaded?

3. **Patterns used**
   - Which patterns from Lessons 1-6 appear in your system?
   - How did reusable patterns speed development?

4. **Lessons learned**
   - What was hardest about coordinating multiple nodes?
   - How would you extend this system (add more sensors, different movements)?
   - What would you change about the architecture?

---

## Try With AI

**Setup:** Reference your complete capstone implementation.

**Prompt 1: Architecture Review**

Ask AI: "Review my multi-node robot system. The agent subscribes to sensors, makes decisions, and sends movement commands. Are there architectural improvements? Should components communicate differently? What's missing?"

**Prompt 2: Scalability**

Ask AI: "My system controls one arm with one decision agent. If I added a second arm and wanted the agent to coordinate both, how would I restructure this? What new challenges arise?"

**Prompt 3: Production Hardening**

Ask AI: "This is a working prototype. What would I need to add to make it production-ready? Error handling, logging, monitoring, configuration management—walk me through the essentials."

**Expected Outcomes:**

Your AI should help clarify:
- Multi-node systems require careful message/action design
- Async callbacks enable responsive system behavior
- System architecture trades complexity for flexibility
- Production systems need robustness beyond the basic prototype

**Self-Reflection:**

After completing the capstone, verify:
- [ ] You understand spec-first development (spec before code)
- [ ] You can design multi-node systems with clear communication
- [ ] You recognize when to use Messages, Services, and Actions
- [ ] You can integrate async callbacks, custom messages, and actions
- [ ] You understand how Lessons 1-6 compose into a working system

