---
sidebar_position: 4
title: "Lesson 4: Custom Message Types"
description: "Define custom message types for structured communication between ROS 2 nodes."
---

# Lesson 4: Custom Message Types

Standard ROS 2 messages like `String`, `Int32`, and `Float64` work for simple data. But what if you need to send a robot's status with multiple fields: battery percentage, temperature, and position coordinates? You could send them as separate messages, but that's awkward. Custom messages let you define structured data that matches your domain.

## Standard Messages vs Custom Messages

**Standard message** (only one value):
```python
from std_msgs.msg import String
msg = String()
msg.data = "battery: 85%"
publisher.publish(msg)
```

**Custom message** (multiple related fields):
```python
from my_package.msg import RobotStatus
msg = RobotStatus()
msg.battery_percent = 85
msg.temperature_celsius = 45.2
msg.arm_position_x = 1.0
msg.arm_position_y = 2.0
publisher.publish(msg)
```

Custom messages group related data into a single, typed structure.

## Defining Custom Message Types

Message definitions go in a `.msg` file. Here's a simple robot status message:

```
# RobotStatus.msg

int32 battery_percent
float32 temperature_celsius
float32 arm_position_x
float32 arm_position_y
string state
```

Each line declares a field: `type name`.

## Common Message Field Types

```
int8, int16, int32, int64      # Signed integers
uint8, uint16, uint32, uint64  # Unsigned integers
float32, float64               # Floating point
bool                           # True/false
string                         # Text (variable length)
time                           # ROS 2 timestamp
```

## Package Organization

Place `.msg` files in a `msg/` subdirectory:

```
my_package/
├── msg/
│   ├── RobotStatus.msg
│   └── ArmCommand.msg
├── my_package/
│   ├── status_publisher.py
│   └── status_subscriber.py
├── package.xml
└── setup.py
```

## Updating package.xml

Add message generation dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>Custom message example</description>

  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <!-- Message generation -->
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <depend>rclpy</depend>
</package>
```

## Updating setup.py

```python
from setuptools import setup, find_packages
import glob
import os

package_name = 'my_package'

# Find .msg files for generation
msg_files = glob.glob(os.path.join('msg', '*.msg'))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', msg_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    description='Custom message example',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'status_pub=my_package.status_publisher:main',
            'status_sub=my_package.status_subscriber:main',
        ],
    },
)
```

After building with `colcon build`, ROS 2 generates Python classes:
- `my_package.msg.RobotStatus`
- `my_package.msg.ArmCommand`

## Publisher with Custom Message

```python
import rclpy
from rclpy.node import Node
from my_package.msg import RobotStatus

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_pub')

        # Publisher for RobotStatus messages
        self.publisher = self.create_publisher(
            RobotStatus, 'robot_status', 10
        )

        # Timer: publish every 1 second
        self.create_timer(1.0, self.publish_status)

        self.battery = 100
        self.temperature = 25.0
        self.arm_x = 0.0
        self.arm_y = 0.0

        self.get_logger().info("Status publisher started")

    def publish_status(self):
        msg = RobotStatus()
        msg.battery_percent = self.battery
        msg.temperature_celsius = self.temperature
        msg.arm_position_x = self.arm_x
        msg.arm_position_y = self.arm_y
        msg.state = "operational"

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published: battery={msg.battery_percent}%, "
            f"temp={msg.temperature_celsius}C"
        )

        # Simulate battery drain and temperature change
        self.battery -= 1
        self.temperature += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = StatusPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscriber with Custom Message

```python
import rclpy
from rclpy.node import Node
from my_package.msg import RobotStatus

class StatusSubscriber(Node):
    def __init__(self):
        super().__init__('status_sub')

        # Subscriber for RobotStatus messages
        self.subscription = self.create_subscription(
            RobotStatus, 'robot_status', self.status_callback, 10
        )

        self.get_logger().info("Status subscriber started")

    def status_callback(self, msg):
        self.get_logger().info(
            f"Status update:\n"
            f"  Battery: {msg.battery_percent}%\n"
            f"  Temperature: {msg.temperature_celsius:.1f}C\n"
            f"  Arm position: ({msg.arm_position_x:.2f}, {msg.arm_position_y:.2f})\n"
            f"  State: {msg.state}"
        )

        # Check for warnings
        if msg.battery_percent < 20:
            self.get_logger().warn("Low battery!")
        if msg.temperature_celsius > 50:
            self.get_logger().warn("High temperature!")

def main(args=None):
    rclpy.init(args=args)
    node = StatusSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Nested Messages (Advanced)

Messages can contain other message types. Define a Position message:

```
# Position.msg
float32 x
float32 y
float32 z
```

Then use it in RobotArm:

```
# RobotArm.msg
int32 id
Position position
float32[] joint_angles
string gripper_state
```

Nested messages work just like standard fields:

```python
msg = RobotArm()
msg.position.x = 1.0
msg.position.y = 2.0
msg.position.z = 0.5
```

## How Custom Messages Are Generated

When you run `colcon build`, ROS 2:

1. **Finds** `.msg` files in your package
2. **Parses** the field definitions
3. **Generates** Python classes with:
   - `__init__()` constructor
   - Field attributes (battery_percent, temperature_celsius, etc.)
   - Serialization for network transmission
4. **Installs** the classes so you can `from my_package.msg import RobotStatus`

## Understanding Message Design

Good message design takes practice. Consider:

**Too many fields** (hard to understand):
```
# Bad: Overloaded with every possible field
msg.battery_percent = 85
msg.battery_voltage = 12.0
msg.battery_current = 2.5
msg.temperature_motor1 = 45
msg.temperature_motor2 = 48
msg.temperature_sensor = 25
msg.arm_x = 1.0
msg.arm_y = 2.0
msg.arm_z = 0.5
# ... 20 more fields
```

**Right-sized** (clear and focused):
```
msg.battery_percent = 85
msg.temperature_highest = 48
msg.arm_position_x = 1.0
msg.arm_position_y = 2.0
msg.arm_position_z = 0.5
```

Group related data together, exclude what's not essential.

## Exercise 4.1: Create Custom Message File

Create `msg/RobotStatus.msg`:

```
int32 battery_percent
float32 temperature_celsius
float32 arm_position_x
float32 arm_position_y
string state
```

Build the package:

```bash
colcon build --packages-select my_package
source install/setup.bash
```

Verify the message was generated:

```bash
ros2 interface show my_package/RobotStatus
```

Expected output:
```
int32 battery_percent
float32 temperature_celsius
float32 arm_position_x
float32 arm_position_y
string state
```

**Success criteria:**
- [ ] `.msg` file created at `msg/RobotStatus.msg`
- [ ] colcon build succeeds
- [ ] `ros2 interface show` displays the message correctly
- [ ] `from my_package.msg import RobotStatus` works in Python

---

## Exercise 4.2: Implement Publisher

Create `my_package/status_publisher.py` using the StatusPublisher class above.

Test:

```bash
ros2 run my_package status_pub
```

In another terminal, verify messages:

```bash
ros2 topic echo /robot_status
```

Expected output:
```
battery_percent: 99
temperature_celsius: 25.1
arm_position_x: 0.0
arm_position_y: 0.0
state: operational
---
battery_percent: 98
temperature_celsius: 25.2
arm_position_x: 0.0
arm_position_y: 0.0
state: operational
---
```

**Success criteria:**
- [ ] Publisher runs without error
- [ ] Messages are published at 1 Hz
- [ ] All fields have expected values
- [ ] `ros2 topic echo` shows correct message structure

---

## Exercise 4.3: Implement Subscriber

Create `my_package/status_subscriber.py` using the StatusSubscriber class above.

Test:

```bash
# Terminal 1: Publisher
ros2 run my_package status_pub

# Terminal 2: Subscriber
ros2 run my_package status_sub
```

Expected output (subscriber):
```
Status update:
  Battery: 99%
  Temperature: 25.1C
  Arm position: (0.00, 0.00)
  State: operational
```

**Success criteria:**
- [ ] Subscriber runs without error
- [ ] Receives messages from publisher
- [ ] All message fields are accessible
- [ ] Status is formatted and logged correctly

---

## Exercise 4.4: Message Design with AI Guidance

Ask your AI tool: "I want to create a robot arm command message with: target position (x, y, z), execution speed (0-100%), and whether to use force control. Design the message structure. Should I use nested messages or flat fields?"

Discuss with AI:
- **Option 1** (flat): All fields at top level (simple, but verbose)
- **Option 2** (nested): Position as separate message (organized, reusable)

**Success criteria:**
- [ ] You understand tradeoffs between flat and nested designs
- [ ] You can justify your message structure choice
- [ ] You recognize when to create nested messages for reusability

---

## Try With AI

**Setup:** Reference your publisher and subscriber code.

**Prompt 1: Message Structure Design**

Ask AI: "I have a message with 12 fields representing robot state: 6 joint angles, battery, temperature, position (x,y,z), and mode. This feels cluttered. How would you restructure this to be cleaner?"

**Prompt 2: Type Selection**

Show AI your message definition and ask: "I defined some fields as float32 but others as int32. How do you decide which type to use? What about fields that change frequently vs. rarely—does that affect the choice?"

**Prompt 3: Backward Compatibility**

Ask AI: "I published messages with 5 fields for the last year. Now I want to add 3 more fields. Will existing subscribers break? How do I avoid breaking changes?"

**Expected Outcomes:**

Your AI should help clarify:
- Custom messages structure data for pub/sub communication
- Message design balances simplicity (fewer fields) with completeness (all needed data)
- Nested messages improve organization and reusability
- Type selection depends on data range and precision needs

**Self-Reflection:**

Before moving to Lesson 5, verify:
- [ ] You can create a .msg file with appropriate fields
- [ ] You understand the difference between basic and nested messages
- [ ] You can publish and subscribe with custom message types
- [ ] You can articulate why a message structure is well-designed
