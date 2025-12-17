---
sidebar_position: 7
title: "Lesson 7: Capstone Project - Multi-Node Robot System"
description: "Implement a complete robot monitoring system from specification"
---

# Lesson 7: Capstone Project - Multi-Node Robot System

**Learning Outcome:** Design and implement a distributed multi-node robot system from specification, integrating all concepts from Lessons 1-6.

**Proficiency Level:** B1

**Estimated Time:** 90 minutes

---

## System Specification

You're building a robot status monitor system. Read this specification carefully—it defines what you need to build:

```
ROBOT STATUS MONITOR SYSTEM

Intent:
Build a distributed monitoring system that tracks robot health metrics
and alerts when thresholds are exceeded.

Components:
1. Battery Publisher — Simulates battery draining 100% → 0%
2. Temperature Sensor — Simulates motor heating 20°C → 80°C
3. Status Monitor — Aggregates data and triggers alerts

Communication:
- Battery data: Topic /robot/battery (String, 1 message per 0.5 seconds)
- Temperature data: Topic /robot/temperature (String, 1 message per 1 second)
- Status queries: Service /robot/status (synchronous)

Thresholds:
- Battery warning: <20% (log "ALERT: Battery critical")
- Temperature warning: >60°C (log "ALERT: Motor overheating")

Quality of Service:
- All topics use RELIABLE QoS (no message drops)
- All components use same QoS policy

Success Criteria:
[ ] Battery publisher starts and publishes correct messages
[ ] Temperature publisher starts and publishes correct messages
[ ] Monitor node subscribed to both topics
[ ] Monitor receives all messages without drops
[ ] Monitor logs alerts when thresholds exceeded
[ ] Service responds to status queries
[ ] System runs without crashes for 60+ seconds
```

---

## Step 1: Understand the Specification

Before writing code, answer these questions:

1. **How many nodes do we need?** (Answer: 3 - battery, temperature, monitor)
2. **What topics are used?** (Answer: /robot/battery, /robot/temperature)
3. **What are the publish frequencies?** (Answer: battery 0.5s, temperature 1.0s)
4. **Which QoS policy?** (Answer: RELIABLE)
5. **What triggers alerts?** (Answer: battery below 20%, temperature above 60°C)

If you can answer these without looking back at the spec, you understand it.

---

## Step 2: Design the System Architecture

Draw a diagram (on paper or text):

```
┌─────────────────┐
│ Battery         │
│ Publisher       │
│ (0.5 Hz)        │
└────────┬────────┘
         │
         │ /robot/battery
         │ (String)
         │
    ┌────▼──────────────────┐
    │                       │
┌───┴─────────┐      ┌──────┴────┐
│ ROS 2       │      │ Monitor   │
│ Topic Bus   │      │ Node      │
└────┬────────┘      └──────┬────┘
     │                      │
     │ /robot/temperature   │
     │ (String)             │
     │                      │
┌────▼─────────────┐        │
│ Temperature      │        │
│ Publisher        │        │
│ (1.0 Hz)         │        │
└──────────────────┘        │
                            │ /robot/status
                            │ Service
```

Key points:
- Two publishers on separate topics
- One monitor subscribing to both
- One service for status queries

---

## Step 3: Implement Battery Publisher

**File: `capstone_battery_publisher.py`**

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')

        # Use RELIABLE QoS as per specification
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)

        self.publisher_ = self.create_publisher(String, 'robot/battery', qos)
        self.get_logger().info('Battery publisher started')

        # Publish every 0.5 seconds (2 Hz)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.battery = 100

    def timer_callback(self):
        msg = String()
        msg.data = f'battery: {self.battery}%'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

        self.battery -= 2  # Decrease by 2% each message
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

**Verify:**
```bash
python3 capstone_battery_publisher.py
```

Should log battery messages every 0.5 seconds.

---

## Step 4: Implement Temperature Publisher

**File: `capstone_temperature_sensor.py`**

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')

        # Use RELIABLE QoS as per specification
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)

        self.publisher_ = self.create_publisher(String, 'robot/temperature', qos)
        self.get_logger().info('Temperature sensor started')

        # Publish every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.temperature = 20.0

    def timer_callback(self):
        msg = String()
        msg.data = f'temperature: {self.temperature:.1f}°C'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

        self.temperature += 1.0  # Increase by 1°C each second
        if self.temperature > 80:
            self.temperature = 20.0


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Verify:**
```bash
python3 capstone_temperature_sensor.py
```

Should log temperature messages every 1 second.

---

## Step 5: Implement Status Monitor

**File: `capstone_status_monitor.py`**

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


class StatusMonitor(Node):
    def __init__(self):
        super().__init__('status_monitor')

        # Use RELIABLE QoS matching publishers
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)

        # Subscribe to both topics
        self.battery_sub = self.create_subscription(
            String, 'robot/battery', self.battery_callback, qos)

        self.temperature_sub = self.create_subscription(
            String, 'robot/temperature', self.temperature_callback, qos)

        self.get_logger().info('Status monitor started')
        self.battery = 100
        self.temperature = 20.0

    def battery_callback(self, msg):
        # Parse message and check threshold
        try:
            battery_str = msg.data.split(': ')[1].rstrip('%')
            self.battery = int(battery_str)
            self.get_logger().info(f'Battery: {self.battery}%')

            if self.battery < 20:
                self.get_logger().error('ALERT: Battery critical')
        except:
            self.get_logger().warn(f'Could not parse battery message: {msg.data}')

    def temperature_callback(self, msg):
        # Parse message and check threshold
        try:
            temp_str = msg.data.split(': ')[1].rstrip('°C')
            self.temperature = float(temp_str)
            self.get_logger().info(f'Temperature: {self.temperature}°C')

            if self.temperature > 60.0:
                self.get_logger().error('ALERT: Motor overheating')
        except:
            self.get_logger().warn(f'Could not parse temperature message: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 6: Run the Complete System

**Terminal 1 - Battery Publisher:**
```bash
python3 capstone_battery_publisher.py
```

**Terminal 2 - Temperature Publisher:**
```bash
python3 capstone_temperature_sensor.py
```

**Terminal 3 - Status Monitor:**
```bash
python3 capstone_status_monitor.py
```

**Terminal 4 - Verify with CLI tools:**
```bash
# Check all topics
ros2 topic list

# View battery messages
ros2 topic echo robot/battery

# Check publisher/subscriber counts
ros2 topic info robot/battery
ros2 topic info robot/temperature

# List all nodes
ros2 node list
```

---

## Step 7: Validate Against Specification

Verify each success criterion:

- [ ] **Battery publisher publishes every 0.5s?**
  ```bash
  ros2 topic hz robot/battery
  # Expected: ~2.0 Hz (every 0.5 seconds)
  ```

- [ ] **Temperature publisher publishes every 1s?**
  ```bash
  ros2 topic hz robot/temperature
  # Expected: ~1.0 Hz (every 1 second)
  ```

- [ ] **Monitor receives all messages?**
  - Count messages in monitor logs for 10 seconds
  - Battery: ~20 messages, Temperature: ~10 messages
  - If messages match, no drops with RELIABLE QoS

- [ ] **Alerts trigger at correct thresholds?**
  - Wait for battery to drop below 20% → See "ALERT: Battery critical"
  - Wait for temperature to exceed 60°C → See "ALERT: Motor overheating"

- [ ] **No crashes after 60+ seconds?**
  - Let system run for 1+ minute
  - All nodes remain running
  - No error messages

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Monitor not receiving messages | Check QoS matches (both RELIABLE) |
| Topic names don't match | Verify topic names: `/robot/battery`, `/robot/temperature` |
| Messages skip numbers | Check QoS is RELIABLE (not BEST_EFFORT) |
| Parser error on messages | Check message format matches parsing code |
| Service not responding | Implement service callback (optional for this capstone) |

---

## Hands-On Practice

### Exercise 7.1: Verify System Behavior

Run the complete system and document:
1. Number of messages received in 60 seconds (should match frequency)
2. When alerts trigger (at what battery/temperature values)
3. Any message drops (should be zero with RELIABLE QoS)

### Exercise 7.2: Debug an Intentional Problem

Modify one of the publishers:
- Change one publisher's topic name (e.g., `/robot/battery_wrong`)
- Use ROS 2 CLI tools to diagnose the mismatch
- Document the diagnostic steps

### Exercise 7.3: Add a Service (Optional Extension)

Implement `/robot/status` service that returns current battery and temperature when queried.

---

## Key Concepts Integrated

This capstone brings together:
- **Lesson 1:** Node creation and lifecycle
- **Lesson 2:** Pub/sub communication through topics
- **Lesson 3:** CLI tools for verification
- **Lesson 4:** (Services - optional extension)
- **Lesson 5:** QoS policies for reliable delivery
- **Lesson 6:** Reusable patterns (all publishers follow same structure)

---

## Try With AI

Extend the capstone system:

**Step 1: Propose enhancements**
Prompt your AI:
```
The robot monitor system works, but needs improvements:
1. What if multiple robots send status to the same topics?
2. How would you log historical data for analysis?
3. Could you add a web dashboard to view robot status?

For each, what ROS 2 concepts would you use?
```

**Step 2: Implement an enhancement**
Pick one of these:
- Add a **historian node** that logs all messages to file
- Add a **web interface** that queries status service
- Extend for **multiple robots** (different topic namespaces)

**Step 3: Validate your extension**
- Does it work with existing system?
- Do all nodes still communicate?
- Can you still use CLI tools to debug?

**Expected Outcome:** A working capstone that demonstrates you've mastered ROS 2 fundamentals and can extend systems independently.
