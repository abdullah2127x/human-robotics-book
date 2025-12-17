---
sidebar_position: 5
title: "Lesson 5: Collision Detection and Contact Sensing"
description: "Implement contact sensors on humanoid feet for real-time collision feedback with AI collaboration."
---

# Lesson 5: Collision Detection and Contact Sensing

## Learning Objectives

By completing this lesson, you will:
- Add contact sensors to humanoid URDF
- Configure Gazebo's contact plugin
- Process ROS 2 ContactsState messages
- Implement contact detection logic for balance control
- Debug sensor issues with AI assistance

**Estimated time**: 120 minutes

---

## Why Contact Sensing Matters

A humanoid robot standing and walking needs constant feedback: "Are my feet touching the ground?" Without this, your controller is flying blind. Contact sensors provide this essential feedback via ROS 2 topics.

In simulation, contact sensors are plugins that publish collision data. In real robots, they're pressure sensors in feet. Here, we implement the simulation version, and the same software patterns work for both.

---

## Adding Contact Sensors to URDF

Your humanoid URDF needs contact sensors on its feet. Open `humanoid.urdf` from Chapter 3 and add sensors.

### Sensor Definition in URDF

```xml
<!-- Add this inside the humanoid's left foot link -->
<link name="left_foot">
  <!-- Existing inertial, collision, visual elements -->

  <!-- Contact sensor: Detects collisions at this link -->
  <sensor name="left_foot_contact" type="contact">
    <always_on>true</always_on>
    <update_rate>50</update_rate>
    <contact>
      <!-- Which link the sensor monitors for contacts -->
      <collision>left_foot_collision</collision>
    </contact>
  </sensor>
</link>

<!-- Identical for right foot -->
<link name="right_foot">
  <!-- Existing elements -->

  <sensor name="right_foot_contact" type="contact">
    <always_on>true</always_on>
    <update_rate>50</update_rate>
    <contact>
      <collision>right_foot_collision</collision>
    </contact>
  </sensor>
</link>
```

### Gazebo Plugin Configuration

In your world file, enable the contact plugin:

```xml
<world name="humanoid_training">
  <!-- ... existing physics, gravity, etc ... -->

  <!-- Plugin: Publishes contact data to ROS 2 -->
  <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact">
  </plugin>

  <!-- Your models and ground plane here -->
</world>
```

---

## Understanding ContactsState Messages

When humanoid feet touch ground, Gazebo publishes `gazebo_msgs/ContactsState` messages.

### Message Structure

```python
# ROS 2 message definition (reference)
from gazebo_msgs.msg import ContactsState

class ContactsState:
    # Header with timestamp
    header: std_msgs.msg.Header

    # Contact information for each collision
    states: list[ContactState]
        # Each ContactState contains:
        # - collision1: Name of first colliding geometry
        # - collision2: Name of second colliding geometry
        # - wrenches: Forces applied at contact points
        # - total_wrench: Sum of all contact forces
        # - contact_positions: 3D positions of contact points
        # - contact_normals: Surface normal directions
        # - depths: Penetration depths at each contact
```

### Example: Processing Contacts

```python
from gazebo_msgs.msg import ContactsState
import rclpy

class ContactListener:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('contact_listener')
        self.subscription = self.node.create_subscription(
            ContactsState,
            '/gazebo/contacts_states',
            self.contact_callback,
            10
        )
        self.left_foot_contact = False
        self.right_foot_contact = False

    def contact_callback(self, msg):
        """Called when contact message received."""
        self.left_foot_contact = False
        self.right_foot_contact = False

        for state in msg.states:
            collision1 = state.collision1
            collision2 = state.collision2

            if 'left_foot' in collision1 and 'ground' in collision2:
                self.left_foot_contact = True
                self.node.get_logger().info(
                    f'Left foot contact: force = {state.total_wrench.force.z:.2f} N'
                )

            if 'right_foot' in collision1 and 'ground' in collision2:
                self.right_foot_contact = True
                self.node.get_logger().info(
                    f'Right foot contact: force = {state.total_wrench.force.z:.2f} N'
                )

    def get_contact_state(self):
        """Returns: (left_contact, right_contact) booleans."""
        return self.left_foot_contact, self.right_foot_contact

    def run(self):
        rclpy.spin(self.node)

if __name__ == '__main__':
    listener = ContactListener()
    listener.run()
```

---

## Exercise 1: Observe Contact Messages

Test your sensor setup:

```bash
# Terminal 1: Gazebo
gz sim -r training_world.sdf

# Terminal 2: Spawn humanoid
python3 spawn_humanoid.py

# Terminal 3: Listen to contacts
ros2 topic echo /gazebo/contacts_states
```

Observe the contacts topic:
- [ ] Messages arrive when humanoid touches ground?
- [ ] Messages stop when humanoid lifted above ground?
- [ ] Contact force increases when humanoid stands?

---

## Collision Filtering and Bitmasks

Sometimes you don't want all objects to collide. Gazebo uses collision bitmasks—bit patterns that determine what collides with what.

### How Collision Filtering Works

Each collision geometry has a bitmask (16-bit integer). Two objects collide only if their bitmasks have overlapping bits.

```xml
<!-- Only this geometry collides with masks that have bit 0 set -->
<collision name="collision">
  <surface>
    <contact>
      <collide_bitmask>0x0001</collide_bitmask>
    </contact>
  </surface>
</collision>
```

### Common Bitmask Patterns

| Mask | Binary | Meaning |
|------|--------|---------|
| 0xffff | 1111111111111111 | Collides with everything |
| 0x0001 | 0000000000000001 | Layer 0 only |
| 0x0002 | 0000000000000010 | Layer 1 only |
| 0x0003 | 0000000000000011 | Layers 0 and 1 |

### Example: Humanoid-Specific Collision

```xml
<!-- Ground collides with everything -->
<model name="ground">
  <link name="link">
    <collision name="ground_collision">
      <surface>
        <contact>
          <collide_bitmask>0xffff</collide_bitmask>
        </contact>
      </surface>
    </collision>
  </link>
</model>

<!-- Humanoid feet collide with ground -->
<model name="humanoid">
  <link name="left_foot">
    <collision name="left_foot_collision">
      <surface>
        <contact>
          <collide_bitmask>0x0001</collide_bitmask>
        </contact>
      </surface>
    </collision>
  </link>
</model>
```

---

## Collaborative Debugging: Contact Sensing Issues

### Scenario 1: No Contacts Reported

**Observation**: Humanoid walks on ground but `/gazebo/contacts_states` is empty.

**Your attempt** (fails):
```python
subscription = self.node.create_subscription(
    ContactsState,
    '/gazebo/contacts_states',
    self.contact_callback,
    10
)
# Result: No messages, or old messages only
```

### Role 1: AI as Teacher

**You ask**:
```
My humanoid's feet touch ground visually, but no contact sensor
messages. The topic exists but is empty. What usually causes this?
```

**AI teaches**:
- "Contact sensors only publish when collision STATE CHANGES"
- "If humanoid stands continuously, no state change = no message"
- "You need to track state transitions or enable continuous publishing"

**What you learned**: A pattern about event-driven vs continuous publishing.

### Role 2: AI as Student

**You refine**:
```
So if humanoid keeps standing, I won't get messages.
But my controller needs continuous feedback "feet are on ground".
How do I get that?
```

**AI adapts**:
- "Solution 1: Track last contact state and republish"
- "Solution 2: Use a different sensor (IMU foot pressure)"
- "Solution 3: Monitor joint torques"
- "Pattern: When sensor events are insufficient, combine signals"

**What happened**: AI learned your actual requirement.

### Role 3: AI as Co-Worker

**Iteration 1** (you add state tracking):
```python
class ContactTracker:
    def __init__(self):
        self.last_left_contact = False
        self.last_right_contact = False

    def contact_callback(self, msg):
        current_left = self.check_left_contact(msg)
        current_right = self.check_right_contact(msg)

        self.left_foot_contact = current_left
        self.right_foot_contact = current_right
```
Result: Still empty when standing still.

**Iteration 2** (AI suggests):
"Set `always_on: true` and increase `update_rate` in sensor definition"

Result: Messages come continuously now!

**Iteration 3** (you document):
"Found it. The sensor had update_rate: 50. Changed to 100."

**Convergence**: Real issue was sensor configuration.

---

## Practical Exercise: Implement Foot Contact Detection

Create a complete node that detects foot contact and publishes status:

```python
#!/usr/bin/env python3

import rclpy
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool
import time

class FootContactDetector:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('foot_contact_detector')

        self.subscription = self.node.create_subscription(
            ContactsState,
            '/gazebo/contacts_states',
            self.contact_callback,
            10
        )

        self.left_contact_pub = self.node.create_publisher(
            Bool, '/humanoid/left_foot_contact', 10
        )
        self.right_contact_pub = self.node.create_publisher(
            Bool, '/humanoid/right_foot_contact', 10
        )

        self.left_contact = False
        self.right_contact = False

    def contact_callback(self, msg):
        """Process contact messages."""
        for state in msg.states:
            collision1 = state.collision1
            collision2 = state.collision2

            if ('left_foot' in collision1 and 'ground' in collision2) or \
               ('ground' in collision1 and 'left_foot' in collision2):
                self.left_contact = True

            if ('right_foot' in collision1 and 'ground' in collision2) or \
               ('ground' in collision1 and 'right_foot' in collision2):
                self.right_contact = True

        self.publish_status()

    def publish_status(self):
        """Publish current foot contact status."""
        left_msg = Bool()
        left_msg.data = self.left_contact
        self.left_contact_pub.publish(left_msg)

        right_msg = Bool()
        right_msg.data = self.right_contact
        self.right_contact_pub.publish(right_msg)

        self.node.get_logger().debug(
            f"Left: {self.left_contact}, Right: {self.right_contact}"
        )

    def run(self):
        """Main loop."""
        rclpy.spin(self.node)

if __name__ == '__main__':
    detector = FootContactDetector()
    detector.run()
```

Test it:

```bash
# Terminal 1: Gazebo
gz sim -r training_world.sdf

# Terminal 2: Spawn humanoid
python3 spawn_humanoid.py

# Terminal 3: Run detector
python3 foot_contact_detector.py

# Terminal 4: Monitor output
ros2 topic echo /humanoid/left_foot_contact
# Should show: True (when touching), False (when not)
```

Success criteria:
- [ ] left_foot_contact publishes True when left foot on ground
- [ ] left_foot_contact publishes False when left foot in air
- [ ] Same for right_foot_contact

---

## Advanced: Force-Based Contact Detection

Contact state alone isn't enough for robust control. Also monitor force magnitude:

```python
class AdvancedFootContactDetector:
    def __init__(self, force_threshold=1.0):
        # ... initialization ...
        self.force_threshold = force_threshold
        self.left_force = 0
        self.right_force = 0

    def contact_callback(self, msg):
        self.left_force = 0
        self.right_force = 0

        for state in msg.states:
            force_z = state.total_wrench.force.z

            if ('left_foot' in state.collision1 and 'ground' in state.collision2):
                self.left_force = max(self.left_force, force_z)

            if ('right_foot' in state.collision1 and 'ground' in state.collision2):
                self.right_force = max(self.right_force, force_z)

        self.publish_status()

    def publish_status(self):
        left_in_contact = self.left_force > self.force_threshold
        right_in_contact = self.right_force > self.force_threshold
        # Publish state and force values
```

This approach handles noise better—small vibrations won't trigger false contacts.

---

## Try With AI

**Prompt 1: Contact Sensor Challenges**
```
I've added contact sensors to humanoid feet, but I'm getting
intermittent messages even when feet are firmly on ground.
What causes this, and how do I get reliable continuous signals?
```

**Prompt 2: Real vs Simulated**
```
In my real humanoid, foot contact is detected via pressure sensors.
In simulation, I use Gazebo contact sensors. How do their
approaches differ? Will simulation results transfer to hardware?
```

**Prompt 3: Multi-Sensor Fusion**
```
I have contact sensors on feet, and I also have an IMU. For robust
"is the humanoid on ground" detection, should I use just one sensor
or combine both?
```

---

**Next: Proceed to Lesson 6: Joint Control and Humanoid Movement**

In Lesson 6, you'll move from sensing to control. You'll create a reusable skill for commanding humanoid joints via ROS 2 trajectory controllers—your first Layer 3 (Intelligence Design) work.
