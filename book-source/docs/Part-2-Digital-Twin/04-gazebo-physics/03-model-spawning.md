---
sidebar_position: 3
title: "Lesson 3: Spawning and Controlling Models"
description: "Spawn URDF models into running Gazebo simulations using spawn_entity service with AI collaboration."
---

# Lesson 3: Spawning and Controlling Models

## Learning Objectives

By completing this lesson, you will:
- Use ROS 2 spawn_entity service to dynamically add models to simulation
- Specify initial poses for spawned models
- Manage namespaces to avoid naming conflicts
- Debug common spawning failures
- Work with AI to troubleshoot simulation issues

**Estimated time**: 120 minutes

---

## Why Spawn Models Dynamically?

In Lesson 2, you wrote world files that contained all objects at startup. That approach works for static environments, but humanoid robotics needs flexibility:
- Testing with multiple robots at different starting positions
- Adding obstacles during simulation
- Spawning new objects without restarting
- Automating test scenarios

ROS 2 provides the `spawn_entity` service for exactly this. Your controller nodes can spawn models programmatically.

---

## Understanding spawn_entity

The `spawn_entity` service is provided by Gazebo's ROS 2 bridge. It accepts:
- **name**: Unique identifier for the model
- **xml**: URDF or SDF string defining the model
- **initial_pose**: Starting position and orientation
- **reference_frame**: Which coordinate frame (usually "world")

---

## Starting Point: Spawn a Simple Box

Before using your humanoid URDF, let's verify spawning works with a simple shape.

### Minimal World File

Create `spawn_test_world.sdf` (no objects inside, just empty ground):

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="spawn_test">
    <physics name="default_physics" default="true" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.4 0.4 0.4</ambient>
      <background>0.7 0.7 0.7</background>
    </scene>

    <light type="directional" name="sun">
      <pose>5 5 5 0 0 0</pose>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>100 100 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>100 100 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

Launch it:

```bash
gz sim -r spawn_test_world.sdf
```

### Spawn a Box via Command Line

In a new terminal:

```bash
ros2 service call /spawn_entity gazebo_msgs/SpawnEntity \
  "{name: my_box, \
    xml: \"<sdf version='1.10'><model name='my_box'><link name='link'><inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial><collision name='collision'><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision><visual name='visual'><geometry><box><size>0.1 0.1 0.1</size></box></geometry><material><ambient>1 0 0 1</ambient></material></visual></link></model></sdf>\", \
    initial_pose: {position: {x: 0, y: 0, z: 1}, \
                   orientation: {x: 0, y: 0, z: 0, w: 1}}}"
```

**Result**: A red box appears at height 1m and falls to the ground.

The XML is wrapped in quotes and escaped. For real code, this is painful. Let's improve it.

---

## Practical Approach: Spawn from Python

Writing complex ROS 2 service calls in bash is error-prone. Python is clearer. Here's a reusable pattern.

### Python Spawning Script

Create `spawn_models.py`:

```python
#!/usr/bin/env python3

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import json

def spawn_model(node, model_name: str, urdf_path: str, pose_dict: dict):
    """
    Spawn a model into Gazebo simulation.

    Args:
        node: ROS 2 node
        model_name: Unique name for the model
        urdf_path: Path to URDF or SDF file
        pose_dict: {'x': float, 'y': float, 'z': float,
                    'roll': float, 'pitch': float, 'yaw': float}
    """
    # Read XML from file
    with open(urdf_path, 'r') as f:
        model_xml = f.read()

    # Build pose
    pose = Pose()
    pose.position.x = pose_dict.get('x', 0)
    pose.position.y = pose_dict.get('y', 0)
    pose.position.z = pose_dict.get('z', 0)

    # Convert RPY to quaternion (simplified - assumes small angles)
    import math
    roll = pose_dict.get('roll', 0)
    pitch = pose_dict.get('pitch', 0)
    yaw = pose_dict.get('yaw', 0)

    # Quaternion from RPY
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    pose.orientation.w = cr * cp * cy + sr * sp * sy
    pose.orientation.x = sr * cp * cy - cr * sp * sy
    pose.orientation.y = cr * sp * cy + sr * cp * sy
    pose.orientation.z = cr * cp * sy - sr * sp * cy

    # Call service
    client = node.create_client(SpawnEntity, '/spawn_entity')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('spawn_entity service not available, waiting...')

    request = SpawnEntity.Request()
    request.name = model_name
    request.xml = model_xml
    request.initial_pose = pose
    request.reference_frame = "world"

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result().success:
        node.get_logger().info(f'Successfully spawned {model_name}')
        return True
    else:
        node.get_logger().error(f'Failed to spawn {model_name}: {future.result().status_message}')
        return False

def main():
    rclpy.init()
    node = rclpy.create_node('spawner')

    # Example 1: Spawn a box at origin
    spawn_model(
        node,
        model_name="test_box_1",
        urdf_path="/path/to/simple_box.urdf",
        pose_dict={'x': 0, 'y': 0, 'z': 1}
    )

    # Example 2: Spawn another box offset
    spawn_model(
        node,
        model_name="test_box_2",
        urdf_path="/path/to/simple_box.urdf",
        pose_dict={'x': 1, 'y': 0, 'z': 1}
    )

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Create Simple Test URDF

Create `simple_box.urdf`:

```xml
<?xml version="1.0" ?>
<robot name="box">
  <link name="link">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>
</robot>
```

Test:

```bash
# Terminal 1: Gazebo
gz sim -r spawn_test_world.sdf

# Terminal 2: Run spawner
python3 spawn_models.py
```

**Observation**: Two red boxes should appear and fall to ground.

---

## Common Failures and Debugging

Spawning often fails silently. Here's how to diagnose and fix.

### Failure 1: File Not Found

**Error**: `status_message: "Error: Unable to find resource file"`

**Cause**: The URDF path is relative, but spawn_entity executes from a different directory.

**Solution**: Use absolute paths.

```python
import os
urdf_path = os.path.abspath("/home/user/humanoid.urdf")
spawn_model(node, "humanoid", urdf_path, {'z': 1})
```

### Failure 2: Duplicate Name

**Error**: `success: False`, `status_message: "Model already exists"`

**Cause**: You spawned a model with the same name twice.

**Solution**: Use unique names, add a counter or UUID.

```python
import time
model_name = f"humanoid_{int(time.time()*1000)}"
spawn_model(node, model_name, urdf_path, {'z': 1})
```

### Failure 3: Objects Overlapping

**Error**: No error, but spawned object sinks through ground or other objects.

**Cause**: Initial pose overlaps with existing collision geometry.

**Solution**: Choose non-overlapping spawn positions.

```python
for i in range(3):
    spawn_model(
        node,
        model_name=f"humanoid_{i}",
        urdf_path=humanoid_urdf,
        pose_dict={'x': i * 2, 'y': 0, 'z': 1}
    )
```

### Failure 4: Spawn Timing Issues

**Error**: Model spawns at wrong position, or doesn't appear until after movement commands sent.

**Cause**: Gazebo world isn't fully ready when you spawn. Or ROS 2 node tries to spawn before service exists.

**Solution**: Wait for world, check services before spawning.

```python
import time
import rclpy
from rclpy.action import ActionClient

def wait_for_spawn_service(node, timeout=10.0):
    """Wait for spawn_entity service to be available."""
    client = node.create_client(SpawnEntity, '/spawn_entity')
    start = time.time()
    while not client.wait_for_service(timeout_sec=1.0):
        elapsed = time.time() - start
        if elapsed > timeout:
            raise TimeoutError(f"spawn_entity not available after {timeout}s")
        node.get_logger().info('Waiting for spawn_entity service...')
    node.get_logger().info('spawn_entity service is ready')

def main():
    rclpy.init()
    node = rclpy.create_node('spawner')

    wait_for_spawn_service(node)
    spawn_model(node, "humanoid", humanoid_urdf, {'z': 1})

    rclpy.shutdown()
```

---

## Working with Your Humanoid URDF

Now apply spawning to your humanoid from Chapter 3.

### Spawn Your Humanoid

Assuming you have `humanoid.urdf` from Chapter 3:

```python
#!/usr/bin/env python3

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import math
import os

def quaternion_from_rpy(roll, pitch, yaw):
    """Convert roll-pitch-yaw to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w

def spawn_humanoid():
    rclpy.init()
    node = rclpy.create_node('humanoid_spawner')

    # Read humanoid URDF
    urdf_path = os.path.expanduser("~/humanoid.urdf")
    with open(urdf_path, 'r') as f:
        urdf_xml = f.read()

    # Create spawn request
    client = node.create_client(SpawnEntity, '/spawn_entity')
    client.wait_for_service()

    request = SpawnEntity.Request()
    request.name = "humanoid_robot"
    request.xml = urdf_xml

    # Set starting pose: standing on ground
    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 1.0

    x, y, z, w = quaternion_from_rpy(0, 0, 0)
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w

    request.initial_pose = pose
    request.reference_frame = "world"

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    if result.success:
        print(f"✓ Humanoid spawned successfully at ({pose.position.x}, {pose.position.y}, {pose.position.z})")
    else:
        print(f"✗ Failed to spawn humanoid: {result.status_message}")

    rclpy.shutdown()

if __name__ == '__main__':
    spawn_humanoid()
```

Run this:

```bash
# Terminal 1: Gazebo
gz sim -r spawn_test_world.sdf

# Terminal 2: Spawn humanoid
python3 spawn_humanoid.py
```

**Success criteria**:
- [ ] Humanoid appears in simulation
- [ ] Humanoid is oriented correctly (standing upright)
- [ ] Humanoid feet are above ground (z=1 is reasonable, adjust if needed)
- [ ] No penetration or intersection with ground

---

## Try With AI

**Prompt 1: Common Spawning Errors**
```
What are the top 5 most common errors when spawning URDF models
into Gazebo simulations? For each, explain the cause and solution.
```

**Prompt 2: Your Humanoid**
```
I have a humanoid URDF from my robotics project. When I spawn it
into Gazebo at z=1.0, sometimes it penetrates the ground and
sometimes it floats above it. What causes this inconsistency?
```

**Prompt 3: Debugging Your Script**
```
My spawn script works the first time, but the second run fails with
"model already exists". I'm trying to spawn multiple humanoids in
one test. How do I fix the naming to allow multiple spawns?
```

---

**Next: Proceed to Lesson 4: Physics Parameter Tuning**

In Lesson 4, you'll take the spawned humanoid and tune its physics parameters to make it stand realistically through collaborative iteration with AI.
