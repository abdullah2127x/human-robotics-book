# Quick Start: Module 1: The Robotic Nervous System (ROS 2)

## Prerequisites

Before starting with this module, ensure you have the following prerequisites:

### System Requirements
- Ubuntu 22.04 LTS (recommended OS for ROS 2 Humble)
- At least 4GB RAM and 20GB free disk space
- Internet connection for package installation

### Software Prerequisites
- Basic Python 3.10+ programming knowledge
- Linux command line familiarity
- Git version control system

## Environment Setup

### 1. Install ROS 2 Humble Hawksbill

```bash
# Set locale
locale  # verify LANG=en_US.UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 GPG key
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS 2
sudo apt update
sudo apt install ros-humble-desktop
```

### 2. Install ROS 2 Python tools
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update
```

### 3. Source ROS 2 environment
```bash
source /opt/ros/humble/setup.bash
# Add to your ~/.bashrc to auto-source on terminal opening
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Creating Your First ROS 2 Package

### 1. Create a workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Create a basic ROS 2 package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_tutorials
```

### 3. Basic Publisher Node Structure
Create `~/ros2_ws/src/my_robot_tutorials/my_robot_tutorials/publisher_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Basic Subscriber Node Structure
Create `~/ros2_ws/src/my_robot_tutorials/my_robot_tutorials/subscriber_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5. Build and run your nodes
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_tutorials
source install/setup.bash
```

Terminal 1 (publisher):
```bash
ros2 run my_robot_tutorials publisher_member_function
```

Terminal 2 (subscriber):
```bash
ros2 run my_robot_tutorials subscriber_member_function
```

## Verifying Communication

### 1. Check available topics
```bash
ros2 topic list
```

### 2. Echo topic data
```bash
ros2 topic echo /topic std_msgs/msg/String
```

### 3. Check node connections
```bash
ros2 run rqt_graph rqt_graph
```

## Launch Files

### 1. Create a Python launch file
Create `~/ros2_ws/src/my_robot_tutorials/launch/two_nodes_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_tutorials',
            executable='publisher_member_function',
            name='publisher',
            output='screen'
        ),
        Node(
            package='my_robot_tutorials',
            executable='subscriber_member_function',
            name='subscriber',
            output='screen'
        )
    ])
```

### 2. Launch the system
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_robot_tutorials two_nodes_launch.py
```

## Creating the ROS 2 Sensor Node Template

### 1. Template Structure
Create `~/ros2_ws/src/my_robot_tutorials/my_robot_tutorials/sensor_node_template.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # or appropriate sensor message type
from std_msgs.msg import String

class ROSSensorNodeTemplate(Node):
    """
    Reusable ROS 2 Sensor Node Template

    This template provides a foundation for creating sensor nodes that can be
    customized for different sensor types. The template includes placeholder
    methods for initialization, data acquisition, and message publishing.
    """

    def __init__(self, node_name='sensor_node'):
        super().__init__(node_name)

        # Initialize sensor-specific parameters
        self.initialize_sensor_parameters()

        # Create publisher for sensor data
        self.sensor_publisher = self.create_publisher(
            LaserScan,  # Change to appropriate message type
            'sensor_data',
            10
        )

        # Timer for periodic sensor reading
        self.timer = self.create_timer(
            0.1,  # Adjust timing as needed
            self.acquire_and_publish_data
        )

        self.get_logger().info(f'{node_name} initialized')

    def initialize_sensor_parameters(self):
        """Placeholder method for sensor-specific initialization"""
        # Override this method in subclass to initialize sensor parameters
        pass

    def acquire_sensor_data(self):
        """Placeholder method for acquiring sensor data"""
        # Override this method in subclass to acquire actual sensor data
        # This is where you would interface with the physical sensor
        return None

    def format_sensor_message(self, raw_data):
        """Placeholder method for formatting sensor data into ROS message"""
        # Override this method in subclass to format data appropriately
        # for the specific sensor message type
        return None

    def acquire_and_publish_data(self):
        """Acquire data and publish it to the topic"""
        raw_data = self.acquire_sensor_data()
        if raw_data is not None:
            formatted_msg = self.format_sensor_message(raw_data)
            if formatted_msg is not None:
                self.sensor_publisher.publish(formatted_msg)
                self.get_logger().info('Published sensor data')

def main(args=None):
    rclpy.init(args=args)
    sensor_node = ROSSensorNodeTemplate()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Using AI for Development

### 1. Three Roles Framework for AI Collaboration
When using AI assistants for code development, apply the Three Roles Framework:

1. **Validator Role**: Ask AI to review and validate your code for correctness
2. **Editor Role**: Ask AI to modify existing code to meet specific requirements
3. **Prompt Engineer Role**: Collaborate with AI to refine prompts for better results

### 2. Example AI Prompt for Launch File Creation
"Create a ROS 2 launch file that starts a publisher, subscriber, and a service server with proper error handling and logging."

## Development Workflow

### 1. Layer 1: Manual Foundation
- Create basic Publisher and Subscriber nodes manually
- Verify package creation and setup.py configuration
- Test topic echoing to verify data flow

### 2. Layer 2: AI Collaboration
- Use AI to create complex launch files
- Apply debugging strategies to large ROS 2 graphs
- Use the Three Roles Framework to correct AI-generated code

### 3. Layer 3: Intelligence Design
- Transform learned patterns into the reusable ROS 2 Sensor Node Template
- Ensure proper class structure with placeholder methods
- Add comprehensive documentation for future reuse

### 4. Layer 4: Spec-Driven Integration
- Write system specification before implementation
- Create multi-node motor controller system
- Integrate all learned concepts in a cohesive project

## Troubleshooting

### Common Issues:
1. **Nodes not connecting**: Ensure both nodes are on the same network and using the same ROS_DOMAIN_ID
2. **Import errors**: Make sure your package is built and sourced (`source install/setup.bash`)
3. **Permission errors**: Check file permissions on your workspace

### Debugging Commands:
```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Check topic info
ros2 topic info /topic_name

# Check node info
ros2 node info /node_name
```

## Next Steps

1. Complete the Module Guide File (docs/module1/index.mdx)
2. Create the Submodule Guide Files for each topic area
3. Develop content files following the 4-layer pedagogical arc
4. Build and test all examples against the target environment
5. Create the final reusable ROS 2 Sensor Node Template