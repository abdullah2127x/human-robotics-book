import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Book sidebar configuration for Physical AI & Humanoid Robotics
 *
 * Structure:
 * - Introduction
 * - Part 1: The Robotic Nervous System (ROS 2)
 *   - Chapter 1: ROS 2 Nodes, Topics, and Services (7 lessons)
 *   - Chapter 2: Bridging Python Agents to ROS Controllers (7 lessons)
 *   - Chapter 3: Understanding URDF for Humanoids (8 lessons)
 * - Part 2: The Digital Twin
 *   - Chapter 4: Simulating Physics in Gazebo (8 lessons)
 *   - Chapter 5: High-Fidelity Rendering and HRI in Unity (8 lessons)
 *   - Chapter 6: Simulating Sensors (9 lessons)
 * - Part 3: Advanced Simulation & Perception
 *   - Chapter 7: NVIDIA Isaac Sim (10 lessons)
 *   - Chapter 8: Isaac ROS Visual SLAM (8 lessons)
 *   - Chapter 9: Nav2 for Humanoid Navigation (9 lessons)
 *
 * Note: Docusaurus strips number prefixes from folder names when generating IDs
 */
const sidebars: SidebarsConfig = {
  bookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Part 1: The Robotic Nervous System',
      link: {
        type: 'doc',
        id: 'Part-1-ROS2-Foundation/index',
      },
      items: [
        {
          type: 'category',
          label: 'Chapter 1: ROS 2 Nodes, Topics, and Services',
          link: {
            type: 'doc',
            id: 'Part-1-ROS2-Foundation/ros2-nodes-topics-services/index',
          },
          items: [
            'Part-1-ROS2-Foundation/ros2-nodes-topics-services/introduction',
            'Part-1-ROS2-Foundation/ros2-nodes-topics-services/understanding-nodes',
            'Part-1-ROS2-Foundation/ros2-nodes-topics-services/topics-and-messages',
            'Part-1-ROS2-Foundation/ros2-nodes-topics-services/publishers-and-subscribers',
            'Part-1-ROS2-Foundation/ros2-nodes-topics-services/services-and-clients',
            'Part-1-ROS2-Foundation/ros2-nodes-topics-services/quality-of-service',
            'Part-1-ROS2-Foundation/ros2-nodes-topics-services/capstone-integration',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Bridging Python Agents to ROS Controllers',
          link: {
            type: 'doc',
            id: 'Part-1-ROS2-Foundation/rclpy-python-development/index',
          },
          items: [
            'Part-1-ROS2-Foundation/rclpy-python-development/package-structure',
            'Part-1-ROS2-Foundation/rclpy-python-development/async-callbacks',
            'Part-1-ROS2-Foundation/rclpy-python-development/action-clients',
            'Part-1-ROS2-Foundation/rclpy-python-development/custom-messages',
            'Part-1-ROS2-Foundation/rclpy-python-development/executors-concurrency',
            'Part-1-ROS2-Foundation/rclpy-python-development/reusable-patterns',
            'Part-1-ROS2-Foundation/rclpy-python-development/capstone-python-agent',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Understanding URDF for Humanoids',
          link: {
            type: 'doc',
            id: 'Part-1-ROS2-Foundation/urdf-humanoid-modeling/index',
          },
          items: [
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/urdf-fundamentals',
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/collision-inertial',
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/joints',
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/inertia-calculations',
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/transform-frames',
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/articulated-arm',
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/xacro-patterns',
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/capstone-humanoid',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Part 2: The Digital Twin',
      link: {
        type: 'doc',
        id: 'Part-2-Digital-Twin/index',
      },
      items: [
        {
          type: 'category',
          label: 'Chapter 4: Simulating Physics in Gazebo',
          link: {
            type: 'doc',
            id: 'Part-2-Digital-Twin/gazebo-physics/index',
          },
          items: [
            'Part-2-Digital-Twin/gazebo-physics/gazebo-architecture',
            'Part-2-Digital-Twin/gazebo-physics/world-files',
            'Part-2-Digital-Twin/gazebo-physics/model-spawning',
            'Part-2-Digital-Twin/gazebo-physics/physics-tuning',
            'Part-2-Digital-Twin/gazebo-physics/collision-detection',
            'Part-2-Digital-Twin/gazebo-physics/joint-control',
            'Part-2-Digital-Twin/gazebo-physics/debugging-optimization',
            'Part-2-Digital-Twin/gazebo-physics/capstone-humanoid-balance',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 5: High-Fidelity Rendering and HRI in Unity',
          link: {
            type: 'doc',
            id: 'Part-2-Digital-Twin/unity-hri/index',
          },
          items: [
            'Part-2-Digital-Twin/unity-hri/unity-ros-bridge',
            'Part-2-Digital-Twin/unity-hri/urdf-import',
            'Part-2-Digital-Twin/unity-hri/environment-design',
            'Part-2-Digital-Twin/unity-hri/human-avatars',
            'Part-2-Digital-Twin/unity-hri/interaction-scripting',
            'Part-2-Digital-Twin/unity-hri/ros-integration',
            'Part-2-Digital-Twin/unity-hri/scene-management',
            'Part-2-Digital-Twin/unity-hri/capstone-hri-demo',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 6: Simulating Sensors',
          link: {
            type: 'doc',
            id: 'Part-2-Digital-Twin/sensors-simulation/index',
          },
          items: [
            'Part-2-Digital-Twin/sensors-simulation/sensor-architecture',
            'Part-2-Digital-Twin/sensors-simulation/urdf-sensors',
            'Part-2-Digital-Twin/sensors-simulation/lidar-simulation',
            'Part-2-Digital-Twin/sensors-simulation/depth-camera',
            'Part-2-Digital-Twin/sensors-simulation/imu-sensor',
            'Part-2-Digital-Twin/sensors-simulation/noise-models',
            'Part-2-Digital-Twin/sensors-simulation/rviz-visualization',
            'Part-2-Digital-Twin/sensors-simulation/sensor-processing',
            'Part-2-Digital-Twin/sensors-simulation/capstone-sensor-suite',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Part 3: Advanced Simulation & Perception',
      link: {
        type: 'doc',
        id: 'Part-3-Advanced-Simulation-Perception/index',
      },
      items: [
        {
          type: 'category',
          label: 'Chapter 7: NVIDIA Isaac Sim',
          link: {
            type: 'doc',
            id: 'Part-3-Advanced-Simulation-Perception/isaac-sim/index',
          },
          items: [
            'Part-3-Advanced-Simulation-Perception/isaac-sim/isaac-sim-installation',
            'Part-3-Advanced-Simulation-Perception/isaac-sim/omniverse-architecture',
            'Part-3-Advanced-Simulation-Perception/isaac-sim/urdf-import',
            'Part-3-Advanced-Simulation-Perception/isaac-sim/photorealistic-rendering',
            'Part-3-Advanced-Simulation-Perception/isaac-sim/domain-randomization',
            'Part-3-Advanced-Simulation-Perception/isaac-sim/replicator-synthetic-data',
            'Part-3-Advanced-Simulation-Perception/isaac-sim/ros2-bridge',
            'Part-3-Advanced-Simulation-Perception/isaac-sim/performance-optimization',
            'Part-3-Advanced-Simulation-Perception/isaac-sim/domain-randomization-skill',
            'Part-3-Advanced-Simulation-Perception/isaac-sim/capstone-training-dataset',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 8: Isaac ROS Visual SLAM',
          link: {
            type: 'doc',
            id: 'Part-3-Advanced-Simulation-Perception/isaac-ros-vslam/index',
          },
          items: [
            'Part-3-Advanced-Simulation-Perception/isaac-ros-vslam/vslam-fundamentals',
            'Part-3-Advanced-Simulation-Perception/isaac-ros-vslam/isaac-ros-installation',
            'Part-3-Advanced-Simulation-Perception/isaac-ros-vslam/visual-odometry',
            'Part-3-Advanced-Simulation-Perception/isaac-ros-vslam/cuda-acceleration',
            'Part-3-Advanced-Simulation-Perception/isaac-ros-vslam/loop-closure',
            'Part-3-Advanced-Simulation-Perception/isaac-ros-vslam/rviz-visualization',
            'Part-3-Advanced-Simulation-Perception/isaac-ros-vslam/vslam-debugging',
            'Part-3-Advanced-Simulation-Perception/isaac-ros-vslam/capstone-realtime-vslam',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 9: Nav2 for Humanoid Navigation',
          link: {
            type: 'doc',
            id: 'Part-3-Advanced-Simulation-Perception/nav2-humanoid/index',
          },
          items: [
            'Part-3-Advanced-Simulation-Perception/nav2-humanoid/nav2-architecture',
            'Part-3-Advanced-Simulation-Perception/nav2-humanoid/costmaps',
            'Part-3-Advanced-Simulation-Perception/nav2-humanoid/path-planners',
            'Part-3-Advanced-Simulation-Perception/nav2-humanoid/controllers',
            'Part-3-Advanced-Simulation-Perception/nav2-humanoid/behavior-trees',
            'Part-3-Advanced-Simulation-Perception/nav2-humanoid/dynamic-obstacles',
            'Part-3-Advanced-Simulation-Perception/nav2-humanoid/vslam-integration',
            'Part-3-Advanced-Simulation-Perception/nav2-humanoid/behavior-tree-skill',
            'Part-3-Advanced-Simulation-Perception/nav2-humanoid/capstone-autonomous-navigation',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Part 4: Vision-Language-Action',
      link: {
        type: 'doc',
        id: 'Part-4-Vision-Language-Action/index',
      },
      items: [
        {
          type: 'category',
          label: 'Chapter 10: Voice-to-Action',
          link: {
            type: 'doc',
            id: 'Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/index',
          },
          items: [
            'Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/prerequisites',
            'Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-01-speech-recognition-fundamentals',
            'Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-02-setting-up-whisper',
            'Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-03-real-time-audio-capture',
            'Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-04-voice-activity-detection',
            'Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-05-designing-intent-schemas',
            'Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-06-implementing-intent-parsers',
            'Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-07-ros2-integration',
            'Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-08-voice-intent-parsing-skill',
            'Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-09-capstone-voice-controlled-navigation',
            'Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/summary',
          ],
        },
      ],
    },
    
  ],
};

export default sidebars;
