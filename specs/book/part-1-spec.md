# Part 1 Specification: The Robotic Nervous System (ROS 2)

**Version:** 1.0.0
**Created:** 2025-12-16
**Status:** Ready for chapter-planner
**Parent:** specs/book/book-spec.md

---

## Part Identity

**Part Title:** The Robotic Nervous System

**Part Number:** 1 of 4

**Technology Focus:** ROS 2 (Robot Operating System 2)

**Unifying Narrative:** "Building the Communication Backbone" - Just as the human nervous system coordinates signals between brain and body, ROS 2 orchestrates communication between all robotic components. Master this, and you understand how robots "think."

---

## Part Purpose

**Why This Part Exists:**
- Establishes the foundational middleware that ALL subsequent parts depend on
- Teaches the "language" robots use to communicate internally
- Provides hands-on experience with real robotics tools from day one

**What Readers Gain:**
- Ability to create ROS 2 nodes that communicate
- Python skills for robot control (rclpy)
- Understanding of robot physical structure (URDF)

**How This Prepares for Later Parts:**
- Part 2: Simulation requires ROS 2 nodes to control robots in Gazebo/Unity
- Part 3: Isaac ROS builds directly on ROS 2 foundation
- Part 4: Voice commands translate to ROS 2 action sequences

---

## Pedagogical Strategy

### Cognitive Load
**Level:** Light → Moderate

| Chapter | Load | Reasoning |
|---------|------|-----------|
| Chapter 1 | Light | New concepts but familiar patterns (pub/sub) |
| Chapter 2 | Light-Moderate | Python integration adds complexity |
| Chapter 3 | Moderate | 3D thinking, XML syntax, physics properties |

### Scaffolding Level
**Level:** Heavy

- Step-by-step guidance for every exercise
- Show-then-explain pattern throughout
- Zero assumptions about robotics background
- Frequent checkpoints to verify understanding

### Concept Density
**Target:** 5-7 key concepts per chapter

- Allows time for absorption
- Prevents cognitive overload for beginners
- Each concept gets dedicated practice

---

## Chapter Specifications

### Chapter 1: ROS 2 Nodes, Topics, and Services

**Chapter Purpose:** Introduce the core ROS 2 architecture - how robotic components communicate.

**Unifying Metaphor:** "The Nervous System" - nodes are neurons, topics are nerve pathways, messages are electrical signals.

#### Learning Outcomes (Bloom's Taxonomy Aligned)

| ID | Outcome | Bloom's Level | Success Criteria |
|----|---------|---------------|------------------|
| LO-1.1 | Explain ROS 2 architecture (nodes, topics, services) | Understand | Can diagram the relationship between nodes, topics, and services |
| LO-1.2 | Create and run ROS 2 nodes | Apply | Successfully creates a node that prints "Hello Robot" |
| LO-1.3 | Implement publishers and subscribers | Apply | Creates pub/sub pair that exchanges messages |
| LO-1.4 | Create and call ROS 2 services | Apply | Implements service server and client |
| LO-1.5 | Configure Quality of Service (QoS) | Apply | Modifies QoS settings and observes behavior changes |
| LO-1.6 | Debug using ROS 2 CLI tools | Apply | Uses ros2 topic, ros2 node, ros2 service to inspect running system |

#### Key Concepts (5-6 concepts)

| Concept | Definition | Why It Matters |
|---------|------------|----------------|
| Node | Independent process that performs computation | Building block of all ROS 2 systems |
| Topic | Named bus for many-to-many messaging | Enables loose coupling between components |
| Publisher | Node that sends messages to a topic | How sensors share data |
| Subscriber | Node that receives messages from a topic | How processors consume data |
| Service | Synchronous request-response pattern | For queries that need immediate answers |
| QoS | Quality of Service policies | Controls reliability vs performance tradeoff |

#### Hands-On Exercises

**Exercise 1.1: Hello Robot Node**
- Task: Create a node that publishes "Hello Robot" every second
- Success: Message appears in `ros2 topic echo`
- Time: 15 minutes

**Exercise 1.2: Temperature Sensor Simulation**
- Task: Create publisher (sensor) and subscriber (display) pair
- Success: Display shows temperature readings from sensor
- Time: 30 minutes

**Exercise 1.3: Robot Status Service**
- Task: Create service that returns robot battery level
- Success: Client receives battery percentage from server
- Time: 30 minutes

**Exercise 1.4: CLI Exploration**
- Task: Use `ros2 topic list`, `ros2 node info`, `ros2 service call`
- Success: Can inspect any running ROS 2 system
- Time: 20 minutes

#### Prerequisites
- Python 3.10+ installed
- Ubuntu 22.04 or WSL2
- ROS 2 Humble installed
- Basic Python knowledge (functions, classes)

#### Chapter Dependencies
- None (this is the first chapter)

#### Prepares For
- Chapter 2: Python patterns used here extend to rclpy
- Chapter 3: Nodes will load and publish URDF data
- All future chapters: Everything uses ROS 2 communication

---

### Chapter 2: Bridging Python Agents to ROS Controllers (rclpy)

**Chapter Purpose:** Connect Python AI agents to the robotic world using rclpy.

**Unifying Metaphor:** "The Translator" - rclpy translates Python decisions into robot commands, bridging the AI world and the physical world.

#### Learning Outcomes (Bloom's Taxonomy Aligned)

| ID | Outcome | Bloom's Level | Success Criteria |
|----|---------|---------------|------------------|
| LO-2.1 | Structure ROS 2 Python packages | Apply | Creates valid package with setup.py, package.xml |
| LO-2.2 | Implement async/await patterns in ROS 2 | Apply | Node handles multiple callbacks without blocking |
| LO-2.3 | Create action clients for long-running tasks | Apply | Client sends goal, receives feedback, gets result |
| LO-2.4 | Work with standard and custom message types | Apply | Defines custom message and uses it in pub/sub |
| LO-2.5 | Build Python agent that controls robot | Apply | Agent makes decisions and sends commands to simulated arm |
| LO-2.6 | Handle callbacks and executors | Understand | Can explain single vs multi-threaded executor tradeoffs |

#### Key Concepts (6 concepts)

| Concept | Definition | Why It Matters |
|---------|------------|----------------|
| rclpy | ROS Client Library for Python | The bridge between Python and ROS 2 |
| Action | Long-running task with feedback | For navigation, manipulation (takes time) |
| Message Type | Data structure for communication | Defines what information flows between nodes |
| Callback | Function triggered by events | How nodes react to incoming data |
| Executor | Manages callback execution | Controls concurrency model |
| Package | Unit of ROS 2 code organization | How code is structured and shared |

#### Hands-On Exercises

**Exercise 2.1: Create ROS 2 Python Package**
- Task: Create package with publisher node using `ros2 pkg create`
- Success: Package builds and node runs
- Time: 20 minutes

**Exercise 2.2: Async Callback Handling**
- Task: Create node with timer + subscriber that don't block each other
- Success: Both callbacks execute independently
- Time: 30 minutes

**Exercise 2.3: Action Client for Arm Movement**
- Task: Send goal to move robot arm, monitor feedback, receive result
- Success: Arm moves through waypoints with progress feedback
- Time: 45 minutes

**Exercise 2.4: Custom Message Type**
- Task: Define RobotStatus.msg with battery, temperature, state fields
- Success: Publish and subscribe using custom message
- Time: 30 minutes

#### Prerequisites
- Completed Chapter 1
- Python async/await basic understanding
- Familiarity with Python classes

#### Chapter Dependencies
- Chapter 1: Nodes, topics, services concepts

#### Prepares For
- Chapter 3: Launch files use Python, URDF loading uses rclpy
- Part 2: All simulation control uses rclpy
- Part 4: LLM outputs execute via rclpy action clients

---

### Chapter 3: Understanding URDF for Humanoids

**Chapter Purpose:** Define robot anatomy using URDF - the skeleton that gives robots physical form.

**Unifying Metaphor:** "The Skeleton" - URDF defines the bones (links) and joints that make a robot body, just like a skeleton defines human structure.

#### Learning Outcomes (Bloom's Taxonomy Aligned)

| ID | Outcome | Bloom's Level | Success Criteria |
|----|---------|---------------|------------------|
| LO-3.1 | Write valid URDF XML syntax | Apply | Creates URDF that passes `check_urdf` validation |
| LO-3.2 | Define links with visual, collision, inertial properties | Apply | Link appears correctly in RViz with all three property types |
| LO-3.3 | Create joints (revolute, prismatic, fixed) | Apply | Joints move correctly with expected limits |
| LO-3.4 | Calculate inertial properties | Apply | Uses formulas to compute mass, inertia for basic shapes |
| LO-3.5 | Visualize URDF in RViz | Apply | Model loads and displays correctly in RViz |
| LO-3.6 | Build humanoid torso and arm | Create | Complete upper body with shoulder, elbow, wrist joints |

#### Key Concepts (7 concepts)

| Concept | Definition | Why It Matters |
|---------|------------|----------------|
| Link | Rigid body in the robot | The "bones" - torso, arm segments, hands |
| Joint | Connection between links | Defines how parts move relative to each other |
| Visual Geometry | What robot looks like | For visualization and debugging |
| Collision Geometry | Simplified shape for physics | For collision detection in simulation |
| Inertial Properties | Mass, center of mass, inertia tensor | Required for realistic physics simulation |
| Joint Types | Revolute, prismatic, fixed, continuous | Different movement patterns |
| TF (Transform) | Coordinate frame relationships | How positions relate across the robot |

#### Hands-On Exercises

**Exercise 3.1: Simple Box Link**
- Task: Create single link with visual, collision, and inertial
- Success: Box appears in RViz with correct dimensions
- Time: 20 minutes

**Exercise 3.2: Two Links with Joint**
- Task: Connect two boxes with revolute joint
- Success: Joint moves in RViz joint_state_publisher
- Time: 25 minutes

**Exercise 3.3: Calculate Inertia**
- Task: Compute inertia tensor for cylinder (arm segment)
- Success: Values match formula results
- Time: 15 minutes

**Exercise 3.4: Humanoid Torso**
- Task: Create torso with head attachment (fixed joint)
- Success: Torso and head display as connected body
- Time: 30 minutes

**Exercise 3.5: Articulated Arm**
- Task: Add shoulder (2 DOF), elbow (1 DOF), wrist (1 DOF)
- Success: Arm moves through full range of motion in RViz
- Time: 45 minutes

**Exercise 3.6: Complete Upper Body**
- Task: Combine torso + two arms into full humanoid upper body
- Success: Both arms move independently, model is complete
- Time: 30 minutes

#### Prerequisites
- Completed Chapters 1-2
- Basic 3D coordinate system understanding (x, y, z axes)
- Familiarity with XML syntax

#### Chapter Dependencies
- Chapter 1: ROS 2 fundamentals
- Chapter 2: Python launch files, rclpy for robot_state_publisher

#### Prepares For
- Chapter 4: URDF spawns in Gazebo for physics simulation
- Chapter 5: URDF imports to Unity for rendering
- Part 3-4: Navigation and manipulation use URDF kinematics

---

## Part 1 Connection Map

```
Chapter 1 (ROS 2 Basics)
    │
    │ Concepts: nodes, topics, services
    │ Skills: CLI tools, basic pub/sub
    │
    ▼
Chapter 2 (Python + ROS 2)
    │
    │ Concepts: rclpy, actions, async
    │ Skills: Package creation, robot control
    │
    ▼
Chapter 3 (URDF)
    │
    │ Concepts: links, joints, physics
    │ Skills: Robot modeling, visualization
    │
    ▼
Part 2 (Simulation) ─────────────────────▶ Uses all Part 1 skills
```

---

## Success Criteria for Part 1

### Measurable Outcomes

| Criteria | Measurement | Target |
|----------|-------------|--------|
| Node Creation | Student creates working node from scratch | 100% can do independently |
| Pub/Sub Implementation | Student implements communication pair | 95% without reference |
| Service Implementation | Student creates service server/client | 90% without reference |
| Package Structure | Student creates valid ROS 2 package | 95% pass colcon build |
| URDF Creation | Student builds multi-link robot | 90% pass check_urdf |
| RViz Visualization | Student loads and manipulates model | 100% can visualize |

### Qualitative Outcomes

- Students feel confident explaining ROS 2 architecture to others
- Students see clear path from Part 1 to simulation (Part 2)
- Students are not overwhelmed by robotics complexity
- Students have working code they can extend

---

## Technical Requirements

### Software
- Ubuntu 22.04 LTS (or WSL2 on Windows)
- ROS 2 Humble Hawksbill
- Python 3.10+
- RViz2
- colcon build tools

### Hardware
- 8GB RAM minimum
- 20GB disk space for ROS 2

### No Hardware Required
- All exercises use simulation or visualization
- No physical robot needed for Part 1

---

## Non-Goals for Part 1

**What We Are NOT Teaching:**

| Non-Goal | Reason | Where It's Covered |
|----------|--------|-------------------|
| Gazebo simulation | Adds complexity too early | Part 2, Chapter 4 |
| Physical robot control | Requires hardware | Not in this book (simulation focus) |
| ROS 2 lifecycle nodes | Advanced topic | Not covered (optional extension) |
| DDS configuration | Deep networking topic | Not covered |
| Multi-machine ROS 2 | Distributed systems | Not covered |

---

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| ROS 2 installation fails | Blocks all progress | Provide Docker container fallback |
| URDF syntax errors frustrate learners | Discouragement | Provide validated templates, clear error messages |
| async/await confuses Python beginners | Chapter 2 bottleneck | Heavy scaffolding, sync alternatives shown first |
| Coordinate frame confusion in URDF | Chapter 3 confusion | Visual diagrams, interactive RViz exploration |

---

## Amendment Log

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-16 | Initial Part 1 specification |
