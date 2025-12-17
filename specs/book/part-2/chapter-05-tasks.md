# Chapter 5: Unity HRI Implementation — Implementation Tasks

**Source**: chapter-05-plan.md
**Status**: Ready for Implementation
**Created**: 2025-12-16

---

## Content Implementation Tasks

### Lesson 1: Unity Setup and ROS-TCP-Connector Bridge

**Task 1.1**: Create lesson introduction
- [ ] Write learning objective and motivational context
- [ ] Explain why bridge setup matters before visual work
- [ ] Provide time estimate (90 minutes)

**Task 1.2**: Teach Unity project fundamentals
- [ ] Explain Unity editor interface (Scene, Hierarchy, Inspector, Assets)
- [ ] Show project folder structure (Assets, Packages, ProjectSettings)
- [ ] Explain Scenes, GameObjects, Components
- [ ] Provide screenshot annotations of key UI elements

**Task 1.3**: Document ROS-TCP-Connector installation
- [ ] Step-by-step instructions: Windows Package Manager or manual installation
- [ ] Configuration file location and structure
- [ ] Network topology diagram: ROS 2 host, ROS master IP, Unity client IP, port numbers
- [ ] Troubleshooting common connection issues

**Task 1.4**: Teach message serialization concepts
- [ ] Explain how ROS 2 messages convert to C# objects
- [ ] Show message registry configuration
- [ ] Example message definition (.msg file)
- [ ] C# class generation from message definition

**Task 1.5**: Hands-on bridge setup activity
- [ ] Create minimal ROS 2 talker node (publishes Std_msgs/String)
- [ ] Configure bridge to receive messages
- [ ] Write C# script to read messages and log to console
- [ ] Checkpoint: Verify messages flow from ROS 2 to Unity

**Task 1.6**: Connection validation exercise
- [ ] Students create two-way communication: ROS 2 → Unity → ROS 2
- [ ] Use bridge monitoring tools to verify message flow
- [ ] Checkpoint: Print "Bridge operational: bidirectional communication verified"
- [ ] Troubleshooting guide for connection failures

**Task 1.7**: Add "Try With AI" section
- [ ] Prompt: "Ask AI about common Unity-ROS 2 bridge setup mistakes"
- [ ] Implement: Students address each potential issue
- [ ] Prompt 2: "Ask AI about scalability: How would you bridge multiple robots?"
- [ ] NO meta-commentary about learning

---

### Lesson 2: URDF Import and Basic Visualization

**Task 2.1**: Lesson introduction
- [ ] Learning objective: Import humanoid URDF successfully
- [ ] Context: "URDF to Unity translation enables simulation-to-visualization pipeline"
- [ ] Time estimate: 90 minutes

**Task 2.2**: Introduce URDF Importer package
- [ ] Show package installation via Package Manager
- [ ] Explain URDF file format (from Part 1) in context of visualization
- [ ] Differences: URDF for physics vs visualization
- [ ] Import options and their effects

**Task 2.3**: URDF import workflow
- [ ] Step-by-step: Copy URDF to Assets folder, configure import settings
- [ ] Explain import hierarchy generation (GameObjects per link)
- [ ] Show prefab creation from imported URDF
- [ ] Parent-child relationship preservation

**Task 2.4**: Model hierarchy understanding
- [ ] Diagram: Show link hierarchy in Hierarchy panel
- [ ] Explain mesh assignment to links
- [ ] Show collider setup (automatic from URDF collision elements)
- [ ] Joint information preservation (for later animation)

**Task 2.5**: Material and shader basics
- [ ] What are materials in Unity (color, texture, shader)
- [ ] Assign default material to imported meshes
- [ ] Show how to make model more visible (colors, lighting)
- [ ] Basic shader overview (Standard shader typical default)

**Task 2.6**: Camera positioning and viewing
- [ ] Set up camera to view humanoid properly
- [ ] Orbit camera around model (manual positioning)
- [ ] Perspective vs orthographic views
- [ ] Save camera position for consistent viewing

**Task 2.7**: Hands-on URDF import practice
- [ ] Students import humanoid URDF from Chapter 4
- [ ] Checkpoint 1: Verify all links appear correctly
- [ ] Checkpoint 2: Examine hierarchy matches URDF definition
- [ ] Checkpoint 3: View model from multiple camera angles
- [ ] Take screenshot of imported humanoid for documentation

**Task 2.8**: Add "Try With AI" section
- [ ] Prompt: "Ask AI why URDF visualization differs from Gazebo rendering"
- [ ] Discuss: How would you improve humanoid appearance?
- [ ] Prompt 2: "Ask AI about preparing URDF for game engine import"
- [ ] NO meta-commentary

---

### Lesson 3: Building Photorealistic Environments with AI Assistance

**Task 3.1**: Lesson introduction and context
- [ ] Learning objective: Create professional-quality HRI environment
- [ ] Motivation: "Visual realism makes interaction studies more credible"
- [ ] Time estimate: 120 minutes

**Task 3.2**: Environment design principles
- [ ] Choose scenario: Living room, office, warehouse, kitchen
- [ ] Scene requirements: Furniture, lighting, realistic materials
- [ ] Design document: Simple sketch or written description
- [ ] Performance constraints: Real-time rendering target (60+ FPS)

**Task 3.3**: Three-point lighting explanation
- [ ] Key light: Main light source (typically bright, creates shadows)
- [ ] Fill light: Secondary light (reduces shadow darkness, shows detail)
- [ ] Back light: Separation light (creates depth, rim highlight)
- [ ] Diagram and practical example

**Task 3.4**: Asset sourcing and licensing
- [ ] Free asset sources: Unity Asset Store free items, free 3D models
- [ ] Copyright/license considerations: Always check attribution requirements
- [ ] FBX vs Prefab import handling
- [ ] Material and texture asset organization

**Task 3.5**: AI collaboration scenarios
- [ ] **Scenario 1 (AI as Teacher)**:
   - Show environment with flat, unrealistic lighting
   - AI teaches three-point lighting technique with explanation
   - Student learns professional approach
- [ ] **Scenario 2 (AI as Student)**:
   - AI suggests ultra-high-resolution textures everywhere
   - Student corrects with performance constraints
   - AI adapts with optimization strategy
- [ ] **Scenario 3 (AI as Co-Worker)**:
   - Problem: Realism vs performance
   - Iteration 1: High detail, low FPS
   - Iteration 2: Low detail, high FPS but cheap-looking
   - Iteration 3: Strategic detail + baked lighting → both
   - Convergence: Professional approach emerges

**Task 3.6**: Hands-on environment creation
- [ ] Students create selected environment scenario
- [ ] Place furniture and decorative objects
- [ ] Implement three-point lighting system
- [ ] Apply materials (test rendering)
- [ ] Measure performance (FPS, draw calls)
- [ ] Optimize if needed (reduce complexity, enable batching)
- [ ] Final checkpoint: Environment looks professional, renders smoothly

**Task 3.7**: Add "Try With AI" section
- [ ] Prompt: "Ask AI for lighting recipes (preset lighting setups)"
- [ ] Try: Different lighting approaches, observe visual impact
- [ ] Prompt 2: "Ask AI about environment design for human subjects studies"
- [ ] NO meta-commentary

---

### Lesson 4: Human Avatar Animation and Character Control

**Task 4.1**: Lesson introduction
- [ ] Learning objective: Implement avatar with working animations
- [ ] Context: "Animation brings avatar to life for interaction"
- [ ] Time estimate: 120 minutes

**Task 4.2**: Humanoid avatar selection and setup
- [ ] Import humanoid avatar (Mixamo, free online character, etc.)
- [ ] Ensure humanoid avatar configuration in Unity
- [ ] Apply to scene, position in environment
- [ ] Verify model quality and animations included

**Task 4.3**: Animation clips overview
- [ ] Show included animations: idle, walking, running, gestures
- [ ] Explain animation clips (FBX sequences or separate files)
- [ ] Duration, loop settings, import options
- [ ] Assigning clips to animator

**Task 4.4**: Animator controller state machine
- [ ] Create Animator controller
- [ ] Add states: Idle, Walking, Gesturing, etc.
- [ ] Create state transitions with conditions
- [ ] Explain parameters (integers, bools, floats) controlling transitions
- [ ] Diagram showing state flow

**Task 4.5**: Animation parameter system
- [ ] Define parameters: IsWalking (bool), Speed (float), Action (int)
- [ ] Link parameters to state transitions
- [ ] Test parameter changes in editor
- [ ] Show inspector for animation debugging

**Task 4.6**: Blend trees for smooth transitions
- [ ] Explain blend trees (1D vs 2D)
- [ ] Create walk/run blend tree based on speed parameter
- [ ] Smooth transitions between motion states
- [ ] Performance consideration: Blending overhead

**Task 4.7**: AI collaboration scenarios
- [ ] **Scenario 1 (AI as Teacher)**:
   - Avatar animations don't play or jerky transitions
   - AI teaches animator patterns and state machine setup
   - Student learns best practices for animation systems
- [ ] **Scenario 2 (AI as Student)**:
   - AI suggests all animations simultaneously
   - Student corrects: Clear states, mutually exclusive transitions
   - AI creates proper state structure
- [ ] **Scenario 3 (AI as Co-Worker)**:
   - Problem: Smooth natural-looking movement
   - Iteration 1: Basic state machine → jerky
   - Iteration 2: Add blend trees → smoother
   - Iteration 3: Tune transition timing → natural
   - Convergence: Polished animation system

**Task 4.8**: Hands-on animator setup
- [ ] Create animator controller for imported avatar
- [ ] Set up states: Idle, Walking, Gesturing
- [ ] Add parameters and transitions
- [ ] Create walk blend tree for variable speed
- [ ] Test avatar animation: Idle → Walk → Gesture → Idle
- [ ] Verify smooth transitions between states

**Task 4.9**: Add "Try With AI" section
- [ ] Prompt: "Ask AI about professional animation rigging"
- [ ] Try: Add new animation clip, integrate into state machine
- [ ] Prompt 2: "Ask AI about animation optimization for real-time"
- [ ] NO meta-commentary

---

### Lesson 5: Scripting Human-Robot Interaction

**Task 5.1**: Lesson introduction
- [ ] Learning objective: Script interactive behavior
- [ ] Context: "Code brings interaction scenarios to life"
- [ ] Time estimate: 120 minutes

**Task 5.2**: C# MonoBehavior fundamentals
- [ ] Review C# basics (classes, variables, methods)
- [ ] Explain MonoBehavior (Update, Start, OnDestroy lifecycle)
- [ ] Show how scripts attach to GameObjects
- [ ] Console debugging with Debug.Log()

**Task 5.3**: Physics-based detection
- [ ] Collision detection: OnTriggerEnter, OnCollisionEnter
- [ ] Raycasting: Detect line-of-sight
- [ ] Distance-based detection: Physics.OverlapSphere
- [ ] Show performance implications of each approach

**Task 5.4**: Event system and callbacks
- [ ] Unity Event system (OnTriggerEnter → trigger event)
- [ ] Custom event pattern (define event, invoke from script)
- [ ] Event subscription (multiple handlers per event)
- [ ] Example: Human approaches → triggers robot response event

**Task 5.5**: UI system for feedback
- [ ] Canvas and UI elements (Text, Image, Button)
- [ ] Display interaction prompts ("Human is approaching", "Interaction complete")
- [ ] Update UI from scripts
- [ ] Screen space vs world space UI

**Task 5.6**: Coroutines for timed sequences
- [ ] Coroutine concept: Time-based execution
- [ ] Syntax: IEnumerator, yield return, StartCoroutine()
- [ ] Animation sequences: Smooth transitions using coroutines
- [ ] Example: Play animation over 2 seconds while updating UI

**Task 5.7**: AI collaboration scenarios
- [ ] **Scenario 1 (AI as Teacher)**:
   - Student uses naive distance calculation in Update
   - AI teaches efficient proximity detection patterns
   - Student learns performance-aware coding
- [ ] **Scenario 2 (AI as Student)**:
   - AI suggests overly complex logic
   - Student clarifies MVP: Just human approaching, robot responds
   - AI simplifies to essential interactions
- [ ] **Scenario 3 (AI as Co-Worker)**:
   - Problem: Responsive but natural-feeling interaction
   - Iteration 1: Instant response → too robotic
   - Iteration 2: Delayed response → sluggish
   - Iteration 3: Response synchronized with avatar animation → natural
   - Convergence: Proper timing found through iteration

**Task 5.8**: Hands-on interaction scripting
- [ ] Script 1: Proximity detector (detects when human approaches)
- [ ] Script 2: Robot responder (triggers when proximity detected)
- [ ] Script 3: UI feedback (shows interaction status)
- [ ] Test interaction flow: Approach → Detection → Response → UI feedback
- [ ] Checkpoint: Natural-feeling interaction sequence works

**Task 5.9**: Add "Try With AI" section
- [ ] Prompt: "Ask AI about C# best practices for game development"
- [ ] Try: Add new interaction type (e.g., human waves → robot waves)
- [ ] Prompt 2: "Ask AI about behavior trees for complex interaction logic"
- [ ] NO meta-commentary

---

### Lesson 6: ROS 2 Integration and Message Publishing

**Task 6.1**: Lesson introduction
- [ ] Learning objective: Connect Unity to ROS 2 ecosystem
- [ ] Context: "ROS 2 integration makes simulation controllable and observable"
- [ ] Time estimate: 120 minutes

**Task 6.2**: ROS 2 message type basics
- [ ] Review message definitions (.msg files)
- [ ] Standard types: Std_msgs, Geometry_msgs, Sensor_msgs
- [ ] Custom messages for this project (e.g., InteractionEvent)
- [ ] Message field types (int, float, string, arrays)

**Task 6.3**: Custom message definition
- [ ] Create custom message: InteractionEvent
   - Fields: human_position (geometry_msgs/Point), action (string), timestamp (float)
- [ ] Message registration with ROS-TCP-Connector
- [ ] C# class generation from message

**Task 6.4**: Publisher implementation
- [ ] ROS Publisher class in Unity
- [ ] Advertise topic, publish message
- [ ] Topic naming conventions
- [ ] Serialization: C# object → ROS message bytes

**Task 6.5**: Subscriber implementation
- [ ] ROS Subscriber for robot state (joint feedback, status)
- [ ] Message callback in C#
- [ ] Deserialization: ROS message bytes → C# object
- [ ] Handling message delays and out-of-order delivery

**Task 6.6**: Event-based publishing
- [ ] Publish only on interaction events (not every frame)
- [ ] Reduce bandwidth vs continuous messages
- [ ] Timestamp attachment for ordering

**Task 6.7**: AI collaboration scenarios
- [ ] **Scenario 1 (AI as Teacher)**:
   - Serialization errors when publishing custom messages
   - AI teaches strict type contracts and message registry
   - Student learns message infrastructure requirements
- [ ] **Scenario 2 (AI as Student)**:
   - AI suggests high-frequency publishing
   - Student corrects: Event-based publishing for efficiency
   - AI adjusts publishing strategy
- [ ] **Scenario 3 (AI as Co-Worker)**:
   - Problem: Synchronization despite network latency
   - Iteration 1: Direct messaging → race conditions
   - Iteration 2: Add acknowledgments → reliable but slow
   - Iteration 3: Smart buffering with timeouts → fast AND reliable
   - Convergence: Robust protocol emerges

**Task 6.8**: Hands-on ROS integration
- [ ] Create InteractionEvent custom message
- [ ] Modify Lesson 5 interaction script to publish events
- [ ] Subscribe to robot state topic (use stub topic for testing)
- [ ] Update robot humanoid based on received joint angles
- [ ] Test: Trigger interaction, verify ROS topic receives message
- [ ] Verify: Subscribe to robot state, verify joint angles update robot model

**Task 6.9**: Debugging ROS messages
- [ ] Use `ros2 topic echo` to monitor published messages
- [ ] Verify message fields contain correct values
- [ ] Check message frequency (should be event-based, not continuous)
- [ ] Latency measurement: Time from interaction trigger to ROS message

**Task 6.10**: Add "Try With AI" section
- [ ] Prompt: "Ask AI about ROS 2 best practices for game engines"
- [ ] Try: Add another message type (e.g., RobotState feedback)
- [ ] Prompt 2: "Ask AI about real-time synchronization challenges"
- [ ] NO meta-commentary

---

### Lesson 7: Advanced Scene Management and UI — Skill Creation

**Task 7.1**: Lesson introduction and skill context
- [ ] Explain Layer 3: "Encode HRI patterns as reusable skill"
- [ ] Learning objective: Develop unity-hri-interaction-skill
- [ ] Time estimate: 90 minutes

**Task 7.2**: Pattern identification from Lessons 3-6
- [ ] Review recurring patterns: Environment setup, avatar animation, interaction, ROS bridge
- [ ] Identify reusable components vs scenario-specific elements
- [ ] Document: "What's the same in every HRI scenario?"

**Task 7.3**: Skill design using Persona + Questions + Principles
- [ ] **Persona**: "Think like a simulation engineer designing reusable HRI test platforms"
- [ ] **Questions**:
   - What environmental elements are essential vs customizable?
   - How do we template avatar behavior for different interaction types?
   - What ROS topics/messages must be standardized?
   - How can we test without ROS 2 running?
- [ ] **Principles**:
   - Modularity: Environment, avatar, interaction independently configurable
   - Scalability: Different robots and interaction types
   - Decoupling: ROS separate from visualization for testing
   - Documentation: Clear interfaces for extension

**Task 7.4**: Skill documentation structure
- [ ] File: `.claude/skills/unity-hri-interaction/SKILL.md`
- [ ] Sections:
   - Overview: What this skill provides
   - Scene template structure (environment prefabs, avatar prefabs, robot model)
   - Interaction event system (triggering, sequencing, validation)
   - ROS message flow (publishers, subscribers, message definitions, topic conventions)
   - Testing without ROS (mock robot controller, simulated feedback)
   - Extension patterns (adding new interaction types, new robot models)
   - Performance optimization (batching, LOD, draw calls)
   - Code examples: Prefab setup, interaction scripts, ROS wrapper
- [ ] Troubleshooting guide

**Task 7.5**: Skill implementation
- [ ] Write complete SKILL.md with all sections
- [ ] Provide C# code templates for standard interactions
- [ ] Document prefab structure and hierarchy
- [ ] Include mock ROS controller for development/testing
- [ ] Performance profiling guidelines

**Task 7.6**: Skill validation exercise
- [ ] Use skill to set up different HRI scenario (e.g., kitchen environment instead of office)
- [ ] Verify all components work with new scenario
- [ ] Checkpoint: New scenario set up and functional using only skill documentation
- [ ] Document: "How did the skill reduce setup time?"

**Task 7.7**: Add "Try With AI" section
- [ ] Prompt: "Ask AI how to extend this skill for multi-agent HRI"
- [ ] Try: Implement two human avatars interacting with robot
- [ ] Prompt 2: "Ask AI about deploying HRI systems from simulation to real robots"
- [ ] NO meta-commentary

---

### Lesson 8: Capstone — Complete HRI Demonstration

**Task 8.1**: Capstone introduction
- [ ] Explain Layer 4: "Specification-first integration of chapter learnings"
- [ ] Learning objective: Complete HRI demonstration system
- [ ] Time estimate: 150 minutes

**Task 8.2**: Specification writing (PRIMARY TASK — BEFORE ANY CODE)
- [ ] Create specs/book/part-2/chapter-5-capstone/spec.md with:
   - **Intent**: "Complete HRI scenario demonstrating human-robot interaction in photorealistic environment"
   - **Constraints**:
      - Both human avatar and humanoid robot in same scene
      - Photorealistic environment (60+ FPS rendering)
      - Real-time bidirectional ROS 2 communication
      - Interaction must demonstrate understanding of chapter concepts
   - **Success Criteria**:
      - Human avatar visible with realistic animation
      - Robot model visible with proper materials
      - When human approaches, robot responds appropriately
      - ROS 2 topics show interaction events
      - Performance maintained at 60+ FPS
   - **Acceptance Tests**:
      - test_scene_load: Renders at 60+ FPS
      - test_avatar_animation: Walking is smooth and realistic
      - test_proximity_trigger: Interaction triggers at correct distance
      - test_ros_communication: ROS topics receive events
      - test_complete_flow: Full scenario runs without errors

**Task 8.3**: Component composition analysis
- [ ] Identify which previous lessons apply:
   - Lesson 3: Environment (living room or scenario of choice)
   - Lesson 4: Avatar with animations
   - Lesson 5: Interaction scripting
   - Lesson 6: ROS 2 publishing
   - Lesson 7: Use unity-hri-interaction-skill for standardized setup
- [ ] Identify gaps (new components):
   - Capstone-specific scripting (orchestrating all components)
   - Performance optimization (target 60+ FPS)

**Task 8.4**: Architecture planning with AI
- [ ] Student provides spec to AI
- [ ] AI generates system architecture outline:
   - Component breakdown
   - ROS topics and message flow
   - Script organization
   - Prefab structure
- [ ] Student reviews and refines plan

**Task 8.5**: Implementation and testing
- [ ] Set up environment (from Lesson 3)
- [ ] Import humanoid robot (from Lesson 2)
- [ ] Add human avatar with animations (from Lesson 4)
- [ ] Implement interaction scripting (from Lesson 5)
- [ ] Integrate ROS 2 communication (from Lesson 6)
- [ ] Test each component, then full system
- [ ] Verify acceptance tests pass

**Task 8.6**: Iterative refinement
- [ ] **Iteration 1**: Scene loads and renders
   - Result: Environment visible, avatar visible, robot visible
   - Issue: No interaction behavior
- [ ] **Iteration 2**: Add interaction logic
   - Avatar can walk toward robot
   - Proximity detection works
   - Robot responds with gesture
   - Issue: Timing feels off or rough
- [ ] **Iteration 3**: Polish and optimize
   - Synchronize animation timing with ROS messages
   - Optimize rendering (target 60+ FPS)
   - Smooth all interactions
   - Result: All acceptance tests pass

**Task 8.7**: Capstone validation and documentation
- [ ] Verify all acceptance tests pass
- [ ] Performance check: 60+ FPS maintained during interaction
- [ ] Student writes reflection (200-300 words):
   - What specification details mattered most?
   - How did reusable skills reduce development time?
   - What would you improve in next iteration?
   - What new skills would benefit future HRI development?
- [ ] Code submission: Clean, organized, documented

**Task 8.8**: Add "Try With AI" section
- [ ] Prompt: "Ask AI about advancing HRI simulation to real robots"
- [ ] Discuss: "What components would need to change for deployment?"
- [ ] Challenge: "Design spec for adding speech recognition to HRI"
- [ ] NO meta-commentary

---

## Integration with Part 2 Book

### File Structure Output
```
book-source/docs/Part-2-Digital-Twin/05-unity-hri/
├── index.md (Chapter 5 introduction)
├── 01-unity-ros-bridge.md (Lesson 1)
├── 02-urdf-import.md (Lesson 2)
├── 03-environment-design.md (Lesson 3)
├── 04-human-avatars.md (Lesson 4)
├── 05-interaction-scripting.md (Lesson 5)
├── 06-ros-integration.md (Lesson 6)
├── 07-scene-management.md (Lesson 7)
├── 08-capstone-hri-demo.md (Lesson 8)
└── examples/ (Code examples and assets)
    ├── unity_project_setup/
    ├── environment_templates/
    ├── avatar_animator_controllers/
    ├── interaction_scripts/
    └── capstone_system/
```

### Reusable Skill
- `.claude/skills/unity-hri-interaction/SKILL.md`

Referenced in future HRI chapters and deployment

---

## Success Criteria for Implementation

- [ ] All 8 lessons written following chapter-05-plan.md
- [ ] All code examples tested and documented
- [ ] All evals (Eval-5.1 through Eval-5.6) achievable by students
- [ ] Reusable skill created (Lesson 7) and used in capstone (Lesson 8)
- [ ] No meta-commentary in "Try With AI" sections
- [ ] Capstone demonstrates spec-first integration
- [ ] Technical review passes: Factual accuracy, code tested, no hallucinations

---

**Chapter 5 Implementation — Ready for Content Writer**
