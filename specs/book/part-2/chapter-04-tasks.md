# Chapter 4: Gazebo Physics Simulation — Implementation Tasks

**Source**: chapter-04-plan.md
**Status**: Ready for Implementation
**Created**: 2025-12-16

---

## Content Implementation Tasks

### Lesson 1: Gazebo Architecture and Ecosystem

**Task 1.1**: Create lesson introduction with learning objectives
- [ ] Write lesson title and learning objective statement
- [ ] Explain why Gazebo architecture understanding matters for control systems
- [ ] Provide expected time estimate (90 minutes)

**Task 1.2**: Teach Gazebo client-server architecture
- [ ] Diagram: Show gzserver and gzclient separation
- [ ] Explain message-based communication between server and client
- [ ] Show how multiple clients can connect to single server
- [ ] Include code/command examples: Starting gzserver and gzclient separately

**Task 1.3**: Explain plugin system
- [ ] Describe plugin categories: physics, sensor, system plugins
- [ ] Show example SDF with plugin declarations
- [ ] Explain plugin lifecycle (load, init, fini)
- [ ] List common physics plugins (ODE, Bullet, DART)

**Task 1.4**: Introduce SDF format basics
- [ ] Show minimal valid SDF document structure
- [ ] Explain key tags: `<sdf>`, `<world>`, `<model>`, `<link>`, `<joint>`, `<geometry>`
- [ ] Contrast SDF vs URDF (when to use each)
- [ ] Show ROS 2 Gazebo bridge integration points

**Task 1.5**: Hands-on exploration activity
- [ ] Provide pre-built Gazebo world with visible objects and forces
- [ ] Instruct students to: Examine world file (text), start simulation, observe behavior
- [ ] Checkpoint: "Modify gravity value in world file, observe effect"
- [ ] Self-reflection: Students write 1-2 sentences explaining what gzserver does

**Task 1.6**: Add "Try With AI" section
- [ ] Prompt: "Ask AI to explain Gazebo plugin loading sequence"
- [ ] Follow-up: "Compare AI's explanation to your understanding. What's new?"
- [ ] Prompt 2: "Ask AI about a real-world robotics scenario using Gazebo. How does architecture support it?"
- [ ] NO meta-commentary (avoid "What AI taught you" explanations)

---

### Lesson 2: Creating and Modifying World Files

**Task 2.1**: Lesson introduction and context
- [ ] Learning objective: Create custom SDF world files
- [ ] Motivation: "World files are blueprints for simulation environments"
- [ ] Time estimate: 120 minutes

**Task 2.2**: SDF world structure walkthrough
- [ ] Show minimal SDF world template with explanations
- [ ] Line-by-line breakdown: XML declaration, root element, physics definition
- [ ] Explain pose representation (x, y, z, roll, pitch, yaw)
- [ ] Show model include syntax for reusing URDF files

**Task 2.3**: Physics engine selection and configuration
- [ ] Compare ODE, Bullet, DART trade-offs (accuracy vs speed)
- [ ] Show SDF physics configuration for each engine
- [ ] Explain key parameters: gravity, gravity_init, physics type, step size
- [ ] Code example: Creating worlds with different physics engines

**Task 2.4**: Gravity and friction configuration
- [ ] Explain gravity vector (standard Earth gravity: 0, 0, -9.81)
- [ ] Show friction model parameters (mu1, mu2, friction)
- [ ] Link parameters to physics behavior: "Higher friction = more grip"
- [ ] Provide examples: Friction for different surfaces (concrete, carpet, ice)

**Task 2.5**: Model placement and collision setup
- [ ] Show geometry types: box, sphere, cylinder, mesh, plane
- [ ] Explain pose element for placement
- [ ] Show material definition for visual distinction
- [ ] Explain collision geometry vs visual geometry

**Task 2.6**: Hands-on world creation
- [ ] Students create world from scratch (NOT copy-paste):
  - Ground plane (large box as terrain)
  - Walls (boxes around perimeter)
  - Obstacles (spheres, boxes scattered)
  - Gravity and friction configured
- [ ] Checkpoint: "Verify each object is visible and collides correctly"
- [ ] Extension: Create 3 variants with different physics engines

**Task 2.7**: Add "Try With AI" section
- [ ] Prompt: "Ask AI how to add a moving platform to your world"
- [ ] Follow-up: "Try implementing the AI suggestion. Does it work?"
- [ ] Prompt 2: "Ask AI why friction values matter for humanoid standing"
- [ ] NO meta-commentary

---

### Lesson 3: Spawning and Controlling Models with AI Assistance

**Task 3.1**: Lesson introduction
- [ ] Learning objective: Spawn URDF into running simulation
- [ ] Context: "Models must be added dynamically in robotics"
- [ ] Time estimate: 120 minutes

**Task 3.2**: spawn_entity service explanation
- [ ] Show ROS 2 service definition for spawn_entity
- [ ] Request fields: name, xml (URDF/SDF), initial_pose, reference_frame
- [ ] Response: success (bool), status_message (string)
- [ ] Example command: `ros2 service call /spawn_entity ...`

**Task 3.3**: Initial pose specification
- [ ] Explain Pose message structure (position, orientation as quaternion)
- [ ] Show alternative: position + RPY angles
- [ ] Demonstrate setting spawn position to avoid collisions
- [ ] Code example: ROS 2 Python client calling spawn_entity

**Task 3.4**: Namespace and naming management
- [ ] Explain model namespaces prevent name conflicts
- [ ] Show how namespace affects topic names (e.g., /robot_1/joint_state)
- [ ] Demonstrate spawning multiple copies with different namespaces
- [ ] Error handling: What happens when name already exists?

**Task 3.5**: AI collaboration demonstrations
- [ ] **Scenario 1 (AI as Teacher)**:
   - Show failed spawn attempt (absolute vs relative path issue)
   - AI teaches path resolution (GAZEBO_MODEL_PATH, absolute paths)
   - Student learns best practice
- [ ] **Scenario 2 (AI as Student)**:
   - AI initially suggests spawning all models at origin
   - Student corrects: "Models need separate positions"
   - AI adapts with positional diversity
- [ ] **Scenario 3 (AI as Co-Worker)**:
   - Problem: Spawn timing creates instability (world not ready)
   - Iteration 1: Simple delay
   - Iteration 2: Check /clock for readiness
   - Convergence: Elegant solution through iteration

**Task 3.6**: Hands-on practice with debugging
- [ ] Students spawn humanoid URDF from Part 1 into Gazebo world from Lesson 2
- [ ] Checkpoint 1: Verify humanoid appears in simulation at correct pose
- [ ] Common failure scenarios: path issues, collision overlap, namespace conflicts
- [ ] Students debug each failure using AI collaboration
- [ ] Success: Humanoid stands stably without penetration

**Task 3.7**: Add "Try With AI" section
- [ ] Prompt: "Ask AI about common spawn_entity errors and solutions"
- [ ] Try implementing: Spawn multiple humanoid copies with different names
- [ ] Prompt 2: "Ask AI how to spawn obstacles around the humanoid"
- [ ] NO meta-commentary about learning or AI teaching

---

### Lesson 4: Physics Parameter Tuning with AI Collaboration

**Task 4.1**: Lesson introduction and context
- [ ] Learning objective: Configure physics parameters for stable simulation
- [ ] Motivation: "Physics tuning makes difference between realistic and unstable"
- [ ] Time estimate: 120 minutes

**Task 4.2**: Physics timestep and solver configuration
- [ ] Explain timestep (simulation step duration): typical 0.001s (1000 Hz)
- [ ] Explain iterations per step: higher = more accurate but slower
- [ ] Show impact on computation: timestep trade-off
- [ ] Code example: SDF physics element configuration

**Task 4.3**: Damping parameters
- [ ] Linear damping: reduces velocity over time
- [ ] Angular damping: reduces rotational velocity
- [ ] Show per-joint configuration in URDF
- [ ] Explain tuning: too high = sluggish, too low = oscillation

**Task 4.4**: Friction and contact physics
- [ ] Friction coefficients (mu1, mu2): affect sliding resistance
- [ ] Contact material properties: elasticity, restitution (bounciness)
- [ ] Show how friction prevents sliding, enables stance
- [ ] Real-world examples: Different surface properties

**Task 4.5**: Stability parameters
- [ ] Contact penetration tolerance: how much overlap before error
- [ ] Contact surface layer: artificial layer for collision detection
- [ ] Constraint force mixing (CFM): solver softness
- [ ] Error reduction parameter (ERP): error correction speed

**Task 4.6**: AI collaboration scenarios
- [ ] **Scenario 1 (AI as Teacher)**:
   - Show humanoid with unrealistic wobbling during stance
   - AI teaches tuning heuristic: "Start conservative (high damping), gradually reduce"
   - Student learns systematic approach
- [ ] **Scenario 2 (AI as Student)**:
   - AI suggests very low timestep (0.0001) for accuracy
   - Student corrects: "Too expensive computationally for real-time"
   - AI provides compromise values with justification
- [ ] **Scenario 3 (AI as Co-Worker)**:
   - Problem: Humanoid feet penetrate ground during stance
   - Iteration 1: Increase friction → helps but incomplete
   - Iteration 2: Reduce penetration tolerance → additional improvement
   - Iteration 3: Combine both + joint limit tuning → stable
   - Convergence: Multi-parameter optimization required

**Task 4.7**: Hands-on tuning exercise
- [ ] Start with "bad" simulation (unrealistic behavior provided)
- [ ] Students systematically modify parameters to achieve stable stance
- [ ] Document: Which parameters affected which behaviors
- [ ] Checkpoint: Humanoid stands stably for 10+ seconds without anomalies
- [ ] Recording: Create before/after videos showing improvement

**Task 4.8**: Add "Try With AI" section
- [ ] Prompt: "Ask AI how physics parameters affect humanoid walking"
- [ ] Implement: Adjust parameters, observe walking behavior changes
- [ ] Prompt 2: "Ask AI about common physics simulation mistakes"
- [ ] NO meta-commentary

---

### Lesson 5: Collision Detection and Contact Sensing

**Task 5.1**: Lesson introduction
- [ ] Learning objective: Implement contact sensors for feedback
- [ ] Motivation: "Robots need touch sensing for balance and object interaction"
- [ ] Time estimate: 120 minutes

**Task 5.2**: Contact sensor plugin explanation
- [ ] Show URDF sensor definition: `<sensor type="contact">`
- [ ] Contact sensor frame and placement in humanoid
- [ ] Plugin element: gazebo_ros_contact_plugin (Humble compatibility)
- [ ] ROS 2 topic publishing: /contact_states

**Task 5.3**: Collision filtering and bits
- [ ] Collision bitmask: selectively enable/disable collisions
- [ ] Link collide_bitmask settings
- [ ] Scenario: Humanoid feet interact with ground but not each other
- [ ] Code example: SDF with collision filtering

**Task 5.4**: ROS 2 ContactsState message
- [ ] Message structure: header, states[], collision1, collision2, wrenches
- [ ] Contact point information: position, normal, depth
- [ ] Force information: normal force magnitude
- [ ] Understanding message timing (when published)

**Task 5.5**: AI collaboration scenarios
- [ ] **Scenario 1 (AI as Teacher)**:
   - Contact sensor added but no messages despite collisions
   - AI teaches: "Sensors publish on state CHANGE, not continuous contact"
   - Student learns message generation pattern
- [ ] **Scenario 2 (AI as Student)**:
   - AI suggests monitoring all contacts
   - Student corrects: "Only care about foot-ground contact"
   - AI filters by link names
- [ ] **Scenario 3 (AI as Co-Worker)**:
   - Problem: Detecting foot contact reliably for walking controller
   - Iteration 1: Force threshold → too sensitive
   - Iteration 2: Contact normal direction → more reliable
   - Iteration 3: Both + hysteresis → robust
   - Convergence: Multi-signal approach better than single criterion

**Task 5.6**: Hands-on sensor implementation
- [ ] Add contact sensors to humanoid feet (URDF modification)
- [ ] Configure Gazebo plugin in world file
- [ ] Write ROS 2 Python node to subscribe to /contact_states
- [ ] Checkpoint 1: Observe contact messages during humanoid movement
- [ ] Visualize in RViz: Display contact points
- [ ] Practice: Detect foot contact during walking vs falling scenarios

**Task 5.7**: Add "Try With AI" section
- [ ] Prompt: "Ask AI how to add sensors to different humanoid links"
- [ ] Implement: Add additional sensors (IMU, force/torque)
- [ ] Prompt 2: "Ask AI about sensor fusion for robust contact detection"
- [ ] NO meta-commentary

---

### Lesson 6: Joint Control and Humanoid Movement — Skill Creation

**Task 6.1**: Lesson introduction and skill context
- [ ] Explain Layer 3: "Creating reusable intelligence for future projects"
- [ ] Learning objective: Develop gazebo-humanoid-control-skill
- [ ] Time estimate: 90 minutes

**Task 6.2**: Pattern identification from Lessons 1-5
- [ ] Review recurring patterns: model spawning, physics tuning, sensor processing
- [ ] Identify control-specific patterns: joint commands, feedback, safety monitoring
- [ ] Document: "What do all humanoid control tasks have in common?"

**Task 6.3**: Skill design using Persona + Questions + Principles
- [ ] **Persona**: "Think like a roboticist standardizing humanoid control interfaces"
- [ ] **Questions**:
   - What joint constraints must control respect?
   - How do we handle controller timeouts gracefully?
   - What feedback signals validate successful execution?
   - How do we sequence complex multi-joint movements?
- [ ] **Principles**:
   - Separation: Joint group control vs individual joint manipulation
   - Degradation: Continue functioning despite partial failures
   - Validation: Always verify goal achievement before proceeding

**Task 6.4**: Skill documentation structure
- [ ] File: `.claude/skills/gazebo-humanoid-control/SKILL.md`
- [ ] Sections:
  - Overview: What this skill does and why it matters
  - Joint coordinate frames: How are joint angles specified/interpreted?
  - Control patterns: Single joint, multi-joint sequences, trajectories
  - Safety constraints: Joint limits, velocity limits, acceleration limits
  - Feedback processing: Reading actual vs commanded joint states
  - Timeout handling: What happens if command doesn't complete?
  - Error recovery: How to handle partial failures
- [ ] Code examples: Python using rclpy for joint trajectory controller

**Task 6.5**: Skill implementation
- [ ] Write complete SKILL.md with all sections above
- [ ] Include Python code patterns for:
  - Publishing joint trajectory commands
  - Monitoring /joint_states feedback
  - Detecting timeout conditions
  - Validating goal achievement
- [ ] Test patterns: Example code that uses skill correctly
- [ ] Common mistakes: Anti-patterns to avoid

**Task 6.6**: Skill validation exercise
- [ ] Apply skill to control humanoid arm reaching toward target
- [ ] Apply skill to control humanoid legs (stance, simple walking preparation)
- [ ] Checkpoint: All movements succeed using skill patterns
- [ ] Documentation: "How did the skill improve your control code?"

**Task 6.7**: Add "Try With AI" section
- [ ] Prompt: "Ask AI how this skill could be extended for complex movements"
- [ ] Try implementing: Use skill to choreograph humanoid movement sequence
- [ ] Prompt 2: "Ask AI about error handling patterns in robotics control"
- [ ] NO meta-commentary

---

### Lesson 7: Debugging and Optimization — Skill Creation

**Task 7.1**: Lesson introduction and skill context
- [ ] Explain Layer 3: "Encode troubleshooting patterns as reusable skill"
- [ ] Learning objective: Develop gazebo-physics-debugging-skill
- [ ] Time estimate: 90 minutes

**Task 7.2**: Problem categorization
- [ ] Identify problem classes from Lesson 4 experience:
  - Penetration (objects sinking through terrain)
  - Oscillation/instability (uncontrolled movement)
  - Performance degradation (simulation running slowly)
  - Sensor failures (contact/IMU not publishing)
- [ ] Document symptoms for each problem class
- [ ] Show diagnostic approach for each

**Task 7.3**: Decision tree creation for diagnostics
- [ ] Build flowchart: "Problem occurs → Check X? → Try Y"
- [ ] Penetration issues:
  - Check geometry definitions
  - Check friction values
  - Check penetration tolerance
  - Check joint limits
- [ ] Oscillation issues:
  - Check damping parameters
  - Check physics timestep
  - Check contact material properties
  - Check constraint solver iterations
- [ ] Performance issues:
  - Check timestep value
  - Check solver iterations
  - Check model complexity
  - Check sensor update rates
- [ ] Sensor failures:
  - Check plugin configuration in world file
  - Check URDF sensor definition
  - Check collision bits/filtering
  - Check ROS 2 topic subscribers

**Task 7.4**: Skill documentation
- [ ] File: `.claude/skills/gazebo-physics-debugging/SKILL.md`
- [ ] Sections:
  - Problem classification: How to identify issue type
  - RViz visualization: What to look for in 3D view
  - Log analysis: Interpreting gzserver console output
  - Incremental diagnosis: How to change one parameter at a time
  - Common pitfalls: Mistakes that make debugging harder
  - Recovery procedures: Step-by-step for each problem class
- [ ] Code examples: RViz configuration files, shell commands for diagnostics

**Task 7.5**: Skill implementation
- [ ] Write complete SKILL.md with decision trees
- [ ] Include RViz plugin configurations for physics visualization
- [ ] Include shell commands for log inspection
- [ ] Document parameter ranges for different problem scenarios

**Task 7.6**: Skill validation exercise
- [ ] Provide "broken" simulation scenarios:
  - Humanoid feet penetrating ground
  - Humanoid oscillating during stance
  - Simulation running too slowly
  - Contact sensors not detecting touches
- [ ] Students apply skill to diagnose and fix each scenario
- [ ] Checkpoint: All scenarios resolved using skill patterns
- [ ] Documentation: "Which diagnostic approach worked for each problem?"

**Task 7.7**: Add "Try With AI" section
- [ ] Prompt: "Ask AI about physics simulation debugging best practices"
- [ ] Implement: Create intentional physics problem, use skill to fix
- [ ] Prompt 2: "Ask AI about performance profiling in Gazebo"
- [ ] NO meta-commentary

---

### Lesson 8: Capstone — Humanoid Standing and Balancing

**Task 8.1**: Capstone introduction
- [ ] Explain Layer 4: "Specification-first integration of accumulated knowledge"
- [ ] Learning objective: Complete humanoid balance control system
- [ ] Time estimate: 150 minutes (1.5 hours teaching + 1.5 hours hands-on)

**Task 8.2**: Specification writing (PRIMARY TASK — BEFORE ANY CODE)
- [ ] Create specs/book/part-2/chapter-4-capstone/spec.md with sections:
  - **Intent**: "Humanoid robot maintains stable stance and recovers from disturbances"
  - **Constraints**:
     - Standing on flat ground (no slope)
     - Using only joint trajectory commands (no active force control)
     - Real-time capable (>20 Hz control loop)
     - Safety limits on joint movements
  - **Success Criteria**:
     - Center of mass remains within foot polygon
     - All joints stay within safe operating range
     - No foot penetration of ground
     - Balance maintained for 10+ seconds
     - Recovers from simulated push disturbances
  - **Acceptance Tests**:
     - test_stable_standing: Stand 10 seconds without falling
     - test_foot_contact: Both feet maintain contact with ground
     - test_balance_recovery: Return to stable pose within 2 seconds after push
     - test_joint_limits: All joints never exceed hard limits
     - test_real_time: Control loop executes at 25+ Hz

**Task 8.3**: Component composition analysis
- [ ] Identify which skills from previous lessons apply:
   - gazebo-humanoid-control-skill: Used for sending joint commands
   - gazebo-physics-debugging-skill: Used for tuning simulation parameters
- [ ] Identify gaps (new components needed):
   - Balance feedback controller: Process foot contact sensor → adjust joint angles
   - Disturbance recovery: Detect push → initiate stability recovery
- [ ] Document architecture: How components connect via ROS 2 topics/services

**Task 8.4**: Implementation planning with AI
- [ ] Student provides spec.md to AI
- [ ] AI generates controller implementation outline:
   - Which nodes/topics involved
   - Which existing skills compose the solution
   - Pseudocode/implementation strategy
- [ ] Student reviews and refines plan

**Task 8.5**: Implementation and testing
- [ ] Implement humanoid balance controller in Python (rclpy)
- [ ] Use gazebo-humanoid-control-skill for joint commands
- [ ] Implement balance feedback loop (read foot sensors → command adjustments)
- [ ] Test against acceptance tests (one at a time)

**Task 8.6**: Iterative refinement
- [ ] **Iteration 1**: Basic stance control
   - Result: Humanoid stands (Eval-4.1, 4.2, 4.4 pass)
   - Issue: Falls when disturbed
   - Feedback: "Must handle disturbances"
- [ ] **Iteration 2**: Add active balance recovery
   - Modify controller to respond to external forces
   - Result: Handles moderate disturbances (Eval-4.5 improvements)
- [ ] **Iteration 3**: Tune for robustness
   - Use gazebo-physics-debugging-skill to optimize parameters
   - Result: All acceptance tests pass (all Evals achieve success)
- [ ] Convergence: Final solution meets all specifications through iteration

**Task 8.7**: Capstone validation and documentation
- [ ] Verify spec adherence: All acceptance tests pass
- [ ] Performance validation: Control loop runs at required frequency
- [ ] Documentation: Write short reflection (200-300 words):
   - What specification details were most important?
   - How did reusable skills (Lessons 6-7) improve development?
   - What would you improve for next iteration?
   - What new skills would help future projects?
- [ ] Code submission: All nodes, launch files, tests in clean structure

**Task 8.8**: Add "Try With AI" section
- [ ] Prompt: "Ask AI how to extend humanoid controller for walking"
- [ ] Discuss: "What new skills would a walking controller need?"
- [ ] Challenge: "Can you sketch specification for walking? What acceptance tests?"
- [ ] NO meta-commentary

---

## Integration with Part 2 Book

### File Structure Output
```
book-source/docs/Part-2-Digital-Twin/04-gazebo-physics/
├── index.md (Part 2 introduction to Chapter 4)
├── 01-gazebo-architecture.md (Lesson 1)
├── 02-world-files.md (Lesson 2)
├── 03-model-spawning.md (Lesson 3)
├── 04-physics-tuning.md (Lesson 4)
├── 05-collision-detection.md (Lesson 5)
├── 06-joint-control.md (Lesson 6)
├── 07-debugging-optimization.md (Lesson 7)
├── 08-capstone-humanoid-balance.md (Lesson 8)
└── examples/ (Code examples directory)
    ├── gazebo_world_templates/
    ├── spawn_entity_scripts/
    ├── joint_control_nodes/
    └── capstone_controller/
```

### Reusable Skills
- `.claude/skills/gazebo-humanoid-control/SKILL.md`
- `.claude/skills/gazebo-physics-debugging/SKILL.md`

Both skills referenced in Chapter 5 and Chapter 6 implementation

---

## Success Criteria for Implementation

- [ ] All 8 lessons written following chapter-04-plan.md structure
- [ ] All code examples tested and working
- [ ] All evals (Eval-4.1 through Eval-4.5) achievable by students
- [ ] Reusable skills created (Lessons 6-7) and used in capstone (Lesson 8)
- [ ] No meta-commentary in "Try With AI" sections
- [ ] Capstone demonstrates spec-first integration (specification BEFORE code)
- [ ] Technical review passes: Factual accuracy, code tested, no hallucinations

---

**Chapter 4 Implementation — Ready for Content Writer**
