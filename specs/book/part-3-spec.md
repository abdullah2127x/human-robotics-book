# Part 3 Specification: The AI-Robot Brain (NVIDIA Isaac)

**Version:** 1.0.0
**Created:** 2025-12-17
**Status:** Active
**Source:** book-spec.md, chapter-index.md, constitutional reasoning

---

## Part Identity

**Part Number:** 3
**Part Title:** The AI-Robot Brain
**Part Subtitle:** Advanced Perception and GPU-Accelerated Training

**Unifying Theme:** Transform simulation from testing ground to intelligent training environment - leverage GPU acceleration for photorealistic rendering, synthetic data generation, and hardware-accelerated perception.

**Part Purpose:** Advanced Integration - bridge the gap between basic simulation (Part 2) and production-ready perception systems using NVIDIA's robotics ecosystem.

---

## Prerequisites

Students must have completed:
- **Part 1: ROS 2 Foundation** (all 3 chapters)
  - ROS 2 nodes, topics, services
  - Python rclpy development
  - URDF humanoid modeling
- **Part 2: The Digital Twin** (all 3 chapters)
  - Gazebo physics simulation
  - Unity HRI scenarios
  - Sensor simulation (LiDAR, depth camera, IMU)

**Technical Requirements:**
- Ubuntu 22.04 LTS (native installation recommended, WSL2 possible but limited GPU support)
- ROS 2 Humble installed and tested
- NVIDIA GPU with 8GB+ VRAM (RTX 3070 or better recommended)
  - RTX 4060/4070: Good performance
  - RTX 3080/3090: Excellent performance
  - RTX 4080/4090: Optimal performance
- NVIDIA GPU drivers (535+ for CUDA 12.x)
- CUDA Toolkit 12.x (will be installed in Chapter 7)
- Isaac Sim 2023.1.1+ (will be installed in Chapter 7)
- Isaac ROS packages (will be installed in Chapter 8)
- 32GB+ RAM recommended (Isaac Sim is memory-intensive)
- 100GB+ free disk space (Isaac Sim + assets)

---

## Part-Level Learning Outcomes

After completing Part 3, students will be able to:

- **LO-3.1:** Install and configure NVIDIA Isaac Sim for humanoid robot simulation
- **LO-3.2:** Generate synthetic training datasets using domain randomization
- **LO-3.3:** Implement hardware-accelerated visual SLAM with Isaac ROS
- **LO-3.4:** Configure Nav2 for bipedal humanoid path planning
- **LO-3.5:** Integrate perception (VSLAM) with navigation (Nav2) for autonomous movement
- **LO-3.6:** Debug VSLAM failures (feature tracking loss, loop closure errors)
- **LO-3.7:** Tune Nav2 behavior trees for humanoid-specific constraints

---

## Scaffolding Strategy

**Scaffolding Level:** Light-Moderate
- Minimal hand-holding (students expected to research and problem-solve)
- Open-ended challenges with multiple valid solutions
- AI collaboration for complex debugging and parameter tuning
- Emphasis on reading official documentation (Isaac Sim docs, Isaac ROS, Nav2)

**Cognitive Load:** Moderate → Heavy
- 7-12 concepts per chapter (higher than Part 2)
- Complex tool integration (Isaac Sim + Isaac ROS + Nav2)
- Performance optimization considerations
- Real-time constraints (VSLAM must run at camera framerate)

**Complexity Tier Progression:**
- Chapter 7: B2 (Intermediate Application - Isaac Sim setup and usage)
- Chapter 8: B2-C1 (Intermediate to Advanced - VSLAM algorithms and CUDA acceleration)
- Chapter 9: B2-C1 (Intermediate to Advanced - Nav2 + bipedal adaptations)

---

## Chapter Specifications

### Chapter 7: NVIDIA Isaac Sim - Photorealistic Simulation and Synthetic Data

**Chapter Focus:** Isaac Sim architecture, Omniverse foundation, domain randomization, synthetic data generation

**Proficiency Tier:** B2 (Intermediate Application)

**Core Concepts (10):**
1. Isaac Sim architecture (Omniverse Kit, PhysX 5, RTX rendering)
2. Omniverse Nucleus (asset management, collaboration)
3. USD (Universal Scene Description) format
4. Photorealistic rendering (RTX ray tracing, materials, lighting)
5. Domain randomization (texture, lighting, pose variation)
6. Replicator (synthetic data generation API)
7. Isaac Sim sensors (cameras, LiDAR with GPU acceleration)
8. ROS 2 bridge in Isaac Sim
9. Performance optimization (LOD, culling, render settings)
10. Isaac Sim vs Gazebo tradeoffs

**Learning Outcomes:**
- **Eval-7.1:** Students install Isaac Sim and verify GPU acceleration
- **Eval-7.2:** Students import humanoid URDF into Isaac Sim scene
- **Eval-7.3:** Students create photorealistic indoor environment with proper lighting
- **Eval-7.4:** Students implement domain randomization for training data diversity
- **Eval-7.5:** Students generate synthetic dataset (10,000+ images) using Replicator
- **Eval-7.6:** Students configure Isaac Sim ROS 2 bridge for sensor data publishing
- **Eval-7.7:** Students analyze Isaac vs Gazebo for specific use cases

**Hands-On Exercises:**
1. Install Isaac Sim via Omniverse Launcher (verify GPU rendering)
2. Import humanoid URDF, validate physics behavior
3. Create indoor scene with photorealistic materials (PBR workflow)
4. Implement domain randomization: textures, lighting, camera poses
5. Generate synthetic RGB-D dataset using Replicator
6. Configure ROS 2 bridge, publish camera + LiDAR data
7. Performance profiling: identify bottlenecks, optimize render settings
8. Capstone: Generate training dataset for object detection (humanoid navigating randomized environments)

**Estimated Duration:** 8-9 lessons (complex tooling, domain randomization workflow)

**Teaching Modality:** Hands-on discovery (students explore Isaac Sim interface, learn through experimentation)

**Reusable Intelligence (Stage 3):**
- **isaac-sim-domain-randomization-skill**: Encode domain randomization patterns (texture variation, lighting changes, pose sampling)
- **isaac-sim-performance-skill**: Performance optimization patterns (render settings, LOD, culling strategies)

---

### Chapter 8: Isaac ROS - Hardware-Accelerated VSLAM and Navigation

**Chapter Focus:** Visual SLAM fundamentals, Isaac ROS packages, CUDA acceleration, stereo vision, localization and mapping

**Proficiency Tier:** B2-C1 (Intermediate Application to Advanced Integration)

**Core Concepts (11):**
1. Visual SLAM fundamentals (feature detection, tracking, mapping)
2. Stereo vision (disparity maps, depth estimation)
3. Isaac ROS architecture (GXF graphs, hardware acceleration)
4. CUDA acceleration for computer vision (GPU vs CPU performance)
5. Visual odometry (frame-to-frame motion estimation)
6. Loop closure detection (recognizing previously visited locations)
7. Map representation (point clouds, occupancy grids, feature maps)
8. Isaac ROS Visual SLAM package (configuration, launch files)
9. VSLAM debugging (feature tracking loss, drift, map quality)
10. ROS 2 tf frames (camera → base_link → map transformations)
11. RViz VSLAM visualization (trajectories, maps, feature matches)

**Learning Outcomes:**
- **Eval-8.1:** Students explain VSLAM algorithm stages (detect → track → map → localize)
- **Eval-8.2:** Students install and configure Isaac ROS packages
- **Eval-8.3:** Students run Isaac Visual SLAM on recorded ROS 2 bag data
- **Eval-8.4:** Students visualize VSLAM output (trajectory, map, features) in RViz
- **Eval-8.5:** Students measure GPU acceleration speedup (CUDA vs CPU baseline)
- **Eval-8.6:** Students debug VSLAM failures (feature loss, drift, loop closure errors)
- **Eval-8.7:** Students tune VSLAM parameters for humanoid robot constraints

**Hands-On Exercises:**
1. Install Isaac ROS packages (isaac_ros_visual_slam, dependencies)
2. Record stereo camera bag in Isaac Sim (humanoid walking through environment)
3. Run Isaac Visual SLAM on bag data (offline processing)
4. Visualize trajectory + map in RViz (validate SLAM quality)
5. Measure performance: GPU vs CPU (quantify CUDA acceleration benefit)
6. Introduce VSLAM failure: low-texture environment (debug feature tracking loss)
7. Introduce VSLAM failure: rapid rotation (debug motion blur, tracking failure)
8. Tune VSLAM parameters: feature detector thresholds, keyframe selection
9. Capstone: Real-time VSLAM on humanoid (Isaac Sim → Isaac ROS → RViz live visualization)

**Estimated Duration:** 8 lessons (focused on VSLAM pipeline)

**Teaching Modality:** Error analysis (students intentionally break VSLAM, learn debugging patterns)

**Reusable Intelligence (Stage 3):**
- **vslam-debugging-skill**: Encode VSLAM failure patterns and debugging strategies (feature loss → increase detector sensitivity, drift → loop closure tuning, etc.)
- **isaac-ros-performance-skill**: CUDA acceleration verification and optimization patterns

---

### Chapter 9: Nav2 - Path Planning for Bipedal Humanoid Movement

**Chapter Focus:** Nav2 stack, path planners, costmaps, behavior trees, bipedal gait constraints

**Proficiency Tier:** B2-C1 (Intermediate Application to Advanced Integration)

**Core Concepts (10):**
1. Nav2 architecture (navigation stack components)
2. Costmaps (local, global, inflation layers)
3. Path planners (Navfn, Smac, TEB for bipedal)
4. Controller plugins (DWB, Regulated Pure Pursuit)
5. Behavior trees (navigation logic, recovery behaviors)
6. Bipedal constraints (stability, center of mass, step planning)
7. Footstep planning (discrete foot placement vs continuous control)
8. Dynamic obstacle avoidance (real-time replanning)
9. Nav2 parameters (tuning for humanoid vs wheeled robot)
10. Integration: VSLAM (Chapter 8) → Nav2 (Chapter 9)

**Learning Outcomes:**
- **Eval-9.1:** Students configure Nav2 stack for humanoid robot
- **Eval-9.2:** Students create costmaps with humanoid-specific inflation radius
- **Eval-9.3:** Students select appropriate path planner for bipedal gait
- **Eval-9.4:** Students implement behavior tree for navigation + recovery
- **Eval-9.5:** Students tune Nav2 parameters collaboratively with AI
- **Eval-9.6:** Students integrate VSLAM (Chapter 8) as Nav2 localization source
- **Eval-9.7:** Students handle dynamic obstacles (real-time replanning)

**Hands-On Exercises:**
1. Install Nav2 stack (nav2_bringup, plugins)
2. Configure costmaps (static map, sensor layers, inflation)
3. Select path planner: compare Navfn (fast, grid-based) vs TEB (trajectory optimization)
4. Implement DWB controller (dynamic window approach for humanoid footstep constraints)
5. Create behavior tree: navigate → recovery (stuck → rotate, clear costmap, etc.)
6. Tune Nav2 parameters with AI: footprint size, max velocities, acceleration limits
7. Integrate VSLAM: Isaac Visual SLAM (Chapter 8) → Nav2 localization
8. Test dynamic obstacles: human walks in front of robot, observe replanning
9. Capstone: Autonomous humanoid navigation (VSLAM localization + Nav2 planning + obstacle avoidance)

**Estimated Duration:** 8-9 lessons (behavior trees, bipedal adaptation complexity)

**Teaching Modality:** Collaborative parameter tuning (student + AI iterate on Nav2 config)

**Reusable Intelligence (Stage 3):**
- **nav2-humanoid-config-skill**: Encode Nav2 parameter patterns for bipedal robots (footprint, velocity limits, costmap inflation)
- **behavior-tree-design-skill**: Behavior tree design patterns for navigation + recovery logic

---

## Part 3 Success Criteria

### Technical Success
- [ ] **SC-3.1:** Isaac Sim installed, GPU acceleration verified (RTX rendering functional)
- [ ] **SC-3.2:** Synthetic dataset generated (10,000+ images with domain randomization)
- [ ] **SC-3.3:** Isaac Visual SLAM running real-time (>10 Hz on GPU)
- [ ] **SC-3.4:** VSLAM map quality validated (low drift, loop closure functional)
- [ ] **SC-3.5:** Nav2 configured for humanoid (bipedal-appropriate parameters)
- [ ] **SC-3.6:** Autonomous navigation functional (VSLAM + Nav2 integration)
- [ ] **SC-3.7:** Dynamic obstacle avoidance verified (real-time replanning)

### Pedagogical Success
- [ ] All lessons follow 4-layer teaching framework (1→2→3→4)
- [ ] Teaching modality varies across chapters (hands-on, error analysis, collaborative tuning)
- [ ] "Try With AI" sections demonstrate AI collaboration
- [ ] Exercises have checkbox success criteria
- [ ] Capstone integrates all 3 chapters (Isaac Sim + VSLAM + Nav2)

### Factual Accuracy Success
- [ ] All Isaac Sim APIs verified against official NVIDIA docs (Context7)
- [ ] All Isaac ROS configurations tested and validated
- [ ] All Nav2 parameters cited with sources
- [ ] Code examples execute successfully (test logs attached)
- [ ] Version compatibility documented (Isaac Sim 2023.1.1+, ROS 2 Humble, CUDA 12.x)

---

## Connection to Other Parts

### From Part 1
- URDF humanoid model → imported into Isaac Sim (Chapter 7)
- ROS 2 topics/services → control Isaac Sim robot (Chapter 7-9)
- Python rclpy skills → process Isaac ROS outputs (Chapter 8-9)

### From Part 2
- Gazebo simulation experience → Compare with Isaac Sim (Chapter 7)
- Sensor simulation knowledge → Understand Isaac Sim sensors (Chapter 7)
- RViz visualization → Visualize VSLAM and Nav2 (Chapter 8-9)
- Sensor data pipelines → Feed into VSLAM (Chapter 8)

### To Part 4
- Synthetic data generation → Train object detection models (Chapter 10-11)
- VSLAM + Nav2 → Autonomous navigation for voice commands (Chapter 10-12)
- Perception pipeline → Object recognition for manipulation (Chapter 12)

---

## Non-Goals (Explicit Exclusions)

### Chapter 7 Non-Goals
- ❌ **Full Omniverse ecosystem**: Focus on Isaac Sim for robotics, not USD authoring, Omniverse Create, or other Omniverse apps
- ❌ **Reinforcement learning**: RL with Isaac Gym is Part 3 scope creep (could be separate advanced chapter)
- ❌ **Multi-robot simulation**: Focus on single humanoid, not fleet coordination
- ❌ **Custom physics plugins**: Use built-in PhysX 5, don't extend physics engine

**Why excluded**: Scope management for B2-C1 tier. These topics deserve dedicated coverage beyond Part 3.

### Chapter 8 Non-Goals
- ❌ **Classical SLAM algorithms**: Focus on visual SLAM (Isaac ROS), not LiDAR SLAM (SLAM Toolbox, Cartographer)
- ❌ **Camera calibration theory**: Assume cameras pre-calibrated (Isaac Sim provides intrinsics)
- ❌ **Loop closure implementation**: Use Isaac ROS built-in loop closure, don't implement from scratch
- ❌ **Map optimization theory**: Treat Isaac Visual SLAM as black box with tunable parameters

**Why excluded**: Isaac ROS abstracts implementation details. Students use the package, don't reimplement VSLAM.

### Chapter 9 Non-Goals
- ❌ **Full ROS 2 navigation stack**: Focus on Nav2 (current standard), not deprecated ROS 1 navigation
- ❌ **Custom path planners**: Use Nav2 plugins (Navfn, Smac, TEB), don't write planners from scratch
- ❌ **Footstep planning algorithms**: High-level Nav2 planning, not low-level bipedal gait generation
- ❌ **Social navigation**: Focus on obstacle avoidance, not human-aware navigation patterns

**Why excluded**: Nav2 for humanoids is already complex. Custom planning and social navigation are advanced research topics.

---

## Risk Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Isaac Sim installation complexity | High | Provide step-by-step installation guide, Docker alternative, troubleshooting section |
| GPU requirement excludes students | High | Provide cloud VM option (AWS EC2 g4dn instances), clarify minimum specs upfront |
| Isaac Sim license/cost concerns | Medium | Clarify free for educational use, provide licensing FAQ |
| Isaac ROS CUDA dependency | Medium | Document CUDA toolkit installation, verify GPU drivers compatible |
| VSLAM failure debugging difficulty | Medium | Create systematic debugging flowchart, common failure patterns |
| Nav2 parameter tuning complexity | Medium | Provide baseline humanoid config, iterate with AI collaboration |
| Performance optimization rabbit hole | Low | Define "good enough" performance thresholds, avoid premature optimization |
| Version compatibility issues | Low | Pin specific versions (Isaac Sim 2023.1.1, ROS 2 Humble, CUDA 12.x), test thoroughly |

---

## Teaching Modality Variation (Anti-Convergence)

**Part 2 Patterns Used:**
- Chapter 4 (Gazebo): Hands-on discovery
- Chapter 5 (Unity): Hands-on discovery + specification-first (capstone)
- Chapter 6 (Sensors): Hands-on discovery

**Part 3 Variation Strategy:**

**Chapter 7 (Isaac Sim):**
- **Primary**: Hands-on discovery (explore Isaac Sim interface, experimentation)
- **Secondary**: Socratic dialogue (Isaac vs Gazebo tradeoffs - when to use each?)
- **Rationale**: Isaac Sim has rich UI, best learned through exploration. Socratic questions force reasoning about tool selection.

**Chapter 8 (VSLAM):**
- **Primary**: Error analysis (intentionally break VSLAM, debug systematically)
- **Secondary**: Specification-first (define VSLAM quality criteria before running)
- **Rationale**: VSLAM failures are pedagogically valuable. Students learn debugging by encountering and fixing failures.

**Chapter 9 (Nav2):**
- **Primary**: Collaborative parameter tuning (student + AI iterate on Nav2 config)
- **Secondary**: Behavior tree design (specification-first for navigation logic)
- **Rationale**: Nav2 tuning has no single "correct" answer. Collaboration with AI explores parameter space efficiently.

**Result**: Zero repeated modalities from Part 2. Each chapter uses distinct teaching approach.

---

## Intelligence Accumulation Plan

### Skills to Create (Stage 3)

**Chapter 7:**
1. **isaac-sim-domain-randomization-skill**
   - Persona: Domain randomization engineer
   - Questions: What variations increase model robustness? How much randomization before diminishing returns?
   - Principles: Systematic variation (texture, lighting, pose), balance realism vs diversity

2. **isaac-sim-performance-skill**
   - Persona: Real-time rendering engineer
   - Questions: What's the performance bottleneck? Which settings impact quality vs speed?
   - Principles: Profile first (don't guess), LOD for distant objects, cull off-screen geometry

**Chapter 8:**
3. **vslam-debugging-skill**
   - Persona: SLAM engineer debugging mapping failures
   - Questions: What failure mode (feature loss, drift, loop closure)? What sensor data reveals issue?
   - Principles: Feature tracking loss → detector sensitivity, drift → loop closure tuning, check RViz for diagnostic visualization

4. **isaac-ros-performance-skill**
   - Persona: Performance engineer validating CUDA acceleration
   - Questions: Is GPU actually being used? What's the speedup vs CPU baseline?
   - Principles: Profile with nvprof/nsys, verify CUDA kernel execution, measure end-to-end latency

**Chapter 9:**
5. **nav2-humanoid-config-skill**
   - Persona: Robotics engineer configuring navigation for bipedal robot
   - Questions: What footprint size? What max velocity given stability constraints? What inflation radius?
   - Principles: Footprint = humanoid width + safety margin, max velocity = gait stability limit, inflation = sensor range + reaction time

6. **behavior-tree-design-skill**
   - Persona: Behavior tree architect designing navigation logic
   - Questions: What recovery behaviors? What failure conditions trigger recovery?
   - Principles: Hierarchical composition (navigate → recover → retry), timeout-based recovery triggers, clear success/failure conditions

### Skills to Reuse (from Part 2)

- **gazebo-physics-debugging-skill** (Part 2 Ch4): Partially applicable to Isaac Sim physics debugging
- **sensor-processing-skill** (Part 2 Ch6): Applies to Isaac Sim sensor data processing

---

## Research Phase Plan (Pre-Spec)

**Total Budget:** 18-25 hours across all 3 chapters

### Chapter 7 Research (10-12 hours)

**Priority 1: Isaac Sim Installation & Setup** (3 hours)
- Official installation docs (Omniverse Launcher workflow)
- GPU requirements and driver compatibility
- Common installation issues (CUDA, Vulkan, drivers)
- Tools: WebFetch (NVIDIA docs), Context7 (if Isaac Sim docs available)

**Priority 2: Domain Randomization & Replicator** (4 hours)
- Replicator API documentation
- Domain randomization examples (texture, lighting, pose)
- Synthetic data generation workflows
- Tools: Context7 (Isaac Sim Replicator docs), code examples

**Priority 3: Isaac Sim vs Gazebo** (2 hours)
- Performance comparisons (GPU acceleration benefits)
- Feature comparison (PhysX vs ODE, RTX vs rasterization)
- Use case selection criteria
- Tools: WebFetch (benchmark articles, community discussions)

**Priority 4: ROS 2 Bridge Configuration** (2 hours)
- Isaac Sim ROS 2 bridge setup
- Sensor data publishing (camera, LiDAR)
- Action server integration
- Tools: Context7 (Isaac Sim ROS 2 docs)

### Chapter 8 Research (6-8 hours)

**Priority 1: VSLAM Fundamentals** (2 hours)
- Visual odometry theory (brief, not deep)
- Feature detection/tracking (ORB, FAST, etc.)
- Loop closure detection
- Tools: WebFetch (VSLAM survey papers, tutorials)

**Priority 2: Isaac ROS Architecture** (3 hours)
- Isaac ROS packages and dependencies
- GXF (Graph Execution Framework) overview
- CUDA acceleration implementation
- isaac_ros_visual_slam package configuration
- Tools: Context7 (Isaac ROS docs), WebFetch (NVIDIA Isaac ROS blogs)

**Priority 3: VSLAM Debugging Patterns** (2 hours)
- Common failure modes (feature tracking loss, drift, loop closure failure)
- Diagnostic tools (RViz VSLAM visualization, log analysis)
- Parameter tuning strategies
- Tools: WebFetch (VSLAM debugging guides, ROS Answers)

### Chapter 9 Research (4-5 hours)

**Priority 1: Nav2 Architecture** (2 hours)
- Nav2 stack components (costmaps, planners, controllers, behavior trees)
- Nav2 plugins and configuration
- ROS 2 Humble compatibility
- Tools: Context7 (Nav2 docs), WebFetch (Nav2 tutorials)

**Priority 2: Bipedal Navigation Constraints** (2 hours)
- Footstep planning vs continuous control
- Center of mass stability constraints
- Nav2 parameters for bipedal robots (max velocity, acceleration, footprint)
- Tools: WebFetch (humanoid navigation papers, Boston Dynamics/Agility Robotics tech blogs)

**Priority 3: Behavior Tree Design** (1 hour)
- Behavior tree fundamentals (sequences, fallbacks, decorators)
- Nav2 behavior tree examples
- Recovery behaviors
- Tools: Context7 (Nav2 behavior tree docs)

---

## File Structure

```
specs/book/
└── part-3-spec.md (this file)

book-source/docs/Part-3-Advanced-Simulation-Perception/
├── index.md (Part 3 introduction)
├── 07-isaac-sim/
│   ├── index.md (Chapter 7 introduction)
│   ├── 01-isaac-sim-installation.md
│   ├── 02-omniverse-architecture.md
│   ├── 03-urdf-import.md
│   ├── 04-photorealistic-rendering.md
│   ├── 05-domain-randomization.md
│   ├── 06-replicator-synthetic-data.md
│   ├── 07-ros2-bridge.md
│   ├── 08-performance-optimization.md
│   └── 09-capstone-training-dataset.md
├── 08-isaac-ros-vslam/
│   ├── index.md (Chapter 8 introduction)
│   ├── 01-vslam-fundamentals.md
│   ├── 02-isaac-ros-installation.md
│   ├── 03-visual-odometry.md
│   ├── 04-cuda-acceleration.md
│   ├── 05-loop-closure.md
│   ├── 06-rviz-visualization.md
│   ├── 07-vslam-debugging.md
│   └── 08-capstone-realtime-vslam.md
└── 09-nav2-humanoid/
    ├── index.md (Chapter 9 introduction)
    ├── 01-nav2-architecture.md
    ├── 02-costmaps.md
    ├── 03-path-planners.md
    ├── 04-controllers.md
    ├── 05-behavior-trees.md
    ├── 06-bipedal-constraints.md
    ├── 07-vslam-integration.md
    ├── 08-dynamic-obstacles.md
    └── 09-capstone-autonomous-navigation.md
```

---

## Amendment Log

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-17 | Initial Part 3 specification created via LoopFlow v2.0 Phase 0 constitutional reasoning |
