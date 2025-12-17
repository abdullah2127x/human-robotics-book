# Part 4 Specification: Vision-Language-Action (VLA)

**Version:** 1.0.0
**Created:** 2025-12-17
**Status:** Active
**Source:** book-spec.md, chapter-index.md, constitutional reasoning

---

## Part Identity

**Part Number:** 4
**Part Title:** Vision-Language-Action (VLA)
**Part Subtitle:** The Convergence of LLMs and Robotics

**Unifying Theme:** "From Words to Worlds" - Transform natural language into physical action. Bridge the gap between human communication and robotic execution through voice interfaces, cognitive planning, and end-to-end autonomous systems.

**Part Purpose:** Integration/Capstone - synthesize all learning from Parts 1-3 into autonomous systems that understand human language, plan intelligent actions, and execute them in the physical world.

---

## Prerequisites

Students must have completed:
- **Part 1: ROS 2 Foundation** (all 3 chapters)
  - ROS 2 nodes, topics, services, actions
  - Python rclpy development
  - URDF humanoid modeling
- **Part 2: The Digital Twin** (all 3 chapters)
  - Gazebo physics simulation
  - Unity HRI scenarios
  - Sensor simulation (LiDAR, depth camera, IMU)
- **Part 3: The AI-Robot Brain** (all 3 chapters)
  - Isaac Sim photorealistic simulation
  - Isaac ROS Visual SLAM
  - Nav2 autonomous navigation

**Technical Requirements:**
- All Part 3 requirements (Ubuntu 22.04, ROS 2 Humble, NVIDIA GPU, Isaac Sim, Nav2)
- Python 3.10+ with audio processing libraries (PyAudio, sounddevice)
- OpenAI API key OR local Whisper model capability
- LLM API access (OpenAI GPT-4, Anthropic Claude, or local alternatives)
- Microphone for voice input (USB microphone recommended)
- 16GB+ RAM (LLM inference can be memory-intensive)
- Internet connectivity for cloud LLM APIs (or local model setup)

---

## Part-Level Learning Outcomes

After completing Part 4, students will be able to:

- **LO-4.1:** Implement real-time speech recognition using OpenAI Whisper
- **LO-4.2:** Parse voice commands into structured intent representations
- **LO-4.3:** Design LLM prompts that translate natural language to robotic action sequences
- **LO-4.4:** Create action primitives that map to ROS 2 actions/services
- **LO-4.5:** Implement error recovery strategies for failed robotic actions
- **LO-4.6:** Build end-to-end autonomous systems: voice → plan → navigate → perceive → act
- **LO-4.7:** Debug and optimize the complete VLA pipeline for real-time performance

---

## Scaffolding Strategy

**Scaffolding Level:** Light (Student-Driven)
- Minimal hand-holding (students expected to independently research, integrate, debug)
- Open-ended challenges with multiple valid approaches
- Heavy emphasis on specification-first development
- AI collaboration as peer/co-worker (not teacher)
- Students design their own solutions, validate with AI assistance

**Cognitive Load:** Heavy
- 10-15 concepts per chapter (highest in book)
- Complex multi-system integration (Whisper + LLM + ROS 2 + Nav2 + Isaac Sim)
- Real-time performance constraints
- Error handling across multiple failure modes
- Production-quality requirements

**Complexity Tier Progression:**
- Chapter 10: C1 (Advanced Integration - Voice pipeline)
- Chapter 11: C1-C2 (Advanced to Professional - LLM planning)
- Chapter 12: C2 (Professional Mastery - Full capstone)

---

## Chapter Specifications

### Chapter 10: Voice-to-Action - Using OpenAI Whisper for Voice Commands

**Chapter Focus:** Speech recognition, Whisper API, intent parsing, ROS 2 integration, real-time audio streaming

**Proficiency Tier:** C1 (Advanced Integration)

**Core Concepts (12):**
1. Speech recognition fundamentals (acoustic models, language models)
2. OpenAI Whisper architecture (encoder-decoder, multilingual)
3. Whisper API vs local model tradeoffs
4. Audio streaming (real-time capture, buffering, VAD)
5. Voice Activity Detection (VAD) for command segmentation
6. Wake word detection (optional but common pattern)
7. Intent parsing (command structure, entities, parameters)
8. Natural Language Understanding (NLU) basics
9. ROS 2 integration (audio → intent → action topic/service)
10. Error handling (unclear speech, background noise, timeout)
11. Latency optimization (streaming vs batch processing)
12. Privacy and safety considerations (voice data handling)

**Learning Outcomes:**
- **Eval-10.1:** Students configure and run Whisper for speech-to-text
- **Eval-10.2:** Students implement real-time audio capture with PyAudio/sounddevice
- **Eval-10.3:** Students detect voice activity and segment commands
- **Eval-10.4:** Students parse voice commands into structured intents
- **Eval-10.5:** Students publish intents to ROS 2 topics
- **Eval-10.6:** Students handle voice recognition errors gracefully
- **Eval-10.7:** Students optimize for acceptable latency (&lt;2 seconds end-to-end)

**Hands-On Exercises:**
1. Install Whisper (API or local model) and verify transcription
2. Implement real-time audio capture with proper buffering
3. Add Voice Activity Detection (silence detection for command boundaries)
4. Create intent parser: "Go to the kitchen" → {action: "navigate", target: "kitchen"}
5. Build ROS 2 node: audio input → Whisper → intent → publish to /voice_intent topic
6. Test with various commands: navigation, object interaction, status queries
7. Handle edge cases: unclear speech, background noise, overlapping speakers
8. Performance profiling: measure and optimize latency
9. Capstone: Real-time voice command pipeline integrated with humanoid

**Estimated Duration:** 8-9 lessons

**Teaching Modality:**
- **Primary:** Specification-first (define intent schema before implementing parser)
- **Secondary:** Iterative refinement (test → fail → debug → improve cycle)
- **Rationale:** Voice interfaces require clear specifications. Intent schemas must be designed before implementation.

**Reusable Intelligence (Stage 3):**
- **voice-intent-parsing-skill**: Encode patterns for parsing natural language commands into robotic intents
- **audio-pipeline-optimization-skill**: Real-time audio processing optimization patterns

---

### Chapter 11: Cognitive Planning - LLMs Translating Language to ROS 2 Actions

**Chapter Focus:** Task decomposition, action primitives, LLM prompting for robotics, action sequence generation, error recovery

**Proficiency Tier:** C1-C2 (Advanced Integration to Professional Mastery)

**Core Concepts (14):**
1. Task decomposition theory (hierarchical task networks)
2. Action primitives (atomic robotic actions)
3. Action primitive library design (navigate, pick, place, speak, wait)
4. LLM prompting for robotics (structured outputs, JSON mode)
5. Chain-of-thought reasoning for planning
6. World state representation (what the robot knows)
7. Preconditions and postconditions (action applicability)
8. Plan validation (checking feasibility before execution)
9. Error recovery strategies (retry, replan, ask for help)
10. ROS 2 action servers (long-running tasks with feedback)
11. Action sequence execution (sequential, parallel, conditional)
12. Grounding language to physical world (entity resolution)
13. Safety constraints (forbidden actions, human safety zones)
14. Explainability (robot explains its plan to user)

**Learning Outcomes:**
- **Eval-11.1:** Students design action primitive library for humanoid robot
- **Eval-11.2:** Students craft LLM prompts that generate valid action sequences
- **Eval-11.3:** Students implement task decomposition for complex commands
- **Eval-11.4:** Students validate plans before execution (feasibility checks)
- **Eval-11.5:** Students implement error recovery (replan on failure)
- **Eval-11.6:** Students integrate LLM planner with ROS 2 action servers
- **Eval-11.7:** Students ground natural language entities to simulation objects

**Hands-On Exercises:**
1. Design action primitive library: 10+ atomic actions for humanoid
2. Create world state representation: objects, locations, robot capabilities
3. Craft LLM system prompt: task decomposition specialist for robotics
4. Implement structured output: LLM returns JSON action sequence
5. Build plan validator: check preconditions, resource availability
6. Create ROS 2 planner node: intent → LLM → action sequence → execute
7. Implement error handling: action failure → replan or ask user
8. Add safety constraints: forbidden actions, workspace limits
9. Test with complex commands: "Make me a coffee" → multi-step plan
10. Capstone: Voice command → LLM plan → ROS 2 execution

**Estimated Duration:** 9-10 lessons (complex LLM integration)

**Teaching Modality:**
- **Primary:** Co-learning with AI (iterative prompt refinement)
- **Secondary:** Error-driven development (design for failure, implement recovery)
- **Rationale:** LLM prompting is best learned through iteration. Students and AI collaborate to refine prompts.

**Reusable Intelligence (Stage 3):**
- **llm-robotics-prompting-skill**: Encode patterns for LLM prompts that generate robotic action sequences
- **action-primitive-design-skill**: Design patterns for robotic action primitive libraries
- **error-recovery-skill**: Error recovery strategy patterns for robotic systems

---

### Chapter 12: Capstone Project - The Autonomous Humanoid

**Chapter Focus:** Full integration, voice → plan → navigate → perceive → act, end-to-end autonomous task completion

**Proficiency Tier:** C2 (Professional Mastery)

**Core Concepts (15):**
1. System integration architecture (all components working together)
2. End-to-end latency analysis (voice → action completion time)
3. Perception-action coupling (see → understand → act)
4. Object detection for task grounding (identify target objects)
5. Manipulation primitives (grasp, place, push, pull)
6. Task sequencing and coordination
7. Failure mode analysis (what can go wrong, how to handle)
8. System monitoring and diagnostics
9. User feedback and confirmation
10. Performance benchmarking
11. Demo preparation and presentation
12. Documentation standards
13. Testing methodology (unit, integration, system)
14. Debugging distributed robotic systems
15. Future extensions and research directions

**Learning Outcomes:**
- **Eval-12.1:** Students integrate all Part 1-4 components into unified system
- **Eval-12.2:** Students implement end-to-end autonomous task completion
- **Eval-12.3:** Students achieve acceptable latency (&lt;10 seconds voice to action start)
- **Eval-12.4:** Students handle failures gracefully with user feedback
- **Eval-12.5:** Students document system architecture and API
- **Eval-12.6:** Students create demo showcasing autonomous capabilities
- **Eval-12.7:** Students identify limitations and propose improvements

**Hands-On Exercises:**
1. Define capstone requirements specification (user stories, acceptance criteria)
2. Design system architecture diagram (all components, data flows)
3. Implement integration layer: voice (Ch10) + planner (Ch11) + navigation (Ch9)
4. Add object detection: identify objects mentioned in commands
5. Implement manipulation primitives (if using manipulator) or navigation goals
6. Build monitoring dashboard: system status, current task, errors
7. Perform end-to-end testing: multiple scenarios, measure success rate
8. Debug failures: use ROS 2 tools (ros2 topic echo, rqt_graph, rosbag)
9. Optimize latency: profile and improve bottlenecks
10. Create demonstration video: voice command to task completion
11. Write documentation: architecture, APIs, deployment guide
12. Capstone demo: live presentation of autonomous humanoid

**Estimated Duration:** 10-12 lessons (full integration complexity)

**Teaching Modality:**
- **Primary:** Project-based learning (student-driven capstone project)
- **Secondary:** Peer review (students review each other's architectures)
- **Rationale:** Capstone is student-driven. Minimal instructor guidance; students own the project.

**Reusable Intelligence (Stage 3):**
- **system-integration-skill**: Patterns for integrating complex robotic systems
- **robotics-debugging-skill**: Debugging patterns for distributed robotic systems
- **demo-preparation-skill**: Patterns for preparing and presenting robotic demonstrations

---

## Part 4 Success Criteria

### Technical Success
- [ ] **SC-4.1:** Voice recognition functional (&lt;2s latency, &gt;90% accuracy on clear speech)
- [ ] **SC-4.2:** Intent parsing correct for defined command vocabulary
- [ ] **SC-4.3:** LLM generates valid action sequences (parseable, executable)
- [ ] **SC-4.4:** Action execution integrates with Nav2 (navigation commands work)
- [ ] **SC-4.5:** Error recovery functional (replan on failure)
- [ ] **SC-4.6:** End-to-end latency acceptable (&lt;10s voice to action start)
- [ ] **SC-4.7:** Capstone demo successful (autonomous task completion from voice)

### Pedagogical Success
- [ ] All lessons follow 4-layer teaching framework (1→2→3→4)
- [ ] Teaching modality varies: specification-first, co-learning, project-based
- [ ] "Try With AI" sections demonstrate AI as co-worker (not teacher)
- [ ] Exercises have checkpoint success criteria
- [ ] Capstone integrates all 4 parts (ROS2 + Simulation + Perception + VLA)

### Factual Accuracy Success
- [ ] Whisper API usage verified against OpenAI documentation
- [ ] LLM prompting patterns tested with multiple LLMs (GPT-4, Claude)
- [ ] ROS 2 action server patterns validated
- [ ] All code examples execute successfully
- [ ] Version compatibility documented

---

## Connection to Other Parts

### From Part 1
- ROS 2 nodes/topics/services → Voice intent publisher (Chapter 10)
- ROS 2 actions → Action primitive execution (Chapter 11)
- Python rclpy skills → All VLA components (Chapters 10-12)
- URDF humanoid → Target platform for voice control

### From Part 2
- Gazebo/Unity simulation → Testing environment for VLA pipeline
- Sensor data → Perception for object detection (Chapter 12)
- HRI concepts → Voice-based human-robot interaction

### From Part 3
- Isaac Sim → High-fidelity testing of autonomous behavior
- VSLAM → Localization for navigation commands
- Nav2 → Navigation execution for "go to X" commands
- Perception skills → Object detection for task grounding

### To Future Work
- Multi-modal inputs (gesture + voice)
- Learning from demonstration
- Real robot deployment
- Multi-robot coordination with voice commands

---

## Non-Goals (Explicit Exclusions)

### Chapter 10 Non-Goals
- ❌ **Custom ASR model training**: Use Whisper as-is, don't train custom speech models
- ❌ **Multi-language support**: Focus on English, multilingual is advanced extension
- ❌ **Speaker identification**: Focus on commands, not who said them
- ❌ **Continuous conversation**: Focus on command/response, not dialogue systems

**Why excluded**: Voice input is a means to an end (robotic action). Deep NLP is scope creep.

### Chapter 11 Non-Goals
- ❌ **Custom LLM training/fine-tuning**: Use pre-trained LLMs via API
- ❌ **Complex manipulation planning**: Focus on navigation, simple manipulation only
- ❌ **Multi-agent planning**: Single robot, not coordinated teams
- ❌ **Symbolic AI planners**: Use LLMs, not PDDL/HTN planners

**Why excluded**: LLM-as-planner is the modern approach. Traditional planning is separate topic.

### Chapter 12 Non-Goals
- ❌ **Physical robot deployment**: Focus on simulation, not hardware
- ❌ **Production hardening**: Focus on demo quality, not production robustness
- ❌ **Performance benchmarking against state-of-art**: Student project, not research paper
- ❌ **Multi-modal perception**: Voice input, visual perception, but not combined reasoning

**Why excluded**: Capstone should be achievable by students in reasonable time.

---

## Risk Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| LLM API costs | High | Provide cost estimation, suggest local models (Ollama), set usage limits |
| Whisper accuracy issues | Medium | Provide troubleshooting guide, test environment recommendations (quiet room) |
| LLM hallucination (invalid actions) | High | Implement plan validation, action whitelist, safety constraints |
| Integration complexity | High | Provide reference architecture, incremental integration guide |
| Real-time performance | Medium | Profile early, optimize bottlenecks, provide fallback (slower but working) |
| API dependency on internet | Medium | Document local alternatives (Whisper local, Ollama for LLMs) |
| Capstone scope creep | High | Provide minimal viable capstone definition, encourage iteration |
| Student overwhelm (C2 tier) | Medium | Break capstone into phases, provide checkpoints, celebrate incremental progress |

---

## Teaching Modality Variation (Anti-Convergence)

**Part 3 Patterns Used:**
- Chapter 7 (Isaac Sim): Hands-on discovery + Socratic dialogue
- Chapter 8 (VSLAM): Error analysis + Specification-first
- Chapter 9 (Nav2): Collaborative parameter tuning + BT design

**Part 4 Variation Strategy:**

**Chapter 10 (Voice):**
- **Primary:** Specification-first (define intent schema before parser)
- **Secondary:** Iterative refinement (test → debug → improve)
- **Rationale:** Voice interfaces require upfront design. Intent schemas must be clear.

**Chapter 11 (LLM Planning):**
- **Primary:** Co-learning with AI (prompt engineering together)
- **Secondary:** Error-driven development (design for failure)
- **Rationale:** LLM prompting is iterative. Student and AI are co-workers refining together.

**Chapter 12 (Capstone):**
- **Primary:** Project-based learning (student-driven)
- **Secondary:** Peer review (architecture review)
- **Rationale:** C2 tier means student independence. They own the project.

**Result**: Zero repeated modalities from Part 3. Each chapter uses distinct teaching approach.

---

## Intelligence Accumulation Plan

### Skills to Create (Stage 3)

**Chapter 10:**
1. **voice-intent-parsing-skill**
   - Persona: Voice interface engineer
   - Questions: What command vocabulary? What entities need extraction? How to handle ambiguity?
   - Principles: Define intent schema first, extract entities systematically, handle unknown gracefully

2. **audio-pipeline-optimization-skill**
   - Persona: Real-time audio engineer
   - Questions: What's the latency budget? Where are the bottlenecks? Buffer size tradeoffs?
   - Principles: Profile first, stream don't batch, VAD before transcription

**Chapter 11:**
3. **llm-robotics-prompting-skill**
   - Persona: LLM prompt engineer for robotics
   - Questions: What action primitives exist? What world state is available? How to ensure valid output?
   - Principles: System prompt defines capabilities, JSON mode for structure, validate before execute

4. **action-primitive-design-skill**
   - Persona: Robotics action architect
   - Questions: What's the atomic action set? What preconditions/postconditions? How to compose?
   - Principles: Actions are atomic, preconditions explicit, postconditions update world state

5. **error-recovery-skill**
   - Persona: Fault-tolerant systems engineer
   - Questions: What can fail? How to detect failure? What recovery strategies?
   - Principles: Detect fast, retry with backoff, replan if retry fails, ask human if stuck

**Chapter 12:**
6. **system-integration-skill**
   - Persona: Systems integration architect
   - Questions: How do components communicate? What's the data flow? Where are the failure points?
   - Principles: Clear interfaces, async where possible, monitor everything, graceful degradation

7. **robotics-debugging-skill**
   - Persona: Robotics debugging specialist
   - Questions: Where in the pipeline did it fail? What data shows the issue? How to reproduce?
   - Principles: Binary search (which component failed), log everything, visualize in RViz/RQt

8. **demo-preparation-skill**
   - Persona: Robotics demo specialist
   - Questions: What's the story? What can go wrong? How to handle failure gracefully?
   - Principles: Practice the happy path, have recovery plan, explain what robot is doing

### Skills to Reuse (from Parts 1-3)

- **rclpy-node-patterns** (Part 1): Python ROS 2 node patterns
- **sensor-processing-skill** (Part 2): Process sensor data for perception
- **nav2-humanoid-config-skill** (Part 3): Navigation configuration
- **behavior-tree-design-skill** (Part 3): Task sequencing patterns
- **vslam-debugging-skill** (Part 3): Localization debugging

---

## Research Phase Plan (Pre-Spec)

**Total Budget:** 20-28 hours across all 3 chapters

### Chapter 10 Research (8-10 hours)

**Priority 1: Whisper Setup & API** (3 hours)
- OpenAI Whisper API documentation
- Local Whisper installation (whisper.cpp, faster-whisper)
- Performance comparison (API vs local)
- Tools: WebFetch (OpenAI docs), Context7 (if available)

**Priority 2: Real-time Audio Processing** (3 hours)
- PyAudio/sounddevice for audio capture
- Voice Activity Detection (VAD) implementations
- Audio buffering strategies
- Tools: WebFetch (audio processing tutorials)

**Priority 3: Intent Parsing Approaches** (2 hours)
- Rule-based intent parsing
- LLM-based intent extraction
- Hybrid approaches
- Tools: WebFetch (NLU frameworks, Rasa concepts)

**Priority 4: ROS 2 Audio Integration** (2 hours)
- audio_common ROS 2 packages
- Custom audio message types
- Real-time ROS 2 patterns
- Tools: Context7 (ROS 2 audio packages)

### Chapter 11 Research (8-10 hours)

**Priority 1: LLM Prompting for Robotics** (4 hours)
- Structured output with LLMs (JSON mode, function calling)
- Chain-of-thought for planning
- Existing robotics + LLM papers (SayCan, PaLM-E concepts)
- Tools: WebFetch (research papers, blog posts)

**Priority 2: Action Primitive Design** (2 hours)
- Robot action libraries (MoveIt concepts, behavior primitives)
- Precondition/postcondition patterns
- Action composition strategies
- Tools: WebFetch (robotics action papers)

**Priority 3: Error Recovery Patterns** (2 hours)
- Fault tolerance in robotics
- Replan-on-failure strategies
- Human-in-the-loop recovery
- Tools: WebFetch (fault-tolerant robotics)

**Priority 4: ROS 2 Action Integration** (2 hours)
- ROS 2 action server patterns
- Action feedback handling
- Action cancellation
- Tools: Context7 (ROS 2 actions)

### Chapter 12 Research (4-6 hours)

**Priority 1: Integration Architecture Patterns** (2 hours)
- Microservices vs monolithic for robotics
- Message passing patterns
- State management
- Tools: WebFetch (robotics architecture papers)

**Priority 2: Debugging Distributed Systems** (2 hours)
- ROS 2 debugging tools (ros2 doctor, rqt)
- Distributed system tracing
- Log aggregation
- Tools: Context7 (ROS 2 debugging)

**Priority 3: Demo Best Practices** (2 hours)
- Robotics demo guidelines
- Failure recovery during demos
- Presentation techniques
- Tools: WebFetch (robotics demo guides)

---

## File Structure

```
specs/book/
└── part-4-spec.md (this file)

book-source/docs/Part-4-Vision-Language-Action/
├── index.md (Part 4 introduction)
├── 10-voice-to-action/
│   ├── index.md (Chapter 10 introduction)
│   ├── 01-speech-recognition-fundamentals.md
│   ├── 02-whisper-setup.md
│   ├── 03-audio-streaming.md
│   ├── 04-voice-activity-detection.md
│   ├── 05-intent-parsing.md
│   ├── 06-ros2-integration.md
│   ├── 07-error-handling.md
│   ├── 08-latency-optimization.md
│   └── 09-capstone-voice-pipeline.md
├── 11-cognitive-planning/
│   ├── index.md (Chapter 11 introduction)
│   ├── 01-task-decomposition.md
│   ├── 02-action-primitives.md
│   ├── 03-llm-prompting-robotics.md
│   ├── 04-structured-output.md
│   ├── 05-world-state.md
│   ├── 06-plan-validation.md
│   ├── 07-error-recovery.md
│   ├── 08-ros2-action-integration.md
│   ├── 09-safety-constraints.md
│   └── 10-capstone-llm-planner.md
└── 12-capstone-autonomous-humanoid/
    ├── index.md (Chapter 12 introduction)
    ├── 01-capstone-requirements.md
    ├── 02-system-architecture.md
    ├── 03-integration-layer.md
    ├── 04-object-detection.md
    ├── 05-manipulation-primitives.md
    ├── 06-system-monitoring.md
    ├── 07-end-to-end-testing.md
    ├── 08-debugging-distributed.md
    ├── 09-latency-optimization.md
    ├── 10-demo-preparation.md
    ├── 11-documentation.md
    └── 12-final-presentation.md
```

---

## Capstone Project Detailed Requirements

### Minimum Viable Capstone

**User Story:** As a user, I can give a voice command to the humanoid robot, and it will autonomously navigate to complete the task.

**Example Commands:**
- "Go to the kitchen"
- "Find the red ball"
- "Come back to me"
- "Stop"

**Success Criteria:**
1. Voice command recognized within 2 seconds
2. Valid plan generated within 3 seconds
3. Robot begins executing within 5 seconds of command
4. Robot navigates to goal (or reports inability)
5. Robot provides feedback (verbal or visual) on completion

### Stretch Goals (Optional Extensions)

- Wake word detection ("Hey Robot")
- Multi-step commands ("Go to kitchen, then come back")
- Object manipulation ("Pick up the red ball")
- Question answering ("Where are you?", "What do you see?")
- Error explanation ("I can't reach that location because...")

### Deliverables

1. **Working Simulation**: Voice → Plan → Execute in Isaac Sim or Gazebo
2. **Architecture Document**: Component diagram, data flows, APIs
3. **Demo Video**: 2-5 minute video showing end-to-end functionality
4. **Source Code**: Documented, runnable ROS 2 packages
5. **Reflection Document**: What worked, what didn't, future improvements

---

## Amendment Log

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-17 | Initial Part 4 specification created using book-scaffolding skill |
