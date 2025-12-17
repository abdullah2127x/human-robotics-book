# Feature Specification: Chapter 10 - Voice-to-Action

**Feature Branch**: `001-voice-to-action`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Chapter 10: Voice-to-Action - Using OpenAI Whisper for Voice Commands based on Part 4 spec"

---

## Overview

Chapter 10 is the first chapter of Part 4 (Vision-Language-Action) and teaches students to build voice command pipelines for humanoid robots. Students learn to capture audio, transcribe speech using OpenAI Whisper, parse natural language into structured intents, and publish those intents to ROS 2 for robotic action execution.

**Part Context**: Part 4 represents the convergence of LLMs and robotics. Chapter 10 establishes the voice input foundation that feeds into Chapter 11 (LLM planning) and Chapter 12 (full autonomous capstone).

**Prerequisites**: Students must have completed Parts 1-3:
- Part 1: ROS 2 nodes, topics, services, Python rclpy
- Part 2: Gazebo/Unity simulation, sensor processing
- Part 3: Isaac Sim, VSLAM, Nav2 navigation

**Proficiency Tier**: C1 (Advanced Integration)
- Concept limit: 10-12 concepts per section
- Scaffolding: Light (student-driven exploration)
- Teaching modality: Specification-first (design intent schemas before implementation)

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Basic Voice Command Recognition (Priority: P1)

A student wants to give voice commands to a humanoid robot and see the commands recognized and transcribed in real-time.

**Why this priority**: This is the foundational capability. Without speech-to-text working, no other voice features can function. Students must verify Whisper setup before building on it.

**Independent Test**: Can be fully tested by speaking into a microphone and seeing transcribed text output. Delivers immediate value by proving the speech recognition pipeline works.

**Acceptance Scenarios**:

1. **Given** a configured Whisper setup (API or local), **When** a student speaks "Go to the kitchen" into the microphone, **Then** the system outputs the transcribed text "Go to the kitchen" within 2 seconds.

2. **Given** Whisper is running, **When** a student speaks a navigation command in a quiet environment, **Then** the transcription accuracy is at least 90% for clear English speech.

3. **Given** Whisper is running, **When** there is silence (no speech), **Then** the system does not produce false transcriptions (no hallucinations).

---

### User Story 2 - Intent Parsing from Natural Language (Priority: P1)

A student wants to transform transcribed text into structured intent objects that a robot can act upon.

**Why this priority**: Raw transcription is not useful to robots. Intent parsing bridges human language to robotic action. This is equally critical to transcription for the voice pipeline to function.

**Independent Test**: Can be tested by providing transcribed text strings and verifying correct intent JSON output. No audio hardware required for isolated testing.

**Acceptance Scenarios**:

1. **Given** the text "Go to the kitchen", **When** passed through the intent parser, **Then** the output is `{action: "navigate", target: "kitchen", parameters: {}}`.

2. **Given** the text "Pick up the red ball", **When** parsed, **Then** the output is `{action: "pick", target: "ball", parameters: {color: "red"}}`.

3. **Given** the text "Hello, how are you?", **When** parsed (non-command), **Then** the output is `{action: "unknown", target: null, parameters: {}}` with appropriate error handling.

4. **Given** an ambiguous command "Go there", **When** parsed, **Then** the system recognizes missing target and returns `{action: "navigate", target: null, error: "target_unspecified"}`.

---

### User Story 3 - Real-Time Audio Capture Pipeline (Priority: P2)

A student wants to capture audio from a microphone in real-time with proper buffering and voice activity detection.

**Why this priority**: Production voice systems require continuous listening with efficient audio handling. This builds on P1 (basic recognition) to enable always-on voice interfaces.

**Independent Test**: Can be tested by monitoring audio buffer contents, verifying VAD triggers on speech vs silence, and measuring latency from speech start to buffer availability.

**Acceptance Scenarios**:

1. **Given** a microphone is connected, **When** audio capture starts, **Then** the system continuously captures audio at 16kHz sample rate with proper buffering.

2. **Given** continuous audio capture, **When** the user starts speaking, **Then** Voice Activity Detection (VAD) triggers within 200ms of speech onset.

3. **Given** VAD is active, **When** the user stops speaking for 1 second, **Then** the system segments the audio as a complete command and sends it for transcription.

4. **Given** background noise (air conditioning, typing), **When** no speech is present, **Then** VAD does not falsely trigger (noise rejection works).

---

### User Story 4 - ROS 2 Integration (Priority: P2)

A student wants to publish parsed intents to ROS 2 topics so downstream systems (Nav2, manipulators) can execute commands.

**Why this priority**: ROS 2 integration connects the voice pipeline to the rest of the robot system. Essential for the capstone but can be deferred while core voice processing is developed.

**Independent Test**: Can be tested by running the voice node and using `ros2 topic echo /voice_intent` to verify intent messages are published correctly.

**Acceptance Scenarios**:

1. **Given** a voice command is recognized and parsed, **When** the intent is ready, **Then** the system publishes to `/voice_intent` topic with custom message type containing action, target, parameters, timestamp, and confidence.

2. **Given** a navigation intent `{action: "navigate", target: "kitchen"}`, **When** published to ROS 2, **Then** the message is compatible with downstream Nav2 goal conversion.

3. **Given** the voice node is running, **When** a subscriber listens to `/voice_intent`, **Then** intents are received within 500ms of original speech completion.

---

### User Story 5 - Error Handling and Recovery (Priority: P3)

A student wants the voice system to handle errors gracefully—unclear speech, background noise, timeouts, and API failures.

**Why this priority**: Robustness is essential for production but can be addressed after core functionality works. Error handling refines the system rather than enabling it.

**Independent Test**: Can be tested by intentionally introducing error conditions (mumbled speech, noise, disconnected API) and verifying graceful degradation.

**Acceptance Scenarios**:

1. **Given** unclear or mumbled speech, **When** transcription confidence is below threshold (70%), **Then** the system asks for clarification: "I didn't understand. Please repeat."

2. **Given** Whisper API is unavailable, **When** a transcription request fails, **Then** the system retries once, then falls back to error state with user notification.

3. **Given** no speech detected for 30 seconds, **When** voice system is in listening mode, **Then** the system remains in standby without crashing or excessive resource usage.

4. **Given** overlapping speakers or cross-talk, **When** multiple voices are detected, **Then** the system either processes the dominant speaker or requests clarification.

---

### User Story 6 - Latency Optimization (Priority: P3)

A student wants to optimize the voice pipeline for acceptable latency, targeting under 2 seconds from speech completion to intent availability.

**Why this priority**: Performance optimization is important for user experience but can be addressed after functional correctness is established.

**Independent Test**: Can be tested by measuring end-to-end latency with timestamps at each pipeline stage: speech end → transcription complete → intent parsed → published.

**Acceptance Scenarios**:

1. **Given** a 3-second voice command, **When** processed through the full pipeline, **Then** the intent is available within 2 seconds of speech completion.

2. **Given** latency profiling tools, **When** measuring each pipeline stage, **Then** the bottleneck is identifiable (audio capture, transcription, parsing, or publishing).

3. **Given** streaming transcription option, **When** enabled, **Then** partial results are available during speech (real-time feedback).

---

### Edge Cases

- What happens when the microphone is disconnected mid-command?
- How does the system handle commands in non-English languages (Whisper is multilingual)?
- What happens if the intent schema doesn't recognize a valid command category?
- How does the system handle extremely long utterances (over 30 seconds)?
- What happens when Whisper hallucinates text during silence?
- How does the system handle commands that span multiple intents ("Go to the kitchen and pick up the cup")?

---

## Requirements *(mandatory)*

### Functional Requirements

**Audio Capture**:
- **FR-001**: System MUST capture audio from USB microphone at 16kHz sample rate, 16-bit depth
- **FR-002**: System MUST implement audio buffering with configurable chunk size (default 1024 samples)
- **FR-003**: System MUST support Voice Activity Detection (VAD) to segment speech from silence
- **FR-004**: System MUST detect speech onset within 200ms of speech start
- **FR-005**: System MUST detect speech offset after configurable silence duration (default 1 second)

**Speech Recognition**:
- **FR-006**: System MUST integrate with OpenAI Whisper for speech-to-text conversion
- **FR-007**: System MUST support both Whisper API (cloud) and local model deployment options
- **FR-008**: System MUST provide transcription results with confidence scores
- **FR-009**: System MUST support English language commands (multilingual is optional extension)
- **FR-010**: System MUST handle audio segments up to 30 seconds duration

**Intent Parsing**:
- **FR-011**: System MUST parse transcribed text into structured intent format: `{action, target, parameters, confidence}`
- **FR-012**: System MUST support navigation intents: "go to X", "navigate to X", "move to X"
- **FR-013**: System MUST support manipulation intents: "pick up X", "put down X", "grab X"
- **FR-014**: System MUST support query intents: "where are you?", "what do you see?"
- **FR-015**: System MUST support control intents: "stop", "pause", "resume", "cancel"
- **FR-016**: System MUST extract entity parameters: colors ("red ball"), locations ("kitchen"), objects ("cup")
- **FR-017**: System MUST handle unknown/unrecognized commands gracefully

**ROS 2 Integration**:
- **FR-018**: System MUST implement as ROS 2 node compatible with Humble distribution
- **FR-019**: System MUST publish intents to `/voice_intent` topic with custom message type
- **FR-020**: System MUST publish diagnostic information to `/voice_diagnostics` topic
- **FR-021**: System MUST expose parameters for runtime configuration (VAD threshold, API keys, etc.)
- **FR-022**: System MUST follow ROS 2 lifecycle node patterns for managed startup/shutdown

**Error Handling**:
- **FR-023**: System MUST detect low-confidence transcriptions and request clarification
- **FR-024**: System MUST implement retry logic for transient API failures (max 2 retries)
- **FR-025**: System MUST timeout stale audio buffers (over 60 seconds old)
- **FR-026**: System MUST log all errors with sufficient context for debugging

**Performance**:
- **FR-027**: System MUST achieve end-to-end latency under 2 seconds for typical commands
- **FR-028**: System MUST process audio without dropping frames at 16kHz rate
- **FR-029**: System MUST operate within 500MB memory footprint (excluding model weights)

### Key Entities

- **AudioBuffer**: Captured audio samples with timestamp, duration, sample rate, and VAD state
- **Transcription**: Raw text output from Whisper with confidence score, language, and timestamps
- **Intent**: Structured command representation with action type, target entity, parameters, confidence, and error state
- **VoiceCommand**: Complete pipeline output combining transcription, intent, and metadata for ROS 2 publishing

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Core Functionality**:
- **SC-001**: Students can transcribe spoken commands with at least 90% word accuracy in quiet environments
- **SC-002**: Students can parse at least 5 distinct command types (navigate, pick, query, control, unknown) correctly
- **SC-003**: Students can capture and process voice commands end-to-end (speak → intent published) within 2 seconds

**Integration**:
- **SC-004**: Voice intents are successfully published to ROS 2 topics and received by subscriber nodes
- **SC-005**: Voice pipeline integrates with existing humanoid simulation from Parts 2-3

**Robustness**:
- **SC-006**: System handles error conditions without crashing (API failures, hardware disconnects, invalid input)
- **SC-007**: System distinguishes speech from background noise with less than 10% false positive rate

**Learning Outcomes**:
- **SC-008**: Students can explain the speech recognition pipeline stages (audio → features → model → text)
- **SC-009**: Students can design intent schemas before implementing parsers (specification-first approach)
- **SC-010**: Students can measure and optimize pipeline latency using profiling techniques
- **SC-011**: Students create reusable voice-intent-parsing skill for future projects

---

## Scope Boundaries

### In Scope

- OpenAI Whisper integration (API and local model options)
- Real-time audio capture with PyAudio/sounddevice
- Voice Activity Detection for command segmentation
- Intent parsing for robotics commands (navigation, manipulation, queries, control)
- ROS 2 node implementation with custom message types
- Error handling and graceful degradation
- Latency measurement and optimization techniques
- Creating reusable skills: voice-intent-parsing-skill, audio-pipeline-optimization-skill

### Out of Scope (Non-Goals)

- **Wake word detection**: Common pattern but adds complexity; students can add as extension
- **Multi-language support**: Whisper supports it, but chapter focuses on English to manage scope
- **Speaker identification**: Who said the command is not relevant for single-user humanoid
- **Continuous conversation/dialogue**: Focus on command/response, not chatbot-style dialogue
- **Custom ASR model training**: Use Whisper as-is, no fine-tuning
- **Hardware selection/purchasing**: Assume USB microphone is available

---

## Assumptions

- Students have completed Parts 1-3 and understand ROS 2 development
- Students have access to a working microphone (USB recommended)
- Students have internet access for Whisper API OR sufficient compute for local model (GPU recommended)
- Isaac Sim or Gazebo simulation environment is available from Parts 2-3
- Students are comfortable with Python async/await patterns from Part 1
- Humanoid robot model from Part 3 is available for integration testing

---

## Dependencies

- **Part 1 Chapter 2**: rclpy Python client library, async patterns
- **Part 2 Chapter 6**: Sensor integration patterns
- **Part 3 Chapter 9**: Nav2 integration (voice → navigation)
- **OpenAI API**: For Whisper cloud transcription (or local Whisper installation)
- **PyAudio or sounddevice**: For audio capture

---

## Risks and Mitigations

| Risk                          | Impact | Mitigation                                                              |
|-------------------------------|--------|-------------------------------------------------------------------------|
| Whisper API costs accumulate  | Medium | Provide local model option, set usage limits in exercises               |
| Audio hardware issues         | Medium | Document troubleshooting, provide recorded test audio files             |
| Latency exceeds target        | Medium | Profile early, teach optimization techniques, accept graceful degradation |
| Intent parsing ambiguity      | Low    | Define clear intent schema, handle unknown gracefully                   |
| Whisper hallucinations        | Medium | Implement confidence thresholds, VAD to avoid silence transcription     |

---

## Teaching Approach

**Primary Modality**: Specification-first
- Students design intent schemas (WHAT commands to support) before implementing parsers
- Students specify latency requirements before optimization
- Mirrors professional voice interface design

**Secondary Modality**: Iterative refinement
- Test → Fail → Debug → Improve cycle for voice recognition tuning
- Real-world voice input is messy; iteration is expected

**Layer Progression**:
- Layer 1 (Manual Foundation): Whisper fundamentals, audio capture basics
- Layer 2 (AI Collaboration): Intent parsing design, error handling strategies
- Layer 3 (Intelligence Design): Create voice-intent-parsing-skill, audio-pipeline-optimization-skill
- Layer 4 (Spec-Driven Capstone): Full voice pipeline integrated with humanoid

**Anti-Convergence**: Chapter 9 used collaborative parameter tuning; Chapter 10 varies by using specification-first for intent schema design.

---

## Amendment Log

| Version | Date       | Changes                                          |
|---------|------------|--------------------------------------------------|
| 1.0.0   | 2025-12-17 | Initial specification created from Part 4 spec  |
