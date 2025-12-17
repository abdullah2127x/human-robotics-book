# Tasks: Chapter 10 - Voice-to-Action

**Input**: Design documents from `/specs/001-voice-to-action/`
**Prerequisites**: plan.md (9 lessons), spec.md (6 user stories)
**Feature Branch**: `001-voice-to-action`
**Created**: 2025-12-17

**Organization**: Tasks are grouped by lesson (Layer progression) mapping to user stories from spec.md.

## Format: `[ID] [P?] [Lesson] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[L#]**: Which lesson this task belongs to (L1-L9)
- Include exact file paths in descriptions

## Path Conventions

- **Book content**: `book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/`
- **Code examples**: `book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/code/`
- **Assets**: `book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/assets/`

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Create chapter structure and scaffolding

- [ ] T001 Create chapter directory structure at book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/
- [ ] T002 [P] Create _category_.json for Chapter 10 navigation in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/
- [ ] T003 [P] Create chapter README/index.md with learning objectives in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/index.md
- [ ] T004 [P] Create code/ subdirectory for Python examples
- [ ] T005 [P] Create assets/ subdirectory for diagrams and images

**Checkpoint**: Chapter structure ready for lesson content

---

## Phase 2: Foundational (Part 4 Introduction)

**Purpose**: Part 4 context that must exist before Chapter 10 lessons

**⚠️ CRITICAL**: Part 4 introduction must be created before chapter content

- [ ] T006 Create Part 4 index.md with Vision-Language-Action overview in book-source/docs/Part-4-Vision-Language-Action/index.md
- [ ] T007 [P] Create Part 4 _category_.json for navigation in book-source/docs/Part-4-Vision-Language-Action/
- [ ] T008 [P] Create prerequisite verification checklist (Parts 1-3 completion) in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/prerequisites.md

**Checkpoint**: Part 4 foundation ready - lesson implementation can begin

---

## Phase 3: User Story 1 - Basic Voice Command Recognition (Priority: P1) 🎯 MVP

**Goal**: Students can transcribe spoken commands with Whisper (Lessons 1-2)

**Independent Test**: Student speaks "Go to the kitchen" and sees transcribed text within 2 seconds

**Maps to Spec**: User Story 1 (Basic Voice Command Recognition)

### Lesson 1: Speech Recognition Fundamentals (Layer 1)

- [ ] T009 [L1] Create lesson-01-speech-recognition-fundamentals.md with learning objectives in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-01-speech-recognition-fundamentals.md
- [ ] T010 [P] [L1] Write "Why Voice Interfaces for Robots?" motivation section explaining natural human-robot communication
- [ ] T011 [P] [L1] Write "Audio Representation" section covering waveforms, samples, sample rates (16kHz)
- [ ] T012 [P] [L1] Write "Feature Extraction" section covering spectrograms and mel-frequency cepstral coefficients
- [ ] T013 [L1] Write "Acoustic and Language Models" section explaining the speech-to-text pipeline
- [ ] T014 [L1] Write "Evaluation Metrics" section covering Word Error Rate (WER) and accuracy measurement
- [ ] T015 [P] [L1] Create code/lesson_01_audio_visualization.py with waveform and spectrogram generation examples
- [ ] T016 [P] [L1] Create assets/speech-pipeline-diagram.png showing audio → features → model → text flow
- [ ] T017 [L1] Write practice exercise: "Record and visualize voice commands" with checkbox success criteria
- [ ] T018 [L1] Write lesson checkpoint: "Explain how audio becomes text" comprehension questions

### Lesson 2: Setting Up OpenAI Whisper (Layer 1)

- [ ] T019 [L2] Create lesson-02-setting-up-whisper.md with learning objectives in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-02-setting-up-whisper.md
- [ ] T020 [P] [L2] Write "Whisper Architecture" section explaining encoder-decoder transformer
- [ ] T021 [P] [L2] Write "Model Size Selection" section covering tiny/base/small/medium/large tradeoffs
- [ ] T022 [L2] Write "Whisper API Setup" section with step-by-step API key creation and Python client usage
- [ ] T023 [L2] Write "Local Whisper Installation" section covering faster-whisper and whisper.cpp options
- [ ] T024 [P] [L2] Create code/lesson_02_whisper_api.py demonstrating Whisper API transcription
- [ ] T025 [P] [L2] Create code/lesson_02_whisper_local.py demonstrating local model transcription
- [ ] T026 [L2] Write "Configuration and Parameters" section (language, temperature, prompt engineering)
- [ ] T027 [L2] Write practice exercise: "Transcribe 10 robot commands and measure WER" with success criteria
- [ ] T028 [L2] Write lesson checkpoint: "Whisper working, model tradeoffs understood"

**Checkpoint**: User Story 1 complete - Students can transcribe voice commands with 90% accuracy

---

## Phase 4: User Story 2 - Intent Parsing from Natural Language (Priority: P1)

**Goal**: Students can parse transcribed text into structured intent objects (Lessons 5-6)

**Independent Test**: Given "Pick up the red ball", parser outputs `{action: "pick", target: "ball", parameters: {color: "red"}}`

**Maps to Spec**: User Story 2 (Intent Parsing from Natural Language)

### Lesson 5: Designing Intent Schemas (Layer 2)

- [ ] T029 [L5] Create lesson-05-designing-intent-schemas.md with learning objectives in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-05-designing-intent-schemas.md
- [ ] T030 [P] [L5] Write "Intent Representation" section covering action, target, parameters, confidence structure
- [ ] T031 [P] [L5] Write "Intent Categories for Robotics" section (navigate, pick, place, query, control)
- [ ] T032 [L5] Write "Entity Extraction" section covering locations, objects, colors, quantities
- [ ] T033 [L5] Write "Slot Filling and Required Parameters" section with examples
- [ ] T034 [L5] Write "Ambiguity Handling" section covering missing targets and unclear commands
- [ ] T035 [P] [L5] Create code/lesson_05_intent_schema.py defining Intent dataclass and schema validation
- [ ] T036 [L5] Write "Try With AI: Schema Design" section demonstrating Three Roles (Teacher, Student, Co-Worker)
- [ ] T037 [L5] Write practice exercise: "Design intents for 5 command categories" with success criteria
- [ ] T038 [L5] Write lesson checkpoint: "Schema covers all command types, handles ambiguity"

### Lesson 6: Implementing Intent Parsers (Layer 2)

- [ ] T039 [L6] Create lesson-06-implementing-intent-parsers.md with learning objectives in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-06-implementing-intent-parsers.md
- [ ] T040 [P] [L6] Write "Rule-Based Parsing" section with regex and keyword matching examples
- [ ] T041 [P] [L6] Write "LLM-Based Parsing" section with prompt engineering for intent extraction
- [ ] T042 [L6] Write "Hybrid Approaches" section (rules first, LLM fallback for ambiguous)
- [ ] T043 [L6] Write "Entity Recognition" section covering regex patterns and spaCy NER
- [ ] T044 [L6] Write "Confidence Scoring" section explaining parser certainty calculation
- [ ] T045 [P] [L6] Create code/lesson_06_rule_parser.py implementing rule-based intent parser
- [ ] T046 [P] [L6] Create code/lesson_06_llm_parser.py implementing LLM-based intent parser
- [ ] T047 [P] [L6] Create code/lesson_06_hybrid_parser.py implementing hybrid parser with fallback
- [ ] T048 [L6] Write "Try With AI: Parser Debugging" section demonstrating Three Roles pattern
- [ ] T049 [L6] Write practice exercise: "Parse 50 test commands with >90% accuracy" with success criteria
- [ ] T050 [L6] Write lesson checkpoint: "Parser achieves target accuracy on test set"

**Checkpoint**: User Story 2 complete - Students can parse 5 command types correctly

---

## Phase 5: User Story 3 - Real-Time Audio Capture Pipeline (Priority: P2)

**Goal**: Students can capture audio with VAD for command segmentation (Lessons 3-4)

**Independent Test**: VAD triggers within 200ms of speech onset, segments commands after 1s silence

**Maps to Spec**: User Story 3 (Real-Time Audio Capture Pipeline)

### Lesson 3: Real-Time Audio Capture (Layer 2)

- [ ] T051 [L3] Create lesson-03-real-time-audio-capture.md with learning objectives in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-03-real-time-audio-capture.md
- [ ] T052 [P] [L3] Write "Audio Input Devices" section covering microphone selection and USB audio
- [ ] T053 [P] [L3] Write "PyAudio vs sounddevice" section comparing library options
- [ ] T054 [L3] Write "Sample Rate Configuration" section explaining 16kHz requirement for Whisper
- [ ] T055 [L3] Write "Audio Buffering" section covering chunk size and callback vs blocking modes
- [ ] T056 [L3] Write "Real-Time Constraints" section explaining processing speed requirements
- [ ] T057 [P] [L3] Create code/lesson_03_audio_capture.py with PyAudio/sounddevice capture examples
- [ ] T058 [P] [L3] Create code/lesson_03_audio_test.py for microphone testing and device enumeration
- [ ] T059 [L3] Write "Try With AI: Debugging Audio Issues" section demonstrating Three Roles pattern
- [ ] T060 [L3] Write practice exercise: "Capture live audio and feed to Whisper" with success criteria
- [ ] T061 [L3] Write lesson checkpoint: "Audio captures at 16kHz without drops"

### Lesson 4: Voice Activity Detection (Layer 2)

- [ ] T062 [L4] Create lesson-04-voice-activity-detection.md with learning objectives in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-04-voice-activity-detection.md
- [ ] T063 [P] [L4] Write "VAD Problem Definition" section explaining speech vs silence vs noise detection
- [ ] T064 [P] [L4] Write "Energy-Based VAD" section with amplitude thresholding examples
- [ ] T065 [L4] Write "WebRTC VAD" section covering Google's production VAD library usage
- [ ] T066 [L4] Write "Silero VAD" section covering neural network-based VAD
- [ ] T067 [L4] Write "Command Segmentation" section covering speech onset/offset and hangover time
- [ ] T068 [P] [L4] Create code/lesson_04_energy_vad.py implementing energy-based VAD
- [ ] T069 [P] [L4] Create code/lesson_04_webrtc_vad.py implementing WebRTC VAD
- [ ] T070 [P] [L4] Create code/lesson_04_silero_vad.py implementing Silero VAD
- [ ] T071 [L4] Write "Try With AI: VAD Threshold Tuning" section demonstrating Three Roles pattern
- [ ] T072 [L4] Write practice exercise: "Test VAD in quiet and noisy environments" with success criteria
- [ ] T073 [L4] Write lesson checkpoint: "VAD segments commands with <10% false positive rate"

**Checkpoint**: User Story 3 complete - Students can capture and segment voice commands

---

## Phase 6: User Story 4 - ROS 2 Integration (Priority: P2)

**Goal**: Students can publish parsed intents to ROS 2 topics (Lesson 7)

**Independent Test**: `ros2 topic echo /voice_intent` shows correct intent messages

**Maps to Spec**: User Story 4 (ROS 2 Integration)

### Lesson 7: ROS 2 Integration (Layer 2)

- [ ] T074 [L7] Create lesson-07-ros2-integration.md with learning objectives in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-07-ros2-integration.md
- [ ] T075 [P] [L7] Write "Custom ROS 2 Message Types" section defining VoiceIntent.msg structure
- [ ] T076 [P] [L7] Write "Voice Node Architecture" section explaining audio → Whisper → parser → publisher flow
- [ ] T077 [L7] Write "Topic Design" section covering /voice_intent and /voice_diagnostics topics
- [ ] T078 [L7] Write "Lifecycle Node Patterns" section for managed startup/shutdown
- [ ] T079 [L7] Write "Parameter Server Integration" section for runtime configuration
- [ ] T080 [P] [L7] Create code/lesson_07_voice_intent_msg.txt with VoiceIntent.msg definition
- [ ] T081 [P] [L7] Create code/lesson_07_voice_command_node.py implementing the ROS 2 voice node
- [ ] T082 [P] [L7] Create code/lesson_07_nav2_bridge.py for intent → Nav2 goal conversion
- [ ] T083 [L7] Write "Integration Testing" section with ros2 topic echo verification
- [ ] T084 [L7] Write practice exercise: "Speak commands, verify intents reach subscribers" with success criteria
- [ ] T085 [L7] Write lesson checkpoint: "Full pipeline: speak → transcribe → parse → publish → receive"

**Checkpoint**: User Story 4 complete - Voice intents published to ROS 2 topics

---

## Phase 7: User Story 5 - Error Handling and Recovery (Priority: P3)

**Goal**: Students can handle voice recognition errors gracefully (Distributed across lessons)

**Independent Test**: System asks for clarification on unclear speech, handles API failures gracefully

**Maps to Spec**: User Story 5 (Error Handling and Recovery)

- [ ] T086 [L2] Add "Handling Whisper Failures" section to Lesson 2 covering API retry logic
- [ ] T087 [L4] Add "Noise Rejection and False Positive Handling" section to Lesson 4
- [ ] T088 [L6] Add "Low Confidence Handling" section to Lesson 6 covering clarification requests
- [ ] T089 [P] Create code/error_handling_examples.py demonstrating retry, fallback, and graceful degradation
- [ ] T090 Write "Error Handling Patterns" appendix in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/appendix-error-handling.md

**Checkpoint**: User Story 5 complete - System handles errors without crashing

---

## Phase 8: User Story 6 - Latency Optimization (Priority: P3)

**Goal**: Students can optimize voice pipeline for <2 second latency (Distributed across lessons)

**Independent Test**: End-to-end latency measured and optimized per pipeline stage

**Maps to Spec**: User Story 6 (Latency Optimization)

- [ ] T091 [L2] Add "Latency Considerations: API vs Local" section to Lesson 2
- [ ] T092 [L3] Add "Buffer Size and Latency Tradeoffs" section to Lesson 3
- [ ] T093 [L6] Add "Parser Performance: Rules vs LLM Latency" section to Lesson 6
- [ ] T094 [P] Create code/latency_profiler.py for measuring pipeline stage latencies
- [ ] T095 Write "Latency Optimization Guide" appendix in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/appendix-latency-optimization.md

**Checkpoint**: User Story 6 complete - Students can measure and optimize latency

---

## Phase 9: Lesson 8 - Voice-Intent-Parsing Skill (Layer 3)

**Goal**: Students create reusable voice-intent-parsing skill (Intelligence Design)

**Maps to Spec**: SC-011 (Students create reusable voice-intent-parsing skill)

- [ ] T096 [L8] Create lesson-08-voice-intent-parsing-skill.md with learning objectives in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-08-voice-intent-parsing-skill.md
- [ ] T097 [L8] Write "Pattern Extraction" section reviewing parsing patterns from Lessons 5-6
- [ ] T098 [L8] Write "Skill Design Framework" section (Persona + Questions + Principles)
- [ ] T099 [L8] Write Persona Definition: "Think like a voice interface designer..."
- [ ] T100 [L8] Write Question Structure (5 questions for intent parser design)
- [ ] T101 [L8] Write Principle Articulation (5 principles: Schema Before Parser, Entity Extraction, etc.)
- [ ] T102 [P] [L8] Create code/voice_intent_parsing_skill.md with the complete skill template
- [ ] T103 [L8] Write "Usage Validation" section testing skill on different command vocabularies
- [ ] T104 [L8] Write practice exercise: "Apply skill to home automation commands" with success criteria
- [ ] T105 [L8] Write lesson checkpoint: "Skill generalizes to new domains"

**Checkpoint**: Layer 3 complete - Reusable skill created

---

## Phase 10: Lesson 9 - Capstone (Layer 4)

**Goal**: Full voice-controlled humanoid navigation integration

**Maps to Spec**: All success criteria integrated (SC-001 through SC-011)

- [ ] T106 [L9] Create lesson-09-capstone-voice-controlled-navigation.md in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/lesson-09-capstone-voice-controlled-navigation.md
- [ ] T107 [L9] Write "Capstone Specification" section (spec-first: define requirements before implementation)
- [ ] T108 [L9] Write "Component Composition" section listing all skills/lessons to integrate
- [ ] T109 [L9] Write "Integration Workflow" step-by-step guide
- [ ] T110 [L9] Write "Nav2 Connection" section bridging to Part 3 Chapter 9
- [ ] T111 [P] [L9] Create code/lesson_09_voice_navigation_system.py integrating all components
- [ ] T112 [P] [L9] Create code/lesson_09_capstone_spec.md template for student specification
- [ ] T113 [L9] Write "Success Validation" section with all capstone criteria as checkboxes
- [ ] T114 [L9] Write "Final Demo" instructions (5-minute voice-controlled navigation demonstration)
- [ ] T115 [L9] Write lesson checkpoint: "Voice command triggers humanoid navigation"

**Checkpoint**: Capstone complete - Full voice pipeline integrated with humanoid

---

## Phase 11: Polish & Cross-Cutting Concerns

**Purpose**: Chapter-wide improvements and finalization

- [ ] T116 [P] Create chapter summary in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/summary.md
- [ ] T117 [P] Create glossary of voice/NLU terms in book-source/docs/Part-4-Vision-Language-Action/Chapter-10-Voice-to-Action/glossary.md
- [ ] T118 [P] Create troubleshooting guide for common audio/Whisper issues
- [ ] T119 Verify all code examples run correctly (Python 3.10+, ROS 2 Humble)
- [ ] T120 Verify all lesson checkpoints have testable criteria
- [ ] T121 Review Three Roles demonstrations in Lessons 3-6 for clarity
- [ ] T122 Ensure specification-first teaching pattern is clear in Lessons 5, 9
- [ ] T123 Final proofreading and formatting consistency check

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - Part 4 intro required before Chapter 10 content
- **User Story 1 (Phase 3)**: Depends on Foundational - Lessons 1-2 (MVP: basic transcription)
- **User Story 2 (Phase 4)**: Depends on Phase 3 - Lessons 5-6 require understanding from L1-2
- **User Story 3 (Phase 5)**: Can run parallel with Phase 4 after Phase 3 - Lessons 3-4
- **User Story 4 (Phase 6)**: Depends on Phases 4, 5 - Lesson 7 integrates all prior lessons
- **User Stories 5-6 (Phases 7-8)**: Distributed additions to existing lessons
- **Layer 3 (Phase 9)**: Depends on Phase 4 - Lesson 8 extracts patterns from L5-6
- **Capstone (Phase 10)**: Depends on all prior phases - Lesson 9 integrates everything
- **Polish (Phase 11)**: Depends on all lesson content being complete

### Lesson Dependency Graph

```
Lesson 1 (Speech Fundamentals) → Lesson 2 (Whisper Setup)
Lesson 2 → Lesson 3 (Audio Capture)
Lesson 3 → Lesson 4 (VAD)
Lesson 4 → Lesson 5 (Intent Schemas)
Lesson 5 → Lesson 6 (Intent Parsers)
Lesson 6 → Lesson 7 (ROS 2 Integration)
Lessons 5-6 → Lesson 8 (Skill Creation)
Lessons 1-8 + Part 3 Nav2 → Lesson 9 (Capstone)
```

### Parallel Opportunities

**Within Setup Phase:**
- T002, T003, T004, T005 can all run in parallel

**Within Each Lesson:**
- Theory sections (Txx1, Txx2, Txx3) can often run in parallel
- Code examples marked [P] can run in parallel
- Practice exercises depend on all theory sections

**Across Phases:**
- Once Phase 3 completes, Phases 4 and 5 can run in parallel
- Phase 7 and 8 additions can run in parallel with Phases 4-6

---

## Implementation Strategy

### MVP First (Lessons 1-2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (Part 4 intro)
3. Complete Phase 3: User Story 1 (Lessons 1-2)
4. **STOP and VALIDATE**: Students can transcribe voice commands
5. Publish preview if ready

### Incremental Delivery

1. Setup + Foundational → Chapter structure ready
2. Add Lessons 1-2 (Phase 3) → Basic transcription working (MVP!)
3. Add Lessons 3-4 (Phase 5) → Audio capture + VAD working
4. Add Lessons 5-6 (Phase 4) → Intent parsing working
5. Add Lesson 7 (Phase 6) → ROS 2 integration complete
6. Add Lesson 8 (Phase 9) → Reusable skill created
7. Add Lesson 9 (Phase 10) → Capstone integrates all
8. Polish (Phase 11) → Chapter finalized

### Suggested Execution Order

For sequential implementation:
1. T001-T005 (Setup)
2. T006-T008 (Foundational)
3. T009-T018 (Lesson 1)
4. T019-T028 (Lesson 2) — **MVP complete here**
5. T051-T061 (Lesson 3)
6. T062-T073 (Lesson 4)
7. T029-T038 (Lesson 5)
8. T039-T050 (Lesson 6)
9. T074-T085 (Lesson 7)
10. T086-T095 (Error handling + Latency additions)
11. T096-T105 (Lesson 8)
12. T106-T115 (Lesson 9)
13. T116-T123 (Polish)

---

## Summary Statistics

| Metric | Count |
|--------|-------|
| **Total Tasks** | 123 |
| **Setup Tasks** | 5 |
| **Foundational Tasks** | 3 |
| **Lesson 1 Tasks** | 10 |
| **Lesson 2 Tasks** | 10 |
| **Lesson 3 Tasks** | 11 |
| **Lesson 4 Tasks** | 12 |
| **Lesson 5 Tasks** | 10 |
| **Lesson 6 Tasks** | 12 |
| **Lesson 7 Tasks** | 12 |
| **Lesson 8 Tasks** | 10 |
| **Lesson 9 Tasks** | 10 |
| **Error Handling Tasks** | 5 |
| **Latency Optimization Tasks** | 5 |
| **Polish Tasks** | 8 |
| **Parallelizable Tasks [P]** | 47 |

### User Story Coverage

| User Story | Priority | Primary Lessons | Tasks |
|------------|----------|-----------------|-------|
| US1: Basic Voice Recognition | P1 | L1, L2 | 20 |
| US2: Intent Parsing | P1 | L5, L6 | 22 |
| US3: Real-Time Audio + VAD | P2 | L3, L4 | 23 |
| US4: ROS 2 Integration | P2 | L7 | 12 |
| US5: Error Handling | P3 | Distributed | 5 |
| US6: Latency Optimization | P3 | Distributed | 5 |

### MVP Scope

**Minimum Viable Chapter**: Phases 1-3 (28 tasks)
- Chapter structure + Part 4 intro
- Lessons 1-2: Students can transcribe voice commands with Whisper
- Delivers immediate value: speech-to-text working

---

## Notes

- [P] tasks = different files, no dependencies within the lesson
- [L#] label maps task to specific lesson for traceability
- Each lesson should be independently completable
- Verify code examples run before marking lesson complete
- Commit after each lesson or logical group
- Stop at any checkpoint to validate lesson independently
- Three Roles demonstrations required in Lessons 3-6 (Layer 2)
- Specification-first pattern emphasized in Lessons 5, 9
