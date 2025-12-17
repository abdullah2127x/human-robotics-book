# Chapter 10: Voice-to-Action

**Using OpenAI Whisper for Voice Commands**

---

## Overview

In this chapter, you will build a complete voice command pipeline for humanoid robots. You'll learn to capture audio from a microphone, transcribe speech using OpenAI Whisper, parse natural language into structured intents, and publish those intents to ROS 2 for robotic action execution.

This is the first chapter of **Part 4: Vision-Language-Action**, which represents the convergence of Large Language Models (LLMs) and robotics. The voice interface you build here will feed into Chapter 11 (LLM planning) and Chapter 12 (full autonomous capstone).

## Learning Objectives

By the end of this chapter, you will be able to:

- [ ] **Understand** speech recognition pipeline stages (audio → features → model → text)
- [ ] **Configure** OpenAI Whisper for speech-to-text transcription (API and local options)
- [ ] **Implement** real-time audio capture with proper buffering and sample rates
- [ ] **Detect** voice activity to segment commands from silence
- [ ] **Design** intent schemas for robotics commands (specification-first approach)
- [ ] **Parse** natural language into structured intents with confidence scoring
- [ ] **Publish** voice intents to ROS 2 topics for downstream action execution
- [ ] **Handle** errors gracefully (unclear speech, API failures, noise)
- [ ] **Optimize** pipeline latency to achieve under 2 second end-to-end response
- [ ] **Create** a reusable voice-intent-parsing skill for future projects

## Prerequisites

Before starting this chapter, ensure you have completed:

- **Part 1**: ROS 2 nodes, topics, services, Python rclpy
- **Part 2**: Gazebo/Unity simulation, sensor processing
- **Part 3**: Isaac Sim, VSLAM, Nav2 navigation

You should also have:

- A working microphone (USB recommended)
- Internet access for Whisper API OR sufficient compute for local model (GPU recommended)
- Python 3.10+ with async/await familiarity

## Chapter Structure

This chapter follows the **4-Layer Teaching Framework**:

| Layer | Lessons | Focus |
|-------|---------|-------|
| **Layer 1**: Manual Foundation | 1-2 | Speech recognition theory, Whisper setup |
| **Layer 2**: AI Collaboration | 3-7 | Audio capture, VAD, intent parsing, ROS 2 |
| **Layer 3**: Intelligence Design | 8 | Create voice-intent-parsing skill |
| **Layer 4**: Spec-Driven Capstone | 9 | Voice-controlled humanoid navigation |

## Lessons

1. [Speech Recognition Fundamentals](./lesson-01-speech-recognition-fundamentals.md) - Understand how audio becomes text
2. [Setting Up OpenAI Whisper](./lesson-02-setting-up-whisper.md) - Install and configure Whisper
3. [Real-Time Audio Capture](./lesson-03-real-time-audio-capture.md) - Capture audio with PyAudio/sounddevice
4. [Voice Activity Detection](./lesson-04-voice-activity-detection.md) - Segment speech from silence
5. [Designing Intent Schemas](./lesson-05-designing-intent-schemas.md) - Structure commands for robots
6. [Implementing Intent Parsers](./lesson-06-implementing-intent-parsers.md) - Parse text to intents
7. [ROS 2 Integration](./lesson-07-ros2-integration.md) - Publish intents to topics
8. [Voice-Intent-Parsing Skill](./lesson-08-voice-intent-parsing-skill.md) - Create reusable skill
9. [Capstone: Voice-Controlled Navigation](./lesson-09-capstone-voice-controlled-navigation.md) - Full integration

## Key Technologies

- **OpenAI Whisper**: State-of-the-art speech recognition model
- **PyAudio/sounddevice**: Real-time audio capture libraries
- **Voice Activity Detection (VAD)**: WebRTC VAD, Silero VAD
- **ROS 2 Humble**: Robot middleware for intent publishing
- **Nav2**: Navigation stack integration from Part 3

## Teaching Approach

This chapter uses **specification-first** teaching:
- You'll design intent schemas BEFORE implementing parsers
- You'll write specifications BEFORE integration
- This mirrors professional voice interface development

Combined with **iterative refinement**:
- Test → Fail → Debug → Improve cycle
- Real-world voice input is messy; iteration is expected

## Estimated Time

- **Total**: ~16 hours
- **MVP (Lessons 1-2)**: ~3.5 hours
- **Full Chapter**: ~16 hours

---

**Ready to give your humanoid robot a voice?** Start with [Lesson 1: Speech Recognition Fundamentals](./lesson-01-speech-recognition-fundamentals.md).
