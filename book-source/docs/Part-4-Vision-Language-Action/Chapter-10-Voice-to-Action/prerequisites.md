# Prerequisites Checklist

Before starting Chapter 10, verify you have completed the following:

## Part 1: ROS 2 Foundation

- [ ] Installed ROS 2 Humble on Ubuntu 22.04
- [ ] Created and built ROS 2 packages with colcon
- [ ] Written Python nodes using rclpy
- [ ] Published and subscribed to topics
- [ ] Created custom message types
- [ ] Used services and parameters
- [ ] Understand async/await patterns in Python

## Part 2: Digital Twin Simulation

- [ ] Run Gazebo Classic or Ignition simulations
- [ ] Spawned robot models in simulation
- [ ] Processed sensor data (cameras, LIDAR)
- [ ] Connected ROS 2 nodes to simulated robots

## Part 3: Advanced Perception and Navigation

- [ ] Configured Isaac Sim for humanoid robots
- [ ] Implemented VSLAM for localization
- [ ] Set up Nav2 navigation stack
- [ ] Created behavior trees for robot actions
- [ ] Navigated humanoid to waypoints

## Hardware Requirements

- [ ] **Microphone**: USB microphone (recommended) or built-in laptop mic
- [ ] **Audio Output**: Speakers/headphones for testing
- [ ] **GPU** (optional): NVIDIA GPU with 4GB+ VRAM for local Whisper models

## Software Requirements

- [ ] Python 3.10 or higher
- [ ] pip package manager
- [ ] Internet access (for Whisper API) OR local compute for Whisper models

## API Keys (Choose One)

### Option A: OpenAI Whisper API (Recommended for beginners)
- [ ] OpenAI account created at https://platform.openai.com
- [ ] API key generated and stored securely
- [ ] Billing set up (Whisper API has usage costs)

### Option B: Local Whisper (No API costs)
- [ ] NVIDIA GPU with CUDA support (recommended)
- [ ] OR CPU with 8GB+ RAM (slower but works)
- [ ] Sufficient disk space (~3GB for large model)

## Python Packages to Install

Run this command to install required packages:

```bash
pip install openai sounddevice numpy scipy webrtcvad
```

For local Whisper:
```bash
pip install faster-whisper
```

## Quick Verification

Run this Python script to verify your audio setup:

```python
import sounddevice as sd
import numpy as np

# List audio devices
print("Available audio devices:")
print(sd.query_devices())

# Test recording (2 seconds)
print("\nRecording 2 seconds of audio...")
recording = sd.rec(int(2 * 16000), samplerate=16000, channels=1, dtype='int16')
sd.wait()
print(f"Recorded {len(recording)} samples")
print("Audio capture working!")
```

If this script runs without errors, you're ready for Chapter 10!

---

**All checkboxes complete?** Proceed to [Lesson 1: Speech Recognition Fundamentals](./lesson-01-speech-recognition-fundamentals.md).
