---
sidebar_position: 3
---

# Lesson 3: Real-Time Audio Capture

**Layer 2: AI Collaboration** | **Estimated Time: 120 minutes**

---

## Learning Objectives

By the end of this lesson, you will be able to:

- [ ] Select and configure audio input devices
- [ ] Implement real-time audio capture with PyAudio or sounddevice
- [ ] Configure sample rates and buffer sizes for Whisper
- [ ] Handle audio buffering for continuous capture
- [ ] Debug common audio capture issues

---

## Audio Input Devices

Before capturing audio, you need to understand your available hardware.

### Microphone Types

| Type | Pros | Cons | Best For |
|------|------|------|----------|
| USB Microphone | Plug-and-play, good quality | External device | Desktop/lab |
| Built-in Laptop Mic | Always available | Lower quality, noise | Quick tests |
| Array Microphone | Directional, noise cancellation | Expensive | Production |
| Headset | Close to mouth, low noise | Requires wearing | Hands-free |

### Listing Devices

```python
import sounddevice as sd

# List all audio devices
print(sd.query_devices())

# Get default input device
default_input = sd.default.device[0]
print(f"Default input: {sd.query_devices(default_input)}")
```

Output example:
```
   0 Microsoft Sound Mapper - Input, MME (2 in, 0 out)
>  1 Microphone (USB Audio Device), MME (2 in, 0 out)
   2 Microsoft Sound Mapper - Output, MME (0 in, 2 out)
```

### Selecting a Device

```python
# Use specific device by index
sd.default.device = (1, None)  # (input_device, output_device)

# Or by name
devices = sd.query_devices()
for i, d in enumerate(devices):
    if "USB" in d["name"] and d["max_input_channels"] > 0:
        sd.default.device = (i, None)
        break
```

---

## PyAudio vs sounddevice

Two popular libraries for audio capture in Python:

### sounddevice (Recommended)

```python
import sounddevice as sd
import numpy as np

# Simple recording
duration = 3  # seconds
sample_rate = 16000

recording = sd.rec(
    int(duration * sample_rate),
    samplerate=sample_rate,
    channels=1,
    dtype='float32'
)
sd.wait()  # Block until done
```

**Pros**:
- Cleaner API
- NumPy integration
- Cross-platform
- Callback and blocking modes

### PyAudio

```python
import pyaudio
import numpy as np

p = pyaudio.PyAudio()

stream = p.open(
    format=pyaudio.paFloat32,
    channels=1,
    rate=16000,
    input=True,
    frames_per_buffer=1024
)

# Read chunks
frames = []
for _ in range(int(16000 / 1024 * 3)):  # 3 seconds
    data = stream.read(1024)
    frames.append(np.frombuffer(data, dtype=np.float32))

stream.stop_stream()
stream.close()
p.terminate()
```

**Pros**:
- Lower-level control
- Widely used
- Good documentation

**Recommendation**: Use **sounddevice** for cleaner code and easier debugging.

---

## Sample Rate Configuration

Whisper expects **16kHz** audio. Using different rates requires resampling.

### Why 16kHz?

- Speech frequencies: 300Hz - 3400Hz (telephone quality)
- Nyquist theorem: Sample at 2x highest frequency
- 16kHz captures all speech information efficiently
- Higher rates waste bandwidth without improving recognition

### Configuring Sample Rate

```python
import sounddevice as sd

SAMPLE_RATE = 16000  # Hz - required for Whisper

# Verify device supports this rate
device_info = sd.query_devices(sd.default.device[0])
print(f"Default sample rate: {device_info['default_samplerate']}")

# Some devices may need resampling
# sounddevice handles this automatically if needed
```

### Resampling (if needed)

```python
from scipy import signal

def resample_audio(audio, original_rate, target_rate=16000):
    """Resample audio to target sample rate."""
    if original_rate == target_rate:
        return audio

    # Calculate new length
    new_length = int(len(audio) * target_rate / original_rate)

    # Resample
    resampled = signal.resample(audio, new_length)
    return resampled.astype(np.float32)
```

---

## Audio Buffering

Real-time audio capture requires careful buffer management.

### Buffer Size Tradeoffs

| Buffer Size | Latency | Stability | Use Case |
|-------------|---------|-----------|----------|
| Small (512) | Low (~32ms) | Risk of drops | Low-latency needed |
| Medium (1024) | Moderate (~64ms) | Good balance | General use |
| Large (4096) | High (~256ms) | Very stable | Batch processing |

### Calculating Latency

```
Latency (ms) = Buffer Size / Sample Rate * 1000

Example: 1024 / 16000 * 1000 = 64ms
```

### Callback vs Blocking Mode

**Blocking Mode** (Simple):
```python
# Blocks until recording is done
recording = sd.rec(duration * sample_rate, ...)
sd.wait()
```

**Callback Mode** (Real-time):
```python
import queue

audio_queue = queue.Queue()

def audio_callback(indata, frames, time, status):
    """Called for each audio buffer."""
    if status:
        print(f"Audio status: {status}")
    audio_queue.put(indata.copy())

# Start stream with callback
with sd.InputStream(
    samplerate=16000,
    channels=1,
    callback=audio_callback,
    blocksize=1024
):
    # Process audio from queue
    while True:
        audio_chunk = audio_queue.get()
        # Process chunk...
```

---

## Real-Time Constraints

For real-time audio, processing must be faster than audio arrival.

### The Rule

```
Processing Time < Buffer Duration

If buffer = 1024 samples at 16kHz:
Buffer Duration = 1024 / 16000 = 64ms
Processing must complete in < 64ms
```

### Common Issues

1. **Buffer Underrun**: Processing too slow, audio drops
   - Solution: Increase buffer size or optimize processing

2. **Buffer Overrun**: Data arriving faster than consumed
   - Solution: Process in separate thread, use queue

3. **Latency Spikes**: Occasional slow processing
   - Solution: Use larger buffer as safety margin

### Thread-Safe Processing

```python
import threading
import queue

class AudioProcessor:
    def __init__(self):
        self.audio_queue = queue.Queue()
        self.running = False

    def audio_callback(self, indata, frames, time, status):
        """Called from audio thread - must be fast!"""
        self.audio_queue.put(indata.copy())

    def process_loop(self):
        """Run in separate thread for processing."""
        while self.running:
            try:
                chunk = self.audio_queue.get(timeout=0.1)
                # Heavy processing here (Whisper, etc.)
                self.process_chunk(chunk)
            except queue.Empty:
                continue

    def process_chunk(self, chunk):
        """Process a single audio chunk."""
        # Your processing logic
        pass
```

---

## Try With AI: Debugging Audio Issues

This section demonstrates AI collaboration for solving common problems.

### AI as Teacher

**Scenario**: Your audio capture has crackling/popping artifacts.

**Student asks**: "My recorded audio has crackling sounds. What's wrong?"

**AI explains**:
> Audio artifacts usually indicate a **buffer underrun** - your processing is slower than audio arrival.
>
> **Solutions**:
> 1. Increase buffer size from 512 to 1024 or 2048
> 2. Use callback mode instead of blocking
> 3. Move heavy processing to a separate thread
>
> **Tradeoff**: Larger buffers increase latency. For a 2048 buffer at 16kHz, latency = 128ms.

### AI as Student

**Scenario**: AI suggests 44.1kHz sample rate (CD quality).

**Student corrects**:
> "Whisper expects 16kHz audio. Higher sample rates waste bandwidth and require resampling, adding latency. Let's use 16kHz."

**AI adapts**: Updates configuration to use 16kHz.

### AI as Co-Worker

**Scenario**: Finding optimal buffer size together.

**Iteration 1**: Student tries 512 samples
- Result: Audio drops when system is busy

**Iteration 2**: AI suggests 8192 samples
- Result: Works but 512ms latency is too high

**Iteration 3**: Together identify 1024-2048 as sweet spot
- Result: Stable capture with acceptable 64-128ms latency

---

## Practice Exercise

### Goal
Implement real-time audio capture and feed directly to Whisper.

### Steps

1. **Install requirements**:
```bash
pip install sounddevice numpy faster-whisper
```

2. **Run the capture script** (see `code/lesson_03_audio_capture.py`)

3. **Test with voice commands**:
   - Speak "Go to the kitchen"
   - Verify transcription appears within 2 seconds
   - Note any audio quality issues

4. **Experiment with buffer sizes**:
   - Try 512, 1024, 2048, 4096
   - Measure latency and stability

### Success Criteria

- [ ] Audio captures at 16kHz without drops
- [ ] Callback mode implemented correctly
- [ ] Can identify optimal buffer size for your system
- [ ] Whisper transcribes captured audio correctly
- [ ] No crackling or audio artifacts

---

## Lesson Checkpoint

Before proceeding, verify you can answer:

1. **Why do we use 16kHz sample rate?**
   > Whisper expects it; captures all speech frequencies efficiently

2. **What causes buffer underrun?**
   > Processing slower than audio arrival; fix with larger buffers

3. **Why use callback mode for real-time?**
   > Non-blocking, allows continuous capture while processing

4. **How do you calculate buffer latency?**
   > Latency = Buffer Size / Sample Rate * 1000 (in ms)

---

## Summary

In this lesson, you learned:

- How to select and configure audio input devices
- PyAudio vs sounddevice (sounddevice recommended)
- Sample rate configuration (16kHz for Whisper)
- Buffer size tradeoffs (latency vs stability)
- Callback vs blocking modes for capture
- Thread-safe processing for real-time audio
- Debugging common audio issues with AI collaboration

**Next**: [Lesson 4: Voice Activity Detection](./lesson-04-voice-activity-detection.md) - Detect when someone is speaking.
