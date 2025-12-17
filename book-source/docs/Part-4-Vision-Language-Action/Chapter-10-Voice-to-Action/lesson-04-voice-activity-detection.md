---
sidebar_position: 4
---

# Lesson 4: Voice Activity Detection

**Layer 2: AI Collaboration** | **Estimated Time: 120 minutes**

---

## Learning Objectives

By the end of this lesson, you will be able to:

- [ ] Explain the VAD problem (speech vs silence vs noise)
- [ ] Implement energy-based VAD with amplitude thresholding
- [ ] Use WebRTC VAD for production-quality detection
- [ ] Configure Silero VAD for neural network-based detection
- [ ] Implement command segmentation with hangover time

---

## VAD Problem Definition

**Voice Activity Detection (VAD)** answers: "Is someone speaking right now?"

### Why VAD Matters

Without VAD, you would:
1. Transcribe silence (wasted compute, potential hallucinations)
2. Send incomplete commands (cut off mid-sentence)
3. Miss command boundaries (where does one command end?)

### The Challenge

```
Audio Stream:  [silence] [speech] [pause] [speech] [silence]
               |         |        |       |        |
               |         |        |       |        +-- Don't transcribe
               |         |        |       +-- Include this
               |         |        +-- Brief pause in command
               |         +-- Detect start
               +-- Don't transcribe
```

### Three States

1. **Silence**: Low energy, no speech - ignore
2. **Speech**: Human voice detected - capture
3. **Noise**: Non-speech sounds - reject

---

## Energy-Based VAD

The simplest approach: threshold on audio amplitude.

### Basic Implementation

```python
import numpy as np

def energy_vad(audio_chunk: np.ndarray, threshold: float = 0.01) -> bool:
    """
    Detect voice activity using RMS energy.

    Args:
        audio_chunk: Audio samples (float32)
        threshold: Energy threshold for speech detection

    Returns:
        True if speech detected
    """
    rms = np.sqrt(np.mean(audio_chunk ** 2))
    return rms > threshold
```

### Adaptive Threshold

Fixed thresholds fail in varying noise conditions. Adapt to environment:

```python
class AdaptiveEnergyVAD:
    def __init__(self, initial_threshold: float = 0.01):
        self.threshold = initial_threshold
        self.noise_level = initial_threshold
        self.alpha = 0.1  # Adaptation rate

    def detect(self, audio_chunk: np.ndarray) -> bool:
        rms = np.sqrt(np.mean(audio_chunk ** 2))

        # Update noise estimate during silence
        if rms < self.threshold:
            self.noise_level = (
                self.alpha * rms +
                (1 - self.alpha) * self.noise_level
            )
            # Threshold slightly above noise
            self.threshold = self.noise_level * 2

        return rms > self.threshold
```

### Limitations

- **Sensitive to noise**: Background noise triggers false positives
- **No speech characteristics**: Loud noise looks like speech
- **Threshold tuning**: Different environments need different values

---

## WebRTC VAD

Google's production VAD, used in Chrome and WebRTC applications.

### Installation

```bash
pip install webrtcvad
```

### Usage

```python
import webrtcvad

# Create VAD instance
# Aggressiveness: 0-3 (higher = more aggressive filtering)
vad = webrtcvad.Vad(2)

def webrtc_vad_detect(audio_chunk: np.ndarray, sample_rate: int = 16000) -> bool:
    """
    Detect speech using WebRTC VAD.

    Args:
        audio_chunk: Audio samples (must be 10, 20, or 30ms)
        sample_rate: Sample rate (8000, 16000, or 32000)

    Returns:
        True if speech detected
    """
    # Convert to 16-bit PCM bytes
    audio_bytes = (audio_chunk * 32767).astype(np.int16).tobytes()

    # WebRTC VAD requires specific frame sizes
    # 10ms at 16kHz = 160 samples
    # 20ms at 16kHz = 320 samples
    # 30ms at 16kHz = 480 samples

    return vad.is_speech(audio_bytes, sample_rate)
```

### Aggressiveness Levels

| Level | Behavior | Use Case |
|-------|----------|----------|
| 0 | Least aggressive | Quiet environments |
| 1 | Moderate | General use |
| 2 | More aggressive | Some background noise |
| 3 | Most aggressive | Noisy environments |

Higher aggressiveness = more likely to classify as non-speech.

---

## Silero VAD

Neural network-based VAD with excellent accuracy.

### Installation

```bash
pip install torch torchaudio
```

### Usage

```python
import torch

# Load model
model, utils = torch.hub.load(
    repo_or_dir='snakers4/silero-vad',
    model='silero_vad',
    trust_repo=True
)

(get_speech_timestamps, _, read_audio, _, _) = utils

def silero_vad_detect(audio: np.ndarray, sample_rate: int = 16000) -> list:
    """
    Detect speech segments using Silero VAD.

    Args:
        audio: Audio samples (float32)
        sample_rate: Sample rate

    Returns:
        List of speech segments with start/end timestamps
    """
    # Convert to torch tensor
    audio_tensor = torch.from_numpy(audio).float()

    # Get speech timestamps
    speech_timestamps = get_speech_timestamps(
        audio_tensor,
        model,
        sampling_rate=sample_rate,
        threshold=0.5,          # Speech probability threshold
        min_speech_duration_ms=250,  # Minimum speech segment
        min_silence_duration_ms=100  # Minimum silence between segments
    )

    return speech_timestamps
```

### Advantages

- **Accurate**: Neural network trained on diverse audio
- **Robust**: Handles various noise types well
- **Timestamps**: Returns exact speech boundaries
- **Configurable**: Many parameters for tuning

---

## Command Segmentation

VAD tells us when speech is happening. Command segmentation determines when a complete command is ready for transcription.

### Speech Onset and Offset

```
Audio:    [silence] [speech....] [silence]
                    ^            ^
                    |            +-- Speech offset
                    +-- Speech onset
```

### Hangover Time

Don't cut off commands during brief pauses:

```python
class CommandSegmenter:
    def __init__(
        self,
        sample_rate: int = 16000,
        onset_threshold: float = 0.02,
        offset_threshold: float = 0.01,
        hangover_ms: int = 800,      # Wait before ending command
        min_speech_ms: int = 200     # Minimum speech duration
    ):
        self.sample_rate = sample_rate
        self.onset_threshold = onset_threshold
        self.offset_threshold = offset_threshold
        self.hangover_samples = int(hangover_ms * sample_rate / 1000)
        self.min_speech_samples = int(min_speech_ms * sample_rate / 1000)

        self.is_speaking = False
        self.speech_buffer = []
        self.silence_count = 0
        self.speech_count = 0

    def process_chunk(self, chunk: np.ndarray) -> tuple:
        """
        Process audio chunk and detect command boundaries.

        Returns:
            (is_command_complete, audio_data or None)
        """
        rms = np.sqrt(np.mean(chunk ** 2))

        if not self.is_speaking:
            # Waiting for speech onset
            if rms > self.onset_threshold:
                self.is_speaking = True
                self.speech_buffer = [chunk]
                self.speech_count = len(chunk)
                self.silence_count = 0
        else:
            # Currently in speech
            self.speech_buffer.append(chunk)

            if rms > self.offset_threshold:
                # Still speaking
                self.silence_count = 0
                self.speech_count += len(chunk)
            else:
                # Possible end of speech
                self.silence_count += len(chunk)

                if self.silence_count >= self.hangover_samples:
                    # Command complete!
                    if self.speech_count >= self.min_speech_samples:
                        audio = np.concatenate(self.speech_buffer)
                        self.reset()
                        return True, audio
                    else:
                        # Too short, ignore
                        self.reset()

        return False, None

    def reset(self):
        """Reset state for next command."""
        self.is_speaking = False
        self.speech_buffer = []
        self.silence_count = 0
        self.speech_count = 0
```

### Tuning Parameters

| Parameter | Effect | Typical Value |
|-----------|--------|---------------|
| Hangover | Time to wait after speech | 500-1000ms |
| Min speech | Reject very short sounds | 100-300ms |
| Onset threshold | Sensitivity to start | 0.01-0.05 |
| Offset threshold | Sensitivity to end | 0.005-0.02 |

---

## Try With AI: VAD Threshold Tuning

### AI as Teacher

**Scenario**: VAD triggers on background noise (false positives).

**Student asks**: "My VAD keeps detecting the air conditioning as speech."

**AI explains**:
> Energy-based VAD is noise-sensitive. For noisy environments:
>
> 1. **Use WebRTC VAD** with aggressiveness=2 or 3
> 2. **Use Silero VAD** which is trained to distinguish speech from noise
> 3. **Increase threshold** if staying with energy-based
>
> **Tradeoff**: Higher thresholds may miss soft speech.

### AI as Student

**Scenario**: AI suggests very short hangover time.

**Student corrects**:
> "Natural speech has pauses for breathing and thinking. A 200ms hangover cuts off commands mid-sentence. We need at least 500-800ms."

**AI adapts**: Updates hangover to 800ms.

### AI as Co-Worker

**Iteration 1**: Low threshold (0.005)
- Result: Triggers on typing sounds

**Iteration 2**: High threshold (0.1)
- Result: Misses soft commands

**Iteration 3**: Adaptive threshold + Silero VAD
- Result: Robust detection in varying conditions

---

## Practice Exercise

### Goal
Implement VAD and test in quiet and noisy environments.

### Steps

1. **Install requirements**:
```bash
pip install webrtcvad torch torchaudio
```

2. **Run the VAD comparison script** (see `code/lesson_04_energy_vad.py`)

3. **Test in different conditions**:
   - Quiet room
   - With music playing
   - With typing/keyboard sounds
   - With fan/AC running

4. **Measure false positive rate**:
   - Record 60 seconds of silence
   - Count how many times VAD triggers
   - Target: under 10% false positive rate

### Success Criteria

- [ ] Energy-based VAD implemented
- [ ] WebRTC VAD configured
- [ ] Silero VAD working
- [ ] Command segmentation with hangover
- [ ] False positive rate under 10% in quiet environment
- [ ] Commands properly segmented

---

## Noise Rejection and False Positive Handling

### Strategies

1. **Multi-stage VAD**: Energy first, then neural network
2. **Confidence thresholds**: Reject low-confidence detections
3. **Duration filtering**: Require minimum speech duration
4. **Spectral analysis**: Speech has specific frequency patterns

### Implementation

```python
def robust_vad(audio_chunk, energy_threshold=0.02, silero_threshold=0.7):
    """
    Two-stage VAD for robust detection.
    """
    # Stage 1: Quick energy check (fast)
    rms = np.sqrt(np.mean(audio_chunk ** 2))
    if rms < energy_threshold:
        return False

    # Stage 2: Neural network verification (slower but accurate)
    # Only run if energy check passes
    speech_prob = silero_model(audio_chunk)
    return speech_prob > silero_threshold
```

---

## Lesson Checkpoint

Before proceeding, verify you can answer:

1. **What is the purpose of VAD?**
   > Detect when someone is speaking to avoid transcribing silence

2. **Why is energy-based VAD insufficient?**
   > Sensitive to noise; cannot distinguish speech from loud sounds

3. **What is hangover time?**
   > Time to wait after speech ends before marking command complete

4. **When should you use Silero VAD vs WebRTC VAD?**
   > Silero for accuracy, WebRTC for low latency/resources

---

## Summary

In this lesson, you learned:

- VAD detects speech vs silence vs noise
- Energy-based VAD is simple but noise-sensitive
- WebRTC VAD is production-quality with aggressiveness tuning
- Silero VAD uses neural networks for best accuracy
- Command segmentation uses hangover time for natural pauses
- Multi-stage VAD combines speed and accuracy

**Next**: [Lesson 5: Designing Intent Schemas](./lesson-05-designing-intent-schemas.md) - Structure voice commands for robots.
