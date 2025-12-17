---
sidebar_position: 2
---

# Lesson 2: Setting Up OpenAI Whisper

**Layer 1: Manual Foundation** | **Estimated Time: 120 minutes**

---

## Learning Objectives

By the end of this lesson, you will be able to:

- [ ] Explain Whisper's encoder-decoder architecture
- [ ] Choose appropriate model sizes based on accuracy/speed tradeoffs
- [ ] Set up Whisper API for cloud transcription
- [ ] Install and run Whisper locally
- [ ] Configure transcription parameters
- [ ] Measure transcription accuracy with WER

---

## Whisper Architecture

**OpenAI Whisper** is a state-of-the-art automatic speech recognition (ASR) system trained on 680,000 hours of multilingual audio data.

### Encoder-Decoder Transformer

Whisper uses an encoder-decoder architecture:

```
Audio Input (30s max)
       |
       v
+------------------+
|  Audio Encoder   |  <- Processes mel spectrogram
|  (Transformer)   |     Outputs contextual representations
+------------------+
       |
       v
  Hidden States
       |
       v
+------------------+
|  Text Decoder    |  <- Generates text token by token
|  (Transformer)   |     Autoregressive (uses previous tokens)
+------------------+
       |
       v
Transcribed Text
```

### Key Components

1. **Log-Mel Spectrogram**: Audio is converted to 80-channel mel spectrogram
2. **Encoder**: 12-32 transformer layers (depending on model size)
3. **Decoder**: Generates text tokens using cross-attention to encoder outputs
4. **Multitask Training**: Trained on transcription, translation, and timestamp prediction

---

## Model Size Selection

Whisper comes in multiple sizes with different accuracy/speed tradeoffs:

| Model | Parameters | VRAM | Relative Speed | English WER |
|-------|------------|------|----------------|-------------|
| tiny | 39M | ~1 GB | ~32x | ~7.6% |
| base | 74M | ~1 GB | ~16x | ~5.0% |
| small | 244M | ~2 GB | ~6x | ~3.4% |
| medium | 769M | ~5 GB | ~2x | ~2.9% |
| large | 1550M | ~10 GB | 1x | ~2.7% |
| large-v3 | 1550M | ~10 GB | 1x | ~2.5% |

### Choosing a Model

**For Development/Testing**:
- Use `base` or `small` for quick iteration
- Fast enough for real-time experiments

**For Production**:
- Use `small` or `medium` for good accuracy
- Balance between quality and latency

**For Best Quality**:
- Use `large-v3` when accuracy is critical
- Requires GPU with 10GB+ VRAM

**For Edge Devices**:
- Use `tiny` or `base` on resource-constrained systems
- Consider quantized versions

---

## Whisper API Setup

The fastest way to get started is using OpenAI's hosted API.

### Step 1: Create OpenAI Account

1. Go to https://platform.openai.com
2. Create an account or sign in
3. Navigate to API Keys section
4. Create a new secret key
5. **Save it securely** - you cannot view it again!

### Step 2: Install the OpenAI Package

```bash
pip install openai
```

### Step 3: Set Your API Key

**Option A: Environment Variable (Recommended)**
```bash
export OPENAI_API_KEY="sk-your-key-here"
```

**Option B: In Code (Not recommended for production)**
```python
from openai import OpenAI
client = OpenAI(api_key="sk-your-key-here")
```

### Step 4: Test Transcription

```python
from openai import OpenAI
import os

client = OpenAI()

# Transcribe an audio file
audio_file = open("audio_recordings/go_to_the_kitchen.wav", "rb")

transcript = client.audio.transcriptions.create(
    model="whisper-1",
    file=audio_file,
    language="en"  # Optional: force English
)

print(transcript.text)
# Output: "Go to the kitchen"
```

### API Pricing

As of 2024:
- **Whisper API**: $0.006 per minute of audio
- Example: 1 hour of audio = $0.36

For development, this is very affordable. For production with heavy usage, consider local deployment.

---

## Local Whisper Installation

For production systems or offline operation, run Whisper locally.

### Option 1: Official OpenAI Whisper

```bash
pip install openai-whisper
```

**Usage**:
```python
import whisper

# Load model (downloads on first use)
model = whisper.load_model("base")

# Transcribe
result = model.transcribe("audio.wav")
print(result["text"])
```

**Pros**: Official implementation, well-tested
**Cons**: Slower, requires more VRAM

### Option 2: Faster-Whisper (Recommended)

**Faster-Whisper** uses CTranslate2 for 4x speedup:

```bash
pip install faster-whisper
```

**Usage**:
```python
from faster_whisper import WhisperModel

# Load model
model = WhisperModel("base", device="cuda", compute_type="float16")
# For CPU: device="cpu", compute_type="int8"

# Transcribe
segments, info = model.transcribe("audio.wav", language="en")

for segment in segments:
    print(f"[{segment.start:.2f}s -> {segment.end:.2f}s] {segment.text}")
```

**Pros**: 4x faster, lower VRAM usage, timestamps
**Cons**: Different API than official

### Option 3: Whisper.cpp (For Edge)

For embedded systems or C++ integration:

```bash
git clone https://github.com/ggerganov/whisper.cpp
cd whisper.cpp
make

# Download model
bash models/download-ggml-model.sh base.en

# Run
./main -m models/ggml-base.en.bin -f audio.wav
```

**Pros**: CPU-optimized, no Python dependencies
**Cons**: Requires compilation, different integration

---

## Configuration and Parameters

### Key Parameters

```python
from faster_whisper import WhisperModel

model = WhisperModel("base")

segments, info = model.transcribe(
    "audio.wav",

    # Language settings
    language="en",           # Force language (None = auto-detect)

    # Decoding parameters
    beam_size=5,             # Beam search width (1 = greedy)
    best_of=5,               # Number of candidates for temperature sampling
    temperature=0.0,         # 0 = deterministic, >0 = sampling

    # Timestamp control
    word_timestamps=True,    # Get word-level timestamps

    # Initial prompt (for context/vocabulary)
    initial_prompt="Navigate, pick up, stop, resume",

    # VAD filtering
    vad_filter=True,         # Filter out non-speech
)
```

### Temperature and Sampling

**Temperature = 0** (Deterministic):
- Always picks the most likely token
- Consistent output
- Best for most use cases

**Temperature > 0** (Sampling):
- Introduces randomness
- Can help with difficult audio
- May produce different results each time

### Initial Prompt

Guide Whisper with expected vocabulary:

```python
# For robotics commands
initial_prompt = """
Navigation: Go to the kitchen. Navigate to the bedroom. Move to location.
Manipulation: Pick up the ball. Put down the cup. Grab the item.
Control: Stop. Pause. Resume. Cancel.
"""

segments, _ = model.transcribe(
    "audio.wav",
    initial_prompt=initial_prompt
)
```

This helps Whisper recognize domain-specific terms.

---

## Practice Exercise

### Goal
Transcribe robot commands and measure Word Error Rate.

### Setup

1. **Install packages**:
```bash
pip install faster-whisper jiwer
```

2. **Prepare test commands** (record or use existing):
   - "Go to the kitchen"
   - "Pick up the red ball"
   - "Navigate to the living room"
   - "Stop"
   - "Put down the cup on the table"
   - "Where are you"
   - "What do you see"
   - "Resume navigation"
   - "Cancel current task"
   - "Move forward two meters"

3. **Run the test script** (see `code/lesson_02_whisper_local.py`)

### Measuring WER

Use the `jiwer` library:

```python
from jiwer import wer

reference = "go to the kitchen"
hypothesis = "go to kitchen"  # "the" was deleted

error_rate = wer(reference, hypothesis)
print(f"WER: {error_rate:.2%}")  # Output: WER: 25.00%
```

### Success Criteria

- [ ] Successfully called Whisper API or local model
- [ ] Transcribed all 10 test commands
- [ ] Calculated WER for each command
- [ ] Achieved average WER below 10% in quiet environment
- [ ] Can explain model size tradeoffs

---

## Handling Whisper Failures

### API Failures

```python
from openai import OpenAI, APIError, RateLimitError
import time

client = OpenAI()

def transcribe_with_retry(audio_path, max_retries=2):
    for attempt in range(max_retries + 1):
        try:
            with open(audio_path, "rb") as f:
                result = client.audio.transcriptions.create(
                    model="whisper-1",
                    file=f
                )
            return result.text
        except RateLimitError:
            if attempt < max_retries:
                time.sleep(2 ** attempt)  # Exponential backoff
                continue
            raise
        except APIError as e:
            print(f"API Error: {e}")
            raise

    return None
```

### Local Model Failures

```python
from faster_whisper import WhisperModel

def load_model_with_fallback():
    """Try GPU first, fall back to CPU."""
    try:
        return WhisperModel("base", device="cuda", compute_type="float16")
    except Exception:
        print("GPU not available, falling back to CPU")
        return WhisperModel("base", device="cpu", compute_type="int8")
```

---

## Latency Considerations

### API vs Local

| Aspect | API | Local (GPU) | Local (CPU) |
|--------|-----|-------------|-------------|
| Latency | ~1-2s network | ~0.3-1s | ~2-10s |
| First-time | Fast | Model download | Model download |
| Offline | No | Yes | Yes |
| Cost | Per-minute | Hardware | Hardware |

### Reducing Latency

1. **Use faster-whisper**: 4x speedup over official
2. **Use smaller models**: `tiny` or `base` for real-time
3. **Use GPU**: 10x faster than CPU
4. **Quantize models**: INT8 reduces memory and speeds up CPU
5. **Batch short utterances**: Process multiple at once

For robotics, target **under 1 second** transcription latency.

---

## Lesson Checkpoint

Before proceeding, verify you can answer:

1. **What is Whisper's architecture?**
   > Encoder-decoder transformer trained on 680k hours of audio

2. **Which model size should you use for development?**
   > `base` or `small` for fast iteration

3. **How do you set up the Whisper API?**
   > Create OpenAI account, get API key, use openai Python package

4. **What is faster-whisper and why use it?**
   > CTranslate2 implementation with 4x speedup, lower VRAM

5. **How does initial_prompt help?**
   > Guides Whisper toward expected vocabulary/domain

---

## Summary

In this lesson, you learned:

- Whisper uses encoder-decoder transformer architecture
- Model sizes trade off accuracy vs speed (tiny to large-v3)
- API setup requires OpenAI account and API key
- Local deployment options: official whisper, faster-whisper, whisper.cpp
- Configuration parameters affect quality and performance
- WER measures transcription accuracy
- Error handling and retry logic for production systems

**MVP Checkpoint**: At this point, you can transcribe voice commands! Try speaking to your system and seeing the text output.

**Next**: [Lesson 3: Real-Time Audio Capture](./lesson-03-real-time-audio-capture.md) - Capture live audio from microphones.
