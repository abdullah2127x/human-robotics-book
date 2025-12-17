---
sidebar_position: 1
---

# Lesson 1: Speech Recognition Fundamentals

**Layer 1: Manual Foundation** | **Estimated Time: 90 minutes**

---

## Learning Objectives

By the end of this lesson, you will be able to:

- [ ] Explain why voice interfaces are valuable for humanoid robots
- [ ] Describe audio representation (waveforms, samples, sample rates)
- [ ] Understand feature extraction (spectrograms, MFCCs)
- [ ] Explain how acoustic and language models work together
- [ ] Measure transcription quality using Word Error Rate (WER)

---

## Why Voice Interfaces for Robots?

Imagine you are working alongside a humanoid robot in a warehouse. You need it to fetch a package from shelf B-12. What is the most natural way to communicate this?

**Option A**: Walk to a control panel, navigate menus, type coordinates
**Option B**: Say "Go to shelf B-12 and bring me the blue package"

Voice is the most natural human interface. We speak before we can type. We speak faster than we type. And we can speak while our hands are busy.

For humanoid robots operating in human environments, voice interfaces enable:

1. **Hands-free operation**: Command the robot while carrying objects
2. **Natural interaction**: No training required for users
3. **Contextual commands**: "Put that over there" (pointing)
4. **Emergency control**: "Stop!" is faster than finding a button

### The Challenge

But voice is also messy:
- Background noise (machines, other people)
- Accents and speech variations
- Ambiguous commands ("Get me that thing")
- Audio hardware variability

This chapter teaches you to handle all of this.

---

## Audio Representation

Before we can recognize speech, we need to understand how audio is represented digitally.

### Analog to Digital

Sound is pressure waves in air. A microphone converts these pressure changes into electrical signals. An **Analog-to-Digital Converter (ADC)** samples this signal at regular intervals.

```
Sound Wave -> Microphone -> Analog Signal -> ADC -> Digital Samples
```

### Key Concepts

**Sample Rate**: How many times per second we measure the audio signal.
- 16,000 Hz (16 kHz) = 16,000 samples per second
- Whisper expects 16 kHz audio
- Higher sample rates capture more detail but require more data

**Bit Depth**: How precisely each sample is measured.
- 16-bit = 65,536 possible values per sample
- Standard for speech recognition

**Waveform**: The raw amplitude values over time.

```python
# A waveform is just an array of numbers
# Each number represents amplitude at that moment
waveform = [-0.02, 0.01, 0.05, 0.12, 0.18, 0.15, 0.08, ...]
```

### Visualizing Audio

The waveform shows:
- **Silence** before and after speech (low amplitude)
- **Speech** in the middle (varying amplitude)
- **Pauses** between words (brief low amplitude)

---

## Feature Extraction

Raw waveforms are hard for machine learning models to process directly. We extract **features** that capture the characteristics of speech.

### Spectrograms

A **spectrogram** shows how frequencies change over time.

- **X-axis**: Time
- **Y-axis**: Frequency (pitch)
- **Color/Intensity**: Energy at that frequency and time

Different sounds have different spectral patterns:
- **Vowels** ("o", "i", "e"): Strong energy in lower frequencies
- **Consonants** ("g", "t", "k"): Brief bursts, often higher frequencies
- **Silence**: Very low energy across all frequencies

### Mel-Frequency Cepstral Coefficients (MFCCs)

MFCCs are a compact representation of the spectral envelope.

Human hearing is logarithmic - we perceive the difference between 100Hz and 200Hz more than between 1000Hz and 1100Hz. The **Mel scale** models this:

```
Mel(f) = 2595 * log10(1 + f/700)
```

MFCCs:
1. Compute spectrogram
2. Apply Mel filterbank (mimics human hearing)
3. Take logarithm (humans perceive loudness logarithmically)
4. Apply Discrete Cosine Transform (decorrelates features)

The result: ~13-40 numbers per time frame that capture speech characteristics.

---

## Acoustic and Language Models

Speech recognition systems traditionally have two components:

### Acoustic Model

Converts audio features (spectrograms/MFCCs) into **phonemes** - the smallest units of speech.

```
Audio Features -> Acoustic Model -> Phonemes
```

The acoustic model learns:
- What spectral patterns correspond to each phoneme
- How context affects pronunciation (coarticulation)
- Speaker variations (accents, speaking rates)

### Language Model

Predicts probable word sequences given phonemes.

```
Phonemes -> Language Model -> Words
```

The language model knows:
- "Go to the kitchen" is more likely than "Go two the kitchen"
- Word probabilities based on context
- Grammar and common phrases

### End-to-End Models (Whisper)

Modern systems like **Whisper** combine both into a single neural network:

```
Audio -> Encoder -> Hidden States -> Decoder -> Text
         (acoustic)                 (language)
```

Whisper is an **encoder-decoder transformer**:
- **Encoder**: Processes audio into contextual representations
- **Decoder**: Generates text autoregressively (word by word)

Benefits of end-to-end:
- No separate phoneme step
- Learns optimal features automatically
- Better handling of context

---

## Evaluation Metrics

How do we know if speech recognition is working well?

### Word Error Rate (WER)

The standard metric for ASR systems:

```
WER = (Substitutions + Insertions + Deletions) / Total Words in Reference
```

**Example**:
- Reference: "Go to the kitchen"
- Hypothesis: "Go to kitchen"

| Word | Reference | Hypothesis | Error |
|------|-----------|------------|-------|
| 1 | Go | Go | Correct |
| 2 | to | to | Correct |
| 3 | the | - | Deletion |
| 4 | kitchen | kitchen | Correct |

WER = (0 + 0 + 1) / 4 = 25%

### Interpreting WER

| WER | Quality | Use Case |
|-----|---------|----------|
| Under 5% | Excellent | Publication quality |
| 5-10% | Good | Production systems |
| 10-20% | Fair | Noisy environments |
| Over 20% | Poor | Needs improvement |

For robotics commands in controlled environments, target **under 10% WER**.

---

## Practice Exercise

### Goal
Record and visualize voice commands to understand audio properties.

### Steps

1. **Install requirements**:
```bash
pip install sounddevice numpy scipy matplotlib
```

2. **Run the visualization script** (see `code/lesson_01_audio_visualization.py`):
```bash
python code/lesson_01_audio_visualization.py
```

3. **Record these commands**:
   - "Go to the kitchen"
   - "Pick up the red ball"
   - "Stop"

4. **Answer these questions**:

- [ ] What is the sample rate of your recordings?
- [ ] How can you identify speech vs. silence in the waveform?
- [ ] What frequencies are most prominent in the spectrogram?
- [ ] How does "Stop" look different from "Go to the kitchen"?

### Success Criteria

- [ ] Successfully recorded 3 voice commands
- [ ] Generated waveform visualizations for each
- [ ] Generated spectrogram visualizations for each
- [ ] Can identify speech regions in waveform
- [ ] Can explain what sample rate means

---

## Lesson Checkpoint

Before proceeding, verify you can answer these questions:

1. **Why is voice input natural for robots?**
   > Hint: Think about hands-free operation and human communication patterns

2. **What does sample rate mean?**
   > Hint: How many times per second we measure the audio signal

3. **How does a spectrogram represent audio?**
   > Hint: Time on X-axis, frequency on Y-axis, energy as color

4. **What is WER and why does it matter?**
   > Hint: Word Error Rate measures transcription accuracy

5. **What is the difference between acoustic and language models?**
   > Hint: Acoustic handles audio to phonemes, language handles phonemes to words

---

## Summary

In this lesson, you learned:

- Voice interfaces enable natural, hands-free robot control
- Audio is represented as digital samples at a specific sample rate
- Spectrograms and MFCCs extract features from raw audio
- Acoustic models convert audio to phonemes; language models convert to words
- Modern systems like Whisper combine both in an end-to-end architecture
- WER measures transcription quality (lower is better)

**Next**: [Lesson 2: Setting Up OpenAI Whisper](./lesson-02-setting-up-whisper.md) - Install and configure Whisper for speech recognition.
