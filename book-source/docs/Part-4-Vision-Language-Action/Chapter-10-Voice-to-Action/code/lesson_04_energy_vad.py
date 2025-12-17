#!/usr/bin/env python3
"""
Lesson 4: Voice Activity Detection
==================================

This script demonstrates:
1. Energy-based VAD
2. WebRTC VAD
3. Command segmentation

Requirements:
    pip install sounddevice numpy webrtcvad

Usage:
    python lesson_04_energy_vad.py
"""

import numpy as np
import sounddevice as sd
import time
from typing import Optional, Tuple
from collections import deque


SAMPLE_RATE = 16000
CHANNELS = 1


class EnergyVAD:
    """Simple energy-based Voice Activity Detection."""

    def __init__(
        self,
        threshold: float = 0.02,
        adaptive: bool = True,
        adaptation_rate: float = 0.1
    ):
        self.threshold = threshold
        self.adaptive = adaptive
        self.adaptation_rate = adaptation_rate
        self.noise_level = threshold * 0.5

    def detect(self, audio_chunk: np.ndarray) -> Tuple[bool, float]:
        """
        Detect voice activity in audio chunk.

        Returns:
            (is_speech, rms_level)
        """
        rms = np.sqrt(np.mean(audio_chunk ** 2))

        is_speech = rms > self.threshold

        # Adapt threshold to noise level
        if self.adaptive and not is_speech:
            self.noise_level = (
                self.adaptation_rate * rms +
                (1 - self.adaptation_rate) * self.noise_level
            )
            self.threshold = max(self.noise_level * 2.5, 0.01)

        return is_speech, rms


class WebRTCVAD:
    """WebRTC-based Voice Activity Detection."""

    def __init__(self, aggressiveness: int = 2):
        """
        Initialize WebRTC VAD.

        Args:
            aggressiveness: 0-3 (higher = more aggressive filtering)
        """
        try:
            import webrtcvad
            self.vad = webrtcvad.Vad(aggressiveness)
            self.available = True
        except ImportError:
            print("webrtcvad not installed. Using energy-based fallback.")
            self.available = False
            self.fallback = EnergyVAD()

    def detect(self, audio_chunk: np.ndarray, sample_rate: int = 16000) -> bool:
        """
        Detect voice activity using WebRTC VAD.

        Note: WebRTC VAD expects 10, 20, or 30ms frames.
        """
        if not self.available:
            return self.fallback.detect(audio_chunk)[0]

        # Convert to 16-bit PCM bytes
        audio_int16 = (audio_chunk * 32767).astype(np.int16)
        audio_bytes = audio_int16.tobytes()

        # Process in 20ms frames (320 samples at 16kHz)
        frame_size = int(sample_rate * 0.02)
        speech_frames = 0
        total_frames = 0

        for i in range(0, len(audio_int16) - frame_size, frame_size):
            frame = audio_int16[i:i + frame_size].tobytes()
            if self.vad.is_speech(frame, sample_rate):
                speech_frames += 1
            total_frames += 1

        # Return True if majority of frames contain speech
        return total_frames > 0 and speech_frames / total_frames > 0.5


class CommandSegmenter:
    """Segment continuous audio into discrete commands."""

    def __init__(
        self,
        sample_rate: int = 16000,
        hangover_ms: int = 800,
        min_speech_ms: int = 200,
        max_command_ms: int = 10000
    ):
        self.sample_rate = sample_rate
        self.hangover_samples = int(hangover_ms * sample_rate / 1000)
        self.min_speech_samples = int(min_speech_ms * sample_rate / 1000)
        self.max_command_samples = int(max_command_ms * sample_rate / 1000)

        self.vad = EnergyVAD(adaptive=True)
        self.reset()

    def reset(self):
        """Reset segmenter state."""
        self.is_speaking = False
        self.speech_buffer = []
        self.silence_count = 0
        self.total_samples = 0

    def process_chunk(self, chunk: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Process audio chunk and detect command boundaries.

        Returns:
            (is_command_complete, audio_data or None)
        """
        is_speech, rms = self.vad.detect(chunk)

        if not self.is_speaking:
            # Waiting for speech onset
            if is_speech:
                self.is_speaking = True
                self.speech_buffer = [chunk]
                self.total_samples = len(chunk)
                self.silence_count = 0
                print("\n[SPEECH START]", end=' ', flush=True)
        else:
            # Currently capturing speech
            self.speech_buffer.append(chunk)
            self.total_samples += len(chunk)

            if is_speech:
                self.silence_count = 0
                print(".", end='', flush=True)
            else:
                self.silence_count += len(chunk)
                print("_", end='', flush=True)

            # Check for command completion
            if self.silence_count >= self.hangover_samples:
                # Hangover exceeded - command complete
                if self.total_samples >= self.min_speech_samples:
                    audio = np.concatenate(self.speech_buffer)
                    duration = len(audio) / self.sample_rate
                    print(f" [END: {duration:.1f}s]")
                    self.reset()
                    return True, audio
                else:
                    print(" [TOO SHORT]")
                    self.reset()

            # Check for maximum length
            if self.total_samples >= self.max_command_samples:
                audio = np.concatenate(self.speech_buffer)
                print(" [MAX LENGTH]")
                self.reset()
                return True, audio

        return False, None


def test_energy_vad():
    """Test energy-based VAD."""
    print("\n=== Energy-Based VAD Test ===")
    print("Speak or make sounds. Press Ctrl+C to stop.\n")

    vad = EnergyVAD(adaptive=True)

    def callback(indata, frames, time_info, status):
        is_speech, rms = vad.detect(indata.flatten())
        bars = int(rms * 100)
        status_str = "SPEECH" if is_speech else "      "
        threshold_bar = int(vad.threshold * 100)
        print(f"\r[{status_str}] {'|' * bars}{' ' * (50 - bars)} "
              f"T:{threshold_bar:2d} RMS:{rms:.3f}", end='', flush=True)

    try:
        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            callback=callback,
            blocksize=1024
        ):
            while True:
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n")


def test_webrtc_vad():
    """Test WebRTC VAD."""
    print("\n=== WebRTC VAD Test ===")
    print("Speak or make sounds. Press Ctrl+C to stop.\n")

    vad = WebRTCVAD(aggressiveness=2)
    if not vad.available:
        print("WebRTC VAD not available. Install: pip install webrtcvad")
        return

    def callback(indata, frames, time_info, status):
        is_speech = vad.detect(indata.flatten())
        status_str = "SPEECH" if is_speech else "      "
        print(f"\r[{status_str}]", end='', flush=True)

    try:
        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            callback=callback,
            blocksize=320  # 20ms frames for WebRTC
        ):
            while True:
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n")


def test_command_segmentation():
    """Test command segmentation."""
    print("\n=== Command Segmentation Test ===")
    print("Speak commands. Wait for segmentation. Press Ctrl+C to stop.\n")

    segmenter = CommandSegmenter(
        hangover_ms=800,
        min_speech_ms=200
    )

    commands_captured = []
    audio_buffer = []

    def callback(indata, frames, time_info, status):
        audio_buffer.append(indata.copy())

    try:
        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            callback=callback,
            blocksize=1024
        ):
            while True:
                if audio_buffer:
                    chunk = audio_buffer.pop(0).flatten()
                    is_complete, audio = segmenter.process_chunk(chunk)

                    if is_complete and audio is not None:
                        duration = len(audio) / SAMPLE_RATE
                        commands_captured.append(audio)
                        print(f"  Command #{len(commands_captured)}: "
                              f"{duration:.2f}s, {len(audio)} samples")
                else:
                    time.sleep(0.01)

    except KeyboardInterrupt:
        print(f"\n\nCaptured {len(commands_captured)} commands")


def main():
    """Main demonstration function."""
    print("=" * 60)
    print("Lesson 4: Voice Activity Detection")
    print("=" * 60)

    while True:
        print("\nSelect test:")
        print("1. Energy-based VAD")
        print("2. WebRTC VAD")
        print("3. Command Segmentation")
        print("4. Exit")

        choice = input("\nChoice (1-4): ").strip()

        if choice == "1":
            test_energy_vad()
        elif choice == "2":
            test_webrtc_vad()
        elif choice == "3":
            test_command_segmentation()
        elif choice == "4":
            break
        else:
            print("Invalid choice")

    print("\nDemo complete!")


if __name__ == "__main__":
    main()
