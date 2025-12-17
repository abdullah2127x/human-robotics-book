#!/usr/bin/env python3
"""
Lesson 3: Real-Time Audio Capture
=================================

This script demonstrates real-time audio capture with:
1. Callback-based streaming
2. Thread-safe processing
3. Direct Whisper transcription

Requirements:
    pip install sounddevice numpy faster-whisper

Usage:
    python lesson_03_audio_capture.py
"""

import sounddevice as sd
import numpy as np
import queue
import threading
import time
import sys
from typing import Optional


# Audio Configuration
SAMPLE_RATE = 16000  # Whisper expects 16kHz
CHANNELS = 1
BUFFER_SIZE = 1024  # Adjust for latency/stability tradeoff
SILENCE_THRESHOLD = 0.01


class RealTimeAudioCapture:
    """Real-time audio capture with continuous streaming."""

    def __init__(
        self,
        sample_rate: int = SAMPLE_RATE,
        channels: int = CHANNELS,
        buffer_size: int = BUFFER_SIZE
    ):
        self.sample_rate = sample_rate
        self.channels = channels
        self.buffer_size = buffer_size

        self.audio_queue = queue.Queue()
        self.is_recording = False
        self.stream: Optional[sd.InputStream] = None

        # Statistics
        self.chunks_captured = 0
        self.chunks_dropped = 0

    def audio_callback(self, indata, frames, time_info, status):
        """
        Called for each audio buffer from the sound card.

        IMPORTANT: This runs in a separate thread and must be fast!
        Do not do heavy processing here.
        """
        if status:
            print(f"Audio status: {status}", file=sys.stderr)
            if status.input_overflow:
                self.chunks_dropped += 1

        # Copy data to queue (fast operation)
        self.audio_queue.put(indata.copy())
        self.chunks_captured += 1

    def start(self):
        """Start audio capture stream."""
        print(f"Starting audio capture...")
        print(f"  Sample rate: {self.sample_rate} Hz")
        print(f"  Buffer size: {self.buffer_size} samples")
        print(f"  Buffer latency: {self.buffer_size / self.sample_rate * 1000:.1f} ms")

        self.is_recording = True
        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=self.channels,
            dtype='float32',
            blocksize=self.buffer_size,
            callback=self.audio_callback
        )
        self.stream.start()
        print("Audio capture started!")

    def stop(self):
        """Stop audio capture stream."""
        self.is_recording = False
        if self.stream:
            self.stream.stop()
            self.stream.close()
            self.stream = None
        print(f"\nAudio capture stopped.")
        print(f"  Chunks captured: {self.chunks_captured}")
        print(f"  Chunks dropped: {self.chunks_dropped}")

    def get_audio_chunk(self, timeout: float = 0.1) -> Optional[np.ndarray]:
        """
        Get next audio chunk from queue.

        Args:
            timeout: Maximum time to wait for chunk

        Returns:
            Audio chunk or None if timeout
        """
        try:
            return self.audio_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def collect_audio(self, duration: float) -> np.ndarray:
        """
        Collect audio for specified duration.

        Args:
            duration: Duration in seconds

        Returns:
            Concatenated audio array
        """
        chunks = []
        samples_needed = int(duration * self.sample_rate)
        samples_collected = 0

        while samples_collected < samples_needed:
            chunk = self.get_audio_chunk(timeout=0.5)
            if chunk is not None:
                chunks.append(chunk)
                samples_collected += len(chunk)

        return np.concatenate(chunks).flatten()


def list_audio_devices():
    """List available audio input devices."""
    print("\n=== Available Audio Devices ===")
    devices = sd.query_devices()
    for i, d in enumerate(devices):
        if d['max_input_channels'] > 0:
            marker = '>' if i == sd.default.device[0] else ' '
            print(f"{marker} [{i}] {d['name']} ({d['max_input_channels']} ch)")


def test_audio_levels(capture: RealTimeAudioCapture, duration: float = 5.0):
    """Test audio capture and show levels."""
    print(f"\n=== Testing Audio Levels ({duration}s) ===")
    print("Speak into the microphone...")
    print("Level: ", end='', flush=True)

    start_time = time.time()
    while time.time() - start_time < duration:
        chunk = capture.get_audio_chunk()
        if chunk is not None:
            # Calculate RMS level
            rms = np.sqrt(np.mean(chunk ** 2))
            # Display as bar
            bars = int(rms * 50)
            print(f"\rLevel: {'|' * bars}{' ' * (50 - bars)} {rms:.4f}", end='', flush=True)

    print("\n")


def demo_with_whisper():
    """Demonstrate real-time capture with Whisper transcription."""
    try:
        from faster_whisper import WhisperModel
    except ImportError:
        print("faster-whisper not installed. Skipping Whisper demo.")
        print("Install with: pip install faster-whisper")
        return

    print("\n=== Real-Time Whisper Demo ===")
    print("Loading Whisper model...")

    try:
        model = WhisperModel("base", device="cuda", compute_type="float16")
        print("Using GPU")
    except Exception:
        model = WhisperModel("base", device="cpu", compute_type="int8")
        print("Using CPU")

    capture = RealTimeAudioCapture()
    capture.start()

    print("\nSpeak commands (3 seconds each). Press Ctrl+C to stop.\n")

    try:
        while True:
            print("Listening... ", end='', flush=True)

            # Collect 3 seconds of audio
            audio = capture.collect_audio(3.0)

            # Check if there's speech
            rms = np.sqrt(np.mean(audio ** 2))
            if rms < SILENCE_THRESHOLD:
                print("(silence)")
                continue

            print("Transcribing... ", end='', flush=True)

            # Transcribe
            start = time.time()
            segments, _ = model.transcribe(audio, language="en")
            text = " ".join(s.text.strip() for s in segments)
            elapsed = time.time() - start

            print(f"Done ({elapsed:.2f}s)")
            print(f">>> {text}\n")

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        capture.stop()


def main():
    """Main demonstration function."""
    print("=" * 60)
    print("Lesson 3: Real-Time Audio Capture")
    print("=" * 60)

    # List devices
    list_audio_devices()

    # Create capture instance
    capture = RealTimeAudioCapture()

    # Start capture
    capture.start()

    # Test levels
    test_audio_levels(capture, duration=5.0)

    # Stop capture
    capture.stop()

    # Demo with Whisper
    demo_with_whisper()

    print("\n" + "=" * 60)
    print("Demo Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
