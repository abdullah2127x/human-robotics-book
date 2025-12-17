#!/usr/bin/env python3
"""
Lesson 1: Audio Visualization for Speech Recognition
=====================================================

This script demonstrates:
1. Recording audio from microphone
2. Visualizing waveforms
3. Generating spectrograms
4. Understanding audio properties

Requirements:
    pip install sounddevice numpy scipy matplotlib

Usage:
    python lesson_01_audio_visualization.py
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.io import wavfile
import sounddevice as sd
import os
from datetime import datetime


# Audio Configuration
SAMPLE_RATE = 16000  # Whisper expects 16kHz
DURATION = 3  # seconds
CHANNELS = 1  # mono audio


def list_audio_devices():
    """List available audio input devices."""
    print("\n=== Available Audio Devices ===")
    print(sd.query_devices())
    print(f"\nDefault input device: {sd.default.device[0]}")
    print(f"Default output device: {sd.default.device[1]}")


def record_audio(duration: float = DURATION) -> np.ndarray:
    """
    Record audio from the default microphone.

    Args:
        duration: Recording duration in seconds

    Returns:
        numpy array of audio samples
    """
    print(f"\n Recording for {duration} seconds...")
    print("Speak now!")

    # Record audio
    recording = sd.rec(
        int(duration * SAMPLE_RATE),
        samplerate=SAMPLE_RATE,
        channels=CHANNELS,
        dtype='float32'
    )
    sd.wait()  # Wait until recording is finished

    print("Recording complete!")
    return recording.flatten()


def plot_waveform(audio: np.ndarray, title: str = "Waveform"):
    """
    Plot the waveform of an audio signal.

    Args:
        audio: Audio samples
        title: Plot title
    """
    time = np.linspace(0, len(audio) / SAMPLE_RATE, len(audio))

    plt.figure(figsize=(12, 4))
    plt.plot(time, audio, linewidth=0.5)
    plt.xlabel('Time (seconds)')
    plt.ylabel('Amplitude')
    plt.title(title)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

    return plt.gcf()


def plot_spectrogram(audio: np.ndarray, title: str = "Spectrogram"):
    """
    Plot the spectrogram of an audio signal.

    Args:
        audio: Audio samples
        title: Plot title
    """
    # Compute spectrogram
    frequencies, times, spectrogram = signal.spectrogram(
        audio,
        fs=SAMPLE_RATE,
        nperseg=256,
        noverlap=128
    )

    plt.figure(figsize=(12, 4))
    plt.pcolormesh(
        times,
        frequencies,
        10 * np.log10(spectrogram + 1e-10),  # Convert to dB
        shading='gouraud',
        cmap='viridis'
    )
    plt.ylabel('Frequency (Hz)')
    plt.xlabel('Time (seconds)')
    plt.title(title)
    plt.colorbar(label='Power (dB)')
    plt.ylim(0, 8000)  # Focus on speech frequencies
    plt.tight_layout()

    return plt.gcf()


def analyze_audio(audio: np.ndarray):
    """
    Print audio statistics.

    Args:
        audio: Audio samples
    """
    print("\n=== Audio Statistics ===")
    print(f"Sample rate: {SAMPLE_RATE} Hz")
    print(f"Duration: {len(audio) / SAMPLE_RATE:.2f} seconds")
    print(f"Number of samples: {len(audio)}")
    print(f"Bit depth: 32-bit float (converted to 16-bit for Whisper)")
    print(f"Max amplitude: {np.max(np.abs(audio)):.4f}")
    print(f"Mean amplitude: {np.mean(np.abs(audio)):.4f}")
    print(f"RMS (loudness): {np.sqrt(np.mean(audio**2)):.4f}")


def save_audio(audio: np.ndarray, filename: str):
    """
    Save audio to WAV file.

    Args:
        audio: Audio samples (float32)
        filename: Output filename
    """
    # Convert to 16-bit integer for WAV file
    audio_int16 = (audio * 32767).astype(np.int16)
    wavfile.write(filename, SAMPLE_RATE, audio_int16)
    print(f"Saved to {filename}")


def main():
    """Main demonstration function."""
    print("=" * 60)
    print("Lesson 1: Audio Visualization for Speech Recognition")
    print("=" * 60)

    # List available devices
    list_audio_devices()

    # Create output directory
    output_dir = "audio_recordings"
    os.makedirs(output_dir, exist_ok=True)

    # Commands to record
    commands = [
        "Go to the kitchen",
        "Pick up the red ball",
        "Stop"
    ]

    recordings = []

    for i, command in enumerate(commands, 1):
        print(f"\n{'='*60}")
        print(f"Recording {i}/{len(commands)}: \"{command}\"")
        print("="*60)

        input("Press Enter when ready to record...")

        # Record
        audio = record_audio()
        recordings.append((command, audio))

        # Analyze
        analyze_audio(audio)

        # Save
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_command = command.replace(" ", "_").lower()
        filename = f"{output_dir}/{safe_command}_{timestamp}.wav"
        save_audio(audio, filename)

        # Visualize
        plot_waveform(audio, f"Waveform: \"{command}\"")
        plt.savefig(f"{output_dir}/{safe_command}_waveform.png", dpi=150)

        plot_spectrogram(audio, f"Spectrogram: \"{command}\"")
        plt.savefig(f"{output_dir}/{safe_command}_spectrogram.png", dpi=150)

        print(f"Visualizations saved to {output_dir}/")

    # Show all plots
    plt.show()

    print("\n" + "="*60)
    print("Exercise Complete!")
    print("="*60)
    print(f"\nCheck the '{output_dir}/' directory for:")
    print("  - WAV audio files")
    print("  - Waveform visualizations")
    print("  - Spectrogram visualizations")
    print("\nAnswer the questions in the lesson to verify understanding.")


if __name__ == "__main__":
    main()
