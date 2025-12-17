#!/usr/bin/env python3
"""
Lesson 2: Local Whisper Transcription with WER Measurement
==========================================================

This script demonstrates:
1. Running Whisper locally using faster-whisper
2. Measuring Word Error Rate (WER)
3. Testing on robotics commands

Requirements:
    pip install faster-whisper jiwer

Usage:
    python lesson_02_whisper_local.py
"""

import os
import sys
from typing import List, Tuple
from jiwer import wer
import time


def load_whisper_model(model_size: str = "base"):
    """
    Load Whisper model with GPU/CPU fallback.

    Args:
        model_size: Model size (tiny, base, small, medium, large)

    Returns:
        WhisperModel instance
    """
    from faster_whisper import WhisperModel

    try:
        # Try GPU first
        print(f"Loading {model_size} model on GPU...")
        model = WhisperModel(model_size, device="cuda", compute_type="float16")
        print("GPU model loaded successfully!")
        return model
    except Exception as e:
        print(f"GPU not available ({e}), falling back to CPU...")
        model = WhisperModel(model_size, device="cpu", compute_type="int8")
        print("CPU model loaded successfully!")
        return model


def transcribe_audio(
    model,
    audio_path: str,
    language: str = "en",
    initial_prompt: str = None
) -> Tuple[str, float]:
    """
    Transcribe audio file using local Whisper model.

    Args:
        model: WhisperModel instance
        audio_path: Path to audio file
        language: Language code
        initial_prompt: Optional vocabulary guidance

    Returns:
        Tuple of (transcribed text, processing time)
    """
    start_time = time.time()

    segments, info = model.transcribe(
        audio_path,
        language=language,
        initial_prompt=initial_prompt,
        beam_size=5,
        word_timestamps=False,
        vad_filter=True
    )

    # Collect all segments
    text_parts = []
    for segment in segments:
        text_parts.append(segment.text.strip())

    text = " ".join(text_parts)
    elapsed = time.time() - start_time

    return text, elapsed


def calculate_wer(reference: str, hypothesis: str) -> float:
    """
    Calculate Word Error Rate between reference and hypothesis.

    Args:
        reference: Ground truth text
        hypothesis: Transcribed text

    Returns:
        WER as a float (0.0 to 1.0+)
    """
    # Normalize: lowercase, strip
    ref = reference.lower().strip()
    hyp = hypothesis.lower().strip()

    return wer(ref, hyp)


def run_wer_test(
    model,
    test_cases: List[Tuple[str, str]],
    initial_prompt: str = None
) -> List[dict]:
    """
    Run WER test on multiple audio files.

    Args:
        model: WhisperModel instance
        test_cases: List of (audio_path, reference_text) tuples
        initial_prompt: Optional vocabulary guidance

    Returns:
        List of result dictionaries
    """
    results = []

    for audio_path, reference in test_cases:
        if not os.path.exists(audio_path):
            print(f"  Skipping (not found): {audio_path}")
            continue

        text, elapsed = transcribe_audio(
            model, audio_path,
            initial_prompt=initial_prompt
        )

        error_rate = calculate_wer(reference, text)

        result = {
            "audio": audio_path,
            "reference": reference,
            "hypothesis": text,
            "wer": error_rate,
            "time": elapsed
        }
        results.append(result)

        # Print result
        status = "PASS" if error_rate < 0.1 else "FAIL"
        print(f"  [{status}] WER: {error_rate:.2%} | Time: {elapsed:.2f}s")
        print(f"       Reference: \"{reference}\"")
        print(f"       Hypothesis: \"{text}\"")
        print()

    return results


def main():
    """Main demonstration function."""
    print("=" * 60)
    print("Lesson 2: Local Whisper Transcription with WER Testing")
    print("=" * 60)

    # Load model
    print("\n1. Loading Model...")
    print("-" * 40)
    model = load_whisper_model("base")

    # Define robotics command vocabulary
    robotics_prompt = """
    Robotics commands: go to the kitchen, navigate to the bedroom,
    pick up the red ball, put down the cup, stop, pause, resume,
    cancel, where are you, what do you see, move forward
    """

    # Test cases (audio_path, reference_text)
    # Adjust paths based on your recorded files
    test_cases = [
        ("audio_recordings/go_to_the_kitchen.wav", "go to the kitchen"),
        ("audio_recordings/pick_up_the_red_ball.wav", "pick up the red ball"),
        ("audio_recordings/stop.wav", "stop"),
    ]

    print("\n2. Running WER Tests...")
    print("-" * 40)
    print()

    results = run_wer_test(model, test_cases, initial_prompt=robotics_prompt)

    # Summary
    if results:
        avg_wer = sum(r["wer"] for r in results) / len(results)
        avg_time = sum(r["time"] for r in results) / len(results)
        passed = sum(1 for r in results if r["wer"] < 0.1)

        print("=" * 60)
        print("Summary")
        print("=" * 60)
        print(f"Tests Run: {len(results)}")
        print(f"Tests Passed (WER < 10%): {passed}/{len(results)}")
        print(f"Average WER: {avg_wer:.2%}")
        print(f"Average Processing Time: {avg_time:.2f}s")

        if avg_wer < 0.1:
            print("\n SUCCESS: Target WER (<10%) achieved!")
        else:
            print("\n WARNING: Average WER exceeds 10% target")
            print("   Tips: Try recording in quieter environment,")
            print("   speak clearly, or use a better microphone.")
    else:
        print("\nNo audio files found for testing.")
        print("Run lesson_01_audio_visualization.py first to record samples.")

    print("\n" + "=" * 60)
    print("WER Test Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
