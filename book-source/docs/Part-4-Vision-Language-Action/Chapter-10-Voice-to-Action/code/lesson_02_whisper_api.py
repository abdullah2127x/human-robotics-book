#!/usr/bin/env python3
"""
Lesson 2: Whisper API Transcription
===================================

This script demonstrates using OpenAI's Whisper API for transcription.

Requirements:
    pip install openai

Setup:
    export OPENAI_API_KEY="sk-your-key-here"

Usage:
    python lesson_02_whisper_api.py audio_file.wav
"""

import os
import sys
from openai import OpenAI, APIError, RateLimitError
import time


def transcribe_with_api(audio_path: str, language: str = "en") -> dict:
    """
    Transcribe audio using OpenAI Whisper API.

    Args:
        audio_path: Path to audio file (WAV, MP3, etc.)
        language: Language code (e.g., "en" for English)

    Returns:
        dict with text and metadata
    """
    client = OpenAI()

    with open(audio_path, "rb") as audio_file:
        # Basic transcription
        transcript = client.audio.transcriptions.create(
            model="whisper-1",
            file=audio_file,
            language=language,
            response_format="verbose_json"  # Get additional metadata
        )

    return {
        "text": transcript.text,
        "language": transcript.language,
        "duration": transcript.duration,
    }


def transcribe_with_retry(
    audio_path: str,
    max_retries: int = 2,
    language: str = "en"
) -> str:
    """
    Transcribe with exponential backoff retry on failure.

    Args:
        audio_path: Path to audio file
        max_retries: Maximum retry attempts
        language: Language code

    Returns:
        Transcribed text or None on failure
    """
    client = OpenAI()

    for attempt in range(max_retries + 1):
        try:
            with open(audio_path, "rb") as f:
                result = client.audio.transcriptions.create(
                    model="whisper-1",
                    file=f,
                    language=language
                )
            return result.text

        except RateLimitError:
            if attempt < max_retries:
                wait_time = 2 ** attempt
                print(f"Rate limited. Waiting {wait_time}s before retry...")
                time.sleep(wait_time)
                continue
            print("Max retries exceeded due to rate limiting")
            raise

        except APIError as e:
            print(f"API Error: {e}")
            if attempt < max_retries:
                time.sleep(1)
                continue
            raise

    return None


def transcribe_with_prompt(
    audio_path: str,
    prompt: str,
    language: str = "en"
) -> str:
    """
    Transcribe with an initial prompt for vocabulary guidance.

    Args:
        audio_path: Path to audio file
        prompt: Context/vocabulary to guide transcription
        language: Language code

    Returns:
        Transcribed text
    """
    client = OpenAI()

    with open(audio_path, "rb") as f:
        result = client.audio.transcriptions.create(
            model="whisper-1",
            file=f,
            language=language,
            prompt=prompt
        )

    return result.text


def main():
    """Main demonstration function."""
    print("=" * 60)
    print("Lesson 2: Whisper API Transcription")
    print("=" * 60)

    # Check for API key
    if not os.environ.get("OPENAI_API_KEY"):
        print("\nError: OPENAI_API_KEY environment variable not set")
        print("Run: export OPENAI_API_KEY='sk-your-key-here'")
        sys.exit(1)

    # Get audio file from command line or use default
    if len(sys.argv) > 1:
        audio_path = sys.argv[1]
    else:
        audio_path = "audio_recordings/go_to_the_kitchen.wav"

    if not os.path.exists(audio_path):
        print(f"\nError: Audio file not found: {audio_path}")
        print("Usage: python lesson_02_whisper_api.py <audio_file.wav>")
        sys.exit(1)

    print(f"\nTranscribing: {audio_path}")
    print("-" * 40)

    # Basic transcription
    print("\n1. Basic Transcription:")
    result = transcribe_with_api(audio_path)
    print(f"   Text: {result['text']}")
    print(f"   Language: {result['language']}")
    print(f"   Duration: {result['duration']:.2f}s")

    # Transcription with robotics prompt
    print("\n2. Transcription with Robotics Prompt:")
    robotics_prompt = """
    Robotics commands: navigate, go to, move to, pick up, put down,
    grab, stop, pause, resume, cancel, where are you, what do you see
    """
    text = transcribe_with_prompt(audio_path, robotics_prompt)
    print(f"   Text: {text}")

    # Transcription with retry logic
    print("\n3. Transcription with Retry Logic:")
    text = transcribe_with_retry(audio_path)
    print(f"   Text: {text}")

    print("\n" + "=" * 60)
    print("Transcription Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
