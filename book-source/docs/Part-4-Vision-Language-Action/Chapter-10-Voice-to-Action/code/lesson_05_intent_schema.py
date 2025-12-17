#!/usr/bin/env python3
"""
Lesson 5: Intent Schema Definition
==================================

This module defines the intent schema for voice commands.
Design your schema BEFORE implementing parsers (specification-first).

Usage:
    from lesson_05_intent_schema import Intent, IntentSchema
"""

from dataclasses import dataclass, field
from typing import Dict, Any, Optional, List
from enum import Enum
import re


class ActionType(Enum):
    """Supported action categories."""
    NAVIGATE = "navigate"
    PICK = "pick"
    PLACE = "place"
    QUERY = "query"
    CONTROL = "control"
    UNKNOWN = "unknown"


@dataclass
class Intent:
    """
    Structured representation of a voice command.

    Attributes:
        action: What to do (navigate, pick, query, control)
        target: Target of action (location, object, query type)
        parameters: Additional details (color, size, speed)
        confidence: Parser confidence (0.0 to 1.0)
        raw_text: Original transcribed text
        error: Error description if parsing failed
    """
    action: str
    target: Optional[str] = None
    parameters: Dict[str, Any] = field(default_factory=dict)
    confidence: float = 1.0
    raw_text: str = ""
    error: Optional[str] = None

    def is_valid(self) -> bool:
        """Check if intent has required information."""
        if self.action in ["navigate", "pick", "place"]:
            return self.target is not None and self.confidence >= 0.5
        return self.confidence >= 0.5

    def needs_clarification(self) -> bool:
        """Check if clarification should be requested."""
        return self.confidence < 0.6 or self.error is not None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for ROS 2 message."""
        return {
            "action": self.action,
            "target": self.target or "",
            "parameters": self.parameters,
            "confidence": self.confidence,
            "raw_text": self.raw_text,
            "error": self.error or ""
        }


class IntentSchema:
    """
    Schema definition for voice command intents.

    This class defines:
    - Supported action types
    - Required/optional slots per action
    - Entity types and valid values
    """

    # Valid entity values
    LOCATIONS = [
        "kitchen", "bedroom", "living room", "bathroom",
        "garage", "office", "hallway", "entrance", "exit"
    ]

    OBJECTS = [
        "ball", "cup", "bottle", "book", "phone", "remote",
        "box", "package", "bag", "tool", "key"
    ]

    COLORS = [
        "red", "blue", "green", "yellow", "black", "white",
        "orange", "purple", "pink", "brown", "gray"
    ]

    SIZES = ["small", "big", "large", "tiny", "medium"]

    QUERY_TYPES = ["location", "status", "objects", "path", "battery"]

    CONTROL_TYPES = ["stop", "pause", "resume", "cancel", "emergency"]

    # Slot requirements per action type
    SLOTS = {
        "navigate": {
            "required": ["target"],
            "optional": ["speed", "path_type"]
        },
        "pick": {
            "required": ["target"],
            "optional": ["color", "size", "location"]
        },
        "place": {
            "required": ["target", "destination"],
            "optional": []
        },
        "query": {
            "required": ["target"],
            "optional": []
        },
        "control": {
            "required": [],
            "optional": ["immediate"]
        },
        "unknown": {
            "required": [],
            "optional": []
        }
    }

    @classmethod
    def validate_intent(cls, intent: Intent) -> List[str]:
        """
        Validate intent against schema.

        Returns:
            List of validation errors (empty if valid)
        """
        errors = []

        # Check action type
        valid_actions = [a.value for a in ActionType]
        if intent.action not in valid_actions:
            errors.append(f"Unknown action: {intent.action}")

        # Check required slots
        slots = cls.SLOTS.get(intent.action, {})
        for slot in slots.get("required", []):
            if slot == "target" and not intent.target:
                errors.append(f"Missing required slot: {slot}")
            elif slot in ["destination"] and slot not in intent.parameters:
                errors.append(f"Missing required parameter: {slot}")

        # Validate entity values
        if intent.target:
            if intent.action == "navigate" and intent.target not in cls.LOCATIONS:
                # Allow unknown locations but flag
                if not any(loc in intent.target.lower() for loc in cls.LOCATIONS):
                    errors.append(f"Unknown location: {intent.target}")

            if intent.action in ["pick", "place"] and intent.target not in cls.OBJECTS:
                if not any(obj in intent.target.lower() for obj in cls.OBJECTS):
                    errors.append(f"Unknown object: {intent.target}")

        # Validate parameter values
        if "color" in intent.parameters:
            if intent.parameters["color"] not in cls.COLORS:
                errors.append(f"Unknown color: {intent.parameters['color']}")

        return errors

    @classmethod
    def extract_entities(cls, text: str) -> Dict[str, Any]:
        """
        Extract entities from text.

        Returns:
            Dictionary of extracted entities
        """
        text_lower = text.lower()
        entities = {}

        # Extract color
        for color in cls.COLORS:
            if color in text_lower:
                entities["color"] = color
                break

        # Extract size
        for size in cls.SIZES:
            if size in text_lower:
                entities["size"] = size
                break

        # Extract location
        for location in cls.LOCATIONS:
            if location in text_lower:
                entities["location"] = location
                break

        # Extract object
        for obj in cls.OBJECTS:
            if obj in text_lower:
                entities["object"] = obj
                break

        # Extract numbers
        number_map = {
            "one": "1", "two": "2", "three": "3",
            "four": "4", "five": "5"
        }
        for word, num in number_map.items():
            if word in text_lower:
                entities["quantity"] = num
                break

        numbers = re.findall(r'\b(\d+)\b', text)
        if numbers and "quantity" not in entities:
            entities["quantity"] = numbers[0]

        return entities


def test_schema():
    """Test the intent schema."""
    print("=" * 60)
    print("Intent Schema Test")
    print("=" * 60)

    # Test valid intents
    test_cases = [
        Intent(
            action="navigate",
            target="kitchen",
            confidence=0.95,
            raw_text="Go to the kitchen"
        ),
        Intent(
            action="pick",
            target="ball",
            parameters={"color": "red"},
            confidence=0.90,
            raw_text="Pick up the red ball"
        ),
        Intent(
            action="query",
            target="location",
            confidence=0.85,
            raw_text="Where are you?"
        ),
        Intent(
            action="control",
            target="stop",
            confidence=0.98,
            raw_text="Stop"
        ),
        Intent(
            action="pick",
            target=None,  # Missing target
            confidence=0.3,
            error="target_unspecified",
            raw_text="Pick that up"
        ),
    ]

    for i, intent in enumerate(test_cases, 1):
        print(f"\nTest {i}: {intent.raw_text}")
        print(f"  Intent: {intent.to_dict()}")
        print(f"  Valid: {intent.is_valid()}")
        print(f"  Needs clarification: {intent.needs_clarification()}")

        errors = IntentSchema.validate_intent(intent)
        if errors:
            print(f"  Validation errors: {errors}")
        else:
            print(f"  Validation: PASSED")

    # Test entity extraction
    print("\n" + "=" * 60)
    print("Entity Extraction Test")
    print("=" * 60)

    texts = [
        "Pick up the red ball",
        "Go to the kitchen",
        "Put the small cup on the table",
        "Get me three books",
    ]

    for text in texts:
        entities = IntentSchema.extract_entities(text)
        print(f"\n  '{text}'")
        print(f"  Entities: {entities}")


if __name__ == "__main__":
    test_schema()
