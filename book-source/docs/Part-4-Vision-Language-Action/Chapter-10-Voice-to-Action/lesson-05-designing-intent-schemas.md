---
sidebar_position: 5
---

# Lesson 5: Designing Intent Schemas

**Layer 2: AI Collaboration** | **Estimated Time: 120 minutes**

---

## Learning Objectives

By the end of this lesson, you will be able to:

- [ ] Design structured intent representations (action, target, parameters)
- [ ] Define intent categories for robotics (navigate, pick, query, control)
- [ ] Extract entities from natural language (locations, objects, colors)
- [ ] Handle ambiguous commands with confidence scoring
- [ ] Apply specification-first approach to parser design

---

## Intent Representation

An **intent** is a structured representation of what the user wants the robot to do.

### Basic Structure

```python
from dataclasses import dataclass
from typing import Dict, Any, Optional

@dataclass
class Intent:
    action: str              # What to do: "navigate", "pick", "query"
    target: Optional[str]    # Target: "kitchen", "ball", None
    parameters: Dict[str, Any]  # Additional info: {"color": "red"}
    confidence: float        # How certain: 0.0 to 1.0
    raw_text: str           # Original transcription
    error: Optional[str]     # Error if any: "target_unspecified"
```

### Example Intents

| Command | Intent |
|---------|--------|
| "Go to the kitchen" | `{action: "navigate", target: "kitchen", parameters: {}, confidence: 0.95}` |
| "Pick up the red ball" | `{action: "pick", target: "ball", parameters: {color: "red"}, confidence: 0.90}` |
| "Where are you?" | `{action: "query", target: "location", parameters: {}, confidence: 0.85}` |
| "Stop" | `{action: "control", target: "stop", parameters: {}, confidence: 0.98}` |
| "Get that thing" | `{action: "pick", target: null, parameters: {}, confidence: 0.3, error: "target_unspecified"}` |

---

## Intent Categories for Robotics

Design your schema based on robot capabilities.

### Navigation Intents

```python
NAVIGATION_PATTERNS = [
    "go to {location}",
    "navigate to {location}",
    "move to {location}",
    "head to {location}",
    "walk to {location}",
]

# Example intent
{
    "action": "navigate",
    "target": "kitchen",      # Extracted location
    "parameters": {
        "speed": "normal",    # Optional: "slow", "normal", "fast"
        "avoid_obstacles": True
    }
}
```

### Manipulation Intents

```python
MANIPULATION_PATTERNS = [
    "pick up {object}",
    "grab {object}",
    "put down {object}",
    "place {object} on {location}",
    "hand me {object}",
]

# Example intent
{
    "action": "pick",
    "target": "ball",
    "parameters": {
        "color": "red",       # Extracted from "red ball"
        "size": "small",      # If mentioned
        "location": "table"   # Where to find it
    }
}
```

### Query Intents

```python
QUERY_PATTERNS = [
    "where are you",
    "what do you see",
    "what is in front of you",
    "how far to {location}",
    "can you reach {object}",
]

# Example intent
{
    "action": "query",
    "target": "location",     # What to query about
    "parameters": {}
}
```

### Control Intents

```python
CONTROL_PATTERNS = [
    "stop",
    "pause",
    "resume",
    "cancel",
    "emergency stop",
]

# Example intent
{
    "action": "control",
    "target": "stop",         # Control type
    "parameters": {
        "immediate": True     # For emergency stop
    }
}
```

---

## Entity Extraction

Entities are the specific values extracted from commands.

### Common Entity Types

| Entity Type | Examples | Extraction Method |
|-------------|----------|-------------------|
| Location | kitchen, bedroom, shelf B-12 | Pattern matching, NER |
| Object | ball, cup, package | Pattern matching, NER |
| Color | red, blue, green | Keyword list |
| Size | small, large, big | Keyword list |
| Quantity | two, three, 5 | Number parsing |
| Direction | left, right, forward | Keyword list |

### Entity Extraction Code

```python
import re
from typing import Dict, List

COLORS = ["red", "blue", "green", "yellow", "black", "white", "orange", "purple"]
SIZES = ["small", "big", "large", "tiny", "huge"]
LOCATIONS = ["kitchen", "bedroom", "living room", "bathroom", "garage", "office"]

def extract_entities(text: str) -> Dict[str, str]:
    """Extract entities from transcribed text."""
    text_lower = text.lower()
    entities = {}

    # Extract color
    for color in COLORS:
        if color in text_lower:
            entities["color"] = color
            break

    # Extract size
    for size in SIZES:
        if size in text_lower:
            entities["size"] = size
            break

    # Extract location
    for location in LOCATIONS:
        if location in text_lower:
            entities["location"] = location
            break

    # Extract numbers
    numbers = re.findall(r'\b(\d+|one|two|three|four|five)\b', text_lower)
    if numbers:
        entities["quantity"] = numbers[0]

    return entities
```

---

## Slot Filling

Some intents require specific information (slots) to be filled.

### Required vs Optional Slots

```python
INTENT_SLOTS = {
    "navigate": {
        "required": ["target"],      # Must have destination
        "optional": ["speed"]
    },
    "pick": {
        "required": ["target"],      # Must specify object
        "optional": ["color", "size", "location"]
    },
    "query": {
        "required": ["target"],      # What to query
        "optional": []
    },
    "control": {
        "required": [],              # No required slots
        "optional": ["immediate"]
    }
}

def validate_intent(intent: Intent) -> bool:
    """Check if all required slots are filled."""
    slots = INTENT_SLOTS.get(intent.action, {})
    required = slots.get("required", [])

    for slot in required:
        if slot == "target" and not intent.target:
            return False
        if slot in intent.parameters and not intent.parameters[slot]:
            return False

    return True
```

---

## Ambiguity Handling

Real commands are often ambiguous. Handle gracefully.

### Types of Ambiguity

1. **Missing target**: "Pick that up" - what object?
2. **Multiple targets**: "Move the ball and cup" - which first?
3. **Unclear reference**: "Go there" - where?
4. **Similar commands**: "Go/Walk/Head to kitchen" - same intent

### Confidence-Based Handling

```python
def handle_ambiguity(intent: Intent) -> Intent:
    """Handle ambiguous intents."""

    # Missing target
    if intent.action in ["navigate", "pick"] and not intent.target:
        intent.confidence = 0.3
        intent.error = "target_unspecified"

    # Unclear reference
    pronouns = ["it", "that", "this", "there", "here"]
    if intent.target and intent.target.lower() in pronouns:
        intent.confidence = 0.4
        intent.error = "unclear_reference"

    return intent

def should_request_clarification(intent: Intent) -> bool:
    """Determine if we should ask for clarification."""
    return intent.confidence < 0.6 or intent.error is not None
```

### Clarification Responses

```python
CLARIFICATION_MESSAGES = {
    "target_unspecified": "I didn't catch what you want me to {action}. Could you specify?",
    "unclear_reference": "When you say '{target}', what are you referring to?",
    "low_confidence": "I'm not sure I understood. Did you mean {suggestion}?",
}
```

---

## Try With AI: Schema Design

### AI as Teacher

**Scenario**: Student designs flat intent schema without entity extraction.

**Student's schema**:
```python
intents = ["go_to_kitchen", "go_to_bedroom", "pick_up_ball", ...]
```

**AI explains**:
> Flat schemas don't generalize. For every new location, you need a new intent.
>
> **Better approach**: Extract entities separately:
> ```python
> intent = {action: "navigate", target: "kitchen"}
> ```
> Now any location works without adding new intents!

### AI as Student

**Scenario**: AI suggests complex nested schema.

**AI proposes**:
```python
{
    "intent": {
        "type": "action",
        "category": "navigation",
        "subcategory": "destination_based",
        "action_type": "go",
        ...
    }
}
```

**Student corrects**:
> "Our humanoid only needs 5 action types. This hierarchy adds parsing complexity without benefit. Keep it flat: action + target + parameters."

### AI as Co-Worker

**Iteration 1**: Ignore ambiguous commands
- Result: Robot picks up wrong object

**Iteration 2**: Reject all ambiguous commands
- Result: "I don't understand" too often - frustrating

**Iteration 3**: Confidence-based approach
- High confidence: Execute
- Low confidence: Ask specific clarification
- Result: Natural, helpful interaction

---

## Practice Exercise

### Goal
Design a complete intent schema for your humanoid robot.

### Steps

1. **Define action categories** (at least 4):
   - Navigation
   - Manipulation
   - Query
   - Control

2. **For each category, define**:
   - Pattern variations (3-5 phrases)
   - Required slots
   - Optional slots
   - Example intents

3. **Define entity types**:
   - Locations (room names)
   - Objects (things robot can manipulate)
   - Modifiers (colors, sizes)

4. **Create test commands** (20 total):
   - 5 clear navigation commands
   - 5 clear manipulation commands
   - 5 query/control commands
   - 5 ambiguous commands

5. **Run schema validation** (see `code/lesson_05_intent_schema.py`)

### Success Criteria

- [ ] 4+ action categories defined
- [ ] Entity extraction working for colors, locations, objects
- [ ] Ambiguity handling with confidence scores
- [ ] 20 test commands classified correctly
- [ ] Schema documented before parser implementation

---

## Lesson Checkpoint

Before proceeding, verify you can answer:

1. **What are the four components of an intent?**
   > action, target, parameters, confidence

2. **Why extract entities separately from actions?**
   > Generalizes to new locations/objects without new intent types

3. **How should you handle ambiguous commands?**
   > Assign low confidence, request specific clarification

4. **Why design the schema before the parser?**
   > Specification-first ensures clear requirements before implementation

---

## Summary

In this lesson, you learned:

- Intent structure: action, target, parameters, confidence
- Four main categories: navigate, pick/place, query, control
- Entity extraction for locations, objects, colors
- Slot filling with required vs optional parameters
- Confidence-based ambiguity handling
- Specification-first approach: schema before parser

**Next**: [Lesson 6: Implementing Intent Parsers](./lesson-06-implementing-intent-parsers.md) - Build parsers for your schema.
