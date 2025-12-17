---
sidebar_position: 6
---

# Lesson 6: Implementing Intent Parsers

**Layer 2: AI Collaboration** | **Estimated Time: 120 minutes**

---

## Learning Objectives

By the end of this lesson, you will be able to:

- [ ] Implement rule-based parsing with regex and keywords
- [ ] Use LLM-based parsing for complex commands
- [ ] Build hybrid parsers (rules first, LLM fallback)
- [ ] Extract entities using pattern matching
- [ ] Calculate confidence scores for parsed intents

---

## Rule-Based Parsing

Fast and predictable parsing using patterns and keywords.

### Basic Pattern Matching

```python
import re
from lesson_05_intent_schema import Intent, IntentSchema

class RuleBasedParser:
    """Parse intents using regex patterns."""

    PATTERNS = {
        "navigate": [
            r"(?:go|navigate|move|head|walk)\s+to\s+(?:the\s+)?(.+)",
            r"take\s+me\s+to\s+(?:the\s+)?(.+)",
        ],
        "pick": [
            r"(?:pick\s+up|grab|get|fetch)\s+(?:the\s+)?(.+)",
            r"bring\s+me\s+(?:the\s+)?(.+)",
        ],
        "place": [
            r"(?:put|place|set)\s+(?:the\s+)?(.+?)\s+(?:on|in|at)\s+(?:the\s+)?(.+)",
        ],
        "query": [
            r"where\s+(?:are\s+you|am\s+i)",
            r"what\s+(?:do\s+you\s+see|is\s+(?:in\s+front|around))",
        ],
        "control": [
            r"^(stop|pause|resume|cancel)$",
            r"emergency\s+(stop)",
        ],
    }

    def parse(self, text: str) -> Intent:
        """Parse text into intent."""
        text_lower = text.lower().strip()

        for action, patterns in self.PATTERNS.items():
            for pattern in patterns:
                match = re.match(pattern, text_lower)
                if match:
                    # Extract target from capture group
                    groups = match.groups()
                    target = groups[0] if groups else None

                    # Extract additional entities
                    entities = IntentSchema.extract_entities(text)

                    return Intent(
                        action=action,
                        target=self._clean_target(target, action),
                        parameters=entities,
                        confidence=0.9,
                        raw_text=text
                    )

        # No pattern matched
        return Intent(
            action="unknown",
            confidence=0.2,
            raw_text=text,
            error="no_pattern_match"
        )

    def _clean_target(self, target: str, action: str) -> str:
        """Clean extracted target."""
        if not target:
            return None

        # Remove articles and clean up
        target = re.sub(r"^(the|a|an)\s+", "", target.strip())

        # For pick actions, extract object name
        if action == "pick":
            # Remove color/size modifiers to get object
            for word in IntentSchema.COLORS + IntentSchema.SIZES:
                target = target.replace(word, "").strip()

        return target if target else None
```

### Keyword-Based Parsing

```python
class KeywordParser:
    """Simple keyword-based intent detection."""

    KEYWORDS = {
        "navigate": ["go", "navigate", "move", "head", "walk", "take me"],
        "pick": ["pick", "grab", "get", "fetch", "bring"],
        "place": ["put", "place", "set", "drop"],
        "query": ["where", "what", "how far", "can you"],
        "control": ["stop", "pause", "resume", "cancel", "emergency"],
    }

    def detect_action(self, text: str) -> str:
        """Detect action type from keywords."""
        text_lower = text.lower()

        for action, keywords in self.KEYWORDS.items():
            if any(kw in text_lower for kw in keywords):
                return action

        return "unknown"
```

---

## LLM-Based Parsing

Use language models for complex, ambiguous commands.

### OpenAI Function Calling

```python
from openai import OpenAI

client = OpenAI()

INTENT_SCHEMA = {
    "name": "parse_robot_command",
    "description": "Parse a voice command into structured intent",
    "parameters": {
        "type": "object",
        "properties": {
            "action": {
                "type": "string",
                "enum": ["navigate", "pick", "place", "query", "control", "unknown"],
                "description": "The type of action requested"
            },
            "target": {
                "type": "string",
                "description": "Target of the action (location, object, etc.)"
            },
            "parameters": {
                "type": "object",
                "properties": {
                    "color": {"type": "string"},
                    "size": {"type": "string"},
                    "location": {"type": "string"},
                },
            },
            "confidence": {
                "type": "number",
                "description": "Confidence in parsing (0.0 to 1.0)"
            }
        },
        "required": ["action", "confidence"]
    }
}

class LLMParser:
    """Parse intents using LLM with function calling."""

    def parse(self, text: str) -> Intent:
        """Parse text using GPT function calling."""
        response = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {
                    "role": "system",
                    "content": "You are a robot command parser. Parse the user's voice command into a structured intent."
                },
                {"role": "user", "content": text}
            ],
            functions=[INTENT_SCHEMA],
            function_call={"name": "parse_robot_command"}
        )

        # Extract function call result
        func_call = response.choices[0].message.function_call
        result = json.loads(func_call.arguments)

        return Intent(
            action=result.get("action", "unknown"),
            target=result.get("target"),
            parameters=result.get("parameters", {}),
            confidence=result.get("confidence", 0.5),
            raw_text=text
        )
```

---

## Hybrid Approaches

Combine speed of rules with flexibility of LLM.

```python
class HybridParser:
    """Rules first, LLM fallback for ambiguous cases."""

    def __init__(self):
        self.rule_parser = RuleBasedParser()
        self.llm_parser = LLMParser()
        self.confidence_threshold = 0.7

    def parse(self, text: str) -> Intent:
        """Parse with rules, fall back to LLM if uncertain."""
        # Try rule-based first (fast)
        intent = self.rule_parser.parse(text)

        # If confident, return rule-based result
        if intent.confidence >= self.confidence_threshold:
            return intent

        # Fall back to LLM for uncertain cases
        try:
            llm_intent = self.llm_parser.parse(text)
            llm_intent.parameters["parse_method"] = "llm"
            return llm_intent
        except Exception as e:
            # LLM failed, return rule-based with low confidence
            intent.error = f"llm_fallback_failed: {e}"
            return intent
```

---

## Entity Recognition

Extract structured entities from natural language.

### Using spaCy NER

```python
import spacy

# Load English model
nlp = spacy.load("en_core_web_sm")

def extract_entities_spacy(text: str) -> dict:
    """Extract entities using spaCy NER."""
    doc = nlp(text)
    entities = {}

    for ent in doc.ents:
        if ent.label_ == "LOC":
            entities["location"] = ent.text
        elif ent.label_ == "PRODUCT":
            entities["object"] = ent.text

    return entities
```

### Combined Extraction

```python
def extract_all_entities(text: str) -> dict:
    """Combine multiple extraction methods."""
    entities = {}

    # Schema-based extraction (fast, domain-specific)
    entities.update(IntentSchema.extract_entities(text))

    # spaCy NER (slower, general)
    try:
        spacy_entities = extract_entities_spacy(text)
        # Only add if not already found
        for key, value in spacy_entities.items():
            if key not in entities:
                entities[key] = value
    except:
        pass  # spaCy not available

    return entities
```

---

## Confidence Scoring

Calculate how certain we are about the parsed intent.

```python
def calculate_confidence(text: str, intent: Intent) -> float:
    """Calculate confidence score for parsed intent."""
    confidence = 1.0

    # Penalize if no pattern match
    if intent.error == "no_pattern_match":
        confidence *= 0.3

    # Penalize missing required slots
    if intent.action in ["navigate", "pick"] and not intent.target:
        confidence *= 0.4

    # Penalize unclear references
    if intent.target and intent.target.lower() in ["it", "that", "this", "there"]:
        confidence *= 0.5

    # Boost for exact keyword matches
    keywords = {
        "navigate": ["go to", "navigate to"],
        "pick": ["pick up", "grab"],
        "control": ["stop", "pause"],
    }
    for action, kws in keywords.items():
        if intent.action == action and any(kw in text.lower() for kw in kws):
            confidence *= 1.1  # Small boost

    return min(confidence, 1.0)
```

---

## Parser Performance

Balance accuracy and latency.

### Benchmarking

```python
import time

def benchmark_parsers(test_commands: list):
    """Compare parser performance."""
    rule_parser = RuleBasedParser()
    hybrid_parser = HybridParser()

    results = {"rule": [], "hybrid": []}

    for cmd in test_commands:
        # Rule-based timing
        start = time.time()
        rule_parser.parse(cmd)
        results["rule"].append(time.time() - start)

        # Hybrid timing
        start = time.time()
        hybrid_parser.parse(cmd)
        results["hybrid"].append(time.time() - start)

    print(f"Rule-based: avg {sum(results['rule'])/len(results['rule'])*1000:.1f}ms")
    print(f"Hybrid: avg {sum(results['hybrid'])/len(results['hybrid'])*1000:.1f}ms")
```

### Performance Guidelines

| Parser Type | Latency | Use Case |
|-------------|---------|----------|
| Rule-based | Under 5ms | Clear commands, real-time |
| LLM-based | 200-500ms | Complex/ambiguous commands |
| Hybrid | 5-500ms | Production systems |

---

## Practice Exercise

### Goal
Implement and test a hybrid parser achieving >90% accuracy.

### Steps

1. **Implement rule-based parser** for 5 command types
2. **Add LLM fallback** for unclear commands
3. **Test with 50 commands** (see test set below)
4. **Measure accuracy and latency**

### Test Commands

```python
TEST_COMMANDS = [
    # Clear commands (should use rules)
    ("Go to the kitchen", "navigate", "kitchen"),
    ("Pick up the red ball", "pick", "ball"),
    ("Stop", "control", "stop"),

    # Variations (rules should handle)
    ("Navigate to the bedroom", "navigate", "bedroom"),
    ("Grab the cup", "pick", "cup"),

    # Ambiguous (may need LLM)
    ("Get that thing over there", "pick", None),
    ("Head over to where you were before", "navigate", None),
]
```

### Success Criteria

- [ ] Rule-based parser handles 80%+ of clear commands
- [ ] LLM fallback handles ambiguous cases
- [ ] Overall accuracy over 90% on test set
- [ ] Rule-based latency under 10ms
- [ ] Hybrid latency under 500ms for LLM cases

---

## Lesson Checkpoint

Before proceeding, verify you can answer:

1. **When should you use rule-based vs LLM parsing?**
   > Rules for clear patterns (fast), LLM for ambiguous (accurate)

2. **How does hybrid parsing work?**
   > Try rules first; if confidence low, fall back to LLM

3. **What affects confidence scoring?**
   > Pattern match quality, required slots, unclear references

4. **Why is latency important for robotics?**
   > Real-time response needed; under 2s end-to-end target

---

## Summary

In this lesson, you learned:

- Rule-based parsing with regex patterns
- LLM parsing using function calling
- Hybrid approach for speed + accuracy
- Entity extraction with pattern matching and NER
- Confidence scoring based on parse quality
- Performance benchmarking and optimization

**Next**: [Lesson 7: ROS 2 Integration](./lesson-07-ros2-integration.md) - Publish intents to ROS 2.
