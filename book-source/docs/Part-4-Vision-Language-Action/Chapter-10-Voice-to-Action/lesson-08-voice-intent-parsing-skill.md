---
sidebar_position: 8
---

# Lesson 8: Voice-Intent-Parsing Skill

**Layer 3: Intelligence Design** | **Estimated Time: 90 minutes**

---

## Learning Objectives

By the end of this lesson, you will be able to:

- [ ] Extract reusable patterns from your parsing work
- [ ] Design skills using the Persona + Questions + Principles framework
- [ ] Create a voice-intent-parsing skill for future projects
- [ ] Validate the skill on different command domains

---

## Pattern Extraction

Review what we learned in Lessons 5-6 to identify reusable patterns.

### What Worked Well?

| Pattern | Why It Works |
|---------|--------------|
| Action + Target + Parameters | Universal structure for commands |
| Entity extraction before parsing | Separates concerns, reusable |
| Confidence scoring | Enables graceful degradation |
| Hybrid parsing | Speed + accuracy |
| Specification-first | Clear requirements before code |

### What's Reusable?

These patterns apply to ANY voice interface:
- Intent schema design
- Entity extraction approach
- Confidence-based clarification
- Hybrid parsing strategy
- Error handling patterns

---

## Skill Design Framework

A **skill** encodes expertise for AI collaboration using three components:

1. **Persona**: Who should the AI think like?
2. **Questions**: What does the AI need to know?
3. **Principles**: What rules should guide decisions?

---

## Voice-Intent-Parsing Skill

### Persona Definition

```markdown
## Persona

Think like a **voice interface designer** creating intent parsers
for robotics applications.

Your goal is to transform natural language commands into structured,
actionable intents that robots can execute reliably.

You have expertise in:
- Natural language processing
- Schema design for robotics
- Balancing accuracy vs latency
- Handling ambiguity gracefully
```

### Question Structure

```markdown
## Questions to Ask

Before designing an intent parser, gather this information:

1. **Command Categories**
   What actions can the robot perform?
   (navigation, manipulation, queries, control, other?)

2. **Entity Types**
   What things need to be identified in commands?
   (locations, objects, colors, sizes, quantities?)

3. **Confidence Threshold**
   What confidence level triggers execution vs clarification?
   (typical: 0.6-0.8)

4. **Ambiguity Strategy**
   How should unclear commands be handled?
   (reject, ask clarification, best guess?)

5. **Latency Budget**
   What's the maximum acceptable parsing latency?
   (real-time: under 100ms, interactive: under 500ms, batch: over 500ms)
```

### Principle Articulation

```markdown
## Design Principles

1. **Schema Before Parser**
   Always define the intent schema before implementing the parser.
   What intents will you support? What are required vs optional fields?

2. **Entity Extraction First**
   Extract entities (locations, objects, colors) into a reusable
   structure rather than hardcoding (e.g., "go_to_kitchen").
   This enables generalization to new targets.

3. **Confidence-Based Execution**
   - High confidence (>0.8): Execute immediately
   - Medium confidence (0.5-0.8): Execute with confirmation
   - Low confidence (<0.5): Request clarification

4. **Hybrid Parsing Strategy**
   Use rule-based parsing for common patterns (fast).
   Fall back to LLM for complex/ambiguous cases (accurate).
   Never LLM-only for simple commands (wasteful).

5. **Graceful Fallback**
   Unknown commands should acknowledge and suggest alternatives.
   Never crash, never ignore, always respond helpfully.
```

---

## Complete Skill Template

```markdown
# Voice Intent Parsing Skill

## Persona

Think like a voice interface designer creating intent parsers
for robotics applications. Your goal is to transform natural
language commands into structured, actionable intents.

## Questions

Before designing a parser, answer:

1. What command categories does the robot support?
2. What entities need extraction? (locations, objects, modifiers)
3. What confidence threshold for execution vs clarification?
4. How should ambiguous commands be handled?
5. What latency budget exists for parsing?

## Principles

1. **Schema Before Parser**: Define intent structure before coding
2. **Entity Extraction**: Separate entities from actions for reuse
3. **Confidence-Based**: High→execute, Low→clarify
4. **Hybrid Parsing**: Rules for common, LLM for complex
5. **Graceful Fallback**: Always respond helpfully to unknowns

## Application Checklist

When applying this skill:

- [ ] Document supported command categories
- [ ] Define intent schema with required/optional fields
- [ ] List all entity types and valid values
- [ ] Set confidence thresholds
- [ ] Implement rule patterns for >80% of commands
- [ ] Add LLM fallback for remaining cases
- [ ] Test with 50+ diverse commands
- [ ] Measure accuracy (target: >90%)
- [ ] Measure latency (target: <500ms average)
```

---

## Usage Validation

Test the skill on different domains.

### Domain: Home Automation

```python
# Apply skill to home automation

# 1. Command categories
CATEGORIES = ["control_light", "control_thermostat", "query_status"]

# 2. Entities
ENTITIES = {
    "rooms": ["living room", "bedroom", "kitchen"],
    "states": ["on", "off", "dim"],
    "temperatures": ["warmer", "cooler", "72 degrees"]
}

# 3. Example intents
{
    "action": "control_light",
    "target": "living room",
    "parameters": {"state": "on"},
    "confidence": 0.95
}
```

### Domain: Manufacturing Robot

```python
# Apply skill to factory robot

# 1. Command categories
CATEGORIES = ["move_arm", "pick_part", "inspect", "emergency"]

# 2. Entities
ENTITIES = {
    "positions": ["position A", "home", "conveyor"],
    "parts": ["bolt", "bracket", "housing"],
    "tools": ["gripper", "welder", "camera"]
}

# 3. Example intents
{
    "action": "pick_part",
    "target": "bolt",
    "parameters": {"from": "bin 3", "tool": "gripper"},
    "confidence": 0.88
}
```

---

## Practice Exercise

### Goal
Apply the voice-intent-parsing skill to a new domain.

### Steps

1. **Choose a domain** (not humanoid navigation):
   - Smart home assistant
   - Manufacturing robot
   - Drone control
   - Customer service bot

2. **Apply the Questions**:
   - Document command categories (5+)
   - List entity types and values
   - Set confidence thresholds
   - Define ambiguity strategy
   - Specify latency budget

3. **Apply the Principles**:
   - Design intent schema
   - Implement entity extraction
   - Add confidence scoring
   - Create hybrid parser

4. **Validate**:
   - Test with 20 commands from new domain
   - Measure accuracy
   - Document lessons learned

### Success Criteria

- [ ] Skill applied to non-robotics domain
- [ ] 5+ command categories defined
- [ ] Intent schema documented
- [ ] 20 test commands parsed
- [ ] Accuracy >85% on new domain
- [ ] Skill generalizes without code changes

---

## Lesson Checkpoint

Before proceeding, verify you can answer:

1. **What are the three components of a skill?**
   > Persona, Questions, Principles

2. **Why "Schema Before Parser"?**
   > Clear requirements prevent wasted implementation effort

3. **When should you use LLM vs rules?**
   > Rules for common patterns, LLM for complex/ambiguous

4. **How do you validate a skill generalizes?**
   > Apply to different domain without code changes

---

## Summary

In this lesson, you learned:

- Extract reusable patterns from implementation experience
- Design skills using Persona + Questions + Principles
- The voice-intent-parsing skill for any voice interface
- Validation through application to different domains

The skill you created can be reused in Chapter 11 (LLM planning) and beyond!

**Next**: [Lesson 9: Capstone - Voice-Controlled Navigation](./lesson-09-capstone-voice-controlled-navigation.md) - Full integration project.
