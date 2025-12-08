---
id: 0001
title: Initial constitution update
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: ["constitution", "update", "initial"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md (modified)
  - .specify/templates/plan-template.md (modified)
tests: []
---

## Prompt

```text

# Content Constitution: Physical AI & Humanoid Robotics

## Design Mandate

This constitution activates the system’s reasoning mode to ensure high-quality, verifiable, and pedagogically robust content. The mandate focuses strictly on governance, methodology, and instructional structure. Topic-specific material is outside scope.

## Core Thesis

Reusable Intelligence (Specifications, Architectures, Skills) replaces Reusable Code as the primary educational asset. “Code” is interpreted as mechanical implementation. All claims, processes, and outputs must be verified using Execution Logs, Test Suites, and Official API Documentation across Robotics, SaaS, and Data Science contexts.

---

## I. The Core Pedagogical Strategy (4-Layer Framework)

### 1. Manual Foundation

Students must first demonstrate independent comprehension, correct execution, and the ability to recognize errors. Advancement requires validated conceptual mastery.

### 2. AI Collaboration

Students develop prompting, validation, and cross-checking skills through structured bidirectional learning using the Three Roles Framework. Advancement requires consistent pattern recognition indicating emerging reusability.

### 3. Intelligence Design

Students convert identified patterns into reusable Skills or Subagents using Persona, Questions, and Principles. Advancement requires evidence of intelligence accumulation across at least three components and demonstrated Specification Capability.

### 4. Spec-Driven Integration

Students orchestrate accumulated intelligence to complete a capstone project governed entirely by clear, testable specifications.

---

## II. Foundational Principles (Mandates)

### Specification Primacy

Specification of intent (WHAT) must precede implementation (HOW). Code exists solely as the output of a verified specification.

### Progressive Complexity

All content must be rigorously chunked to match learner tiers (A1–C2), ensuring cognitive load remains appropriate.

### Factual Accuracy

Every claim must be verified again
### Coherent Pedagogical Structure

All chapters must follow the arc: Foundation → Application → Integration → Validation → Mastery.

### Intelligence Accumulation

Content must inherit and build upon all previously established intelligence. Chapters must never reset context.

### Anti-Convergence Variation

Teaching modalities must vary between consecutive chapters (e.g., Error Analysis, Socratic Dialogue, Reverse Specification).

### Minimal Sufficient Content

Only essential instructional content is permitted. Final sections such as “Summary” or “Key Takeaways” are explicitly prohibited.

---

## III. Quality and Enforcement

### Meta-Commentary Prohibition

Internal scaffolding (“AI is teaching you,” “Here is the framework used”) is prohibited. Instead, prompts must rely on Active Collaboration and Self-Reflection without revealing internal mechanics.

### Agent Coordination

All agent handoffs must maintain reasoning continuity. Major decisions must be justified through the underlying framework without referencing system internals.

### Success Definition

Success is defined by:

1. Zero violations of specifications or mandates.
2. Demonstrated student comprehension at 80% or higher.
3. Verified composition of reusable intelligence by the learner.
```

## Response snapshot

The project constitution has been updated to version 1.0.0, incorporating principles, pedagogical strategy, quality, and governance rules as provided by the user. The `plan-template.md` file was also updated to include a `Constitution Check` section with the new principles. Manual follow-up is required to address an unknown ratification date and to ensure full alignment of other templates.

## Outcome

- ✅ Impact: Constitution and related planning template updated to reflect new project principles.
- 🧪 Tests: No automated tests were run.
- 📁 Files: .specify/memory/constitution.md, .specify/templates/plan-template.md
- 🔁 Next prompts: Review remaining templates for full alignment.
- 🧠 Reflection: Successfully updated the constitution and a related template. Path handling for PHR creation needs to be more robust for cross-platform compatibility.

## Evaluation notes (flywheel)

- Failure modes observed: Initial PHR file read failed due to path formatting.
- Graders run and results (PASS/FAIL): Not applicable.
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Improve PHR script to return platform-agnostic paths or for the agent to use `pathlib` for path construction.
