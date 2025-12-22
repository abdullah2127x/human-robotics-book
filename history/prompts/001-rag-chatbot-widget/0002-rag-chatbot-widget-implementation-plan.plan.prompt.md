---
id: 0002
title: RAG chatbot widget implementation plan
stage: plan
date: 2025-12-22
surface: agent
model: claude-sonnet-4-5
feature: 001-rag-chatbot-widget
branch: 001-rag-chatbot-widget
user: claude-agent
command: /sp.plan
labels: ["planning", "architecture", "rag-chatbot", "docusaurus", "react"]
links:
  spec: specs/001-rag-chatbot-widget/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot-widget/plan.md
  - specs/001-rag-chatbot-widget/research.md
  - specs/001-rag-chatbot-widget/data-model.md
  - specs/001-rag-chatbot-widget/quickstart.md
  - specs/001-rag-chatbot-widget/contracts/sse-events.md
tests:
  - N/A (planning phase, no tests yet)
---

## Prompt

/sp.plan

Context: Create implementation plan for the RAG chatbot widget feature based on the approved specification. The feature integrates a chatbot UI into Docusaurus with:
- Floating button and slide-in panel
- SSE streaming for real-time AI responses
- Text selection context feature
- Source citations with confidence badges
- Chat history persistence across pages (session-scoped)
- Integration with existing backend at http://localhost:8000/chat

Technical stack: TypeScript 5.6.2, React 19, Docusaurus 3.9.2, Tailwind CSS 4.1.18

## Response snapshot

Created comprehensive implementation plan with:

**Phase 0 - Research (research.md)**:
- 6 technology decisions documented with rationale
- Root.tsx swizzling for Docusaurus integration
- Fetch API with ReadableStream for SSE (not EventSource - POST required)
- selectionchange + mouseup events for text selection
- SessionStorage for session-scoped persistence
- Pure CSS transitions for GPU-accelerated animations
- Class-based dark mode with useColorMode hook

**Phase 1 - Design (data-model.md + contracts/)**:
- TypeScript interfaces: ChatMessage, SourceCitation, SSEEvent (discriminated union)
- State management model: ChatState, SelectionState, UIState
- API contract: SSE events (5 types - source, content, suggestion, done, error)
- Storage schema: SessionStorage structure with version field
- Message lifecycle: pending → streaming → complete

**Project Structure**:
- Components: 9 React components (ChatButton, ChatWidget, ChatMessage, etc.)
- Hooks: 3 custom hooks (useRagChat, useTextSelection, useChatHistory)
- Integration: Root.tsx swizzling (non-invasive Docusaurus pattern)
- Architecture: Atomic design (atoms/molecules/organisms)

**Constitution Check**: ✅ PASS
- Specification Primacy: Spec created first, plan derived from spec
- Factual Accuracy: All API contracts verified against FRONTEND_API.md
- Minimal Sufficient Content: 23 functional requirements, no feature creep
- Progressive Complexity: Logical build phases (research → design → implement)

**Developer Quickstart** (quickstart.md):
- Prerequisites and installation steps
- Development workflow with npm start
- File locations and key components
- Testing checklist (manual + type checking)
- Common issues and debugging tips

## Outcome

- ✅ Impact: Complete architecture ready for task generation and implementation
- 🧪 Tests: N/A (planning phase, TypeScript type checking only)
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, contracts/sse-events.md
- 🔁 Next prompts: /sp.tasks 001-rag-chatbot-widget (generate implementation tasks)
- 🧠 Reflection: Plan comprehensively addresses all 23 functional requirements with zero external dependencies beyond existing stack. Architecture follows Docusaurus best practices (swizzling) and React patterns (hooks, atomic design). All technology decisions justified with rationale and alternatives considered.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully with all design artifacts
- Graders run and results (PASS/FAIL): Constitution check PASS (4/4 applicable principles met)
- Prompt variant (if applicable): Standard /sp.plan workflow with Phase 0 (research) and Phase 1 (design)
- Next experiment (smallest change to try): N/A - proceed to task generation with /sp.tasks
