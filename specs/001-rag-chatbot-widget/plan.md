# Implementation Plan: RAG Chatbot Widget for Docusaurus

**Branch**: `001-rag-chatbot-widget` | **Date**: 2025-12-22 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot-widget/spec.md`

## Summary

Implement a RAG chatbot widget integrated into the Docusaurus documentation site with floating button, slide-in panel, Server-Sent Events (SSE) streaming, text selection context feature, and source citations. The chatbot connects to an existing backend API at `http://localhost:8000/chat` and provides real-time AI-generated answers to documentation questions with confidence indicators and clickable source references.

**Technical Approach**: React-based component system using Docusaurus theme swizzling, custom hooks for SSE streaming and text selection, SessionStorage for chat history persistence, and Tailwind CSS for styling with light/dark mode support.

## Technical Context

**Language/Version**: TypeScript 5.6.2 with React 19.0.0
**Primary Dependencies**:
- @docusaurus/core 3.9.2 (documentation framework)
- React 19.0.0 + React DOM (UI library)
- Tailwind CSS 4.1.18 (styling)
- Native Fetch API (SSE streaming, no external libraries)

**Storage**: SessionStorage (browser-native, persists chat history across page navigation within session)
**Testing**: TypeScript type checking (`tsc`), manual integration testing with live backend
**Target Platform**: Modern browsers (Chrome, Firefox, Safari, Edge) supporting ES2020+, mobile responsive (375px+)
**Project Type**: Web (frontend-only, integrates with existing Docusaurus site)
**Performance Goals**:
- First response chunk in <3 seconds
- Chat panel open/close animation <300ms
- No UI lag with 20+ messages in history
- SSE stream processing <50ms per chunk

**Constraints**:
- Must not block or interfere with main documentation content
- Must respect Docusaurus theme (light/dark mode)
- No build-time configuration changes to backend API
- Client-side validation only (query 1-2000 chars, selected_text max 5000 chars)

**Scale/Scope**:
- Single-page application component
- ~10 TypeScript files (~1500 lines total)
- 5 React components + 3 custom hooks
- Integration with existing Docusaurus site (no backend changes)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Since this project is focused on implementing a chatbot widget (not educational content), most constitutional principles about pedagogy don't directly apply. However, the following software development principles do apply:

### Applicable Principles:

1. **Specification Primacy** ✅
   - Specification (spec.md) created before implementation planning
   - All requirements documented with clear acceptance criteria
   - Success criteria defined with measurable metrics

2. **Factual Accuracy** ✅
   - All API integrations verified against FRONTEND_API.md
   - SSE event types documented from authoritative source
   - No hallucinated APIs or features

3. **Minimal Sufficient Content** ✅
   - Implementation focused on 23 functional requirements
   - No feature creep beyond specification
   - Scope clearly bounded to chatbot UI (no backend changes)

4. **Progressive Complexity** ✅
   - Implementation plan follows logical build order:
     1. Foundation: Core components and hooks
     2. Integration: SSE streaming and state management
     3. Enhancement: Text selection and history persistence
     4. Polish: Styling, animations, error handling

### Non-Applicable Principles:
- Layer 1-4 pedagogical progression (N/A - software implementation, not teaching content)
- Three Roles Framework (N/A - implementing tool, not collaborative learning)
- Anti-Convergence Variation (N/A - not chapter content)

**GATE STATUS**: ✅ PASS - Specification complete, requirements clear, implementation approach sound

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot-widget/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (technology decisions, patterns)
├── data-model.md        # Phase 1 output (TypeScript interfaces, state model)
├── quickstart.md        # Phase 1 output (developer setup guide)
├── contracts/           # Phase 1 output (API contract reference)
│   └── sse-events.md    # SSE event type definitions
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
book-source/
├── src/
│   ├── components/
│   │   └── RagChatbot/
│   │       ├── index.ts                    # Public exports
│   │       ├── ChatButton.tsx              # Floating button component
│   │       ├── ChatWidget.tsx              # Main panel container
│   │       ├── ChatMessage.tsx             # Message display component
│   │       ├── ChatInput.tsx               # Input field with send button
│   │       ├── SourceCitation.tsx          # Citation card component
│   │       ├── ConfidenceBadge.tsx         # High/medium/low badge
│   │       ├── SuggestionPrompt.tsx        # "Did you mean..." component
│   │       ├── TextSelectionButton.tsx     # "Ask about this" button
│   │       ├── hooks/
│   │       │   ├── useRagChat.ts           # SSE streaming + state management
│   │       │   ├── useTextSelection.ts     # Text selection detection
│   │       │   └── useChatHistory.ts       # SessionStorage persistence
│   │       ├── types.ts                    # TypeScript interfaces
│   │       └── styles.module.css           # Component-specific styles
│   └── theme/
│       └── Root.tsx                         # Docusaurus Root wrapper (global integration)
├── package.json
└── tsconfig.json
```

**Structure Decision**: Web application pattern selected. Frontend-only implementation integrating with existing Docusaurus site structure. Components follow atomic design (atoms: badges/buttons, molecules: message/input, organisms: widget/panel). Hooks encapsulate business logic (SSE, selection, history). Root.tsx provides global context without modifying Docusaurus core.

## Complexity Tracking

No constitutional violations detected. All gates passed without requiring complexity justification.

---

## Phase 0: Research & Technology Decisions

*Output: research.md*

### Research Tasks:

1. **Docusaurus Theme Integration Patterns**
   - Research: How to add global components to Docusaurus without modifying core
   - Decision needed: Root.tsx swizzling vs plugin approach
   - Alternatives: Custom theme, layout wrapper, plugin architecture

2. **Server-Sent Events (SSE) in React**
   - Research: Best practices for SSE streaming in React 19
   - Decision needed: Native EventSource vs Fetch API streaming
   - Alternatives: EventSource (limited), Fetch with ReadableStream (flexible)

3. **Text Selection Detection**
   - Research: Cross-browser text selection API patterns
   - Decision needed: Selection event handling + positioning strategy
   - Alternatives: `selectionchange` event, `mouseup` event, Tooltip libraries

4. **SessionStorage vs LocalStorage for Chat History**
   - Research: Persistence tradeoffs for chat history
   - Decision needed: Session-scoped vs persistent storage
   - Alternatives: SessionStorage (session-only), LocalStorage (persistent), IndexedDB (complex)

5. **Animation Performance**
   - Research: CSS animations vs JavaScript for slide-in panel
   - Decision needed: CSS transitions vs React Spring vs Framer Motion
   - Alternatives: Pure CSS (lightweight), React Spring (complex), Framer Motion (heavy)

6. **Tailwind Dark Mode Integration**
   - Research: How to respect Docusaurus dark mode with Tailwind
   - Decision needed: Dark mode class strategy
   - Alternatives: Class-based (`dark:` prefix), media query, Docusaurus theme hook

**Research Output**: `research.md` will document decisions with rationale for each area.

---

## Phase 1: Design & Contracts

*Output: data-model.md, contracts/sse-events.md, quickstart.md*

### Data Model Design

From spec entities, extract TypeScript interfaces:

**Key Entities:**
1. **ChatMessage**
   - Fields: id, role ('user' | 'assistant'), content, sources?, confidence?, timestamp
   - Relationships: Array of SourceCitation
   - State transitions: pending → streaming → complete

2. **SourceCitation**
   - Fields: text (excerpt), source (URL), pageTitle?, section?, score
   - Relationships: Belongs to ChatMessage
   - Validation: score 0.0-1.0, text max 200 chars

3. **SSEEvent**
   - Fields: type ('source' | 'content' | 'suggestion' | 'done' | 'error'), data, timestamp
   - Relationships: Processed into ChatMessage updates
   - State transitions: event → parser → state update

**State Management Model:**
- Chat state: messages array, isStreaming flag, error state
- Selection state: selectedText string, isSelectionActive boolean
- UI state: isPanelOpen boolean, inputValue string

### API Contracts

From FRONTEND_API.md, document SSE event contracts in `/contracts/sse-events.md`:

**Request Contract:**
```typescript
POST http://localhost:8000/chat
Content-Type: application/json

{
  query: string (1-2000 chars, required)
  selected_text?: string (max 5000 chars, optional)
}
```

**Response Contract:**
```typescript
Content-Type: text/event-stream

Event Types:
- source: { type, source: { text, source, page_title?, section?, score }, timestamp }
- content: { type, text, timestamp }
- suggestion: { type, text, suggestion, timestamp }
- done: { type, text: 'high' | 'medium' | 'low', timestamp }
- error: { type, text, timestamp }
```

### Developer Quickstart

`quickstart.md` will include:
1. Prerequisites (Node 20+, backend running on :8000)
2. Installation steps (no additional deps needed)
3. Development workflow (npm start, view chatbot)
4. Testing approach (type checking, manual testing)
5. Integration points (where to add to existing site)

### Agent Context Update

Run `.specify/scripts/bash/update-agent-context.sh claude` to update agent-specific context file with:
- Docusaurus 3.9.2 patterns
- React 19 hooks patterns
- SSE streaming patterns
- SessionStorage patterns
- Tailwind dark mode patterns

---

## Constitution Re-Check (Post-Design)

**After Phase 1 design, re-evaluate gates:**

1. **Specification Primacy** ✅
   - Data model derived from spec entities
   - API contracts reference authoritative FRONTEND_API.md
   - No implementation details added beyond specification

2. **Factual Accuracy** ✅
   - All API contracts verified against FRONTEND_API.md
   - SSE event types match backend documentation
   - No invented APIs or features

3. **Minimal Sufficient Content** ✅
   - Data model includes only entities needed for requirements
   - No speculative features added
   - Architecture focused on 23 functional requirements

4. **Progressive Complexity** ✅
   - Implementation plan follows logical phases:
     - Phase 0: Research decisions
     - Phase 1: Design data model + contracts
     - Phase 2: Tasks generation (next command)

**GATE STATUS**: ✅ PASS - Design phase complete, ready for task generation with `/sp.tasks`

---

## Next Steps

**This command (`/sp.plan`) stops here.** Phase 2 (task generation) is handled by `/sp.tasks` command.

**To proceed:**
```bash
/sp.tasks 001-rag-chatbot-widget
```

This will generate `tasks.md` with concrete implementation tasks broken down by component, including:
- Task dependencies and order
- Acceptance criteria per task
- Test cases for validation
- Integration checkpoints

**Artifacts Generated:**
- ✅ `plan.md` (this file)
- 🔄 `research.md` (next: Phase 0)
- 🔄 `data-model.md` (next: Phase 1)
- 🔄 `contracts/sse-events.md` (next: Phase 1)
- 🔄 `quickstart.md` (next: Phase 1)
- ⏭️ `tasks.md` (requires `/sp.tasks` command)

**Branch**: `001-rag-chatbot-widget` (ready for implementation)
**Spec**: `specs/001-rag-chatbot-widget/spec.md` (complete)
**Plan**: `specs/001-rag-chatbot-widget/plan.md` (this file)
