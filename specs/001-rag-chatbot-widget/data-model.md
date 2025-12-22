# Data Model: RAG Chatbot Widget

**Feature**: 001-rag-chatbot-widget
**Date**: 2025-12-22
**Purpose**: Define TypeScript interfaces and state management model

---

## TypeScript Interfaces

### Core Entities

#### ChatMessage

Represents a single message in the chat conversation.

```typescript
interface ChatMessage {
  id: string;                    // Unique identifier (UUID or timestamp-based)
  role: 'user' | 'assistant';    // Message sender
  content: string;                // Message text content
  sources?: SourceCitation[];     // Citations (assistant messages only)
  confidence?: 'high' | 'medium' | 'low';  // Confidence level (assistant messages only)
  timestamp: string;              // ISO 8601 timestamp
  isStreaming?: boolean;          // True while AI response is being streamed
}
```

**State Transitions**:
```
pending → streaming → complete
  |          |           |
  v          v           v
User sends → AI streams → AI finishes
             (isStreaming=true) (isStreaming=false, confidence set)
```

**Validation Rules**:
- `id`: Must be unique across all messages
- `role`: Must be either 'user' or 'assistant'
- `content`: Required, non-empty string
- `sources`: Optional, only present for assistant messages
- `confidence`: Optional, only present for completed assistant messages
- `timestamp`: ISO 8601 format (e.g., "2025-12-22T14:30:00Z")

---

#### SourceCitation

Represents a citation to documentation source material.

```typescript
interface SourceCitation {
  text: string;          // Excerpt from source (first 200 chars)
  source: string;        // URL or filename
  pageTitle?: string;    // Page title (if available)
  section?: string;      // Section within page (if available)
  score: number;         // Similarity score (0.0 to 1.0)
}
```

**Validation Rules**:
- `text`: Max 200 characters, non-empty
- `source`: Valid URL or filename
- `score`: Float between 0.0 and 1.0 inclusive
- `pageTitle` and `section`: Optional strings

**Display Formatting**:
```typescript
// Example citation display
const formatScore = (score: number) => `${Math.round(score * 100)}%`;
// e.g., 0.87 → "87%"

const formatCitation = (citation: SourceCitation) => {
  const scorePercent = formatScore(citation.score);
  const title = citation.pageTitle || citation.source;
  return `📄 ${title} (${scorePercent})`;
};
// e.g., "📄 ROS Wiki - URDF (87%)"
```

---

#### SSEEvent

Represents an incoming Server-Sent Event from the backend.

```typescript
interface SSEEvent {
  type: 'source' | 'content' | 'suggestion' | 'done' | 'error';
  timestamp: string;  // ISO 8601 timestamp
  // Type-specific data (discriminated union)
}

// Discriminated union for type-safe event handling
type SSESourceEvent = SSEEvent & {
  type: 'source';
  source: SourceCitation;
};

type SSEContentEvent = SSEEvent & {
  type: 'content';
  text: string;  // Text chunk to append
};

type SSESuggestionEvent = SSEEvent & {
  type: 'suggestion';
  text: string;        // Display message (e.g., "Did you mean: urdf")
  suggestion: string;  // Corrected query
};

type SSEDoneEvent = SSEEvent & {
  type: 'done';
  text: 'high' | 'medium' | 'low';  // Confidence level
};

type SSEErrorEvent = SSEEvent & {
  type: 'error';
  text: string;  // Error message
};

type ParsedSSEEvent =
  | SSESourceEvent
  | SSEContentEvent
  | SSESuggestionEvent
  | SSEDoneEvent
  | SSEErrorEvent;
```

**Event Processing Flow**:
```
Raw SSE → Parse → Discriminate Type → Update State
   |        |           |                  |
"data:..." JSON.parse  type='content'    append to message
```

---

### State Management Model

#### ChatState

Main application state for chat functionality.

```typescript
interface ChatState {
  messages: ChatMessage[];
  isStreaming: boolean;
  error: string | null;
  currentStreamingMessageId: string | null;
}

// Initial state
const initialChatState: ChatState = {
  messages: [],
  isStreaming: false,
  error: null,
  currentStreamingMessageId: null,
};
```

**State Actions**:
```typescript
type ChatAction =
  | { type: 'ADD_USER_MESSAGE'; payload: { content: string; selectedText?: string } }
  | { type: 'START_STREAMING'; payload: { messageId: string } }
  | { type: 'ADD_SOURCE'; payload: { messageId: string; source: SourceCitation } }
  | { type: 'APPEND_CONTENT'; payload: { messageId: string; text: string } }
  | { type: 'SET_CONFIDENCE'; payload: { messageId: string; confidence: 'high' | 'medium' | 'low' } }
  | { type: 'SET_ERROR'; payload: { error: string } }
  | { type: 'CLEAR_ERROR' }
  | { type: 'LOAD_HISTORY'; payload: { messages: ChatMessage[] } };
```

---

#### SelectionState

State for text selection feature.

```typescript
interface SelectionState {
  selectedText: string | null;
  isSelectionActive: boolean;
  buttonPosition: { x: number; y: number } | null;
}

const initialSelectionState: SelectionState = {
  selectedText: null,
  isSelectionActive: false,
  buttonPosition: null,
};
```

---

#### UIState

State for UI interactions.

```typescript
interface UIState {
  isPanelOpen: boolean;
  inputValue: string;
  showSuggestion: { text: string; suggestion: string } | null;
}

const initialUIState: UIState = {
  isPanelOpen: false,
  inputValue: '',
  showSuggestion: null,
};
```

---

## State Flow Diagrams

### Message Lifecycle

```
User types query → Send to backend → Receive SSE events → Update message state
     |                  |                   |                      |
inputValue         POST /chat          Parse events         messages array
     |                  |                   |                      |
     v                  v                   v                      v
"What is URDF?"   {query: "..."}    type='source'         Add citation
                                    type='content'         Append text
                                    type='done'            Set confidence
```

### Streaming Flow

```
1. User sends message
   → ADD_USER_MESSAGE: { content: "What is URDF?" }
   → messages = [{ id: '1', role: 'user', content: "What is URDF?", ... }]

2. Start streaming
   → START_STREAMING: { messageId: '2' }
   → messages = [..., { id: '2', role: 'assistant', content: '', isStreaming: true, ... }]

3. Receive sources (0-5 events)
   → ADD_SOURCE: { messageId: '2', source: {...} }
   → messages[1].sources = [citation1, citation2, ...]

4. Receive content chunks (streaming)
   → APPEND_CONTENT: { messageId: '2', text: 'URDF stands for...' }
   → messages[1].content += 'URDF stands for...'

5. Done event
   → SET_CONFIDENCE: { messageId: '2', confidence: 'high' }
   → messages[1].confidence = 'high'
   → messages[1].isStreaming = false
```

---

## Storage Schema

### SessionStorage Structure

```typescript
// Key: 'rag-chat-history'
// Value: JSON serialized ChatMessage[]

interface StoredHistory {
  version: '1.0';  // Schema version for future migrations
  messages: ChatMessage[];
  lastUpdated: string;  // ISO timestamp
}

// Example stored data
{
  "version": "1.0",
  "lastUpdated": "2025-12-22T14:35:00Z",
  "messages": [
    {
      "id": "msg-001",
      "role": "user",
      "content": "What is URDF?",
      "timestamp": "2025-12-22T14:30:00Z"
    },
    {
      "id": "msg-002",
      "role": "assistant",
      "content": "URDF (Unified Robot Description Format) is...",
      "sources": [
        {
          "text": "URDF is an XML format...",
          "source": "/docs/ros/urdf",
          "pageTitle": "ROS URDF Documentation",
          "score": 0.87
        }
      ],
      "confidence": "high",
      "timestamp": "2025-12-22T14:30:15Z"
    }
  ]
}
```

**Storage Limits**:
- SessionStorage typical limit: 5-10 MB per origin
- Average message size: ~500 bytes (text) + ~200 bytes per citation
- Target: 20 messages = ~20 KB (well within limits)

---

## Relationships

```
ChatState
  └─ messages: ChatMessage[]
       ├─ role: 'user' | 'assistant'
       ├─ content: string
       ├─ sources?: SourceCitation[]  (assistant only)
       │    ├─ text: string
       │    ├─ source: string
       │    ├─ pageTitle?: string
       │    ├─ section?: string
       │    └─ score: number
       ├─ confidence?: 'high' | 'medium' | 'low'  (assistant only)
       └─ timestamp: string

SelectionState
  ├─ selectedText: string | null
  ├─ isSelectionActive: boolean
  └─ buttonPosition: { x, y } | null

UIState
  ├─ isPanelOpen: boolean
  ├─ inputValue: string
  └─ showSuggestion: { text, suggestion } | null
```

---

## Type Exports

All interfaces will be exported from a central types file:

```typescript
// book-source/src/components/RagChatbot/types.ts

export type {
  ChatMessage,
  SourceCitation,
  SSEEvent,
  ParsedSSEEvent,
  SSESourceEvent,
  SSEContentEvent,
  SSESuggestionEvent,
  SSEDoneEvent,
  SSEErrorEvent,
  ChatState,
  ChatAction,
  SelectionState,
  UIState,
  StoredHistory,
};
```

---

## Next Steps

These interfaces will be implemented in the following hooks and components:

- **useRagChat**: Manages ChatState, handles SSE events
- **useTextSelection**: Manages SelectionState
- **useChatHistory**: Handles SessionStorage persistence with StoredHistory schema
- **ChatWidget**: Main container consuming all state
- **ChatMessage**: Renders individual messages with sources
- **SourceCitation**: Displays citation cards
- **ConfidenceBadge**: Displays confidence indicators

See `contracts/sse-events.md` for backend API contract details.
