# API Contract: SSE Events

**Feature**: 001-rag-chatbot-widget
**Date**: 2025-12-22
**Authority**: FRONTEND_API.md (root directory)
**Purpose**: Document Server-Sent Events contract between frontend and backend

---

## Base URL

```
http://localhost:8000
```

**Note**: In production, this will be replaced with the actual deployment URL via environment variable or configuration.

---

## Endpoint

### POST /chat

Sends a query to the RAG chatbot and receives streaming response via SSE.

**Request:**

```http
POST http://localhost:8000/chat HTTP/1.1
Content-Type: application/json

{
  "query": string,              // Required, 1-2000 characters
  "selected_text": string | null // Optional, max 5000 characters
}
```

**Request Schema:**

```typescript
interface ChatRequest {
  query: string;              // User's question
  selected_text?: string;     // Optional context from text selection
}
```

**Client-Side Validation:**
- `query`: Must be 1-2000 characters (validate before sending)
- `selected_text`: If provided, max 5000 characters (truncate if needed)

**Response:**

```http
HTTP/1.1 200 OK
Content-Type: text/event-stream
Cache-Control: no-cache
Connection: keep-alive

data: {"type":"source","source":{...},"timestamp":"..."}

data: {"type":"content","text":"...","timestamp":"..."}

data: {"type":"done","text":"high","timestamp":"..."}
```

---

## SSE Event Types

### 1. Source Event

**Purpose**: Provide citation sources for the AI response

**Timing**: Sent first, before content streaming begins (0-5 events)

**Format**:
```json
{
  "type": "source",
  "source": {
    "text": "Excerpt from documentation (first 200 chars)",
    "source": "https://example.com/docs/page OR /docs/relative-path",
    "page_title": "Page Title" | null,
    "section": "Section Name" | null,
    "score": 0.87
  },
  "timestamp": "2025-12-22T14:30:00.123Z"
}
```

**TypeScript Interface**:
```typescript
interface SSESourceEvent {
  type: 'source';
  source: {
    text: string;        // Max 200 characters
    source: string;      // URL or filename
    page_title: string | null;
    section: string | null;
    score: number;       // 0.0 to 1.0
  };
  timestamp: string;     // ISO 8601
}
```

**Frontend Handling**:
- Collect all source events before displaying
- Render as clickable citation cards
- Display score as percentage (e.g., 0.87 → "87%")
- Link to source URL when clicked

**Example**:
```json
{
  "type": "source",
  "source": {
    "text": "URDF (Unified Robot Description Format) is an XML format for representing a robot model. It defines the kinematic and dynamic properties...",
    "source": "/docs/ros/urdf-basics",
    "page_title": "ROS URDF Documentation",
    "section": "Introduction",
    "score": 0.87
  },
  "timestamp": "2025-12-22T14:30:00.123Z"
}
```

---

### 2. Content Event

**Purpose**: Stream AI-generated response text in real-time

**Timing**: Sent continuously after sources, until response completes

**Format**:
```json
{
  "type": "content",
  "text": "chunk of text",
  "timestamp": "2025-12-22T14:30:01.456Z"
}
```

**TypeScript Interface**:
```typescript
interface SSEContentEvent {
  type: 'content';
  text: string;        // Text chunk to append
  timestamp: string;   // ISO 8601
}
```

**Frontend Handling**:
- Append each chunk to the current assistant message
- Update UI incrementally (streaming effect)
- Avoid flickering by appending, not replacing

**Example Sequence**:
```json
{"type":"content","text":"URDF stands for ","timestamp":"..."}
{"type":"content","text":"Unified Robot Description Format. ","timestamp":"..."}
{"type":"content","text":"It is an XML format used to ","timestamp":"..."}
{"type":"content","text":"describe robot models in ROS.","timestamp":"..."}
```

---

### 3. Suggestion Event

**Purpose**: Suggest corrected query when typo detected and no results found

**Timing**: Sent instead of content events when fuzzy search finds correction

**Format**:
```json
{
  "type": "suggestion",
  "text": "Did you mean: urdf",
  "suggestion": "urdf",
  "timestamp": "2025-12-22T14:30:02.789Z"
}
```

**TypeScript Interface**:
```typescript
interface SSESuggestionEvent {
  type: 'suggestion';
  text: string;         // Display message for user
  suggestion: string;   // Corrected query string
  timestamp: string;    // ISO 8601
}
```

**Frontend Handling**:
- Display suggestion prompt with clickable corrected term
- When user clicks suggestion, send new query automatically
- Clear suggestion after user interaction

**Example UI**:
```
⚠️ No results found for "UDF"
Did you mean: urdf
[Search for "urdf" instead]
```

---

### 4. Done Event

**Purpose**: Signal end of streaming and provide confidence level

**Timing**: Sent last, after all content chunks

**Format**:
```json
{
  "type": "done",
  "text": "high",
  "timestamp": "2025-12-22T14:30:03.012Z"
}
```

**TypeScript Interface**:
```typescript
interface SSEDoneEvent {
  type: 'done';
  text: 'high' | 'medium' | 'low';  // Confidence level
  timestamp: string;                 // ISO 8601
}
```

**Frontend Handling**:
- Mark message as complete (stop streaming indicator)
- Display confidence badge:
  - `high` → 🟢 Green badge
  - `medium` → 🟡 Yellow badge
  - `low` → 🔴 Red badge
- Close SSE connection

**Confidence Badge Examples**:
```tsx
{confidence === 'high' && <span className="text-green-600">🟢 High Confidence</span>}
{confidence === 'medium' && <span className="text-yellow-600">🟡 Medium Confidence</span>}
{confidence === 'low' && <span className="text-red-600">🔴 Low Confidence</span>}
```

---

### 5. Error Event

**Purpose**: Report errors during query processing

**Timing**: Sent if an error occurs at any point

**Format**:
```json
{
  "type": "error",
  "text": "Error message describing what went wrong",
  "timestamp": "2025-12-22T14:30:04.345Z"
}
```

**TypeScript Interface**:
```typescript
interface SSEErrorEvent {
  type: 'error';
  text: string;        // Error description
  timestamp: string;   // ISO 8601
}
```

**Frontend Handling**:
- Display error message to user
- Provide "Try again" button
- Log error for debugging
- Close SSE connection

**Example Errors**:
- "Backend service unavailable"
- "Query too long (max 2000 characters)"
- "Invalid request format"

---

## Event Flow

### Successful Query Flow

```
1. POST /chat { query: "What is URDF?" }
   ↓
2. SSE Connection Established
   ↓
3. source event (citation 1)
   ↓
4. source event (citation 2)
   ↓
5. source event (citation 3)
   ↓
6. content event ("URDF stands for ")
   ↓
7. content event ("Unified Robot...")
   ↓
8. content event ("It is used...")
   ↓
9. done event { text: "high" }
   ↓
10. Connection Closed
```

### Typo Correction Flow

```
1. POST /chat { query: "What is UDF?" }
   ↓
2. SSE Connection Established
   ↓
3. suggestion event { text: "Did you mean: urdf", suggestion: "urdf" }
   ↓
4. done event { text: "low" }
   ↓
5. Connection Closed
```

### Error Flow

```
1. POST /chat { query: "..." }
   ↓
2. SSE Connection Established
   ↓
3. error event { text: "Backend service unavailable" }
   ↓
4. Connection Closed
```

---

## Implementation Notes

### Parsing SSE Stream

```typescript
async function parseSS EStream(response: Response) {
  const reader = response.body?.getReader();
  const decoder = new TextDecoder();
  let buffer = '';

  while (true) {
    const { done, value } = await reader!.read();
    if (done) break;

    buffer += decoder.decode(value, { stream: true });
    const lines = buffer.split('\n\n');
    buffer = lines.pop() || '';

    for (const line of lines) {
      if (line.startsWith('data: ')) {
        const eventData = JSON.parse(line.slice(6));
        handleSSEEvent(eventData);
      }
    }
  }
}
```

### Event Handler

```typescript
function handleSSEEvent(event: ParsedSSEEvent) {
  switch (event.type) {
    case 'source':
      addSourceToMessage(event.source);
      break;
    case 'content':
      appendContentToMessage(event.text);
      break;
    case 'suggestion':
      showSuggestionPrompt(event.text, event.suggestion);
      break;
    case 'done':
      setMessageConfidence(event.text);
      setStreamingComplete();
      break;
    case 'error':
      displayError(event.text);
      setStreamingComplete();
      break;
  }
}
```

---

## Error Responses (Non-Streaming)

In some cases, the backend may return a non-streaming JSON error response:

```http
HTTP/1.1 400 Bad Request
Content-Type: application/json

{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Query too long (max 2000 characters)"
  }
}
```

**Frontend Handling**:
- Check `response.headers.get('content-type')`
- If not `text/event-stream`, parse as JSON error
- Display error message to user

---

## Testing

### Test with cURL

```bash
# Successful query
curl -N -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is URDF?"}'

# Typo correction
curl -N -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is UDF?"}'

# With selected text context
curl -N -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "Explain this", "selected_text": "URDF stands for Unified Robot Description Format"}'
```

---

## References

- **Authority Document**: `FRONTEND_API.md` (root directory)
- **SSE Specification**: https://html.spec.whatwg.org/multipage/server-sent-events.html
- **MDN SSE Guide**: https://developer.mozilla.org/en-US/docs/Web/API/Server-sent_events

---

**Contract Version**: 1.0
**Last Updated**: 2025-12-22
**Status**: Stable (matches existing backend implementation)
