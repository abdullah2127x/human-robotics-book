# RAG Chatbot - Frontend API Guide

## API Endpoint

**Base URL:** `http://localhost:8000`

**Endpoint:** `POST /chat`

---

## Request Format

```typescript
interface ChatRequest {
  query: string;              // Required, 1-2000 characters
  selected_text?: string;     // Optional, max 5000 characters
}
```

**Example:**
```json
{
  "query": "What is URDF?",
  "selected_text": null
}
```

---

## Response Format

**Type:** Server-Sent Events (SSE) stream

**Content-Type:** `text/event-stream`

---

## Event Types

### 1. Source Event
Received first, contains citation sources.

```typescript
{
  type: "source",
  source: {
    text: string,           // Excerpt (first 200 chars)
    source: string,         // URL or filename
    page_title: string | null,
    section: string | null,
    score: number           // 0.0 to 1.0 (similarity score)
  },
  timestamp: string         // ISO8601
}
```

### 2. Content Event
Streamed as AI generates response (append these chunks).

```typescript
{
  type: "content",
  text: string,             // Text chunk to append
  timestamp: string
}
```

### 3. Suggestion Event
Only sent when typo detected and no results found.

```typescript
{
  type: "suggestion",
  text: string,             // Display message
  suggestion: string,       // Corrected query
  timestamp: string
}
```

**Example:** `"Did you mean: urdf"`

### 4. Done Event
Final event, closes the stream.

```typescript
{
  type: "done",
  text: "high" | "medium" | "low",  // Confidence level
  timestamp: string
}
```

### 5. Error Event
Sent if an error occurs.

```typescript
{
  type: "error",
  text: string,             // Error message
  timestamp: string
}
```

---

## Implementation Example

```typescript
async function sendChatQuery(query: string, selectedText?: string) {
  const response = await fetch('http://localhost:8000/chat', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      query,
      selected_text: selectedText || null
    })
  });

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
        const event = JSON.parse(line.slice(6));

        switch (event.type) {
          case 'source':
            // Display source citation
            addSource(event.source);
            break;

          case 'content':
            // Append text chunk to response
            appendContent(event.text);
            break;

          case 'suggestion':
            // Show "Did you mean?" prompt
            showSuggestion(event.suggestion);
            break;

          case 'done':
            // Show confidence badge and finalize
            showConfidence(event.text); // "high" | "medium" | "low"
            return;

          case 'error':
            // Display error message
            showError(event.text);
            return;
        }
      }
    }
  }
}
```

---

## React Hook Example

```tsx
import { useState } from 'react';

export function useRagChat() {
  const [messages, setMessages] = useState([]);
  const [isStreaming, setIsStreaming] = useState(false);

  const sendMessage = async (query: string) => {
    setIsStreaming(true);
    setMessages(prev => [...prev, { role: 'user', content: query }]);

    const response = await fetch('http://localhost:8000/chat', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ query })
    });

    const reader = response.body?.getReader();
    const decoder = new TextDecoder();
    let buffer = '';
    let content = '';
    let sources = [];

    setMessages(prev => [...prev, { role: 'assistant', content: '', sources: [] }]);

    while (true) {
      const { done, value } = await reader!.read();
      if (done) break;

      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split('\n\n');
      buffer = lines.pop() || '';

      for (const line of lines) {
        if (line.startsWith('data: ')) {
          const event = JSON.parse(line.slice(6));

          if (event.type === 'source') {
            sources.push(event.source);
          } else if (event.type === 'content') {
            content += event.text;
            setMessages(prev => {
              const updated = [...prev];
              updated[updated.length - 1] = { role: 'assistant', content, sources };
              return updated;
            });
          } else if (event.type === 'done') {
            setIsStreaming(false);
            return;
          }
        }
      }
    }
  };

  return { messages, isStreaming, sendMessage };
}
```

---

## UI Recommendations

### Source Citations
Display each source as a card/chip:
- Show page title and score percentage
- Make it clickable to view full source
- Example: `📄 ROS Wiki - URDF (87%)`

### Confidence Badge
- `high` → 🟢 Green badge
- `medium` → 🟡 Yellow badge
- `low` → 🔴 Red badge

### Suggestion Prompt
When `type: "suggestion"` received:
```
⚠️ No results found for "UDF"
Did you mean: urdf
[Search for "urdf" instead]
```

---

## Error Handling

**Client-side validation before sending:**
- Query: 1-2000 characters
- Selected text: max 5000 characters

**Server error responses (non-streaming):**
```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Query too long (max 2000 characters)"
  }
}
```

---

## Testing

### Start backend:
```bash
uv run uvicorn src.main:app --reload --port 8000
```

### Test with cURL:
```bash
curl -N -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is URDF?"}'
```

### Test fuzzy search (typo):
```bash
curl -N -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is UDF?"}'
```

Expected: `type: "suggestion"` event with `suggestion: "urdf"`

---

## Event Flow Summary

```
1. Send POST request with query
2. Receive "source" events (0-5 sources)
3. Receive "content" events (streaming response)
   OR receive "suggestion" event (if typo + no results)
4. Receive "done" event (with confidence level)
```

**Note:** If no results found, you'll get a `suggestion` event OR a `content` event saying "no relevant information found", then immediately a `done` event. No need to wait for LLM generation.
