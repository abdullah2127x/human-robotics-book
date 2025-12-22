# Research & Technology Decisions: RAG Chatbot Widget

**Feature**: 001-rag-chatbot-widget
**Date**: 2025-12-22
**Purpose**: Document technology choices and rationale for implementation decisions

---

## 1. Docusaurus Theme Integration Patterns

### Decision: Root.tsx Swizzling

**Rationale**:
- Docusaurus provides official "swizzling" mechanism to customize theme components
- Root.tsx is a special wrapper component that renders on every page
- Allows adding global UI elements without modifying Docusaurus core
- Maintains upgrade compatibility with future Docusaurus versions

**Alternatives Considered**:

| Approach | Pros | Cons | Why Rejected |
|----------|------|------|--------------|
| Custom Plugin | Full control, reusable | Complex setup, overkill for single component | Too heavy for single widget feature |
| Layout Wrapper | Clean separation | Requires modifying all layouts | Invasive, maintenance burden |
| Direct Core Modification | Simple | Breaks upgrades, unmaintainable | Violates Docusaurus best practices |

**Implementation Pattern**:
```typescript
// book-source/src/theme/Root.tsx
import React from 'react';
import { RagChatbot } from '@site/src/components/RagChatbot';

export default function Root({children}) {
  return (
    <>
      {children}
      <RagChatbot />
    </>
  );
}
```

**References**:
- Docusaurus Swizzling Docs: https://docusaurus.io/docs/swizzling
- Root component pattern: https://docusaurus.io/docs/advanced/client#root

---

## 2. Server-Sent Events (SSE) in React

### Decision: Fetch API with ReadableStream

**Rationale**:
- Fetch API provides more control than EventSource
- Supports custom headers and request bodies (POST method required)
- EventSource limited to GET requests only
- ReadableStream allows chunk-by-chunk processing
- Native browser API, no external dependencies

**Alternatives Considered**:

| Approach | Pros | Cons | Why Rejected |
|----------|------|------|--------------|
| EventSource | Built-in SSE support | GET only, no custom headers | Backend requires POST with JSON body |
| WebSocket | Bidirectional | Overkill, backend uses SSE | Backend already implemented SSE, no need to change |
| Polling | Simple fallback | Inefficient, high latency | SSE provides real-time streaming |

**Implementation Pattern**:
```typescript
async function streamChatResponse(query: string, selectedText?: string) {
  const response = await fetch('http://localhost:8000/chat', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ query, selected_text: selectedText || null })
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
        // Process event...
      }
    }
  }
}
```

**References**:
- MDN Fetch Streams: https://developer.mozilla.org/en-US/docs/Web/API/Streams_API/Using_readable_streams
- SSE Format Spec: https://html.spec.whatwg.org/multipage/server-sent-events.html

---

## 3. Text Selection Detection

### Decision: `selectionchange` Event + `mouseup` Positioning

**Rationale**:
- `selectionchange` event fires whenever selection changes
- Debounced to avoid excessive processing
- `mouseup` event provides accurate cursor position for button placement
- Cross-browser compatible (Chrome, Firefox, Safari, Edge)
- No external libraries needed

**Alternatives Considered**:

| Approach | Pros | Cons | Why Rejected |
|----------|------|------|--------------|
| mouseup only | Simple | May miss keyboard selection | Users may select with keyboard (Shift+arrows) |
| Tooltip library | Pre-built UI | Heavy dependency | Violates minimal dependencies principle |
| Range API only | Precise | Complex positioning logic | Requires custom positioning anyway |

**Implementation Pattern**:
```typescript
useEffect(() => {
  const handleSelectionChange = () => {
    const selection = window.getSelection();
    const text = selection?.toString().trim();
    if (text && text.length > 0) {
      setSelectedText(text);
    } else {
      setSelectedText(null);
    }
  };

  const handleMouseUp = (e: MouseEvent) => {
    if (selectedText) {
      // Position button near cursor
      setButtonPosition({ x: e.clientX, y: e.clientY });
      setShowButton(true);
    }
  };

  document.addEventListener('selectionchange', debounce(handleSelectionChange, 150));
  document.addEventListener('mouseup', handleMouseUp);

  return () => {
    document.removeEventListener('selectionchange', handleSelectionChange);
    document.removeEventListener('mouseup', handleMouseUp);
  };
}, [selectedText]);
```

**References**:
- MDN Selection API: https://developer.mozilla.org/en-US/docs/Web/API/Selection
- selectionchange event: https://developer.mozilla.org/en-US/docs/Web/API/Document/selectionchange_event

---

## 4. SessionStorage vs LocalStorage for Chat History

### Decision: SessionStorage

**Rationale**:
- Specification requires "current browsing session" persistence (FR-020)
- SessionStorage automatically clears when tab closes (matches session scope)
- Prevents history pollution across different browsing sessions
- Privacy-friendly (no persistent tracking)
- Adequate for 20+ message history within session

**Alternatives Considered**:

| Approach | Pros | Cons | Why Rejected |
|----------|------|------|--------------|
| LocalStorage | Persistent across sessions | Violates session-scope requirement | Spec explicitly states "within browsing session" |
| IndexedDB | Large capacity, structured | Over-engineered, complex API | 20 messages don't need database |
| In-memory only | Simple | Lost on page refresh | Spec requires cross-page persistence (SC-009) |

**Implementation Pattern**:
```typescript
// Save to SessionStorage
const saveChatHistory = (messages: ChatMessage[]) => {
  try {
    sessionStorage.setItem('rag-chat-history', JSON.stringify(messages));
  } catch (error) {
    console.warn('Failed to save chat history:', error);
  }
};

// Load from SessionStorage
const loadChatHistory = (): ChatMessage[] => {
  try {
    const stored = sessionStorage.getItem('rag-chat-history');
    return stored ? JSON.parse(stored) : [];
  } catch (error) {
    console.warn('Failed to load chat history:', error);
    return [];
  }
};
```

**References**:
- MDN SessionStorage: https://developer.mozilla.org/en-US/docs/Web/API/Window/sessionStorage
- Storage quota limits: https://web.dev/storage-for-the-web/

---

## 5. Animation Performance

### Decision: Pure CSS Transitions

**Rationale**:
- CSS transitions hardware-accelerated by browsers (GPU)
- Lightweight, no JavaScript libraries needed
- Meets <300ms animation requirement (SC-002)
- Simple slide-in effect doesn't need physics-based animation
- Excellent browser support

**Alternatives Considered**:

| Approach | Pros | Cons | Why Rejected |
|----------|------|------|--------------|
| React Spring | Physics-based, smooth | 20KB+ bundle, complex API | Overkill for simple slide-in |
| Framer Motion | Declarative, powerful | 40KB+ bundle, heavy | Feature set exceeds needs |
| JavaScript RAF | Full control | Manual optimization needed | CSS handles this better |

**Implementation Pattern**:
```css
/* styles.module.css */
.chatPanel {
  position: fixed;
  right: 0;
  top: 0;
  height: 100vh;
  width: 400px;
  transform: translateX(100%);
  transition: transform 250ms ease-out;
  will-change: transform;
}

.chatPanel.open {
  transform: translateX(0);
}
```

**Performance Optimization**:
- `will-change: transform` hints browser to optimize
- `transform` over `left`/`right` for GPU acceleration
- `ease-out` timing function for natural deceleration

**References**:
- CSS Transitions Performance: https://web.dev/animations-guide/
- GPU Acceleration: https://www.html5rocks.com/en/tutorials/speed/high-performance-animations/

---

## 6. Tailwind Dark Mode Integration

### Decision: Class-Based with Docusaurus Theme Hook

**Rationale**:
- Docusaurus manages dark mode via `data-theme` attribute on `<html>`
- Tailwind's `dark:` prefix works with class-based dark mode
- `useColorMode` hook from Docusaurus provides reactive theme state
- Automatic synchronization with Docusaurus theme toggle
- No manual media query handling needed

**Alternatives Considered**:

| Approach | Pros | Cons | Why Rejected |
|----------|------|------|--------------|
| Media Query | Automatic OS sync | Ignores Docusaurus toggle | Users expect site toggle to work |
| Manual Class Toggle | Full control | Duplicate theme management | Docusaurus already handles this |
| CSS Variables | Flexible | Doesn't leverage Tailwind dark: | More boilerplate needed |

**Implementation Pattern**:
```typescript
// In component
import { useColorMode } from '@docusaurus/theme-common';

export function ChatWidget() {
  const { colorMode } = useColorMode();

  return (
    <div className={`${colorMode === 'dark' ? 'dark' : ''}`}>
      <div className="bg-white dark:bg-gray-900 text-gray-900 dark:text-gray-100">
        {/* Chat UI */}
      </div>
    </div>
  );
}
```

**Tailwind Configuration**:
```javascript
// tailwind.config.js
module.exports = {
  darkMode: 'class', // Enable class-based dark mode
  // ...
};
```

**References**:
- Docusaurus useColorMode: https://docusaurus.io/docs/api/themes/configuration#use-color-mode
- Tailwind Dark Mode: https://tailwindcss.com/docs/dark-mode

---

## Summary of Decisions

| Area | Decision | Key Reason |
|------|----------|------------|
| **Docusaurus Integration** | Root.tsx Swizzling | Official, maintainable, non-invasive |
| **SSE Streaming** | Fetch API with ReadableStream | POST support, full control, native |
| **Text Selection** | selectionchange + mouseup | Cross-browser, keyboard support, lightweight |
| **Storage** | SessionStorage | Matches session-scope requirement |
| **Animations** | Pure CSS Transitions | GPU-accelerated, lightweight, fast |
| **Dark Mode** | Class-based + useColorMode hook | Docusaurus integration, automatic sync |

**Architecture Benefits**:
- Zero external dependencies beyond existing project (React, Tailwind, Docusaurus)
- Lightweight implementation (<1500 LOC total)
- Performance-optimized (GPU acceleration, debouncing, streaming)
- Maintainable (follows Docusaurus best practices)
- Scalable (patterns support future enhancements)

**Next Phase**: Generate data-model.md with TypeScript interfaces based on these decisions.
