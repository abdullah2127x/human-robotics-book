# Developer Quickstart: RAG Chatbot Widget

**Feature**: 001-rag-chatbot-widget
**Date**: 2025-12-22
**Purpose**: Get developers up and running with chatbot development

---

## Prerequisites

### Required Software

- **Node.js**: 20.0 or higher (check: `node --version`)
- **npm**: 10.0 or higher (check: `npm --version`)
- **Git**: For version control
- **Code Editor**: VS Code recommended with TypeScript/React extensions

### Backend Requirement

The RAG chatbot backend must be running:

```bash
# Backend should be accessible at:
http://localhost:8000

# Test backend is running:
curl http://localhost:8000/health
# Expected: {"status": "ok"} or similar
```

**If backend is not running**, see backend repository README for setup instructions.

---

## Installation

### 1. Clone and Navigate

```bash
cd physical-ai-and-humanoid-robotics/book-source
```

### 2. Install Dependencies

No additional dependencies needed! The chatbot uses existing packages:
- React 19.0.0 (already installed)
- Tailwind CSS 4.1.18 (already installed)
- TypeScript 5.6.2 (already installed)

```bash
# Just ensure dependencies are up to date:
npm install
```

### 3. Verify Installation

```bash
# Run type checking:
npm run typecheck

# Expected: No errors
```

---

## Development Workflow

### Start Development Server

```bash
npm start
```

**Expected Output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

### View Chatbot

1. Open browser to `http://localhost:3000`
2. Look for floating chat button in bottom-right corner
3. Click to open chat panel
4. Type a question and press send

---

## Project Structure

### File Locations

```
book-source/
├── src/
│   ├── components/
│   │   └── RagChatbot/               # Main chatbot component directory
│   │       ├── index.ts              # Public exports
│   │       ├── ChatButton.tsx        # Floating button
│   │       ├── ChatWidget.tsx        # Main panel
│   │       ├── ChatMessage.tsx       # Message display
│   │       ├── ChatInput.tsx         # Input field
│   │       ├── SourceCitation.tsx    # Citation cards
│   │       ├── ConfidenceBadge.tsx   # Confidence indicators
│   │       ├── SuggestionPrompt.tsx  # Typo suggestions
│   │       ├── TextSelectionButton.tsx  # "Ask about this" button
│   │       ├── hooks/
│   │       │   ├── useRagChat.ts     # SSE streaming logic
│   │       │   ├── useTextSelection.ts  # Text selection handler
│   │       │   └── useChatHistory.ts # SessionStorage persistence
│   │       ├── types.ts              # TypeScript interfaces
│   │       └── styles.module.css     # Component styles
│   └── theme/
│       └── Root.tsx                   # Docusaurus global wrapper
├── package.json
└── tsconfig.json
```

### Key Files to Know

| File | Purpose | Edit When |
|------|---------|-----------|
| `Root.tsx` | Global integration point | Adding/removing chatbot globally |
| `ChatWidget.tsx` | Main component container | Changing overall layout |
| `useRagChat.ts` | SSE streaming logic | Modifying API interaction |
| `types.ts` | TypeScript interfaces | Adding new data structures |
| `styles.module.css` | Component styles | Styling changes |

---

## Development Tasks

### Task 1: Add New Component

**Example: Add a typing indicator**

1. Create component file:
```bash
touch src/components/RagChatbot/TypingIndicator.tsx
```

2. Implement component:
```typescript
// src/components/RagChatbot/TypingIndicator.tsx
import React from 'react';

export function TypingIndicator() {
  return (
    <div className="flex space-x-2 p-3">
      <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce" />
      <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce delay-100" />
      <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce delay-200" />
    </div>
  );
}
```

3. Export from index:
```typescript
// src/components/RagChatbot/index.ts
export { TypingIndicator } from './TypingIndicator';
```

4. Use in ChatWidget:
```typescript
// src/components/RagChatbot/ChatWidget.tsx
import { TypingIndicator } from './TypingIndicator';

// Inside render:
{isStreaming && <TypingIndicator />}
```

### Task 2: Modify SSE Handling

**Example: Add logging for debugging**

```typescript
// src/components/RagChatbot/hooks/useRagChat.ts

function handleSSEEvent(event: ParsedSSEEvent) {
  // Add logging
  console.log('[SSE Event]', event.type, event);

  switch (event.type) {
    case 'source':
      // existing logic...
      break;
    // ...
  }
}
```

### Task 3: Customize Styling

**Example: Change chat panel width**

```css
/* src/components/RagChatbot/styles.module.css */

.chatPanel {
  width: 450px;  /* Changed from 400px */
}

/* Mobile responsive */
@media (max-width: 768px) {
  .chatPanel {
    width: 100%;  /* Full width on mobile */
  }
}
```

---

## Testing Approach

### Type Checking

```bash
# Run TypeScript compiler
npm run typecheck

# Fix type errors before proceeding
```

### Manual Testing Checklist

- [ ] Floating button appears in bottom-right
- [ ] Click button opens panel with slide animation
- [ ] Type question and send
- [ ] Response streams in real-time
- [ ] Source citations appear and are clickable
- [ ] Confidence badge displays correctly
- [ ] Close panel and reopen (history preserved)
- [ ] Navigate to different page (history persists)
- [ ] Select text and "Ask about this" button appears
- [ ] Dark mode toggle (chatbot follows theme)

### Test with Backend

**Successful Query**:
```
Query: "What is URDF?"
Expected:
- Sources appear first
- Response streams
- Confidence badge: High
```

**Typo Correction**:
```
Query: "What is UDF?"
Expected:
- Suggestion prompt: "Did you mean: urdf"
- Click suggestion → New query sent
```

**Error Handling**:
```
Stop backend (Ctrl+C)
Query: "Any question"
Expected:
- Error message displayed
- "Try again" button appears
```

---

## Integration Points

### Adding Chatbot to Site

The chatbot is globally integrated via `Root.tsx`:

```typescript
// src/theme/Root.tsx
import React from 'react';
import { RagChatbot } from '@site/src/components/RagChatbot';

export default function Root({children}) {
  return (
    <>
      {children}
      <RagChatbot />  {/* Renders on every page */}
    </>
  );
}
```

**To disable on specific pages:**

```typescript
import { useLocation } from '@docusaurus/router';

export default function Root({children}) {
  const location = useLocation();
  const showChatbot = !location.pathname.startsWith('/blog');

  return (
    <>
      {children}
      {showChatbot && <RagChatbot />}
    </>
  );
}
```

### Backend Configuration

If backend URL changes (e.g., production deployment):

```typescript
// src/components/RagChatbot/hooks/useRagChat.ts

// Option 1: Environment variable (recommended)
const BACKEND_URL = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';

// Option 2: Configuration file
import { config } from '../config';
const BACKEND_URL = config.backendUrl;

async function sendQuery(query: string, selectedText?: string) {
  const response = await fetch(`${BACKEND_URL}/chat`, {
    method: 'POST',
    // ...
  });
}
```

---

## Debugging

### Common Issues

#### Chatbot Not Appearing

**Check**:
1. Is Root.tsx correctly swizzled?
   ```bash
   ls src/theme/Root.tsx
   # Should exist
   ```
2. Is component exported?
   ```bash
   grep "export" src/components/RagChatbot/index.ts
   ```
3. Check browser console for errors

#### SSE Connection Fails

**Check**:
1. Backend running?
   ```bash
   curl http://localhost:8000/health
   ```
2. CORS enabled on backend?
3. Check Network tab in browser DevTools

#### Chat History Not Persisting

**Check**:
1. SessionStorage available?
   ```javascript
   console.log(typeof sessionStorage);  // Should be "object"
   ```
2. Check quota limits (unlikely with 20 messages)
3. Private browsing mode? (SessionStorage may be disabled)

### Browser DevTools

**Console Logging**:
```typescript
// Add strategic console.logs
console.log('[Chat] Sending query:', query);
console.log('[SSE] Received event:', event);
console.log('[History] Loaded messages:', messages.length);
```

**Network Tab**:
- Look for POST to `/chat`
- Check request payload
- Verify SSE stream (EventStream)

**React DevTools**:
- Inspect component state
- Track hook updates
- Debug re-renders

---

## Performance Optimization

### Monitoring

```typescript
// Measure SSE chunk processing time
const start = performance.now();
handleSSEEvent(event);
const duration = performance.now() - start;
console.log('[Performance] Event processed in', duration, 'ms');
// Target: <50ms per chunk
```

### Optimization Tips

1. **Debounce text selection** (already implemented)
2. **Throttle content appending** if streaming is too fast
3. **Virtualize message list** if >50 messages (unlikely)
4. **Memoize expensive computations**:
   ```typescript
   const formattedScore = useMemo(() =>
     Math.round(citation.score * 100),
     [citation.score]
   );
   ```

---

## Next Steps

After setup:

1. **Review Architecture**: Read `data-model.md` for state management
2. **Study API Contract**: Read `contracts/sse-events.md` for SSE details
3. **Implement Tasks**: See `tasks.md` (generated by `/sp.tasks` command)
4. **Run Tests**: Execute type checking and manual testing
5. **Deploy**: Build production bundle (`npm run build`)

---

## Resources

### Documentation
- **Docusaurus**: https://docusaurus.io/docs
- **React 19**: https://react.dev
- **Tailwind CSS**: https://tailwindcss.com/docs
- **TypeScript**: https://www.typescriptlang.org/docs

### Project Docs
- **Spec**: `specs/001-rag-chatbot-widget/spec.md`
- **Plan**: `specs/001-rag-chatbot-widget/plan.md`
- **Research**: `specs/001-rag-chatbot-widget/research.md`
- **Data Model**: `specs/001-rag-chatbot-widget/data-model.md`
- **API Contract**: `specs/001-rag-chatbot-widget/contracts/sse-events.md`

### Support
- Check GitHub issues for known problems
- Review FRONTEND_API.md for backend API documentation
- Ask team members for architecture questions

---

**Ready to start developing!** 🚀

Run `npm start` and open `http://localhost:3000` to see the chatbot in action.
