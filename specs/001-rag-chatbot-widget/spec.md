# Feature Specification: RAG Chatbot Widget for Docusaurus

**Feature Branch**: `001-rag-chatbot-widget`
**Created**: 2025-12-22
**Status**: Draft
**Input**: User description: "Docusaurus RAG chatbot integration with floating button, slide-in panel, SSE streaming, text selection context, and source citations"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Questions About Book Content (Priority: P1)

A reader is browsing the documentation and wants to ask a question about the content they're reading. They click the floating chat button, type their question, and receive an AI-generated answer with relevant source citations from the book.

**Why this priority**: This is the core value proposition of the chatbot - helping readers get instant answers to their questions about the documentation content without having to search manually.

**Independent Test**: Can be fully tested by loading any documentation page, clicking the chat button, typing "What is URDF?", and verifying that a streaming response appears with source citations and confidence badges.

**Acceptance Scenarios**:

1. **Given** a user is on any documentation page, **When** they click the floating chat button in the bottom-right corner, **Then** a slide-in chat panel opens from the right side without blocking the main content
2. **Given** the chat panel is open, **When** the user types a question and clicks send, **Then** the question appears in the chat history and a streaming response begins to appear with source citations
3. **Given** the AI is generating a response, **When** the response completes, **Then** a confidence badge (green/yellow/red) appears and the user can see all source citations with clickable links
4. **Given** the user has asked a question, **When** they click on a source citation, **Then** they are navigated to the relevant page in the documentation

---

### User Story 2 - Ask About Selected Text (Priority: P2)

A reader selects text from the documentation and wants to ask questions specifically about that selected content. They select text, click a contextual button or icon, and the chatbot opens with the selected text as context for a more relevant answer.

**Why this priority**: This enhances the user experience by providing context-aware assistance, allowing users to get specific clarifications about confusing passages or technical terms they're reading.

**Independent Test**: Can be fully tested by selecting any text on a documentation page, clicking the "Ask about this" button/icon that appears, and verifying that the chatbot opens with the selected text pre-loaded and provides contextually relevant answers.

**Acceptance Scenarios**:

1. **Given** a user is reading documentation, **When** they select text on the page, **Then** a contextual "Ask about this" button or tooltip appears near the selection
2. **Given** text is selected, **When** the user clicks the "Ask about this" button, **Then** the chat panel opens and sends the selected text as context along with a prompt asking for clarification
3. **Given** the chatbot receives selected text context, **When** the AI generates a response, **Then** the response is more focused and relevant to the selected content

---

### User Story 3 - Handle Typos and Suggestions (Priority: P3)

A reader makes a typo when asking a question (e.g., types "UDF" instead of "URDF"). The chatbot detects the typo, shows a "Did you mean..." suggestion, and allows the user to quickly search for the corrected term.

**Why this priority**: This improves user experience by being forgiving of typos and reducing frustration, though it's not essential for the core functionality.

**Independent Test**: Can be fully tested by typing a misspelled query like "What is UDF?" and verifying that the chatbot displays a suggestion prompt with the corrected term and a button to search again.

**Acceptance Scenarios**:

1. **Given** a user types a query with a typo, **When** the backend detects no results and fuzzy search finds a match, **Then** a suggestion event is displayed showing "Did you mean: [corrected term]"
2. **Given** a suggestion is displayed, **When** the user clicks on the suggested term, **Then** a new query is automatically sent with the corrected term

---

### User Story 4 - View Real-Time Streaming Responses (Priority: P1)

A reader asks a question and wants to see the answer appear in real-time as the AI generates it, rather than waiting for the complete response. They see text streaming in word by word, giving immediate feedback that their query is being processed.

**Why this priority**: Real-time streaming provides better user experience and perceived performance, making the chatbot feel more responsive and engaging. This is critical for user satisfaction.

**Independent Test**: Can be fully tested by sending any query and observing that the response text appears incrementally in real-time chunks rather than all at once after generation completes.

**Acceptance Scenarios**:

1. **Given** a user sends a query, **When** the backend begins generating a response, **Then** text chunks appear in the chat panel incrementally as they are received via SSE
2. **Given** streaming is in progress, **When** new content events arrive, **Then** each chunk is appended to the existing message without flickering or re-rendering the entire message

---

### User Story 5 - Maintain Chat History (Priority: P2)

A reader wants to review their previous questions and answers during their current browsing session. They can scroll through the chat history to see all their past queries and the AI's responses.

**Why this priority**: This allows users to reference previous answers and build on their learning, creating a more cohesive educational experience.

**Independent Test**: Can be fully tested by asking multiple questions in sequence and verifying that all questions and answers remain visible in the chat panel when scrolling up.

**Acceptance Scenarios**:

1. **Given** a user has asked multiple questions, **When** they scroll up in the chat panel, **Then** all previous questions and answers are visible in chronological order
2. **Given** the chat panel is closed, **When** the user reopens it during the same session, **Then** the chat history is preserved and still visible

---

### Edge Cases

- What happens when the user asks a question with no relevant results in the documentation?
  - Expected: The chatbot responds with "No relevant information found in the documentation" and shows a low confidence badge
- What happens when the backend API is unavailable or returns an error?
  - Expected: The chatbot displays an error message explaining the issue and suggests trying again later
- What happens when the user selects text and tries to ask about it but closes the chat panel before sending?
  - Expected: The selected text context is cleared when the panel closes
- What happens when the user asks a very long question (>2000 characters)?
  - Expected: Client-side validation prevents sending and shows an error message indicating the character limit
- What happens when the user selects a large amount of text (>5000 characters)?
  - Expected: The selected text is truncated to 5000 characters with a warning message displayed
- What happens when the chat panel is open and the user navigates to a different page?
  - Expected: The chat history is preserved across page navigation during the same session
- What happens when the user receives a response with no source citations?
  - Expected: The response is displayed without a sources section, but still shows the confidence badge
- What happens when the user tries to interact with the page while the chat panel is open?
  - Expected: The page remains fully interactive; the chat panel doesn't block or interfere with page content

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display a floating chat button/icon in the bottom-right corner of all documentation pages
- **FR-002**: System MUST open a slide-in chat panel from the right side when the chat button is clicked
- **FR-003**: System MUST allow users to type questions in a text input field with a send button
- **FR-004**: System MUST send user queries to the backend API at `POST /chat` with JSON payload containing `query` and optional `selected_text`
- **FR-005**: System MUST handle Server-Sent Events (SSE) streaming responses from the backend
- **FR-006**: System MUST display source citations as they are received via SSE `source` events, showing excerpt text, source URL/filename, page title, section, and similarity score
- **FR-007**: System MUST append text chunks to the AI response as they are received via SSE `content` events
- **FR-008**: System MUST display "Did you mean..." suggestions when SSE `suggestion` events are received
- **FR-009**: System MUST display confidence badges (green for high, yellow for medium, red for low) when SSE `done` events are received
- **FR-010**: System MUST display error messages when SSE `error` events are received
- **FR-011**: System MUST allow users to select text on documentation pages
- **FR-012**: System MUST show a contextual button or icon when text is selected
- **FR-013**: System MUST send the selected text as `selected_text` parameter when the user asks a question about selected content
- **FR-014**: System MUST validate that user queries are between 1-2000 characters before sending to backend
- **FR-015**: System MUST validate that selected text is maximum 5000 characters before sending to backend
- **FR-016**: System MUST maintain chat history (questions and answers) within the current browsing session
- **FR-017**: System MUST allow users to close the chat panel by clicking a close button or clicking outside the panel
- **FR-018**: System MUST make source citations clickable, navigating to the referenced documentation page/section when clicked
- **FR-019**: System MUST display a loading indicator while waiting for the first response chunk
- **FR-020**: System MUST preserve chat history when users navigate between documentation pages during the same session
- **FR-021**: System MUST display messages in a scrollable container with automatic scroll to bottom when new messages arrive
- **FR-022**: System MUST prevent sending new queries while a streaming response is in progress
- **FR-023**: System MUST style the chat UI to match the Vectra branding and Docusaurus theme (light/dark mode support)

### Key Entities

- **Chat Message**: Represents a single message in the chat history. Contains role (user or assistant), content text, optional source citations array, optional confidence level, and timestamp.
- **Source Citation**: Represents a reference to documentation content. Contains excerpt text (first 200 chars), source URL or filename, optional page title, optional section, and similarity score (0.0 to 1.0).
- **SSE Event**: Represents a server-sent event from the backend. Contains type (source/content/suggestion/done/error), type-specific data (text, source object, suggestion string, etc.), and timestamp.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can successfully ask questions and receive streaming responses in under 3 seconds for the first response chunk
- **SC-002**: The chat panel opens and closes with smooth animation in under 300 milliseconds
- **SC-003**: 95% of queries with relevant content in the documentation return at least one source citation
- **SC-004**: The chatbot UI remains responsive and usable on mobile devices (screens 375px width and above)
- **SC-005**: The chatbot supports at least 20 messages in the chat history without performance degradation
- **SC-006**: Text selection feature works across all modern browsers (Chrome, Firefox, Safari, Edge)
- **SC-007**: Users can complete the workflow of selecting text, asking a question, and receiving a response in under 10 seconds
- **SC-008**: The chat panel does not block or interfere with the main documentation content when open
- **SC-009**: Chat history persists across page navigation for at least 20 page views within the same session
- **SC-010**: Error states are clearly communicated to users with actionable next steps (e.g., "Try again" button)
