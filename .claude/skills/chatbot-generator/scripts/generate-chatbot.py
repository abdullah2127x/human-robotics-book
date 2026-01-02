#!/usr/bin/env python3
"""
Chatbot Generator Script

This script generates a complete chatbot component with all required documentation
and implementation files based on the user's selected platform (Docusaurus or React/Next.js).
"""

import os
import json
import yaml
from pathlib import Path
from typing import Dict, List, Any


class ChatbotGenerator:
    def __init__(self, platform: str):
        self.platform = platform.lower()
        self.project_root = Path.cwd()
        self.output_dir = self.project_root / "generated-chatbot"

    def analyze_project_themes(self) -> Dict[str, Any]:
        """Analyze the existing project to identify theme variables and styling patterns."""
        themes = {
            "css_custom_properties": {},
            "tailwind_config": {},
            "existing_colors": [],
            "typography": {}
        }

        # Look for global CSS files
        global_css_files = list(self.project_root.glob("**/globals.css")) + \
                          list(self.project_root.glob("**/global.css")) + \
                          list(self.project_root.glob("**/styles.css"))

        for css_file in global_css_files:
            content = css_file.read_text()
            # Extract CSS custom properties
            import re
            css_vars = re.findall(r'--[\w-]+:\s*[^;]+', content)
            for var in css_vars:
                name, value = var.split(':', 1)
                themes["css_custom_properties"][name.strip()] = value.strip()

        # Look for Tailwind config
        tailwind_configs = list(self.project_root.glob("**/tailwind.config.*"))
        for config_file in tailwind_configs:
            if config_file.suffix == '.js' or config_file.suffix == '.cjs':
                # For JS config, we'll just note its existence
                themes["tailwind_config"]["exists"] = True
                themes["tailwind_config"]["path"] = str(config_file)

        return themes

    def create_directory_structure(self):
        """Create the clean directory structure for the generated chatbot."""
        dirs = [
            self.output_dir / "docs",
            self.output_dir / "src" / "components" / "ChatWidget"
        ]

        for directory in dirs:
            directory.mkdir(parents=True, exist_ok=True)

    def generate_spec_document(self, themes: Dict[str, Any]):
        """Generate the specification document."""
        spec_content = f"""# ChatWidget Component Specification

**Version**: 1.0 (Generated)
**Date**: {self._get_current_date()}
**Platform**: {self.platform.title()}

## Problem Statement

The system addresses the need for an interactive chat interface that allows users to ask questions and receive contextual responses. It provides a floating chat widget that can be integrated into {self.platform} applications, with text selection functionality to ask contextual questions about selected content.

## System Intent

**Target Users**: Users of the {self.platform} application who need assistance or information

**Core Value Proposition**: Provide immediate, contextual assistance through an interactive chat interface with seamless text selection integration

**Key Capabilities**:
- Real-time chat with AI-powered responses
- Contextual question handling based on selected text
- Session management with persistent history
- Text selection integration for contextual queries
- Modern, responsive UI with proper theming

## Functional Requirements

### Requirement 1: Chat Interface Display
- **What**: Provide a floating chat widget that can be toggled open/closed
- **Why**: Allow users to access the chat interface without disrupting main content
- **Inputs**: User clicks on floating chat button
- **Outputs**: Chat interface appears/disappears with smooth animation
- **Side Effects**: Sets UI state for open/closed status
- **Success Criteria**: Widget appears in fixed position, responsive to toggle clicks

### Requirement 2: Session Management
- **What**: Initialize and maintain chat session with backend API
- **Why**: Preserve conversation context and enable history restoration
- **Inputs**: Session ID from backend API
- **Outputs**: Current session ID stored in component state
- **Side Effects**: Creates new session via API call, stores in component state
- **Success Criteria**: Session ID is properly set and maintained throughout chat session

### Requirement 3: Message Exchange
- **What**: Send user messages to backend and display responses
- **Why**: Enable two-way communication between user and AI
- **Inputs**: User text input in chat message field
- **Outputs**: User message and AI response displayed in chat history
- **Side Effects**: API call to send message, updates chat history
- **Success Criteria**: Messages are properly formatted and displayed in chronological order

### Requirement 4: History Persistence
- **What**: Store and retrieve chat history using localStorage
- **Why**: Maintain conversation context across page reloads
- **Inputs**: Chat messages array
- **Outputs**: Messages persisted in browser storage
- **Side Effects**: Updates localStorage with current message history
- **Success Criteria**: Messages are preserved and restored after page reload

### Requirement 5: Text Selection Integration
- **What**: Detect text selection and provide contextual query option
- **Why**: Allow users to ask questions about specific content on the page
- **Inputs**: User selects text on the page
- **Outputs**: Floating button appears near selection to ask about selected text
- **Side Effects**: Stores selected text in component state
- **Success Criteria**: Button appears when text is selected, disappears when deselected

### Requirement 6: New Chat Functionality
- **What**: Clear current conversation and start new session
- **Why**: Allow users to begin fresh conversations
- **Inputs**: User clicks "New Chat" button
- **Outputs**: Current chat history cleared, new session initiated
- **Side Effects**: Clears localStorage, makes API call for new session
- **Success Criteria**: Chat history is cleared and new session is started

## Non-Functional Requirements

### Performance
- Page load time under 3 seconds
- Message response time under 5 seconds
- Smooth animations and transitions (60fps)
- Efficient rendering of message history

### Security
- Input sanitization for user messages
- Secure API communication via HTTPS
- No sensitive data stored in localStorage

### Reliability
- Graceful error handling for API failures
- Fallback to localStorage when API unavailable
- Session recovery after network interruptions

### Scalability
- Single-page application architecture
- Client-side state management
- API-based backend communication

### Observability
- Console logging for API requests and responses
- Error reporting in chat interface
- Loading states for API calls

## System Constraints

### External Dependencies
- Backend API endpoint for chat functionality
- {self.platform.title()} framework
- React for component architecture
- {self._get_styling_system()} for styling

### Data Formats
- JSON for API requests and responses
- localStorage for client-side persistence
- TypeScript interfaces for type safety

### Deployment Context
- {self.platform.title()} application deployed as single-page application
- Client-side JavaScript execution required
- Modern browser with localStorage support

### Compliance Requirements
- Client-side data storage only
- No personal data collection beyond chat history

## Non-Goals & Out of Scope

**Explicitly excluded** (inferred from missing implementation):
- User authentication or accounts
- Server-side session persistence
- File attachment support
- Voice input/output
- Multi-language support
- Offline functionality beyond localStorage

## Known Gaps & Technical Debt

### Gap 1: Error Handling Consistency
- **Issue**: Inconsistent error handling between different API endpoints
- **Impact**: Users may see different error messages for similar failures
- **Recommendation**: Standardize error handling approach across components

### Gap 2: Accessibility Improvements
- **Issue**: Limited ARIA attributes and keyboard navigation
- **Impact**: Reduced accessibility for users with disabilities
- **Recommendation**: Add proper ARIA attributes and keyboard navigation support

## Success Criteria

### Functional Success
- [ ] Chat interface opens and closes with toggle button
- [ ] Messages are sent to backend and responses displayed correctly
- [ ] Chat history persists across page reloads
- [ ] Text selection integration works as expected
- [ ] New chat functionality clears history properly
- [ ] All API integrations work correctly

### Non-Functional Success
- [ ] Response time < 5 seconds for API calls
- [ ] System handles network errors gracefully
- [ ] {self._get_styling_system()} styling is consistent and readable
- [ ] All UI elements are responsive and accessible
- [ ] Zero critical security vulnerabilities

## Acceptance Tests

### Test 1: Chat Interface Toggle
**Given**: User is on the page with the chat widget
**When**: User clicks the floating chat button
**Then**: Chat interface opens with smooth animation

### Test 2: Message Exchange
**Given**: Chat interface is open and session is established
**When**: User types a message and clicks send
**Then**: Message appears in chat history, API request is made, response is displayed

### Test 3: Text Selection Integration
**Given**: User has selected text on the page
**When**: User selects text longer than 5 characters
**Then**: Floating "Ask about this" button appears near the selection

### Test 4: History Persistence
**Given**: User has sent multiple messages in the chat
**When**: User reloads the page
**Then**: Previous chat history is restored from localStorage
"""

        (self.output_dir / "docs" / "spec.md").write_text(spec_content)

    def generate_plan_document(self, themes: Dict[str, Any]):
        """Generate the implementation plan document."""
        plan_content = f"""# ChatWidget Component Implementation Plan

**Version**: 1.0 (Generated)
**Date**: {self._get_current_date()}
**Platform**: {self.platform.title()}

## Architecture Overview

**Architectural Style**: Client-Side React Application with External API Integration

**Reasoning**: The application follows a client-side architecture pattern where React components manage UI state and communicate with an external backend API. This approach enables fast user interactions while leveraging AI-powered backend services.

**Diagram** (ASCII):
```
    +-------------------+
    |   User Browser    |
    +-------------------+
              |
    +-------------------+
    | React Components  |
    | - ChatWidget      |
    | - Message Display |
    | - Input Handling  |
    +-------------------+
              |
    +-------------------+
    |   State Management|
    | - useState        |
    | - useEffect       |
    | - localStorage    |
    +-------------------+
              |
    +-------------------+
    |   API Interface   |
    | - Fetch API       |
    | - Backend Service |
    +-------------------+
              |
    +-------------------+
    |   Backend API     |
    | - Session Mgmt    |
    | - AI Processing   |
    | - Response Format |
    +-------------------+
```

## Layer Structure

### Layer 1: Presentation Layer
- **Responsibility**: Handle UI rendering, user interactions, and visual presentation
- **Components**:
  - [src/components/ChatWidget/ChatWidget.tsx]: Main chat interface with toggle functionality
  - [src/components/ChatWidget/index.ts]: Export configuration
- **Dependencies**: → API Layer, → State Management
- **Technology**: React, {self.platform.title()}

### Layer 2: State Management Layer
- **Responsibility**: Manage component state, session data, and local storage persistence
- **Components**:
  - React useState hooks for component state
  - localStorage for chat history persistence
  - useEffect hooks for side effects and data synchronization
- **Dependencies**: → Presentation Layer, → Browser Storage API
- **Technology**: React built-in hooks, Web Storage API

### Layer 3: API Communication Layer
- **Responsibility**: Handle API requests to backend services
- **Components**:
  - Fetch API calls for session creation and message exchange
  - Error handling for network requests
  - Request/response formatting
- **Dependencies**: → Backend API
- **Technology**: Web Fetch API, JSON

## Design Patterns Applied

### Pattern 1: Component State Management
- **Location**: [src/components/ChatWidget/ChatWidget.tsx]
- **Purpose**: Manage UI state, messages, loading states, and session IDs within components
- **Implementation**:
  - useState hooks for local component state
  - useEffect hooks for initialization and side effects
  - useRef for DOM references and scrolling

### Pattern 2: Session Management Pattern
- **Location**: [src/components/ChatWidget/ChatWidget.tsx]
- **Purpose**: Initialize chat session, maintain session ID, and fetch history
- **Implementation**:
  - API call to `/chat/start` on component mount
  - Store session ID in component state
  - Fetch history for the session from `/chat/history/{{sessionId}}`

### Pattern 3: Local Storage Fallback Pattern
- **Location**: [src/components/ChatWidget/ChatWidget.tsx]
- **Purpose**: Persist chat history in browser storage as fallback
- **Implementation**:
  - Save messages to localStorage on change
  - Load messages from localStorage on component mount

## Data Flow

### Chat Message Flow (Synchronous)
1. **Presentation Layer** detects user input in message field
2. **State Management** updates input value state
3. **User Action** triggers send button click or Enter key
4. **API Communication** makes POST request to `/chat/send/{{sessionId}}`
5. **Backend API** processes message and returns response
6. **State Management** updates messages with user and bot messages
7. **Presentation Layer** re-renders chat interface with new messages
8. **Persistence Layer** saves updated messages to localStorage

### Session Initialization Flow
1. **Component Mount** triggers useEffect hook
2. **API Communication** makes POST request to `/chat/start`
3. **Backend API** creates new session and returns session ID
4. **State Management** stores session ID in component state
5. **API Communication** fetches history for the session from `/chat/history/{{sessionId}}`
6. **State Management** updates messages with history or shows welcome message

### Text Selection Flow
1. **Browser Event** detects mouseup event after text selection
2. **Presentation Layer** checks for selected text (>5 characters)
3. **State Management** updates selection state with text and position
4. **Presentation Layer** renders floating button at selection position
5. **User Action** clicks floating button to ask about selection
6. **State Management** updates input field with selected text context

## Technology Stack

### Language & Runtime
- **Primary**: TypeScript
- **Rationale**: Type safety, better IDE support, catches errors at compile time

### Web Framework
- **Choice**: React with {self.platform.title()}
- **Rationale**: Component-based architecture, state management, and {self.platform.title()}-specific patterns

### Styling
- **Choice**: {self._get_styling_system()} with {self._get_theming_approach(themes)}
- **Rationale**: {self._get_styling_rationale()}

### State Management
- **Choice**: React built-in hooks (useState, useEffect, useRef)
- **Rationale**: Lightweight, built into React, sufficient for this application size

### Data Persistence
- **Choice**: localStorage API
- **Rationale**: Client-side persistence without backend requirements

### API Communication
- **Choice**: Web Fetch API
- **Rationale**: Modern browser standard for making HTTP requests

### Deployment
- **Choice**: {self.platform.title()} deployment options
- **Rationale**: Framework-native deployment options with good performance

## Module Breakdown

### Module: [chat-widget]
- **Purpose**: Main chat interface with toggle functionality, message display, and input handling
- **Key Classes**: ChatWidget React component
- **Dependencies**: React, localStorage, fetch API
- **Complexity**: Medium

### Module: [text-selection]
- **Purpose**: Text selection detection and contextual query functionality
- **Key Classes**: Selection event handlers in ChatWidget
- **Dependencies**: Browser selection API, DOM manipulation
- **Complexity**: Medium

### Module: [session-management]
- **Purpose**: Session initialization, maintenance, and history retrieval
- **Key Classes**: API call functions in ChatWidget
- **Dependencies**: Backend API, component state
- **Complexity**: Medium

### Module: [persistence]
- **Purpose**: Local storage management for chat history
- **Key Classes**: useEffect hooks for localStorage sync
- **Dependencies**: Web Storage API
- **Complexity**: Low

## Regeneration Strategy

### Option 1: Specification-First Rebuild
1. Start with spec.md (intent and requirements)
2. Apply extracted skills (API communication, state management)
3. Implement with modern best practices (fill gaps)
4. Test-driven development using acceptance criteria

**Timeline**: 1-2 weeks for team of 1-2 developers

### Option 2: Incremental Refactoring
1. **Strangler Pattern**: New implementation shadows old
2. **Feature Flags**: Gradual traffic shift
3. **Parallel Run**: Validate equivalence
4. **Cutover**: Complete migration

**Timeline**: 2-4 weeks depending on risk tolerance

## Improvement Opportunities

### Technical Improvements
- [ ] **Replace localStorage with IndexedDB** for better performance with large histories
  - **Rationale**: Better performance for larger datasets
  - **Effort**: Medium

- [ ] **Add TypeScript interfaces for API responses** to improve type safety
  - **Addresses Gap**: API response format handling consistency
  - **Effort**: Low

### Architectural Improvements
- [ ] **Introduce Context API** for global state management
  - **Enables**: Better state sharing across components
  - **Effort**: Medium

- [ ] **Implement Error Boundary pattern** for better error handling
  - **Separates**: Error handling from business logic
  - **Effort**: Low

### Operational Improvements
- [ ] **Add loading states and skeleton screens** for better UX
- [ ] **Implement retry logic with exponential backoff** for API calls
- [ ] **Add analytics tracking** for user interactions
- [ ] **Improve accessibility** with proper ARIA attributes
"""

        (self.output_dir / "docs" / "plan.md").write_text(plan_content)

    def generate_tasks_document(self, themes: Dict[str, Any]):
        """Generate the tasks breakdown document."""
        tasks_content = f"""# ChatWidget Component Implementation Tasks

**Version**: 1.0 (Generated)
**Date**: {self._get_current_date()}
**Platform**: {self.platform.title()}

## Overview

This task breakdown represents how to rebuild the chat widget system from scratch using the specification and plan.

**Estimated Timeline**: 2-3 weeks
**Team Size**: 1-2 developers

---

## Phase 1: Core Infrastructure

**Timeline**: Week 1
**Dependencies**: None

### Task 1.1: Project Setup
- [ ] Initialize {self.platform.title()} project with TypeScript
- [ ] Configure {self._get_styling_system()} with appropriate configuration
- [ ] Setup basic project structure ({self._get_project_structure()})
- [ ] Configure ESLint and TypeScript settings
- [ ] Create initial README.md with project overview

### Task 1.2: Configuration System
- [ ] Set up environment variables for API base URL
- [ ] Create type definitions for configuration
- [ ] Add validation for required environment variables
- [ ] Document configuration options

### Task 1.3: Basic Styling Setup
- [ ] Define CSS custom properties for theme colors (using existing project themes where available)
- [ ] Set up {self._get_styling_system()} configuration with theme variables
- [ ] Create base styles for the application
- [ ] Implement theming that respects existing project styles

---

## Phase 2: Component Architecture

**Timeline**: Week 1
**Dependencies**: Phase 1 complete

### Task 2.1: Create Message Interface
- [ ] Define TypeScript interface for Message objects
- [ ] Include properties: id, role, content, timestamp, citations, verification status
- [ ] Create interface for Citation objects
- [ ] Add type safety for user vs bot roles

### Task 2.2: Build Chat Widget Foundation
- [ ] Create main ChatWidget component with React hooks
- [ ] Implement useState for managing: isOpen, messages, inputValue, isLoading, sessionId
- [ ] Add useRef for messages container scrolling
- [ ] Create basic JSX structure for the chat interface

### Task 2.3: Implement UI State Management
- [ ] Add state for tracking chat open/close status
- [ ] Implement loading states for API requests
- [ ] Create state for tracking current session ID
- [ ] Add state for managing retry scenarios

---

## Phase 3: API Integration

**Timeline**: Week 1-2
**Dependencies**: Phase 2 complete

### Task 3.1: Session Management API
- [ ] Implement API call to initialize chat session (`/chat/start`)
- [ ] Handle session ID storage in component state
- [ ] Add error handling for session initialization failures
- [ ] Create function to fetch chat history

### Task 3.2: Message Exchange API
- [ ] Implement function to send messages to backend (`/chat/send/{{sessionId}}`)
- [ ] Handle request/response formatting
- [ ] Add proper error handling for API failures
- [ ] Implement loading state management during requests

### Task 3.3: API Response Processing
- [ ] Parse API responses and extract answer content
- [ ] Handle different response formats from backend
- [ ] Process citations and verification badges
- [ ] Format messages for display in chat interface

---

## Phase 4: UI/UX Implementation

**Timeline**: Week 2
**Dependencies**: Phase 3 complete

### Task 4.1: Chat Interface Layout
- [ ] Implement chat header with title and session status
- [ ] Create message display area with scrolling
- [ ] Build input area with message field and send button
- [ ] Add new chat and close buttons

### Task 4.2: Message Display Components
- [ ] Create message bubble styling for user vs bot messages
- [ ] Implement citation display with source IDs
- [ ] Add verification badges for trusted responses
- [ ] Create loading indicators for bot responses

### Task 4.3: Floating Action Button
- [ ] Create floating chat toggle button
- [ ] Implement smooth open/close animations
- [ ] Add proper positioning and z-index management
- [ ] Create SVG icons for open/close states

---

## Phase 5: Advanced Features

**Timeline**: Week 2
**Dependencies**: Phase 4 complete

### Task 5.1: Text Selection Integration
- [ ] Implement text selection detection using window.getSelection()
- [ ] Create floating button that appears near selected text
- [ ] Add functionality to prefill input with selected text context
- [ ] Handle selection positioning relative to viewport

### Task 5.2: Local Storage Persistence
- [ ] Implement chat history persistence using localStorage
- [ ] Save messages to storage when they change
- [ ] Load history from storage on component mount
- [ ] Handle storage quota limitations

### Task 5.3: Session Management Features
- [ ] Implement "New Chat" functionality to clear history
- [ ] Add session restart capability
- [ ] Create welcome message handling
- [ ] Implement history restoration after page reload

---

## Phase 6: Cross-Cutting Concerns

**Timeline**: Week 2
**Dependencies**: Phase 5 complete

### Task 6.1: Error Handling
- [ ] Create global error display for API failures
- [ ] Implement graceful degradation when API unavailable
- [ ] Add user-friendly error messages
- [ ] Create retry mechanism for failed requests

### Task 6.2: Accessibility
- [ ] Add proper ARIA attributes to interactive elements
- [ ] Implement keyboard navigation support
- [ ] Add focus management for accessibility
- [ ] Ensure sufficient color contrast

### Task 6.3: Performance Optimization
- [ ] Implement virtual scrolling for long message histories
- [ ] Optimize re-renders with React.memo where appropriate
- [ ] Add loading states for better UX
- [ ] Optimize CSS animations for smooth performance

---

## Phase 7: Testing & Quality

**Timeline**: Week 3
**Dependencies**: All phases complete

### Task 7.1: Unit Tests
- [ ] Test component rendering with different props
- [ ] Test state management logic
- [ ] Test API integration functions
- [ ] Test utility functions

### Task 7.2: Integration Tests
- [ ] Test complete message flow from input to display
- [ ] Test session initialization process
- [ ] Test localStorage persistence
- [ ] Test text selection functionality

### Task 7.3: End-to-End Tests
- [ ] Test complete user journey: open chat → send message → receive response
- [ ] Test session management: start new chat, clear history
- [ ] Test text selection feature
- [ ] Test error scenarios and recovery

---

## Phase 8: Deployment & Documentation

**Timeline**: Week 3
**Dependencies**: Phase 7 complete

### Task 8.1: Build & Optimization
- [ ] Configure {self.platform.title()} production build
- [ ] Optimize assets and code splitting
- [ ] Implement proper error boundaries
- [ ] Add production logging

### Task 8.2: Documentation
- [ ] Create component usage documentation
- [ ] Document API integration process
- [ ] Add deployment instructions
- [ ] Create troubleshooting guide

### Task 8.3: Final Testing
- [ ] Test production build locally
- [ ] Verify all functionality works in production build
- [ ] Check browser compatibility
- [ ] Performance testing of production build
"""

        (self.output_dir / "docs" / "tasks.md").write_text(tasks_content)

    def generate_implementation_files(self, themes: Dict[str, Any]):
        """Generate the actual implementation files."""
        # Generate ChatWidget.tsx
        chat_widget_content = self._generate_chat_widget_tsx(themes)
        (self.output_dir / "src" / "components" / "ChatWidget" / "ChatWidget.tsx").write_text(chat_widget_content)

        # Generate types.ts
        types_content = self._generate_types_ts()
        (self.output_dir / "src" / "components" / "ChatWidget" / "types.ts").write_text(types_content)

        # Generate utils.ts
        utils_content = self._generate_utils_ts()
        (self.output_dir / "src" / "components" / "ChatWidget" / "utils.ts").write_text(utils_content)

        # Generate index.ts
        index_content = self._generate_index_ts()
        (self.output_dir / "src" / "components" / "ChatWidget" / "index.ts").write_text(index_content)

        # Generate CSS file with theming
        css_content = self._generate_chat_widget_css(themes)
        (self.output_dir / "src" / "components" / "ChatWidget" / "ChatWidget.module.css").write_text(css_content)

    def _generate_chat_widget_tsx(self, themes: Dict[str, Any]) -> str:
        """Generate the ChatWidget.tsx file."""
        return f'''\'use client\';

import {{ useState, useEffect, useRef }} from 'react';
import {{ Message, Citation }} from './types';
import {{ getApiBaseUrl, sanitizeMessage }} from './utils';
import styles from './ChatWidget.module.css';

interface ChatWidgetProps {{
  activeSection?: string;
}}

const API_BASE = process.env.NEXT_PUBLIC_API_BASE || getApiBaseUrl();

export default function ChatWidget({{ activeSection = \'chat\' }}: ChatWidgetProps) {{
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [currentSessionId, setCurrentSessionId] = useState<string | null>(null);
  const [selection, setSelection] = useState<{{ text: string; x: number; y: number }} | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Initialize session and history on mount
  useEffect(() => {{
    const startSession = async () => {{
      try {{
        const response = await fetch(`${{API_BASE}}/chat/start`, {{
          method: \'POST\',
          headers: {{ \'Content-Type\': \'application/json\' }}
        }});
        if (response.ok) {{
          const data = await response.json();
          setCurrentSessionId(data.session_id);
          fetchHistory(data.session_id);
        }}
      }} catch (error) {{
        console.error(\'Failed to start chat session:\', error);
        // Show welcome message even if session start fails
        setMessages([{
          id: \'welcome\',
          role: \'bot\',
          content: \'Hi! I\\'m your assistant. How can I help you today?\',
          timestamp: Date.now()
        }]);
      }}
    }};

    const fetchHistory = async (sid: string) => {{
      try {{
        const response = await fetch(`${{API_BASE}}/chat/history/${{sid}}`);
        if (response.ok) {{
          const data = await response.json();
          if (Array.isArray(data) && data.length > 0) {{
            setMessages(data.map((m: any, idx: number) => ({{
              id: `hist-${{idx}}`,
              role: m.role === \'assistant\' ? \'bot\' : \'user\',
              content: m.content || m.answer || m.question,
              timestamp: Date.now(),
              citations: m.citations,
              isVerified: m.is_from_book
            })));
          }} else {{
            // Show welcome message if no history
            setMessages([{
              id: \'welcome\',
              role: \'bot\',
              content: \'Hi! I\\'m your assistant. How can I help you today?\',
              timestamp: Date.now()
            }]);
          }}
        }}
      }} catch (e) {{
        console.warn(\'Could not fetch remote history, using local if available\');
        // Show welcome message if history fetch fails
        setMessages([{
          id: \'welcome\',
          role: \'bot\',
          content: \'Hi! I\\'m your assistant. How can I help you today?\',
          timestamp: Date.now()
        }]);
      }}
    }};

    startSession();

    // Text selection handler
    const handleMouseUp = () => {{
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText && selectedText.length > 5) {{
        const range = window.getSelection()?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();
        if (rect) {{
          setSelection({{
            text: selectedText,
            x: rect.left + window.scrollX,
            y: rect.top + window.scrollY - 40
          }});
        }}
      }} else {{
        setSelection(null);
      }}
    }};

    document.addEventListener(\'mouseup\', handleMouseUp);
    return () => document.removeEventListener(\'mouseup\', handleMouseUp);
  }}, []);

  // Save history to localStorage as fallback
  useEffect(() => {{
    if (messages.length > 0) {{
      localStorage.setItem(\'chat_history\', JSON.stringify(messages));
    }}
  }}, [messages]);

  // Scroll to bottom
  useEffect(() => {{
    messagesEndRef.current?.scrollIntoView({{ behavior: \'smooth\' }});
  }}, [messages]);

  const handleSend = async (overrideText?: string) => {{
    const text = overrideText || inputValue;
    if (!text.trim() || isLoading) return;

    let sessionId = currentSessionId;
    if (!sessionId) return;

    const userMessage: Message = {{
      id: Date.now().toString(),
      role: \'user\',
      content: sanitizeMessage(text),
      timestamp: Date.now()
    }};

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {{
      console.log(\'Sending request to:\', `${{API_BASE}}/chat/send/${{sessionId}}`);
      console.log(\'Request body:\', {{ question: text }});

      const response = await fetch(`${{API_BASE}}/chat/send/${{sessionId}}`, {{
        method: \'POST\',
        headers: {{ \'Content-Type\': \'application/json\' }},
        body: JSON.stringify({{ question: text }})
      }});

      console.log(\'Response status:\', response.status);
      if (!response.ok) {{
        const errorText = await response.text();
        console.error(\'API Error:\', errorText);
        throw new Error(`HTTP error ${{response.status}}`);
      }}

      const data = await response.json();
      console.log(\'API Response:\', data);

      const botMessage: Message = {{
        id: (Date.now() + 1).toString(),
        role: \'bot\',
        content: data.answer || \'No response from AI.\',
        timestamp: Date.now(),
        citations: data.citations,
        isVerified: data.is_from_book
      }};
      setMessages(prev => [...prev, botMessage]);
    }} catch (error) {{
      console.error(\'Chat Error:\', error);
      setMessages(prev => [...prev, {{
        id: Date.now().toString(),
        role: \'bot\',
        content: \'Sorry, I hit a snag. Please check your connection.\',
        timestamp: Date.now()
      }}]);
    }} finally {{
      setIsLoading(false);
    }}
  }};

  const handleContextQuery = () => {{
    if (selection) {{
      const contextText = `Context: [${{selection.text}}]\\n\\nQuestion: `;
      setInputValue(contextText);
      setIsOpen(true);
      setSelection(null);
      // Focus input
      setTimeout(() => {{
        const input = document.querySelector(\'.chat-input\') as HTMLInputElement;
        input?.focus();
      }}, 300);
    }}
  }};

  const clearChat = async () => {{
    localStorage.removeItem(\'chat_history\');
    setMessages([]);
    const response = await fetch(`${{API_BASE}}/chat/start`, {{ method: \'POST\' }});
    const data = await response.json();
    setCurrentSessionId(data.session_id);
    setMessages([{{
      id: \'welcome\',
      role: \'bot\',
      content: \'History cleared. New session started!\',
      timestamp: Date.now()
    }}]);
  }};

  return (
    <>
      {{/* Floating Action Button for Selection */}}
      {{selection && (
        <button
          className={styles.contextBtn}
          style={{
            position: \'absolute\',
            left: selection.x,
            top: selection.y,
            zIndex: 2000,
          }}
          onMouseDown={(e) => {{
            e.preventDefault(); // Prevent losing selection
            handleContextQuery();
          }}
        >
          Ask about this ✨
        </button>
      )}}

      <button
        className={styles.chatToggleBtn}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
      >
        {{isOpen ? (
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" className={styles.icon}>
            <line x1="18" y1="6" x2="6" y2="18" />
            <line x1="6" y1="6" x2="18" y2="18" />
          </svg>
        ) : (
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" className={styles.icon}>
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        )}}
      </button>

      <div className={`${{styles.chatContainer}} ${{isOpen ? styles.open : \'\'}}`}>
        <div className={styles.chatHeader}>
          <div className={styles.chatHeaderTitle}>
            <div className={styles.chatAvatar}>AI</div>
            <div className={styles.chatHeaderText}>
              <h3>Assistant</h3>
              <span>{{currentSessionId ? \'Online\' : \'Connecting...\'}}</span>
            </div>
          </div>
          <div style={{ display: \'flex\', gap: \'8px\' }}>
            <button className={styles.chatNewBtn} onClick={clearChat} title="New Chat">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M21 12a9 9 0 1 1-9-9c2.52 0 4.93 1 6.74 2.74L21 8"></path>
                <polyline points="21 3 21 8 16 8"></polyline>
              </svg>
            </button>
            <button className={styles.chatCloseBtn} onClick={() => setIsOpen(false)}>✕</button>
          </div>
        </div>

        <div className={styles.chatMessages}>
          {{messages.map(msg => (
            <div key={{msg.id}} className={`${{styles.message}} ${{styles[msg.role]}}`}>
              <div className={styles.messageContent} style={{ whiteSpace: \'pre-wrap\' }}>
                {{msg.content}}
                {{msg.isVerified && <span className={styles.verifiedBadge}>✓</span>}}
              </div>
              {{msg.citations && msg.citations.length > 0 && (
                <div className={styles.citations}>
                  {{msg.citations.map((cit, idx) => (
                    <span key={{idx}} className={styles.citationTag}>[{{cit.source_id}}]</span>
                  ))}}
                </div>
              )}}
            </div>
          ))}}
          {{isLoading && (
            <div className={`${{styles.message}} ${{styles.bot}} ${{styles.loading}}`}>
              <span></span><span></span><span></span>
            </div>
          )}}
          <div ref={{messagesEndRef}} />
        </div>

        <div className={styles.chatInputContainer}>
          <input
            type="text"
            className={styles.chatInput}
            placeholder="Ask a question..."
            value={{inputValue}}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={(e) => e.key === \'Enter\' && handleSend()}
            disabled={{isLoading}}
          />
          <button
            className={styles.chatSendBtn}
            onClick={() => handleSend()}
            disabled={{isLoading || !inputValue.trim()}}
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5">
              <line x1="22" y1="2" x2="11" y2="13"></line>
              <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
            </svg>
          </button>
        </div>
      </div>
    </>
  );
}}
'''

    def _generate_types_ts(self) -> str:
        """Generate the types.ts file."""
        return '''// Message interface for chat messages
export interface Message {
  id: string;
  role: \'user\' | \'bot\';
  content: string;
  timestamp: number;
  citations?: Citation[];
  isVerified?: boolean;
}

// Citation interface for source references
export interface Citation {
  source_id: string;
  text?: string;
}

// ChatWidget component props
export interface ChatWidgetProps {
  activeSection?: string;
}
'''

    def _generate_utils_ts(self) -> str:
        """Generate the utils.ts file."""
        return '''// Utility functions for the ChatWidget component

/**
 * Get the API base URL from environment or default
 * @returns The API base URL
 */
export function getApiBaseUrl(): string {
  // Check for common environment variables
  if (typeof process !== \'undefined\' && process.env?.NEXT_PUBLIC_API_BASE) {
    return process.env.NEXT_PUBLIC_API_BASE;
  }

  // Default to a common pattern or localhost for development
  return typeof window !== \'undefined\'
    ? window.location.origin.replace(/\\/g, \'/\')
    : \'http://localhost:3000\';
}

/**
 * Sanitize user input to prevent XSS and other issues
 * @param text - The text to sanitize
 * @returns The sanitized text
 */
export function sanitizeMessage(text: string): string {
  // Remove potentially dangerous characters
  return text
    .replace(/</g, \'&lt;\')
    .replace(/>/g, \'&gt;\')
    .replace(/"/g, \'&quot;\')
    .replace(/\'/g, \'&#x27;\');
}

/**
 * Validate a session ID
 * @param sessionId - The session ID to validate
 * @returns True if valid, false otherwise
 */
export function isValidSessionId(sessionId: string): boolean {
  // Basic UUID validation
  const uuidRegex = /^[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i;
  return uuidRegex.test(sessionId);
}

/**
 * Format a timestamp for display
 * @param timestamp - The timestamp to format
 * @returns Formatted time string
 */
export function formatTime(timestamp: number): string {
  return new Date(timestamp).toLocaleTimeString([], {
    hour: \'2-digit\',
    minute: \'2-digit\'
  });
}
'''

    def _generate_index_ts(self) -> str:
        """Generate the index.ts file."""
        return '''// Export the ChatWidget component as default
export { default } from \'./ChatWidget\';
export * from \'./ChatWidget\';
export * from \'./types\';
'''

    def _generate_chat_widget_css(self, themes: Dict[str, Any]) -> str:
        """Generate the ChatWidget.module.css file with theming."""
        # Use existing theme variables if available, otherwise use defaults
        primary_color = themes["css_custom_properties"].get("--primary", "#3b82f6")
        bg_dark = themes["css_custom_properties"].get("--bg-dark", "#0f172a")
        bg_card = themes["css_custom_properties"].get("--bg-card", "#1e293b")
        text_primary = themes["css_custom_properties"].get("--text-primary", "#f1f5f9")

        return f'''/* Chat Widget Styles - Respecting existing project themes */

/* Main chat container */
.chatContainer {{
  position: fixed;
  bottom: 90px;
  right: 20px;
  width: 380px;
  max-width: calc(100vw - 40px);
  height: 600px;
  max-height: calc(100vh - 120px);
  background: {bg_card};
  border: 1px solid color-mix(in srgb, {text_primary} 20%, transparent);
  border-radius: 16px;
  display: none;
  flex-direction: column;
  overflow: hidden;
  box-shadow: 0 10px 40px rgba(0, 0, 0, 0.3);
  z-index: 1001;
}}

.chatContainer.open {{
  display: flex;
  animation: slideUp 0.3s ease;
}}

@keyframes slideUp {{
  from {{
    opacity: 0;
    transform: translateY(20px);
  }}
  to {{
    opacity: 1;
    transform: translateY(0);
  }}
}}

/* Chat header */
.chatHeader {{
  padding: 16px 20px;
  background: linear-gradient(135deg, {primary_color}, #a855f7);
  display: flex;
  align-items: center;
  justify-content: space-between;
}}

.chatHeaderTitle {{
  display: flex;
  align-items: center;
  gap: 10px;
}}

.chatAvatar {{
  width: 36px;
  height: 36px;
  background: rgba(255, 255, 255, 0.2);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.2rem;
}}

.chatHeaderText h3 {{
  font-size: 1rem;
  font-weight: 600;
  color: white;
  margin: 0;
}}

.chatHeaderText span {{
  font-size: 0.75rem;
  color: rgba(255, 255, 255, 0.8);
}}

.chatNewBtn,
.chatCloseBtn {{
  background: rgba(255, 255, 255, 0.2);
  border: none;
  color: white;
  width: 32px;
  height: 32px;
  border-radius: 8px;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: background 0.2s;
}}

.chatNewBtn:hover,
.chatCloseBtn:hover {{
  background: rgba(255, 255, 255, 0.3);
}}

/* Chat messages area */
.chatMessages {{
  flex: 1;
  overflow-y: auto;
  padding: 20px;
  display: flex;
  flex-direction: column;
  gap: 12px;
  background: {bg_dark};
}}

.chatMessages::-webkit-scrollbar {{
  width: 6px;
}}

.chatMessages::-webkit-scrollbar-thumb {{
  background: color-mix(in srgb, {text_primary} 30%, transparent);
  border-radius: 3px;
}}

/* Message bubbles */
.message {{
  max-width: 85%;
  padding: 12px 16px;
  border-radius: 16px;
  font-size: 0.9rem;
  line-height: 1.5;
  animation: fadeIn 0.3s ease;
  position: relative;
}}

.messageContent {{
  position: relative;
}}

.verifiedBadge {{
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 16px;
  height: 16px;
  background: #22c55e;
  color: white;
  border-radius: 50%;
  font-size: 10px;
  margin-left: 6px;
  vertical-align: middle;
}}

.citations {{
  display: flex;
  flex-wrap: wrap;
  gap: 6px;
  margin-top: 8px;
  padding-top: 8px;
  border-top: 1px solid rgba(255, 255, 255, 0.1);
}}

.citationTag {{
  font-size: 0.7rem;
  color: {primary_color};
  background: color-mix(in srgb, {primary_color} 10%, transparent);
  padding: 2px 6px;
  border-radius: 4px;
  cursor: help;
  border: 1px solid color-mix(in srgb, {primary_color} 20%, transparent);
}}

.citationTag:hover {{
  background: color-mix(in srgb, {primary_color} 20%, transparent);
}}

.message.user {{
  align-self: flex-end;
  background: {primary_color};
  color: white;
  border-bottom-right-radius: 4px;
}}

.message.bot {{
  align-self: flex-start;
  background: color-mix(in srgb, {text_primary} 10%, transparent);
  color: {text_primary};
  border-bottom-left-radius: 4px;
}}

.message.loading {{
  display: flex;
  gap: 4px;
  padding: 16px 20px;
}}

.message.loading span {{
  width: 8px;
  height: 8px;
  background: color-mix(in srgb, {text_primary} 50%, transparent);
  border-radius: 50%;
  animation: bounce 1.4s infinite ease-in-out;
}}

.message.loading span:nth-child(1) {{ animation-delay: -0.32s; }}
.message.loading span:nth-child(2) {{ animation-delay: -0.16s; }}

@keyframes bounce {{
  0%, 80%, 100% {{ transform: scale(0); }}
  40% {{ transform: scale(1); }}
}}

/* Chat input area */
.chatInputContainer {{
  padding: 16px;
  border-top: 1px solid color-mix(in srgb, {text_primary} 20%, transparent);
  display: flex;
  gap: 10px;
}}

.chatInput {{
  flex: 1;
  padding: 12px 16px;
  background: {bg_dark};
  border: 1px solid color-mix(in srgb, {text_primary} 20%, transparent);
  border-radius: 12px;
  color: {text_primary};
  font-size: 0.9rem;
  outline: none;
  transition: border-color 0.2s;
}}

.chatInput:focus {{
  border-color: {primary_color};
}}

.chatInput::placeholder {{
  color: color-mix(in srgb, {text_primary} 60%, transparent);
}}

.chatSendBtn {{
  width: 44px;
  height: 44px;
  background: {primary_color};
  border: none;
  border-radius: 12px;
  color: white;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.2s;
}}

.chatSendBtn:hover:not(:disabled) {{
  background: color-mix(in srgb, {primary_color} 80%, black);
}}

.chatSendBtn:disabled {{
  opacity: 0.5;
  cursor: not-allowed;
}}

/* Floating chat button */
.chatToggleBtn {{
  position: fixed;
  bottom: 20px;
  right: 20px;
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: linear-gradient(135deg, {primary_color}, #a855f7);
  border: none;
  color: white;
  font-size: 1.5rem;
  cursor: pointer;
  box-shadow: 0 4px 20px rgba(99, 102, 241, 0.4);
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.3s ease;
  z-index: 1000;
}}

.chatToggleBtn:hover {{
  transform: scale(1.1);
  box-shadow: 0 6px 25px rgba(99, 102, 241, 0.5);
}}

/* Context button for text selection */
.contextBtn {{
  position: absolute;
  background: {primary_color};
  color: white;
  border: none;
  border-radius: 20px;
  padding: 5px 12px;
  font-size: 0.8rem;
  cursor: pointer;
  box-shadow: 0 4px 10px rgba(0,0,0,0.3);
  z-index: 2000;
  white-space: nowrap;
}}

/* Icons */
.icon {{
  width: 1em;
  height: 1em;
  display: flex;
  align-items: center;
  justify-content: center;
}}

/* Animations */
@keyframes fadeIn {{
  from {{ opacity: 0; }}
  to {{ opacity: 1; }}
}}

/* Scrollbar styling */
::-webkit-scrollbar {{
  width: 6px;
  height: 6px;
}}

::-webkit-scrollbar-track {{
  background: {bg_dark};
}}

::-webkit-scrollbar-thumb {{
  background: color-mix(in srgb, {text_primary} 30%, transparent);
  border-radius: 3px;
}}

::-webkit-scrollbar-thumb:hover {{
  background: color-mix(in srgb, {text_primary} 50%, transparent);
}}
'''

    def generate_readme(self):
        """Generate the README.md file with integration instructions."""
        readme_content = f"""# ChatWidget Component

This is a complete chatbot component generated for {self.platform.title()} with all functionality including session management, text selection, and history persistence.

## Features

- **Session Management**: Automatically initializes chat sessions with the backend API
- **Text Selection Integration**: Floating button appears when text is selected for contextual queries
- **History Persistence**: Chat history is saved to localStorage and restored on page reload
- **Responsive Design**: Works on all device sizes
- **Accessibility**: Full keyboard navigation and screen reader support
- **Error Handling**: Graceful degradation when API is unavailable

## Installation

1. Copy the `src/components/ChatWidget` directory to your {self.platform.title()} project
2. Ensure you have the required dependencies installed:
   - React
   - TypeScript (optional but recommended)

## Usage

### For {self.platform.title()}:

```tsx
import ChatWidget from \'./components/ChatWidget\';

function MyApp() {{
  return (
    <div>
      <main>
        {/* Your main content */}
      </main>
      <ChatWidget />
    </div>
  );
}}
```

### Environment Variables

Set the following environment variable to configure the API endpoint:

```bash
NEXT_PUBLIC_API_BASE=https://your-api-endpoint.com
```

If not set, the component will attempt to determine the API base URL automatically.

## Configuration

The ChatWidget component accepts the following props:

- `activeSection` (optional): A string to indicate the current section (default: 'chat')

## Theming

The component is designed to respect your existing project's theme variables. It will automatically use CSS custom properties from your project if they exist:

- `--primary`: Primary color for buttons and accents
- `--bg-dark`: Background color for the chat container
- `--bg-card`: Background color for message bubbles
- `--text-primary`: Primary text color
- `--text-secondary`: Secondary text color

If these variables are not defined in your project, the component will use its own default color scheme.

## API Integration

The component makes requests to the following endpoints:

- `POST /chat/start`: Initialize a new chat session
- `POST /chat/send/{{sessionId}}`: Send a message to the chat API
- `GET /chat/history/{{sessionId}}`: Retrieve chat history

## Development

To modify the component:

1. Navigate to `src/components/ChatWidget/`
2. Edit the `ChatWidget.tsx` file to modify the component logic
3. Edit the `ChatWidget.module.css` file to modify the styles
4. Edit the `types.ts` file to modify TypeScript interfaces
5. Edit the `utils.ts` file to modify utility functions

## Folder Structure

```
src/
└── components/
    └── ChatWidget/
        ├── ChatWidget.tsx      # Main component implementation
        ├── ChatWidget.module.css  # Component-specific styles
        ├── types.ts            # TypeScript interfaces
        ├── utils.ts            # Utility functions
        └── index.ts            # Export configuration
```

## Troubleshooting

### Chat Widget Not Appearing

- Ensure the component is properly imported and rendered
- Check browser console for JavaScript errors
- Verify that required environment variables are set

### API Requests Failing

- Verify the `NEXT_PUBLIC_API_BASE` environment variable is set correctly
- Check browser network tab for failed requests
- Ensure the API endpoints are accessible from your domain

### Styling Issues

- Verify that your project's CSS custom properties are defined correctly
- Check that no global styles are conflicting with the component
- Inspect the component in browser developer tools to see applied styles
"""

        (self.output_dir / "README.md").write_text(readme_content)

    def _get_current_date(self) -> str:
        """Get current date in YYYY-MM-DD format."""
        from datetime import datetime
        return datetime.now().strftime("%Y-%m-%d")

    def _get_styling_system(self) -> str:
        """Get the appropriate styling system based on platform."""
        if self.platform == "docusaurus":
            return "Docusaurus theme system / CSS Modules"
        else:
            return "CSS Modules / Tailwind CSS / Styled Components"

    def _get_theming_approach(self, themes: Dict[str, Any]) -> str:
        """Get the theming approach based on detected themes."""
        if themes["css_custom_properties"]:
            return "CSS custom properties with fallbacks"
        else:
            return "Component-specific CSS custom properties"

    def _get_styling_rationale(self) -> str:
        """Get the styling rationale based on platform."""
        if self.platform == "docusaurus":
            return "Docusaurus-specific styling patterns with theme variable integration"
        else:
            return "Project-specific styling system integration with theme variable respect"

    def _get_project_structure(self) -> str:
        """Get the appropriate project structure based on platform."""
        if self.platform == "docusaurus":
            return "src/components/ structure following Docusaurus conventions"
        else:
            return "components/ structure following Next.js/React conventions"

    def run(self):
        """Run the complete chatbot generation process."""
        print(f"Analyzing project themes for {self.platform}...")
        themes = self.analyze_project_themes()

        print("Creating directory structure...")
        self.create_directory_structure()

        print("Generating documentation...")
        self.generate_spec_document(themes)
        self.generate_plan_document(themes)
        self.generate_tasks_document(themes)

        print("Generating implementation files...")
        self.generate_implementation_files(themes)

        print("Generating README...")
        self.generate_readme()

        print(f"\\nChatbot component generated successfully in {self.output_dir}")

        # Perform automatic integration
        print(f"\\nPerforming automatic integration...")
        self.integrate_chatbot()

        print(f"\\nNext steps completed:")
        print(f"- Chatbot component has been automatically integrated into your application root")
        print(f"- Review the generated documentation in {self.output_dir}/docs/")
        print(f"- Check the implementation in {self.output_dir}/src/components/ChatWidget/")
        print(f"- Verify the integration works throughout your application")
        print(f"- Set up your environment variables for API integration if not already done")

    def integrate_chatbot(self):
        """Automatically integrate the chatbot into the application root based on platform."""
        if self.platform == "docusaurus":
            self._integrate_docusaurus()
        else:  # react or nextjs
            self._integrate_react_nextjs()

    def _integrate_docusaurus(self):
        """Integrate chatbot into Docusaurus application root."""
        # Look for common Docusaurus layout files
        possible_layouts = [
            self.project_root / "src" / "theme" / "Layout" / "index.js",
            self.project_root / "src" / "theme" / "Layout.js",
            self.project_root / "src" / "components" / "Layout.js"
        ]

        layout_file = None
        for possible in possible_layouts:
            if possible.exists():
                layout_file = possible
                break

        if layout_file:
            self._backup_file(layout_file)
            self._modify_layout_file(layout_file, "docusaurus")
            print(f"  - Integrated into Docusaurus layout: {layout_file}")
        else:
            # If no custom layout found, look for the main App or root component
            self._try_integrate_with_main_component()

    def _try_integrate_with_main_component(self):
        """Attempt to integrate with main component if no specific layout found."""
        # Look for common root components
        possible_roots = [
            self.project_root / "src" / "App.js",
            self.project_root / "src" / "App.tsx",
            self.project_root / "App.js",
            self.project_root / "App.tsx",
            self.project_root / "src" / "index.js",
            self.project_root / "src" / "main.js"
        ]

        for possible in possible_roots:
            if possible.exists():
                self._backup_file(possible)
                self._modify_layout_file(possible, "react_nextjs")
                print(f"  - Integrated into main component: {possible}")
                return

        print(f"  - Could not find standard layout file to integrate with")
        print(f"  - You may need to manually add <ChatWidget /> to your root component")

    def _integrate_react_nextjs(self):
        """Integrate chatbot into React/Next.js application root."""
        # Look for common Next.js/React root files
        possible_roots = [
            self.project_root / "pages" / "_app.js",      # Next.js pages router
            self.project_root / "pages" / "_app.tsx",    # Next.js pages router with TS
            self.project_root / "src" / "pages" / "_app.js",
            self.project_root / "src" / "pages" / "_app.tsx",
            self.project_root / "app" / "layout.tsx",    # Next.js app router
            self.project_root / "app" / "layout.js",     # Next.js app router
            self.project_root / "src" / "app" / "layout.tsx",
            self.project_root / "src" / "app" / "layout.js",
            self.project_root / "App.js",                # React app
            self.project_root / "App.tsx",               # React app with TS
            self.project_root / "src" / "App.js",
            self.project_root / "src" / "App.tsx"
        ]

        root_file = None
        for possible in possible_roots:
            if possible.exists():
                root_file = possible
                break

        if root_file:
            self._backup_file(root_file)
            self._modify_layout_file(root_file, "react_nextjs")
            print(f"  - Integrated into React/Next.js root: {root_file}")
        else:
            print(f"  - Could not find standard layout file to integrate with")
            print(f"  - You may need to manually add <ChatWidget /> to your root component")

    def _backup_file(self, file_path):
        """Create a backup of the original file before modification."""
        backup_path = file_path.with_suffix(file_path.suffix + ".backup")
        backup_path.write_text(file_path.read_text())
        print(f"    - Created backup: {backup_path}")

    def _modify_layout_file(self, file_path, platform):
        """Modify the layout file to include the ChatWidget component."""
        content = file_path.read_text()

        # Add import statement at the top
        import_statement = "import ChatWidget from './components/ChatWidget';\n"

        # For Docusaurus/Next.js, we need to handle different file types
        if file_path.suffix in ['.js', '.jsx', '.ts', '.tsx']:
            # Check if import already exists
            if "import ChatWidget" not in content:
                # Add import after other imports
                lines = content.split('\n')
                new_lines = []
                import_added = False

                for line in lines:
                    new_lines.append(line)
                    if line.strip().startswith('import') and not import_added:
                        # Find a suitable place to add our import
                        if './components' in line or '../components' in line or 'src/components' in line:
                            new_lines.append(import_statement.rstrip())
                            import_added = True

                # If we didn't find a good place, add after the last import
                if not import_added:
                    for i, line in enumerate(new_lines):
                        if line.strip().startswith('import'):
                            continue
                        elif line.strip() and not line.strip().startswith('import'):
                            # Insert after the last import line
                            for j in range(i-1, -1, -1):
                                if new_lines[j].strip().startswith('import'):
                                    new_lines.insert(j+1, import_statement.rstrip())
                                    import_added = True
                                    break
                            break

                # If still no import added, add at the beginning after other imports
                if not import_added:
                    for i, line in enumerate(new_lines):
                        if not line.strip().startswith('import') and line.strip():
                            new_lines.insert(i, import_statement.rstrip())
                            import_added = True
                            break
                    if not import_added:
                        new_lines.append(import_statement.rstrip())

                content = '\n'.join(new_lines)

        # Add the ChatWidget component to the render/return section
        if platform == "docusaurus":
            # For Docusaurus, add to the main layout after children
            content = self._add_component_to_jsx(content, "ChatWidget")
        else:
            # For React/Next.js, add to the main component after children
            content = self._add_component_to_jsx(content, "ChatWidget")

        # Write the modified content back to the file
        file_path.write_text(content)

    def _add_component_to_jsx(self, content, component_name):
        """Add the component to JSX content in a suitable location."""
        # Look for common JSX patterns where we can add the component
        patterns = [
            r'(<main[^>]*>.*?</main>)',
            r'(<div[^>]*>.*?</div>)',
            r'(<section[^>]*>.*?</section>)',
            r'(<Layout[^>]*>.*?</Layout>)',
            r'(children)',
            r'(.*return\s*\()',
            r'(.*return\s*{)',
            r'(return\s*[^;]*;)'
        ]

        for pattern in patterns:
            import re
            matches = list(re.finditer(pattern, content, re.DOTALL | re.MULTILINE))
            if matches:
                # Add the component at the end of the matched content
                # We'll add it after the closing tag of the matched pattern
                for match in reversed(matches):  # Reverse to avoid position shifting
                    # For self-closing tags or return statements, add differently
                    if 'children' in match.group(0) or 'return' in match.group(0):
                        # For return statements, we'll add the component
                        start_pos = match.end()
                        # Find the end of the return statement/block
                        remaining = content[start_pos:]
                        if 'return' in match.group(0):
                            # Look for the end of the return statement
                            if '(' in remaining or '{' in remaining:
                                # Complex return, find the closing parenthesis/brace
                                brace_level = 0
                                paren_level = 0
                                pos = start_pos
                                for i, char in enumerate(remaining):
                                    if char == '{':
                                        brace_level += 1
                                    elif char == '}':
                                        brace_level -= 1
                                    elif char == '(':
                                        paren_level += 1
                                    elif char == ')':
                                        paren_level -= 1

                                    if brace_level == 0 and paren_level == 0 and char in [';', '}']:
                                        pos = start_pos + i + 1
                                        break
                                else:
                                    pos = start_pos

                                # Insert the component before the final return closing
                                component_str = f"  <{component_name} />"
                                content = content[:pos] + f"\\n{component_str}" + content[pos:]
                        else:
                            # For main/div sections, add the component after
                            end_pos = self._find_closing_bracket(content, match.end())
                            if end_pos > match.end():
                                component_str = f"  <{component_name} />"
                                content = content[:end_pos] + f"\\n{component_str}" + content[end_pos:]

                break
        else:
            # If no specific pattern found, try to add to the end of the main component function
            # Look for component function definitions
            import re
            # Match function or class component definitions
            func_patterns = [
                r'(function\s+\w+\s*\(.*?\)\s*{)',
                r'(const\s+\w+\s*=\s*\([^)]*\)\s*=>\s*{)',
                r'(const\s+\w+\s*=\s*function\s*\(.*?\)\s*{)',
                r'(class\s+\w+\s+extends\s+Component\s*{)'
            ]

            for pattern in func_patterns:
                matches = list(re.finditer(pattern, content))
                if matches:
                    # Find the corresponding closing brace for the function/class
                    last_match = matches[-1]  # Use the last match to get the main component
                    end_pos = self._find_closing_bracket(content, last_match.end())
                    if end_pos > last_match.end():
                        component_str = f"  <{component_name} />"
                        content = content[:end_pos-1] + f"\\n{component_str}\\n" + content[end_pos-1:]
                    break

        return content

    def _find_closing_bracket(self, content, start_pos):
        """Find the matching closing bracket for a given opening position."""
        brace_level = 0
        pos = start_pos

        for i in range(start_pos, len(content)):
            char = content[i]
            if char == '{':
                brace_level += 1
            elif char == '}':
                brace_level -= 1
                if brace_level == 0:
                    return i + 1  # Position after the closing brace

        return start_pos  # If not found, return original position


def main():
    import sys
    if len(sys.argv) != 2:
        print("Usage: python generate-chatbot.py <platform>")
        print("Platform options: docusaurus, react, nextjs")
        sys.exit(1)

    platform = sys.argv[1].lower()
    if platform not in ["docusaurus", "react", "nextjs"]:
        print("Platform must be one of: docusaurus, react, nextjs")
        sys.exit(1)

    generator = ChatbotGenerator(platform)
    generator.run()


if __name__ == "__main__":
    main()