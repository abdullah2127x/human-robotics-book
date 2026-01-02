# Chatbot Implementation Patterns

## Platform-Specific Patterns

### Docusaurus Implementation Pattern
- **Component Location**: `src/components/ChatWidget/`
- **Styling**: Use Docusaurus theme variables and CSS custom properties
- **Integration**: Hook into Docusaurus layout system via theme components
- **Markdown Compatibility**: Ensure compatibility with MDX if needed

### React/Next.js Implementation Pattern
- **Component Location**: `components/ChatWidget/` or `src/components/ChatWidget/`
- **Styling**: Use project's existing styling system (Tailwind, CSS Modules, etc.)
- **Integration**: Standard React component integration patterns
- **SSR Considerations**: Handle chat state appropriately for server-side rendering

## Theming Patterns

### CSS Custom Properties Pattern
- **Detection**: Scan for existing `:root` or theme variables
- **Usage**: Use existing color, spacing, and typography variables
- **Fallback**: Define fallbacks only when necessary

### Tailwind Integration Pattern
- **Theme Detection**: Check `tailwind.config.js` for custom theme
- **Class Application**: Use existing utility classes where possible
- **Custom Extensions**: Extend theme only when required

## Architecture Patterns

### Component Structure Pattern
```
ChatWidget/
├── ChatWidget.tsx          # Main component with all functionality
├── ChatWidget.module.css   # Component-specific styles
├── types.ts                # TypeScript interfaces
├── utils.ts                # Helper functions
└── index.ts                # Export file
```

### State Management Pattern
- **Local State**: Use React useState for UI state
- **Session State**: Manage session ID and chat history
- **Persistence**: Use localStorage as fallback
- **API Integration**: Handle API calls with proper error handling

### API Integration Pattern
- **Session Management**: Initialize session on component mount
- **Message Exchange**: Send/receive messages with proper error handling
- **History Persistence**: Fetch and store chat history
- **Loading States**: Handle loading and error states gracefully

## Feature Implementation Patterns

### Text Selection Integration
- **Detection**: Use `window.getSelection()` for text selection
- **Positioning**: Calculate position for floating action button
- **Context**: Pre-fill input with selected text context
- **Accessibility**: Ensure keyboard navigation compatibility

### Responsive Design Pattern
- **Mobile-First**: Design for mobile and scale up
- **Breakpoints**: Use existing project breakpoints
- **Touch Targets**: Ensure adequate touch target sizes
- **Adaptive Layout**: Adjust layout based on screen size

### Accessibility Pattern
- **ARIA Labels**: Proper labels for interactive elements
- **Keyboard Navigation**: Full keyboard support
- **Screen Reader**: Proper semantic structure
- **Focus Management**: Clear focus indicators

## Error Handling Patterns

### API Error Handling
- **Network Errors**: Graceful degradation when API unavailable
- **Response Validation**: Handle various response formats
- **User Feedback**: Clear error messages to users
- **Fallback Behavior**: Use localStorage when API fails

### State Error Handling
- **Invalid States**: Handle unexpected component states
- **User Input**: Validate and sanitize user input
- **Session Recovery**: Handle session initialization failures
- **Graceful Degradation**: Maintain functionality when features fail

## Performance Patterns

### Rendering Optimization
- **Memoization**: Use React.memo for performance
- **Conditional Rendering**: Render only when necessary
- **Virtual Scrolling**: For long message histories
- **Code Splitting**: Lazy load when appropriate

### API Optimization
- **Debouncing**: Prevent excessive API calls
- **Caching**: Cache responses when appropriate
- **Retry Logic**: Implement intelligent retry strategies
- **Connection Management**: Handle connection states properly