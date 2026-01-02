---
name: chatbot-generator
description: |
  Generate a complete chatbot component with specification, plan, tasks, and implementation
  for either Docusaurus or React/Next.js, with proper theming and clean folder structure.
  Use this skill when creating chatbot components that need to integrate with existing
  projects while respecting current theme variables and maintaining clean architecture.
version: "1.0.0"
constitution_alignment: "v4.0.1"
---

# Skill: Chatbot Component Generator

**Purpose**: Generate a complete chatbot component with specification, plan, tasks, and implementation for either Docusaurus or React/Next.js, with proper theming and clean folder structure.

**Constitution Alignment**: v4.0.1 emphasizing:
- **Principle 1**: Specification Primacy ("Specs Are the New Syntax")
- **Section IIa**: Complete Implementation (spec → plan → tasks → implementation)
- **Nine Pillars**: AI-First IDEs, SDD, Composable Skills, Clean Architecture

**Status**: Reusable skill for chatbot component generation
**Application**: Any Docusaurus or React/Next.js project requiring a chatbot component

---

## Core Principles

### 1. Platform-First Specification
❌ **DON'T**: Create generic chatbot without considering target platform
✅ **DO**: Ask platform type first, then generate platform-specific implementation

**Why**: Ensures proper integration with target platform's architecture and conventions.

**Process**:
- Ask: "What platform would you like to create the chatbot for?"
- Options: Docusaurus, React/Next.js
- Generate implementation following platform-specific patterns

### 2. Theme Respect Principle
❌ **DON'T**: Create new theme variables that conflict with existing project
✅ **DO**: Analyze existing project for theme variables and use them

**Why**: Maintains visual consistency with existing project.

**Process**:
- Scan project for existing CSS custom properties, Tailwind themes, or styling patterns
- Use existing variables for colors, spacing, typography
- Create new variables only when necessary

### 3. Complete SDD Cycle
❌ **DON'T**: Generate only code without documentation
✅ **DO**: Create spec, plan, tasks, and implementation in sequence

**Why**: Enables understanding, maintenance, and extension of generated components.

**Process**:
- spec.md: Requirements and functionality specification
- plan.md: Architecture and implementation approach
- tasks.md: Implementation task breakdown
- Implementation: Complete component code

### 4. Clean Folder Structure
❌ **DON'T**: Create messy, unorganized file structure
✅ **DO**: Use clean, organized, platform-appropriate structure

**Why**: Eases maintenance and understanding of the generated code.

**Structure**:
```
generated-chatbot/
├── docs/
│   ├── spec.md
│   ├── plan.md
│   └── tasks.md
├── src/
│   └── components/
│       └── ChatWidget/
│           ├── ChatWidget.tsx
│           ├── ChatWidget.module.css (or appropriate styling)
│           └── index.ts
└── README.md
```

### 5. Full Feature Parity
❌ **DON'T**: Generate simplified chatbot without key features
✅ **DO**: Include all functionality from existing implementation

**Why**: Ensures generated chatbot has complete functionality users expect.

**Features to Include**:
- Session management with API integration
- Text selection with floating action button (appears when text is selected)
- History persistence with localStorage fallback
- Loading states and error handling
- Responsive design and accessibility
- API integration with backend endpoints
- Citation support with source IDs
- Verified content indicators
- Context-aware querying with selected text
- Message timestamps and user/bot differentiation
- Smooth scrolling to latest messages
- Retry mechanisms for failed requests

### 6. Integration-Ready Implementation
❌ **DON'T**: Generate components that are hard to integrate
✅ **DO**: Provide clear integration instructions and proper exports

**Why**: Ensures generated component can be easily integrated into the target project.

**Process**:
- Proper TypeScript interfaces
- Clear import/export structure
- Integration documentation in README.md
- Example usage patterns

---

## The Chatbot Generation Workflow

### Phase 1: Platform Selection
**Input**: User selects target platform (Docusaurus or React/Next.js)
**Output**: Platform-specific generation strategy

**Questions to Ask**:
1. What platform would you like to create the chatbot for? (Docusaurus/React/Next.js)

### Phase 2: Project Analysis
**Input**: Current project structure and existing theme variables
**Output**: Analysis of existing styling patterns and conventions

**Process**:
- Scan for CSS custom properties in global styles
- Look for Tailwind theme configurations
- Identify existing color palettes and design tokens
- Analyze component structure conventions

### Phase 3: Documentation Generation
**Input**: Analyzed project and platform requirements
**Output**: Complete documentation set

**Generate**:
- `docs/spec.md`: Complete specification following the reverse-engineered pattern
- `docs/plan.md`: Implementation architecture and approach
- `docs/tasks.md`: Detailed task breakdown for implementation

### Phase 4: Component Implementation
**Input**: Platform type and existing theme analysis
**Output**: Complete chatbot component implementation

**For Docusaurus**:
- Create in `src/components/ChatWidget/`
- Use Docusaurus styling conventions
- Integrate with Docusaurus layout system
- Follow MDX component patterns if applicable

**For React/Next.js**:
- Create in `components/ChatWidget/` or appropriate location
- Use Next.js/React patterns
- Integrate with existing styling system
- Follow project-specific component conventions

### Phase 5: Automatic Integration
**Input**: Complete implementation and target platform
**Output**: Automatic integration into application root

**For Docusaurus**:
- Detect Docusaurus layout files (`src/theme/Layout/index.js`, `docusaurus.config.js`)
- Modify layout to include ChatWidget component
- Ensure proper placement in DOM structure
- Maintain Docusaurus theme compatibility

**For React/Next.js**:
- Detect main layout files (`pages/_app.js`, `app/layout.tsx`, `App.js`)
- Modify root component to include ChatWidget
- Ensure proper placement in component tree
- Maintain Next.js/React conventions

**Process**:
- Analyze project structure to identify root layout component
- Create backup of original files before modification
- Inject ChatWidget import and component into root
- Verify integration doesn't conflict with existing components
- Test integration by verifying component appears throughout application

---

## Quality Standards Checklist

All generated chatbot components MUST:
- ✅ **Follow platform-specific conventions**: Docusaurus or React/Next.js patterns
- ✅ **Respect existing themes**: Use project's existing CSS variables and styling
- ✅ **Include complete functionality**: All features from reference implementation
- ✅ **Have proper TypeScript typing**: Complete type safety
- ✅ **Include accessibility features**: ARIA attributes, keyboard navigation
- ✅ **Be responsive**: Work on all screen sizes
- ✅ **Include error handling**: Graceful degradation and error states
- ✅ **Follow clean architecture**: Organized folder structure

## Acceptance Checks

- [ ] Platform-specific implementation follows best practices
- [ ] Uses existing project theme variables where available
- [ ] Includes complete functionality from reference implementation
- [ ] Has proper TypeScript interfaces and typing
- [ ] Includes accessibility considerations
- [ ] Responsive design implemented
- [ ] Clear integration instructions provided
- [ ] Proper folder structure maintained

---

## Success Metrics

For each generated chatbot:

| Metric | Success Measure | Target |
|--------|-----------------|--------|
| **Platform Integration** | Component follows platform conventions | 100% platform-appropriate |
| **Theme Consistency** | Uses existing project themes | 100% theme variable usage |
| **Feature Completeness** | All reference features implemented | 100% feature parity |
| **Code Quality** | Clean, maintainable, well-typed | 100% TypeScript safety |
| **Accessibility** | Proper ARIA, keyboard support | 100% accessible |
| **Integration Ease** | Clear integration instructions | 100% integration-ready |

---

## How to Use This Skill

### When Generating a Chatbot Component

```markdown
Use the chatbot-generator skill to:
1. Ask for target platform (Docusaurus or React/Next.js)
2. Analyze existing project theme variables
3. Generate complete documentation (spec, plan, tasks)
4. Create full-featured implementation
5. Provide integration instructions
```

### When Customizing Generated Components

```markdown
Use the chatbot-generator skill to:
1. Maintain theme consistency with existing project
2. Preserve all functionality from reference implementation
3. Follow platform-specific best practices
4. Ensure proper TypeScript typing
5. Include accessibility features
```

---

**Status**: Ready for use on all Docusaurus and React/Next.js projects requiring chatbot components. Proven with the reference implementation from the learning-chatbots-ui-creation project.