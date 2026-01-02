# Chatbot Generator Skill

This skill generates a complete chatbot component with specification, plan, tasks, and implementation for either Docusaurus or React/Next.js projects, with proper theming and clean folder structure.

## Overview

The chatbot-generator skill creates a complete, production-ready chatbot component that:
- Matches the functionality of the existing reference implementation
- Respects existing project themes and styling
- Includes comprehensive documentation (spec, plan, tasks)
- Follows platform-specific best practices
- Maintains clean, organized folder structure

## Usage

To use this skill, run:

```bash
# This skill will prompt for platform selection
```

The skill will ask you to select between:
- Docusaurus
- React/Next.js

Then it will:
1. Analyze your project's existing themes and structure
2. Generate the complete chatbot implementation with all functionality
3. Automatically integrate the chatbot into your application root
4. Create comprehensive documentation (spec, plan, tasks)

The chatbot will be automatically added throughout your application via the root layout component.

## Output Structure

The skill generates:

```
generated-chatbot/
├── docs/
│   ├── spec.md          # Complete specification
│   ├── plan.md          # Implementation plan
│   └── tasks.md         # Task breakdown
├── src/
│   └── components/
│       └── ChatWidget/
│           ├── ChatWidget.tsx
│           ├── ChatWidget.module.css
│           ├── types.ts
│           ├── utils.ts
│           └── index.ts
└── README.md            # Integration instructions
```

## Features Included

- Session management with API integration
- Text selection with floating action button
- History persistence with localStorage fallback
- Loading states and error handling
- Responsive design and accessibility
- Proper TypeScript typing
- Platform-specific integration patterns

## Platform-Specific Implementation

### For Docusaurus:
- Follows Docusaurus component conventions
- Integrates with Docusaurus theme system
- Compatible with MDX content

### For React/Next.js:
- Follows Next.js/React best practices
- Integrates with existing styling systems
- Handles SSR considerations appropriately

## Theming

The generated component automatically:
- Detects existing CSS custom properties
- Uses Tailwind theme variables if present
- Maintains visual consistency with the project
- Falls back to appropriate defaults when needed