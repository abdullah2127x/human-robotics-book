# Quick Start Guide: Physical AI & Humanoid Robotics Book Infrastructure

## Overview

This guide provides instructions for setting up, developing, and deploying the Physical AI & Humanoid Robotics book infrastructure using Docusaurus v3.

## Prerequisites

- Node.js 18.0 or higher
- npm or yarn package manager
- Git
- A GitHub account (for deployment)

## Installation

### 1. Clone or Create the Repository

```bash
# If starting fresh
npx create-docusaurus@latest website classic --typescript

# If working with existing repository
git clone <repository-url>
cd website
```

### 2. Install Dependencies

```bash
npm install
# OR
yarn install
```

## Configuration

### 1. Update Site Configuration

Edit `docusaurus.config.ts`:

```typescript
export default {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging Digital Minds to Physical Bodies',
  url: 'https://your-github-username.github.io',
  baseUrl: '/repository-name/',
  organizationName: 'your-github-username',
  projectName: 'repository-name',
  deploymentBranch: 'gh-pages',
  // ... other configuration
} as Config;
```

### 2. Configure Navigation

In `docusaurus.config.ts`, update the navbar items:

```typescript
themeConfig: {
  navbar: {
    title: 'Physical AI & Humanoid Robotics',
    items: [
      {
        type: 'doc',
        position: 'left',
        label: 'Learn',
        docId: 'intro',
      },
      {
        to: '/resources',
        label: 'Resources',
        position: 'left',
      },
      {
        to: '/capstone',
        label: 'Capstone',
        position: 'left',
      },
      {
        label: 'Login/Personalize',
        position: 'right',
        // This is a placeholder - implement actual functionality later
      },
    ],
  },
  // ... other theme config
} as Config;
```

### 3. Set Up Sidebar Structure

Create or update `sidebars.ts`:

```typescript
export default {
  docs: [
    {
      type: 'category',
      label: 'Intro',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1',
      items: ['module1/ros-intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2',
      items: ['module2/digital-twin'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3',
      items: ['module3/ai-reasoning'],
      collapsed: false,
    },
  ],
} satisfies SidebarsConfig;
```

## Development

### 1. Start Development Server

```bash
npm run start
# OR
yarn start
```

This will start a local development server at `http://localhost:3000`.

### 2. Create Documentation Pages

Create markdown files in the `docs/` directory following the module structure:

```
docs/
├── intro.md
├── module1/
│   ├── index.md
│   └── ros-intro.md
├── module2/
│   ├── index.md
│   └── digital-twin.md
└── module3/
    ├── index.md
    └── ai-reasoning.md
```

### 3. Add Content to Pages

Each documentation page should follow this structure:

```markdown
---
title: Page Title
description: Brief description of the page content
---

# Page Title

Content goes here...

## Section

More content...

### Subsection

Detailed information...
```

## Styling Setup

### 1. Install Tailwind CSS

```bash
npm install -D tailwindcss postcss autoprefixer
npx tailwindcss init -p
```

### 2. Configure Tailwind

Update `tailwind.config.js`:

```javascript
/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    './src/**/*.{js,jsx,ts,tsx}',
    './docs/**/*.{md,mdx}',
    './blog/**/*.{md,mdx}',
  ],
  theme: {
    extend: {},
  },
  plugins: [],
}
```

### 3. Add Tailwind to CSS

In `src/css/custom.css`:

```css
@tailwind base;
@tailwind components;
@tailwind utilities;
```

## Building and Deployment

### 1. Build the Site

```bash
npm run build
# OR
yarn build
```

This creates a `build/` directory with the static files.

### 2. Deploy to GitHub Pages

```bash
npm run deploy
# OR
yarn deploy
```

This command:
1. Builds the site
2. Pushes the build files to the `gh-pages` branch
3. Configures GitHub Pages to serve from that branch

## Running Tests

### 1. Unit Tests

```bash
npm run test
# OR
yarn test
```

### 2. End-to-End Tests

```bash
npm run test:e2e
# OR
yarn run test:e2e
```

## Useful Commands

```bash
# Serve built site locally (for testing)
npm run serve
# OR
yarn serve

# Clear build cache
npm run clear
# OR
yarn clear

# Check for broken links
npm run swizzle
```

## Project Structure

```
website/                    # Docusaurus project root
├── blog/                   # Blog posts (if needed)
├── docs/                   # Documentation files organized by modules
│   ├── intro.md            # Introduction page
│   ├── module1/            # Module 1 documentation
│   │   ├── index.md
│   │   └── ros-intro.md
│   ├── module2/            # Module 2 documentation
│   │   ├── index.md
│   │   └── digital-twin.md
│   └── module3/            # Module 3 documentation
│       ├── index.md
│       └── ai-reasoning.md
├── src/
│   ├── components/         # Custom React components
│   ├── css/                # Custom CSS and Tailwind setup
│   │   └── custom.css
│   └── pages/              # Custom pages
├── static/                 # Static assets (images, files)
├── docusaurus.config.ts    # Main configuration file
├── sidebars.ts             # Sidebar navigation configuration
├── package.json            # Project dependencies and scripts
├── tsconfig.json           # TypeScript configuration
├── postcss.config.js       # PostCSS configuration for Tailwind
└── tailwind.config.js      # Tailwind CSS configuration
```

## Troubleshooting

### Common Issues

1. **Page not found after navigation update**
   - Ensure the docId in navbar links matches a file in docs/
   - Check for typos in file names and paths

2. **Styles not applying**
   - Verify Tailwind CSS is properly configured
   - Check that custom CSS is imported in `src/css/custom.css`

3. **GitHub Pages deployment failing**
   - Verify organizationName and projectName in docusaurus.config.ts
   - Ensure you have push permissions to the repository

### Getting Help

- [Docusaurus documentation](https://docusaurus.io/docs)
- [GitHub Discussions](https://github.com/facebook/docusaurus/discussions)
- Check the project's issue tracker for similar problems