# Docusaurus Book Generator Skill

This skill automates the complete setup of a Docusaurus-based book website with Tailwind CSS, TypeScript, and proper GitHub Pages deployment configuration.

## Purpose
Generate a fully configured Docusaurus book website with Tailwind CSS, TypeScript, and GitHub Actions workflow in a single command. The skill accepts book metadata and creates a ready-to-use book structure with automated deployment.

## Input Parameters
- `title` (required): The main title of the book
- `tagline` (optional): Custom tagline; if not provided, generates from title
- `description` (optional): Custom description; if not provided, generates from title
- `github_org` (optional): GitHub organization name for deployment
- `github_repo` (optional): GitHub repository name for deployment
- `output_dir` (optional): Directory name for the website (default: "book-source")

## Process

### 1. Project Initialization
- Run `npx create-docusaurus@latest book-source classic --typescript` to create a new Docusaurus v3 project
- Install Tailwind CSS v4 with PostCSS plugin (`npm install -D tailwindcss @tailwindcss/postcss autoprefixer`)
- Note: Tailwind v4 does NOT use `tailwind.config.js` or `postcss.config.js` - configuration is done via CSS and Docusaurus plugins

### 2. Dynamic Configuration Setup
- **Title Generation**: If user provides main title, automatically generate:
  - Site title: User-provided title
  - Tagline: Generate from title (e.g., "Exploring [Title]" if not provided)
  - Meta description: Generate relevant description based on title
  - If user provides specific tagline/description, use those instead
- Update `docusaurus.config.ts` with provided/configured title, tagline, and metadata
- Configure proper GitHub Pages settings (organizationName, projectName, baseUrl) based on provided GitHub details

### 3. Styling Configuration (Tailwind CSS v4)
- **CRITICAL CHANGE**: Tailwind v4 uses a completely different configuration approach than v3
- Do NOT create `postcss.config.js` or `tailwind.config.js` - these are not used in v4
- Add PostCSS plugin configuration directly in `docusaurus.config.ts` using the `plugins` array with `configurePostCss` hook
- Update `src/css/custom.css` to use selective imports:
  ```css
  @import "tailwindcss/theme" layer(theme);
  @import "tailwindcss/utilities" layer(utilities);
  ```
  - **CRITICAL:** Import ONLY theme and utilities layers, NOT the base layer
  - `tailwindcss/theme` = CSS variables for colors, spacing, fonts
  - `tailwindcss/utilities` = Utility classes (bg-*, text-*, p-*, etc.)
  - Skipping `tailwindcss/base` prevents CSS reset from breaking Docusaurus layout
  - Without this selective import, all Docusaurus styling will be destroyed
- Add any custom brand colors or styling to CSS variables in custom.css

### 4. Content Cleanup
- Remove all default Docusaurus tutorial content (tutorial-basics, tutorial-extras)
- Remove default blog content and assets
- Remove any default pages that don't fit the book structure
- Keep only essential files like `index.tsx`, `intro.md`, and sidebar configuration

### 5. Custom Content Setup
- Create a basic intro page with placeholder content
- Set up a basic sidebar structure ready for book modules
- Configure the homepage with a simple layout appropriate for a book

### 6. Project Structure Setup
- Prepare project structure ready for GitHub Pages deployment (workflow setup handled separately with docusaurus-deployer skill)
- Configure deployment settings based on provided GitHub details for later workflow setup

### 7. Final Configuration
- Update `.gitignore` appropriately
- Set up proper TypeScript configuration
- Ensure all paths and configurations are consistent

## Output
- Fully configured Docusaurus project with TypeScript and Tailwind CSS
- Clean content structure with default Docusaurus content removed
- Project structure ready for GitHub Pages deployment (use docusaurus-deployer skill for workflow setup)
- Ready-to-use book structure with proper metadata
- All necessary configurations set up for immediate content addition and deployment

## Usage
```
skill: "docusaurus-book-generator"
title: "Your Book Title"
tagline: "Your custom tagline" (optional)
description: "Your custom description" (optional)
github_org: "your-github-org" (optional)
github_repo: "your-repo-name" (optional)
```

## Success Criteria
- Docusaurus project created successfully with TypeScript and Tailwind CSS
- All default Docusaurus content removed
- Configuration files properly updated with provided metadata
- Project structure prepared for GitHub Pages deployment (use docusaurus-deployer skill for workflow setup)
- Project ready for immediate content addition
- Deployment workflow ready for GitHub Pages