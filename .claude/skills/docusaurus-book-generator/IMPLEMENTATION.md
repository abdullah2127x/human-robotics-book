# Docusaurus Book Generator Implementation

This script implements the Docusaurus Book Generator skill that automates the complete setup of a Docusaurus-based book website.

## Implementation Steps

### 1. Project Initialization
```bash
# Create new Docusaurus project with TypeScript and Classic theme
npx create-docusaurus@latest book-source classic --typescript

# Navigate to book-source directory
cd book-source

# Install Tailwind CSS v4 dependencies (IMPORTANT: includes @tailwindcss/postcss)
npm install -D tailwindcss @tailwindcss/postcss autoprefixer
```

### 2. Tailwind CSS v4 Configuration

**IMPORTANT**: Tailwind CSS v4 has a different setup than v3. Do NOT use `npx tailwindcss init -p`.

Create `postcss.config.js` manually:
```javascript
module.exports = {
  plugins: {
    '@tailwindcss/postcss': {},  // NOT 'tailwindcss'
    autoprefixer: {},
  },
};
```

Create `tailwind.config.js` manually:
```javascript
/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    './src/**/*.{js,jsx,ts,tsx}',
    './docs/**/*.{md,mdx}',
  ],
  theme: {
    extend: {},
  },
  plugins: [],
  corePlugins: {
    preflight: false,  // Disable preflight to avoid conflicts with Docusaurus/Infima
  },
};
```

Update `src/css/custom.css` with Tailwind v4 import syntax:
```css
@import "tailwindcss";

/* Rest of your CSS... */
```

**Key differences from Tailwind v3:**
- Use `@import "tailwindcss";` instead of `@tailwind base; @tailwind components; @tailwind utilities;`
- Use `@tailwindcss/postcss` plugin instead of `tailwindcss` in postcss.config.js
- No `npx tailwindcss init` command - create config files manually

### 3. Docusaurus Configuration Setup
- Update `docusaurus.config.ts` with provided title, tagline, and metadata
- Configure GitHub Pages settings if GitHub org/repo provided
- Update navigation structure for book format
- Set `blog: false` to disable blog functionality

### 4. Styling Configuration
- Configure `tailwind.config.js` with proper content paths
- Update `src/css/custom.css` with Tailwind import and custom Infima variables
- Add custom brand styling if needed

### 5. Content Cleanup
- Remove default tutorial content from `docs/` (tutorial-basics, tutorial-extras)
- Remove default blog content from `blog/`
- Remove unnecessary documentation files
- Remove default HomepageFeatures component if creating custom homepage

### 6. Custom Content Creation
- Create basic intro page with provided title
- Set up sidebar structure for book modules (use `bookSidebar` instead of `tutorialSidebar`)
- Configure homepage with book layout and feature sections

### 7. GitHub Actions Workflow
- Create `.github/workflows/deploy.yml` with deployment workflow
- Configure deployment settings based on provided GitHub details
- Set working-directory to `book-source`

### 8. Finalization
- Update package.json scripts
- Verify build succeeds with `npm run build`
- Create initial git commit

## Common Issues

### PostCSS Plugin Error
**Error**: `It looks like you're trying to use 'tailwindcss' directly as a PostCSS plugin`
**Solution**: Install `@tailwindcss/postcss` and use it in postcss.config.js instead of `tailwindcss`

### CSS Import Error
**Error**: Unknown at-rule `@tailwind`
**Solution**: Use `@import "tailwindcss";` syntax (Tailwind v4) instead of `@tailwind base;` etc.

### Sidebar Error
**Error**: `Sidebar category has neither any subitem nor a link`
**Solution**: Empty categories need either items or a `link` property. Remove empty categories or add placeholder content.
