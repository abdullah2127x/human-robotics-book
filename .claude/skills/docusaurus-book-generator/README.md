# Docusaurus Book Generator

A skill that automates the setup of a Docusaurus-based book website with Tailwind CSS v4, TypeScript, and GitHub Pages deployment.

## Features

- Creates a complete Docusaurus v3 project with TypeScript and Classic theme
- Configures Tailwind CSS v4 with `@tailwindcss/postcss` plugin
- Sets up custom title, tagline, and metadata based on provided parameters
- Removes all default Docusaurus content (tutorials, blog posts)
- Sets up project structure ready for GitHub Pages deployment (workflow setup handled separately with docusaurus-deployer skill)
- Creates a basic book structure ready for content addition

## Usage

Run the setup script with your book's information:

```bash
node .claude/skills/docusaurus-book-generator/scripts/setup-book.ts --title "Your Book Title" --tagline "Your custom tagline" --description "Your book description" --github-org "your-github-org" --github-repo "your-repo-name"
```

### Required Parameters

- `--title`: The main title of your book (required)

### Optional Parameters

- `--tagline`: Custom tagline for your book (auto-generated if not provided)
- `--description`: Custom description for your book (auto-generated if not provided)
- `--github-org`: GitHub organization name for deployment
- `--github-repo`: GitHub repository name for deployment
- `--output-dir`: Directory name for the website (default: "book-source")

### Example Usage

```bash
# Basic setup with just a title
node .claude/skills/docusaurus-book-generator/scripts/setup-book.ts --title "My Awesome Book"

# Complete setup with all parameters
node .claude/skills/docusaurus-book-generator/scripts/setup-book.ts --title "Physical AI & Humanoid Robotics" --tagline "Bridging Digital Minds to Physical Bodies" --description "A comprehensive guide to building intelligent robotic systems" --github-org "myorg" --github-repo "my-book-repo"
```

## What the Skill Does

1. Creates a new Docusaurus v3 project with TypeScript and Classic theme
2. Installs Tailwind CSS v4 with `@tailwindcss/postcss` plugin
3. **Configures PostCSS plugin directly in `docusaurus.config.ts`** (Tailwind v4 doesn't use separate config files)
4. Updates `src/css/custom.css` with `@import "tailwindcss";` syntax
5. Updates the Docusaurus configuration with your book's title, tagline, and metadata
6. Removes all default Docusaurus content (tutorials, blog posts)
7. Updates the homepage with your book's information
8. Sets up project structure ready for GitHub Pages deployment
9. Sets up a basic sidebar structure for your book
10. Updates the intro page with your book's information

## Tailwind CSS v4 Configuration

**IMPORTANT**: Tailwind CSS v4 has a completely different configuration approach than v3!

### What's Different in v4:
- ❌ **NO** `postcss.config.js` file needed
- ❌ **NO** `tailwind.config.js` file needed
- ✅ Configuration is done directly in `docusaurus.config.ts` using PostCSS plugin
- ✅ CSS uses `@import "tailwindcss";` instead of `@tailwind` directives

### docusaurus.config.ts
```typescript
const config: Config = {
  // ... other config ...

  // Enable PostCSS plugins for Tailwind CSS v4
  plugins: [
    function tailwindPlugin() {
      return {
        name: 'docusaurus-tailwindcss',
        configurePostCss(postcssOptions) {
          postcssOptions.plugins.push(require('@tailwindcss/postcss'));
          postcssOptions.plugins.push(require('autoprefixer'));
          return postcssOptions;
        },
      };
    },
  ],

  // ... rest of config ...
};
```

### src/css/custom.css
```css
@import "tailwindcss/theme" layer(theme);
@import "tailwindcss/utilities" layer(utilities);

/* Your custom styles... */
```

**IMPORTANT:** We import ONLY the theme (CSS variables) and utilities layers:
- ✅ `tailwindcss/theme` - Provides CSS custom properties for colors, spacing, etc.
- ✅ `tailwindcss/utilities` - Provides utility classes (bg-*, text-*, p-*, etc.)
- ❌ Skips `tailwindcss/base` - The base reset layer that would break Docusaurus/Infima

This prevents Tailwind from overriding Docusaurus's default styles, preserving:
- Layout and responsiveness
- Typography and spacing
- Button and component styles

## Output

After running the script, you'll have:

- A fully configured Docusaurus project in the specified output directory
- Tailwind CSS v4 configured and ready to use
- All default content removed
- Project structure ready for GitHub Pages deployment (use docusaurus-deployer skill for workflow setup)
- A basic book structure ready for content addition
- Proper metadata and SEO configuration

## Next Steps

1. Navigate to your project directory: `cd book-source` (or your specified output directory)
2. Start the development server: `npm run start`
3. Add your book content to the `docs/` directory
4. Update `sidebars.ts` to reflect your book's structure
5. Push to GitHub to trigger the automated deployment workflow (if configured)

## Troubleshooting

### Tailwind Classes Not Working
**Symptom**: Tailwind classes appear in HTML but no styling is applied
**Cause**: Tailwind v4 requires PostCSS plugin configuration in `docusaurus.config.ts`
**Solution**: This skill automatically adds the PostCSS plugin configuration. If you set it up manually, ensure the `plugins` array with `configurePostCss` is present in your config.

### CSS Import Error
**Error**: Unknown at-rule `@tailwind`
**Solution**: Use `@import "tailwindcss";` instead of old `@tailwind base/components/utilities` directives (Tailwind v4 syntax)

### Sidebar Empty Category Error
**Error**: `Sidebar category has neither any subitem nor a link`
**Solution**: Don't create empty sidebar categories - add items or remove the category

### Wrong Tailwind Version Installed
**Issue**: If you have Tailwind v3 patterns (config files) that aren't working
**Solution**: Make sure you're using Tailwind v4 (`tailwindcss@^4.0.0`) and follow the v4 configuration approach (no config files, use `@import` in CSS)
