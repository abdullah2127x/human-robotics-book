# Quickstart Guide: UI & Identity Setup

## Prerequisites

- Node.js 20.x or higher
- npm or yarn package manager
- Git for version control

## Setup Process

### 1. Clone the Repository
```bash
git clone https://github.com/abdullah2127x/human-robotics-book.git
cd human-robotics-book
```

### 2. Navigate to Website Directory
```bash
cd website
```

### 3. Install Dependencies
```bash
npm install
```

### 4. Create Brand Assets
Create the following files in the `static/img/` directory:
- `logo.svg` - Main logo with brain and gear fusion design
- `logo_dark.svg` - Dark theme variant of the logo
- `favicon.png` - Favicon in PNG format

### 5. Update Brand Colors
In `src/css/custom.css`, define the Tech-Blue primary color:
```css
:root {
  --ifm-color-primary: #007bff;  /* Tech-Blue example */
  --ifm-color-primary-dark: #0069d9;
  --ifm-color-primary-darker: #0062cc;
  --ifm-color-primary-darkest: #0051a8;
  --ifm-color-primary-light: #3395ff;
  --ifm-color-primary-lighter: #4da3ff;
  --ifm-color-primary-lightest: #80c4ff;
}
```

### 6. Customize Homepage
Replace the content in `src/pages/index.tsx` with the custom homepage that includes:
- Hero section with title "Physical AI & Humanoid Robotics."
- Tagline "Bridging Digital Minds to Physical Bodies."
- Primary CTA button "Start Learning (Module 1)"
- Secondary CTA button "Chat with Book (Placeholder)"
- Three value proposition blocks

### 7. Configure Footer Links
In `docusaurus.config.ts`, update the footer.links section with:
- Docs: Links to all modules
- Community: GitHub/Discord links
- More: Additional resources

## Running the Development Server

```bash
npm start
```

This will start the development server at http://localhost:3000

## Building for Production

```bash
npm run build
```

This creates a `build/` directory with the static site ready for deployment.

## Testing the Implementation

1. Verify all branding elements appear correctly
2. Check that both light and dark mode logos display properly
3. Confirm the primary brand color is applied throughout the site
4. Test that all CTAs function as expected
5. Verify footer navigation works correctly
6. Ensure the site is responsive across different devices