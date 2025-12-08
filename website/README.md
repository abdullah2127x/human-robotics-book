# Website

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

## Features

The Physical AI & Humanoid Robotics website includes:

- **Brand Identity**: Custom SVG logo featuring a fusion of brain and gear elements representing the connection between AI and robotics
- **Color Scheme**: Tech-Blue primary color with complementary brand colors
- **Responsive Design**: Fully responsive layout that works on mobile, tablet, and desktop devices
- **Dark Mode**: Full dark mode support with optimized color schemes for both light and dark themes
- **Accessibility**: ARIA attributes and semantic HTML for improved accessibility
- **SEO Optimized**: Meta tags, structured data, and optimized content for search engines

## Installation

```bash
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true npm run deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

## User Feedback

We welcome feedback on the Physical AI & Humanoid Robotics website. If you have suggestions for improvements or encounter any issues:

1. **Usability Testing**: If you're part of our target audience (students, researchers, or professionals in AI/Robotics), we'd appreciate your feedback on the clarity and usability of the content.

2. **Accessibility**: Please report any accessibility issues you encounter while using the site.

3. **Browser Compatibility**: Let us know if you experience issues on specific browsers or devices.

To provide feedback, please create an issue in the GitHub repository or contact the development team.
