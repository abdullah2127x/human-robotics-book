#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import { execSync } from 'child_process';

/**
 * Docusaurus Book Generator Script
 *
 * This script automates the setup of a Docusaurus book website with:
 * - Custom title, tagline, and metadata
 * - Tailwind CSS v4 configuration (with @tailwindcss/postcss)
 * - Removal of default content
 * - GitHub Actions workflow for GitHub Pages
 */

interface BookConfig {
  title: string;
  tagline?: string;
  description?: string;
  githubOrg?: string;
  githubRepo?: string;
  outputDir?: string;
}

function generateTagline(title: string): string {
  return `Exploring ${title}`;
}

function generateDescription(title: string): string {
  return `Learn about ${title} - comprehensive guide and documentation`;
}

function generateKeywords(title: string): string {
  return title.toLowerCase().replace(/\s+/g, ', ') + ', documentation, guide, tutorial';
}

function escapeForRegex(str: string): string {
  return str.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
}

function escapeForTemplate(str: string): string {
  return str.replace(/\\/g, '\\\\').replace(/'/g, "\\'").replace(/`/g, '\\`').replace(/\$/g, '\\$');
}

function createDocusaurusProject(outputDir: string): void {
  console.log('Creating Docusaurus project...');
  execSync(`npx create-docusaurus@latest ${outputDir} classic --typescript`, { stdio: 'inherit' });
}

function installTailwind(outputDir: string): void {
  console.log('Installing Tailwind CSS v4...');
  const cwd = process.cwd();
  process.chdir(outputDir);

  // Install Tailwind CSS v4 with @tailwindcss/postcss plugin
  execSync('npm install -D tailwindcss @tailwindcss/postcss autoprefixer', { stdio: 'inherit' });

  process.chdir(cwd);
}

function configureTailwind(outputDir: string): void {
  console.log('Configuring Tailwind CSS v4...');

  // Tailwind v4 doesn't use tailwind.config.js or postcss.config.js
  // It's configured directly via CSS and Docusaurus PostCSS plugin

  // Update src/css/custom.css to use Tailwind v4 import syntax
  const customCssPath = path.join(outputDir, 'src', 'css', 'custom.css');
  if (fs.existsSync(customCssPath)) {
    let customCss = fs.readFileSync(customCssPath, 'utf8');
    // Add Tailwind v4 import at the top (theme + utilities only, skip base layer)
    if (!customCss.includes('@import "tailwindcss')) {
      customCss = '@import "tailwindcss/theme" layer(theme);\n@import "tailwindcss/utilities" layer(utilities);\n\n' + customCss;
      fs.writeFileSync(customCssPath, customCss);
    }
  }
}

function updateDocusaurusConfig(outputDir: string, config: BookConfig): void {
  console.log('Updating Docusaurus configuration...');

  const docusaurusConfigPath = path.join(outputDir, 'docusaurus.config.ts');

  // Read the existing config
  let configContent = fs.readFileSync(docusaurusConfigPath, 'utf8');

  // Add PostCSS plugin configuration for Tailwind v4
  // Insert after the config constant declaration
  const pluginsConfig = `
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
`;

  // Insert plugins configuration after the future block if it exists, or after favicon
  // Use a more specific regex that handles nested braces
  if (configContent.includes('future:')) {
    // Match the entire future block with nested braces
    configContent = configContent.replace(
      /(future:\s*\{[\s\S]*?\},)/,
      `$1${pluginsConfig}`
    );
  } else {
    configContent = configContent.replace(
      /(favicon:\s*'[^']*',)/,
      `$1${pluginsConfig}`
    );
  }

  // Replace title, tagline, and other metadata at the top level
  // Use more specific regex to target the config object properties
  // Escape single quotes in user input to prevent breaking the config
  const safeTitle = config.title.replace(/'/g, "\\'");
  const safeTagline = (config.tagline || generateTagline(config.title)).replace(/'/g, "\\'");

  configContent = configContent.replace(
    /(\nconst config[^{]*\{[\s\S]*?)\btitle:\s*'[^']*',/,
    `$1title: '${safeTitle}',`
  );

  configContent = configContent.replace(
    /(\nconst config[^{]*\{[\s\S]*?)\btagline:\s*'[^']*',/,
    `$1tagline: '${safeTagline}',`
  );

  // Update URL and base URL for GitHub Pages
  if (config.githubOrg && config.githubRepo) {
    configContent = configContent.replace(
      /\burl:\s*'[^']*',/,
      `url: 'https://${config.githubOrg}.github.io',`
    );

    configContent = configContent.replace(
      /\bbaseUrl:\s*'[^']*',/,
      `baseUrl: '/${config.githubRepo}/',`
    );

    // Update organization and project names
    configContent = configContent.replace(
      /\borganizationName:\s*'[^']*',/,
      `organizationName: '${config.githubOrg}',`
    );

    configContent = configContent.replace(
      /\bprojectName:\s*'[^']*',/,
      `projectName: '${config.githubRepo}',`
    );
  }

  // Update navbar title specifically in the themeConfig
  configContent = configContent.replace(
    /(themeConfig:[\s\S]*?navbar:\s*\{[\s\S]*?)\btitle:\s*'[^']*',/,
    `$1title: '${safeTitle}',`
  );

  // Update description in metadata
  if (config.description) {
    configContent = configContent.replace(
      /content: 'Learn .* Docusaurus'/,
      `content: '${config.description}'`
    );
  }

  // Disable blog if not needed for a book
  // Use a more robust regex that handles nested objects
  configContent = configContent.replace(
    /blog:\s*\{[\s\S]*?\n\s*\},/,
    'blog: false, // Disable blog functionality for the book'
  );

  // Update edit URL if GitHub repo is provided
  if (config.githubOrg && config.githubRepo) {
    configContent = configContent.replace(
      /editUrl:\s*'https:\/\/github\.com\/.*',/g,
      `          editUrl:\n            'https://github.com/${config.githubOrg}/${config.githubRepo}/edit/main/${outputDir}/docs/',`
    );
  }

  // Fix sidebar reference in navbar (tutorial -> book)
  configContent = configContent.replace(
    /sidebarId:\s*'tutorialSidebar'/,
    "sidebarId: 'bookSidebar'"
  );

  // Change navbar label from Tutorial to Book
  configContent = configContent.replace(
    /(type:\s*'docSidebar',[\s\S]*?sidebarId:\s*'bookSidebar'[\s\S]*?label:\s*)'Tutorial'/,
    "$1'Book'"
  );

  // Remove blog link from navbar
  configContent = configContent.replace(
    /\{to:\s*'\/blog',\s*label:\s*'Blog',\s*position:\s*'left'\},?\n?\s*/,
    ''
  );

  // Remove blog link from footer
  configContent = configContent.replace(
    /\{\s*label:\s*'Blog',\s*to:\s*'\/blog',?\s*\},?\n?\s*/,
    ''
  );

  fs.writeFileSync(docusaurusConfigPath, configContent);
}

function updateIntroPage(outputDir: string, config: BookConfig): void {
  console.log('Updating intro page...');

  const introPath = path.join(outputDir, 'docs', 'intro.md');
  if (fs.existsSync(introPath)) {
    const introContent = `---
sidebar_position: 1
title: Introduction
description: Introduction to ${config.title}
---

# Introduction

Welcome to ${config.title}. This introduction provides an overview of the concepts and modules you'll encounter throughout this book.
`;
    fs.writeFileSync(introPath, introContent);
  }
}

function updateHomepage(outputDir: string, config: BookConfig): void {
  console.log('Updating homepage...');

  const homepagePath = path.join(outputDir, 'src', 'pages', 'index.tsx');
  if (fs.existsSync(homepagePath)) {
    const homepageContent = `import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

type FeatureItem = {
  title: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Core Concepts',
    description: (
      <>
        Learn the fundamental concepts of ${config.title}
      </>
    ),
  },
  {
    title: 'Practical Applications',
    description: (
      <>
        Apply concepts through hands-on exercises and examples
      </>
    ),
  },
  {
    title: 'Advanced Topics',
    description: (
      <>
        Explore advanced topics and current research
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md padding-vert--lg">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Welcome"
      description="${config.description || generateDescription(config.title)}">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
`;
    fs.writeFileSync(homepagePath, homepageContent);
  }

  // Update index.module.css to include features style
  const cssPath = path.join(outputDir, 'src', 'pages', 'index.module.css');
  if (fs.existsSync(cssPath)) {
    let cssContent = fs.readFileSync(cssPath, 'utf8');
    if (!cssContent.includes('.features')) {
      cssContent += `
.features {
  display: flex;
  align-items: center;
  padding: 2rem 0;
  width: 100%;
}
`;
      fs.writeFileSync(cssPath, cssContent);
    }
  }
}

function removeDefaultContent(outputDir: string): void {
  console.log('Removing default content...');

  // Remove tutorial-basics and tutorial-extras directories
  const tutorialBasicsPath = path.join(outputDir, 'docs', 'tutorial-basics');
  const tutorialExtrasPath = path.join(outputDir, 'docs', 'tutorial-extras');

  if (fs.existsSync(tutorialBasicsPath)) {
    fs.rmSync(tutorialBasicsPath, { recursive: true, force: true });
  }

  if (fs.existsSync(tutorialExtrasPath)) {
    fs.rmSync(tutorialExtrasPath, { recursive: true, force: true });
  }

  // Remove blog directory if we're treating this as a book
  const blogPath = path.join(outputDir, 'blog');
  if (fs.existsSync(blogPath)) {
    fs.rmSync(blogPath, { recursive: true, force: true });
  }

  // Remove default HomepageFeatures component (we create our own in index.tsx)
  const homepageFeaturesPath = path.join(outputDir, 'src', 'components', 'HomepageFeatures');
  if (fs.existsSync(homepageFeaturesPath)) {
    fs.rmSync(homepageFeaturesPath, { recursive: true, force: true });
  }
}


function fixTsConfig(outputDir: string): void {
  console.log('Fixing TypeScript configuration...');

  const tsconfigPath = path.join(outputDir, 'tsconfig.json');
  if (fs.existsSync(tsconfigPath)) {
    let tsconfigContent = fs.readFileSync(tsconfigPath, 'utf8');

    // Add ignoreDeprecations to silence baseUrl deprecation warning in TypeScript 5.x+
    // The baseUrl option is deprecated and will stop functioning in TypeScript 7.0
    if (!tsconfigContent.includes('ignoreDeprecations')) {
      // Insert ignoreDeprecations after baseUrl or jsx
      if (tsconfigContent.includes('"jsx"')) {
        tsconfigContent = tsconfigContent.replace(
          /("jsx":\s*"[^"]*")/,
          '$1,\n    "ignoreDeprecations": "6.0"'
        );
      } else if (tsconfigContent.includes('"baseUrl"')) {
        tsconfigContent = tsconfigContent.replace(
          /("baseUrl":\s*"[^"]*")/,
          '$1,\n    "ignoreDeprecations": "6.0"'
        );
      }
      fs.writeFileSync(tsconfigPath, tsconfigContent);
    }
  }
}

function updateSidebar(outputDir: string): void {
  console.log('Updating sidebar configuration...');

  const sidebarPath = path.join(outputDir, 'sidebars.ts');
  if (fs.existsSync(sidebarPath)) {
    // Simple sidebar that won't cause empty category errors
    const sidebarContent = `import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Book sidebar configuration
 * Add chapters as folders and documents in the docs/ directory.
 */
const sidebars: SidebarsConfig = {
  bookSidebar: [
    'intro',
    // Add more chapters here as you create content
  ],
};

export default sidebars;
`;
    fs.writeFileSync(sidebarPath, sidebarContent);
  }
}

function main(): void {
  // Get command line arguments
  const args = process.argv.slice(2);

  if (args.length === 0) {
    console.error('Usage: node setup-book.js --title "Your Book Title" [--tagline "Your tagline"] [--description "Your description"] [--github-org "your-org"] [--github-repo "your-repo"]');
    process.exit(1);
  }

  const config: BookConfig = { title: '' };

  for (let i = 0; i < args.length; i += 2) {
    const flag = args[i];
    const value = args[i + 1];

    switch (flag) {
      case '--title':
        config.title = value;
        break;
      case '--tagline':
        config.tagline = value;
        break;
      case '--description':
        config.description = value;
        break;
      case '--github-org':
        config.githubOrg = value;
        break;
      case '--github-repo':
        config.githubRepo = value;
        break;
      case '--output-dir':
        config.outputDir = value;
        break;
      default:
        console.error(`Unknown flag: ${flag}`);
        process.exit(1);
    }
  }

  if (!config.title) {
    console.error('Title is required');
    process.exit(1);
  }

  const outputDir = config.outputDir || 'book-source';

  console.log(`Setting up Docusaurus book: ${config.title}`);

  // Create the Docusaurus project
  createDocusaurusProject(outputDir);

  // Install and configure Tailwind CSS v4
  installTailwind(outputDir);
  configureTailwind(outputDir);

  // Update configurations
  updateDocusaurusConfig(outputDir, config);
  updateIntroPage(outputDir, config);
  updateHomepage(outputDir, config);
  updateSidebar(outputDir);
  fixTsConfig(outputDir);

  // Remove default content
  removeDefaultContent(outputDir);

  // GitHub Actions workflow will be set up separately using docusaurus-deployer skill
  console.log('Note: GitHub Actions workflow setup will be handled separately using docusaurus-deployer skill');

  console.log('\nDocusaurus book setup complete!');
  console.log(`Project created in: ${outputDir}`);
  console.log('Next steps:');
  console.log(`1. cd ${outputDir}`);
  console.log('2. npm run start # to start the development server');
  console.log('3. Add your book content to the docs/ directory');
  console.log('4. Update sidebars.ts to reflect your book structure');
}

main();
