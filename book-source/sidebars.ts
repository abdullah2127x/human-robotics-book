import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

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
