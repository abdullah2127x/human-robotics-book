import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
   title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging Digital Minds to Physical Bodies',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://abdullah2127x.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/human-robotics-book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'abdullah2127x', // Usually your GitHub org/user name.
  projectName: 'human-robotics-book', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Additional SEO and meta tags
  trailingSlash: false,
  noIndex: false,

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: false, // Disable blog functionality for the book
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    metadata: [
      {name: 'keywords', content: 'physical AI, humanoid robotics, artificial intelligence, robotics, machine learning, computer science, engineering, technology'},
      {name: 'description', content: 'Learn about Physical AI and Humanoid Robotics - bridging digital minds to physical bodies through advanced robotics and AI technologies'},
      {name: 'author', content: 'Physical AI & Humanoid Robotics Book'},
      {name: 'og:title', content: 'Physical AI & Humanoid Robotics'},
      {name: 'og:description', content: 'Bridging Digital Minds to Physical Bodies'},
      {name: 'og:type', content: 'website'},
      {name: 'og:url', content: 'https://abdullah2127x.github.io/human-robotics-book/'},
    ],
    // Performance and asset loading optimizations
    scripts: [
      {
        src: 'https://polyfill.io/v3/polyfill.min.js?features=es6',
        async: true,
        defer: true,
      },
    ],
    // Additional head tags for progressive enhancement
    headTags: [
      {
        tagName: 'noscript',
        innerHTML: '<style>.hero__title { font-size: 2rem; } .hero__subtitle { font-size: 1.2rem; } .button { display: inline-block; padding: 0.5rem 1rem; } .features { display: block; } .col { display: block; width: 100%; margin-bottom: 2rem; }</style>',
      },
    ],
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.svg',
        srcDark: 'img/logo_dark.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Modules',
        },
        {
          to: '/docs/intro',
          label: 'Start Reading',
          position: 'left',
        },
        {
          href: 'https://github.com/abdullah2127x/human-robotics-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'All Modules',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/abdullah2127x/human-robotics-book',
            },
            {
              label: 'Discord',
              href: 'https://discord.gg/robotics',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/abdullah2127x/human-robotics-book',
            },
            {
              label: 'Discord',
              href: 'https://discord.gg/robotics',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
