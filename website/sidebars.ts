import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar for the Physical AI & Humanoid Robotics book
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/index',
        {
          type: 'category',
          label: 'Core Concepts',
          items: [
            'module1/core-concepts/index',
            'module1/core-concepts/pub-sub',
            'module1/core-concepts/parameters'
          ],
        },
        {
          type: 'category',
          label: 'AI Collaboration',
          items: [
            'module1/agent-bridge/index',
            'module1/agent-bridge/rclpy-intro',
            'module1/agent-bridge/launch-files'
          ],
        },
        {
          type: 'category',
          label: 'Component Design',
          items: [
            'module1/design/index',
            'module1/design/node-template',
            'module1/design/mini-controller-spec',
            'module1/design/mini-controller-impl'
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twins',
      items: ['module2/digital-twin'],
    },
    {
      type: 'category',
      label: 'Module 3: AI Reasoning',
      items: ['module3/ai-reasoning'],
    },
  ],
};

export default sidebars;
