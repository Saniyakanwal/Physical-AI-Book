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
  // Manual sidebar for the Physical AI & Humanoid Robotics textbook
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Modules',
      items: [
        {
          type: 'category',
          label: 'ROS2',
          items: [
            'modules/ros2/intro',
            'modules/ros2/basics',
            'modules/ros2/advanced',
            'modules/ros2/practical-examples',
            'modules/ros2/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Gazebo/Unity',
          items: [
            'modules/gazebo-unity/intro',
            'modules/gazebo-unity/basics',
            'modules/gazebo-unity/advanced',
            'modules/gazebo-unity/practical-examples',
            'modules/gazebo-unity/exercises',
          ],
        },
        {
          type: 'category',
          label: 'NVIDIA Isaac',
          items: [
            'modules/nvidia-isaac/intro',
            'modules/nvidia-isaac/basics',
            'modules/nvidia-isaac/advanced',
            'modules/nvidia-isaac/practical-examples',
            'modules/nvidia-isaac/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Vision-Language-Action',
          items: [
            'modules/vision-language-action/intro',
            'modules/vision-language-action/basics',
            'modules/vision-language-action/advanced',
            'modules/vision-language-action/practical-examples',
            'modules/vision-language-action/exercises',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Weekly Breakdown',
      items: [
        'weeks/week-1',
        'weeks/week-2',
        'weeks/week-3',
        'weeks/week-4',
        'weeks/week-5',
        'weeks/week-6',
        'weeks/week-7',
        'weeks/week-8',
        'weeks/week-9',
        'weeks/week-10',
        'weeks/week-11',
        'weeks/week-12',
        'weeks/week-13',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: ['capstone/intro', 'capstone/project-ideas', 'capstone/submission-guidelines'],
    },
    {
      type: 'category',
      label: 'Hardware Requirements',
      items: ['hardware/intro', 'hardware/components', 'hardware/setup-guide', 'hardware/troubleshooting'],
    },
  ],
};

export default sidebars;
