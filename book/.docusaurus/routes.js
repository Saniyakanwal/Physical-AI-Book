import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '112'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '969'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', '3f8'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'ab6'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '725'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '4b3'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '1da'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'f99'),
    routes: [
      {
        path: '/docs/',
        component: ComponentCreator('/docs/', 'b00'),
        exact: true
      },
      {
        path: '/docs/',
        component: ComponentCreator('/docs/', 'da5'),
        exact: true
      },
      {
        path: '/docs/capstone-project/',
        component: ComponentCreator('/docs/capstone-project/', '342'),
        exact: true,
        sidebar: "tutorialsSidebar"
      },
      {
        path: '/docs/gazebo-simulation/',
        component: ComponentCreator('/docs/gazebo-simulation/', '97c'),
        exact: true,
        sidebar: "tutorialsSidebar"
      },
      {
        path: '/docs/humanoid-locomotion/',
        component: ComponentCreator('/docs/humanoid-locomotion/', 'ff2'),
        exact: true,
        sidebar: "tutorialsSidebar"
      },
      {
        path: '/docs/installation',
        component: ComponentCreator('/docs/installation', 'b2a'),
        exact: true
      },
      {
        path: '/docs/introduction-to-physical-ai/',
        component: ComponentCreator('/docs/introduction-to-physical-ai/', '466'),
        exact: true,
        sidebar: "tutorialsSidebar"
      },
      {
        path: '/docs/local-development',
        component: ComponentCreator('/docs/local-development', '6b5'),
        exact: true
      },
      {
        path: '/docs/nvidia-isaac/',
        component: ComponentCreator('/docs/nvidia-isaac/', '61b'),
        exact: true,
        sidebar: "tutorialsSidebar"
      },
      {
        path: '/docs/quickstart',
        component: ComponentCreator('/docs/quickstart', 'a4b'),
        exact: true
      },
      {
        path: '/docs/ros-2-fundamentals/',
        component: ComponentCreator('/docs/ros-2-fundamentals/', '0cd'),
        exact: true,
        sidebar: "tutorialsSidebar"
      },
      {
        path: '/docs/unity-for-robotics/',
        component: ComponentCreator('/docs/unity-for-robotics/', '91e'),
        exact: true,
        sidebar: "tutorialsSidebar"
      },
      {
        path: '/docs/vision-language-action/',
        component: ComponentCreator('/docs/vision-language-action/', '1ed'),
        exact: true,
        sidebar: "tutorialsSidebar"
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', 'd5b'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
