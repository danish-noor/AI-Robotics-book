import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '2b9'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '98a'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '7f3'),
            routes: [
              {
                path: '/docs/capstone',
                component: ComponentCreator('/docs/capstone', '69d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '5ac'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1',
                component: ComponentCreator('/docs/module-1', 'b56'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2',
                component: ComponentCreator('/docs/module-2', 'f04'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3',
                component: ComponentCreator('/docs/module-3', '115'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4',
                component: ComponentCreator('/docs/module-4', '7af'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
