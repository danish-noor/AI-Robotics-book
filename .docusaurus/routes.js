import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai-book/__docusaurus/debug',
    component: ComponentCreator('/ai-book/__docusaurus/debug', 'c7e'),
    exact: true
  },
  {
    path: '/ai-book/__docusaurus/debug/config',
    component: ComponentCreator('/ai-book/__docusaurus/debug/config', 'ba6'),
    exact: true
  },
  {
    path: '/ai-book/__docusaurus/debug/content',
    component: ComponentCreator('/ai-book/__docusaurus/debug/content', '6fa'),
    exact: true
  },
  {
    path: '/ai-book/__docusaurus/debug/globalData',
    component: ComponentCreator('/ai-book/__docusaurus/debug/globalData', '2a2'),
    exact: true
  },
  {
    path: '/ai-book/__docusaurus/debug/metadata',
    component: ComponentCreator('/ai-book/__docusaurus/debug/metadata', '8ba'),
    exact: true
  },
  {
    path: '/ai-book/__docusaurus/debug/registry',
    component: ComponentCreator('/ai-book/__docusaurus/debug/registry', '099'),
    exact: true
  },
  {
    path: '/ai-book/__docusaurus/debug/routes',
    component: ComponentCreator('/ai-book/__docusaurus/debug/routes', 'd85'),
    exact: true
  },
  {
    path: '/ai-book/search',
    component: ComponentCreator('/ai-book/search', '6a3'),
    exact: true
  },
  {
    path: '/ai-book/docs',
    component: ComponentCreator('/ai-book/docs', 'f78'),
    routes: [
      {
        path: '/ai-book/docs',
        component: ComponentCreator('/ai-book/docs', '9ee'),
        routes: [
          {
            path: '/ai-book/docs',
            component: ComponentCreator('/ai-book/docs', 'e49'),
            routes: [
              {
                path: '/ai-book/docs/capstone/',
                component: ComponentCreator('/ai-book/docs/capstone/', 'd0e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book/docs/intro',
                component: ComponentCreator('/ai-book/docs/intro', '966'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book/docs/module-1/',
                component: ComponentCreator('/ai-book/docs/module-1/', '09c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book/docs/module-2/',
                component: ComponentCreator('/ai-book/docs/module-2/', '507'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book/docs/module-3/',
                component: ComponentCreator('/ai-book/docs/module-3/', '87a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-book/docs/module-4/',
                component: ComponentCreator('/ai-book/docs/module-4/', 'da0'),
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
    path: '/ai-book/',
    component: ComponentCreator('/ai-book/', '9e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
