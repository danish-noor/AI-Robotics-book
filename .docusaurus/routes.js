import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/AI-Robotics-book/docs',
    component: ComponentCreator('/AI-Robotics-book/docs', '181'),
    routes: [
      {
        path: '/AI-Robotics-book/docs',
        component: ComponentCreator('/AI-Robotics-book/docs', 'd11'),
        routes: [
          {
            path: '/AI-Robotics-book/docs',
            component: ComponentCreator('/AI-Robotics-book/docs', '314'),
            routes: [
              {
                path: '/AI-Robotics-book/docs/capstone',
                component: ComponentCreator('/AI-Robotics-book/docs/capstone', 'a8d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI-Robotics-book/docs/intro',
                component: ComponentCreator('/AI-Robotics-book/docs/intro', '52e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI-Robotics-book/docs/module-1',
                component: ComponentCreator('/AI-Robotics-book/docs/module-1', 'e23'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI-Robotics-book/docs/module-2',
                component: ComponentCreator('/AI-Robotics-book/docs/module-2', 'bce'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI-Robotics-book/docs/module-3',
                component: ComponentCreator('/AI-Robotics-book/docs/module-3', '1de'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI-Robotics-book/docs/module-4',
                component: ComponentCreator('/AI-Robotics-book/docs/module-4', '784'),
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
    path: '/AI-Robotics-book/',
    component: ComponentCreator('/AI-Robotics-book/', 'dac'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
