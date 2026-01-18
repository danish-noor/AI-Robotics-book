// // @ts-check
// import {themes as prismThemes} from 'prism-react-renderer';

// const isProd = process.env.NODE_ENV === 'production';

// /** @type {import('@docusaurus/types').Config} */
// const config = {
//   title: 'AI Robotics Book',
//   tagline: 'A Comprehensive Curriculum on AI Robotics',
//   favicon: 'img/favicon.ico',

//   // Set the production url of your site here
//   url: 'https://danish-noor.github.io',
//   // Set the /<baseUrl>/ pathname under which your site is served
//   // For GitHub pages deployment, it is often '/<projectName>/'
//   baseUrl: isProd ? '/AI-Robotics-book/' : '/',

//   // GitHub pages deployment config.
//   organizationName: 'danish-noor', // Usually your GitHub org/user name.
//   projectName: 'AI-Robotics-book', // Usually your repo name.
//   trailingSlash: false,

//   // GitHub Pages configuration
//   // deploymentBranch: 'gh-pages',

//   onBrokenLinks: 'throw',
//   onBrokenMarkdownLinks: 'warn',

//   // Even if you don't use internationalization, you can use this field to set
//   // useful metadata like html lang. For example, if your site is Chinese, you
//   // may want to replace "en" with "zh-Hans".
//   i18n: {
//     defaultLocale: 'en',
//     locales: ['en'],
//   },

//   presets: [
//     [
//       'classic',
//       /** @type {import('@docusaurus/preset-classic').Options} */
//       ({
//         docs: {
//           sidebarPath: './sidebars.js',
//           // Please change this to your repo.
//           // Remove this to remove the "edit this page" links.
//           editUrl:
//             'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
//           // Performance optimizations
//           routeBasePath: '/', // Serve docs at the root
//           showLastUpdateTime: true,
//           editCurrentVersion: true,
//         },
//         blog: false, // Disable blog if not needed
//         theme: {
//           customCss: './src/css/custom.css',
//         },
//         gtag: {
//           trackingID: 'G-XXXXXXXXXX',
//           anonymizeIP: true,
//         },
//       }),
//     ],
//   ],

//   plugins: [
//     [
//       '@docusaurus/plugin-content-docs',
//       {
//         id: 'community',
//         path: 'community',
//         routeBasePath: 'community',
//         sidebarPath: 'sidebarsCommunity.js',
//       },
//     ],
//     [
//       '@docusaurus/plugin-client-redirects',
//       {
//         redirects: [
//           {
//             to: '/docs/intro',
//             from: ['/docs', '/welcome'],
//           },
//         ],
//       },
//     ],
//     // PWA Plugin
//     [
//       '@docusaurus/plugin-pwa',
//       {
//         debug: false,
//         offlineModeActivationStrategies: [
//           'appInstalled',
//           'standalone',
//           'queryString',
//         ],
//         pwaHead: [
//           {
//             tagName: 'link',
//             rel: 'icon',
//             href: '/img/pwa/icon-192x192.png',
//           },
//           {
//             tagName: 'link',
//             rel: 'manifest',
//             href: '/manifest.json',
//           },
//           {
//             tagName: 'meta',
//             name: 'theme-color',
//             content: '#2a6f97',
//           },
//           {
//             tagName: 'meta',
//             name: 'apple-mobile-web-app-capable',
//             content: 'yes',
//           },
//           {
//             tagName: 'meta',
//             name: 'apple-mobile-web-app-status-bar-style',
//             content: '#2a6f97',
//           },
//           {
//             tagName: 'link',
//             rel: 'apple-touch-icon',
//             href: '/img/pwa/icon-192x192.png',
//           },
//           {
//             tagName: 'link',
//             rel: 'mask-icon',
//             href: '/img/pwa/icon-192x192.png',
//             color: '#2a6f97',
//           },
//           {
//             tagName: 'meta',
//             name: 'msapplication-TileImage',
//             content: '/img/pwa/icon-192x192.png',
//           },
//           {
//             tagName: 'meta',
//             name: 'msapplication-TileColor',
//             content: '#2a6f97',
//           },
//         ],
//       },
//     ],
//     // Client redirects for better UX
    
//   ],

//   themeConfig:
//     /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
//     ({
//       // Replace with your project's social card
//       image: 'img/docusaurus-social-card.jpg',
//       navbar: {
//         title: 'AI Robotics Book',
//         logo: {
//           alt: 'AI Robotics Book Logo',
//           src: 'img/ai_robotics_book_logo.svg',
//         },
//         items: [
//           {
//             type: 'docSidebar',
//             sidebarId: 'tutorialSidebar',
//             position: 'left',
//             label: 'Curriculum',
//           },
//           {
//             type: 'dropdown',
//             label: 'Modules',
//             position: 'left',
//             items: [
//               {
//                 label: 'Module 1: ROS 2',
//                 to: '/docs/module-1',
//               },
//               {
//                 label: 'Module 2: Simulation',
//                 to: '/docs/module-2',
//               },
//               {
//                 label: 'Module 3: NVIDIA Isaac',
//                 to: '/docs/module-3',
//               },
//               {
//                 label: 'Module 4: VLA Models',
//                 to: '/docs/module-4',
//               },
//             ],
//           },
//           {
//             type: 'search',
//             position: 'right',
//           },
//           {
//             href: 'https://github.com/facebook/docusaurus',
//             label: 'GitHub',
//             position: 'right',
//           },
//         ],
//       },
//       footer: {
//         style: 'dark',
//         links: [
//           {
//             title: 'Modules',
//             items: [
//               {
//                 label: 'Module 1: ROS 2',
//                 to: '/docs/module-1',
//               },
//               {
//                 label: 'Module 2: Simulation',
//                 to: '/docs/module-2',
//               },
//               {
//                 label: 'Module 3: NVIDIA Isaac',
//                 to: '/docs/module-3',
//               },
//               {
//                 label: 'Module 4: VLA Models',
//                 to: '/docs/module-4',
//               },
//             ],
//           },
//           {
//             title: 'Resources',
//             items: [
//               {
//                 label: 'GitHub',
//                 href: 'https://github.com/facebook/docusaurus',
//               },
//               {
//                 label: 'Documentation',
//                 to: '/docs/intro',
//               },
//             ],
//           },
//           {
//             title: 'Community',
//             items: [
//               {
//                 label: 'Stack Overflow',
//                 href: 'https://stackoverflow.com/questions/tagged/docusaurus',
//               },
//               {
//                 label: 'Discord',
//                 href: 'https://discordapp.com/invite/docusaurus',
//               },
//             ],
//           },
//         ],
//         copyright: `Copyright © ${new Date().getFullYear()} AI Robotics Book. Built by DANISH NOOR.`,
//       },
//       prism: {
//         theme: prismThemes.github,
//         darkTheme: prismThemes.dracula,
//       },
//       algolia: undefined, // Disable Algolia, using built-in search
//       colorMode: {
//         defaultMode: 'light',
//         disableSwitch: false,
//         respectPrefersColorScheme: true,
//       },
//       // Performance optimizations
//       metadata: [
//         {name: 'keywords', content: 'AI, Robotics, Curriculum, Education, Technology'},
//         {name: 'twitter:card', content: 'summary_large_image'},
//       ],
//     }),
// };

// export default config;

// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';

const isProd = process.env.NODE_ENV === 'production';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AI Robotics Book',
  tagline: 'A Comprehensive Curriculum on AI Robotics',
  favicon: 'img/favicon.ico',

  // ✅ GitHub Pages main URL
  url: 'https://danish-noor.github.io',

  // ✅ DEV vs PROD baseUrl (MOST IMPORTANT FIX)
  baseUrl: isProd ? '/AI-Robotics-book/' : '/',

  organizationName: 'danish-noor',
  projectName: 'AI-Robotics-book',

  trailingSlash: false,

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
          showLastUpdateTime: true,
          editCurrentVersion: true,
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'community',
        path: 'community',
        routeBasePath: 'community',
        sidebarPath: 'sidebarsCommunity.js',
      },
    ],

    // ✅ SINGLE redirect plugin (duplicate removed)
    [
      '@docusaurus/plugin-client-redirects',
      {
        redirects: [
          { from: ['/docs', '/welcome'], to: '/docs/intro' },
        ],
      },
    ],

    // ✅ PWA plugin (safe with baseUrl)
    [
      '@docusaurus/plugin-pwa',
      {
        debug: false,
        offlineModeActivationStrategies: [
          'appInstalled',
          'standalone',
          'queryString',
        ],
        pwaHead: [
          { tagName: 'link', rel: 'icon', href: '/img/pwa/icon-192x192.png' },
          { tagName: 'link', rel: 'manifest', href: '/manifest.json' },
          { tagName: 'meta', name: 'theme-color', content: '#2a6f97' },
        ],
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',

    navbar: {
      title: 'AI Robotics Book',
      logo: {
        alt: 'AI Robotics Book Logo',
        src: 'img/ai_robotics_book_logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          label: 'Curriculum',
          position: 'left',
        },
        {
          type: 'dropdown',
          label: 'Modules',
          position: 'left',
          items: [
            { label: 'Module 1: ROS 2', to: '/docs/module-1' },
            { label: 'Module 2: Simulation', to: '/docs/module-2' },
            { label: 'Module 3: NVIDIA Isaac', to: '/docs/module-3' },
            { label: 'Module 4: VLA Models', to: '/docs/module-4' },
          ],
        },
        { type: 'search', position: 'right' },
        {
          href: 'https://github.com/danish-noor/AI-Robotics-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      copyright: `Copyright © ${new Date().getFullYear()} AI Robotics Book. Built by Danish Noor.`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },

    // ✅ Algolia disabled (Chunk error ka reason nahi banega)
    algolia: undefined,

    colorMode: {
      defaultMode: 'light',
      respectPrefersColorScheme: true,
    },
  },
};

export default config;
