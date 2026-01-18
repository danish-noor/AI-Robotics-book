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
