// Basic service worker for caching static assets
const CACHE_NAME = 'ai-robotics-book-v1';
const urlsToCache = [
  '/',
  '/docs/intro',
  '/css/custom.css',
  '/js/pwa-sw-register.js',
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then((cache) => {
        console.log('Opened cache');
        return cache.addAll(urlsToCache);
      })
  );
});

self.addEventListener('fetch', (event) => {
  event.respondWith(
    caches.match(event.request)
      .then((response) => {
        // Return cached version if available, otherwise fetch from network
        if (response) {
          return response;
        }
        return fetch(event.request);
      }
    )
  );
});