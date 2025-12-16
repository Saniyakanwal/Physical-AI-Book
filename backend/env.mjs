/// <reference types="next" />
/// <reference types="next/image-types/global" />

// NOTE: This file should not be version controlled since it contains sensitive data
//       Instead, use environment variables on your hosting platform or CI/CD pipeline

/** @type {import('next').NextConfig} */
const nextConfig = {
  experimental: {
    typedRoutes: true,
  },
  env: {
    DATABASE_URL: process.env.DATABASE_URL,
    AUTH_SECRET: process.env.AUTH_SECRET,
  },
};

export default nextConfig;