import { getServerSession } from 'better-auth/server-actions';
import Link from 'next/link';

export default async function Home() {
  const session = await getServerSession();

  return (
    <div className="max-w-7xl mx-auto py-12 px-4 sm:px-6 lg:px-8">
      <div className="text-center">
        <h1 className="text-4xl font-extrabold text-gray-900 sm:text-5xl sm:tracking-tight lg:text-6xl">
          Welcome to Better-Auth Demo
        </h1>
        <p className="mt-6 max-w-lg mx-auto text-xl text-gray-500">
          A Next.js 15 application with Better-Auth and Neon Postgres integration
        </p>
        
        {!session ? (
          <div className="mt-10">
            <Link
              href="/signup"
              className="inline-block bg-indigo-600 text-white px-6 py-3 rounded-md text-base font-medium hover:bg-indigo-700"
            >
              Get Started
            </Link>
          </div>
        ) : (
          <div className="mt-10">
            <p className="text-gray-600">Welcome back, {session.user.name || session.user.email}!</p>
            <Link
              href="/dashboard"
              className="mt-4 inline-block bg-indigo-600 text-white px-6 py-3 rounded-md text-base font-medium hover:bg-indigo-700"
            >
              View Dashboard
            </Link>
          </div>
        )}
      </div>
    </div>
  );
}