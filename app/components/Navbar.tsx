'use client';

import Link from 'next/link';
import { useSession } from 'better-auth/react';
import { UserBadge } from './UserBadge';

export default function Navbar() {
  const { data: session } = useSession();

  return (
    <nav className="bg-white shadow-sm border-b">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <div className="flex justify-between h-16">
          <div className="flex items-center">
            <Link href="/" className="text-xl font-bold text-indigo-600">
              Better-Auth Demo
            </Link>
          </div>
          <div className="flex items-center space-x-4">
            {!session ? (
              <>
                <Link
                  href="/login"
                  className="text-gray-700 hover:text-indigo-600 px-3 py-2 rounded-md text-sm font-medium"
                >
                  Login
                </Link>
                <Link
                  href="/signup"
                  className="bg-indigo-600 text-white px-4 py-2 rounded-md text-sm font-medium hover:bg-indigo-700"
                >
                  Sign Up
                </Link>
              </>
            ) : (
              <UserBadge user={session.user} />
            )}
          </div>
        </div>
      </div>
    </nav>
  );
}