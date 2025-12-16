'use client';

import { signOut } from 'better-auth/client/actions';
import { useRouter } from 'next/navigation';

interface User {
  email: string;
  name?: string;
}

interface UserBadgeProps {
  user: User;
}

export function UserBadge({ user }: UserBadgeProps) {
  const router = useRouter();

  const handleSignOut = async () => {
    await signOut();
    router.push('/');
  };

  return (
    <div className="flex items-center space-x-3">
      <div className="bg-indigo-100 text-indigo-800 rounded-full w-8 h-8 flex items-center justify-center">
        {user.name?.charAt(0) || user.email.charAt(0)}
      </div>
      <span className="text-sm font-medium text-gray-700 hidden md:block">
        {user.name || user.email}
      </span>
      <button
        onClick={handleSignOut}
        className="text-sm text-gray-500 hover:text-gray-700"
      >
        Logout
      </button>
    </div>
  );
}