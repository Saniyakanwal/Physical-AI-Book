import { auth } from '@/lib/auth';
import { NextRequest } from 'next/server';

export async function GET(req: NextRequest) {
  // Check if user is authenticated
  const session = await auth.api.getSession({
    headers: req.headers,
  });

  if (!session) {
    return new Response(JSON.stringify({ authenticated: false }), {
      status: 200,
      headers: {
        'Content-Type': 'application/json',
      },
    });
  }

  return new Response(JSON.stringify({
    authenticated: true,
    user: {
      id: session.session.userId,
      email: session.user.email,
      name: session.user.name,
    }
  }), {
    status: 200,
    headers: {
      'Content-Type': 'application/json',
    },
  });
}