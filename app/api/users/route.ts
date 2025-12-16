import { auth } from '@/lib/auth';
import { db } from '@/db';
import { users } from '@/db/schema';
import { eq, and, or } from 'drizzle-orm';
import { NextRequest } from 'next/server';

export async function GET(req: NextRequest) {
  // Check if user is authenticated
  const session = await auth.api.getSession({
    headers: req.headers,
  });

  if (!session) {
    return new Response('Unauthorized', { status: 401 });
  }

  // Only allow users to view their own data
  // In a real app, you might have more sophisticated authorization rules

  try {
    // Get query parameters
    const url = new URL(req.url);
    const userId = url.searchParams.get('id');

    let userQuery;
    if (userId && parseInt(userId) === session.session.userId) {
      // User can fetch their own data by ID
      userQuery = db
        .select({
          id: users.id,
          email: users.email,
          name: users.name,
          softwareExperience: users.softwareExperience,
          hardwareAvailable: users.hardwareAvailable,
          roboticsBackground: users.roboticsBackground,
          createdAt: users.createdAt,
        })
        .from(users)
        .where(eq(users.id, parseInt(userId)));
    } else {
      // User can only fetch their own data without ID parameter
      userQuery = db
        .select({
          id: users.id,
          email: users.email,
          name: users.name,
          softwareExperience: users.softwareExperience,
          hardwareAvailable: users.hardwareAvailable,
          roboticsBackground: users.roboticsBackground,
          createdAt: users.createdAt,
        })
        .from(users)
        .where(eq(users.id, session.session.userId));
    }

    const userResults = await userQuery;

    return new Response(JSON.stringify(userResults), {
      status: 200,
      headers: {
        'Content-Type': 'application/json',
      },
    });
  } catch (error) {
    console.error('Error fetching users data:', error);
    return new Response('Internal server error', { status: 500 });
  }
}