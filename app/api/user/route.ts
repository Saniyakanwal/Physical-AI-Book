import { auth } from '@/lib/auth';
import { db } from '@/db';
import { users } from '@/db/schema';
import { eq } from 'drizzle-orm';
import { NextRequest } from 'next/server';

export async function GET(req: NextRequest) {
  // Check if user is authenticated
  const session = await auth.api.getSession({
    headers: req.headers,
  });

  if (!session) {
    return new Response('Unauthorized', { status: 401 });
  }

  try {
    // Fetch the user data from the database
    const user = await db
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
      .where(eq(users.id, session.session.userId))
      .limit(1);

    if (user.length === 0) {
      return new Response('User not found', { status: 404 });
    }

    return new Response(JSON.stringify(user[0]), {
      status: 200,
      headers: {
        'Content-Type': 'application/json',
      },
    });
  } catch (error) {
    console.error('Error fetching user data:', error);
    return new Response('Internal server error', { status: 500 });
  }
}

export async function PUT(req: NextRequest) {
  // Check if user is authenticated
  const session = await auth.api.getSession({
    headers: req.headers,
  });

  if (!session) {
    return new Response('Unauthorized', { status: 401 });
  }

  try {
    const body = await req.json();

    // Update the user data in the database
    const updatedUser = await db
      .update(users)
      .set({
        name: body.name || undefined,
        softwareExperience: body.softwareExperience || undefined,
        hardwareAvailable: body.hardwareAvailable || undefined,
        roboticsBackground: body.roboticsBackground || undefined,
        updatedAt: new Date(),
      })
      .where(eq(users.id, session.session.userId))
      .returning();

    return new Response(JSON.stringify(updatedUser[0]), {
      status: 200,
      headers: {
        'Content-Type': 'application/json',
      },
    });
  } catch (error) {
    console.error('Error updating user data:', error);
    return new Response('Internal server error', { status: 500 });
  }
}