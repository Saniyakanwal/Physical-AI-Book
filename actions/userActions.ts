'use server';

import { db } from '@/db';
import { users } from '@/db/schema';
import { eq } from 'drizzle-orm';
import { revalidatePath } from 'next/cache';

export async function updateUserProfile(userId: number, profileData: {
  softwareExperience?: string;
  hardwareAvailable?: string;
  roboticsBackground?: string;
}) {
  try {
    await db.update(users)
      .set({
        softwareExperience: profileData.softwareExperience,
        hardwareAvailable: profileData.hardwareAvailable,
        roboticsBackground: profileData.roboticsBackground,
        updatedAt: new Date(),
      })
      .where(eq(users.id, userId));

    revalidatePath('/profile');
    return { success: true, message: 'Profile updated successfully' };
  } catch (error) {
    console.error('Error updating user profile:', error);
    return { success: false, message: 'Failed to update profile' };
  }
}

export async function getUserProfile(userId: number) {
  try {
    const user = await db.select().from(users).where(eq(users.id, userId)).limit(1);
    return user[0] || null;
  } catch (error) {
    console.error('Error fetching user profile:', error);
    return null;
  }
}