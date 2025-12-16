'use server';

import { signUp } from 'better-auth/server-actions';
import { db } from '@/db';
import { users } from '@/db/schema';
import { eq } from 'drizzle-orm';
import { redirect } from 'next/navigation';

export async function createUserWithProfile(prevState: { error: string | null; success: boolean }, formData: FormData) {
  const email = formData.get('email') as string;
  const name = formData.get('name') as string;
  const password = formData.get('password') as string;
  const softwareExperience = formData.get('softwareExperience') as string;
  const hardwareAvailable = formData.get('hardwareAvailable') as string;
  const roboticsBackground = formData.get('roboticsBackground') as string;

  try {
    // First, create the user with Better Auth
    const authResponse = await signUp({
      email,
      name,
      password,
      attributes: {
        softwareExperience,
        hardwareAvailable,
        roboticsBackground,
      }
    });

    if (authResponse.error) {
      return {
        error: authResponse.error.message || 'Something went wrong during signup',
        success: false,
      };
    }

    // At this point, Better Auth has created the user in the database
    // If needed, we can update the user record with additional data
    
    // Redirect to dashboard on successful signup
    redirect('/dashboard');
    
    return {
      error: null,
      success: true,
    };
  } catch (error: any) {
    return {
      error: error.message || 'Something went wrong',
      success: false,
    };
  }
}