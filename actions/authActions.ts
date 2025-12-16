'use server';

import { signIn, signOut } from 'better-auth/server-actions';
import { revalidatePath } from 'next/cache';
import { redirect } from 'next/navigation';

export async function loginWithEmailAndPassword(prevState: any, formData: FormData) {
  const email = formData.get('email') as string;
  const password = formData.get('password') as string;

  try {
    const response = await signIn({
      email,
      password,
    });

    if (response.error) {
      return {
        error: response.error.message || 'Something went wrong',
        success: false,
      };
    }

    revalidatePath('/');
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

export async function logoutAction() {
  try {
    await signOut();
    revalidatePath('/');
    redirect('/');
  } catch (error) {
    console.error('Logout error:', error);
    // Redirect anyway even if there's an error
    redirect('/');
  }
}

export async function updateProfileAction(prevState: any, formData: FormData) {
  // Update profile action would go here
  // This would use the updateUserProfile function from userActions
  return {
    error: null,
    success: true,
  };
}