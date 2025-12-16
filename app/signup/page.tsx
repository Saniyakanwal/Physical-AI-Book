'use client';

import { useState } from 'react';
import { signUp } from 'better-auth/client/actions';
import { useFormState, useFormStatus } from 'react-dom';
import { createUserWithProfile } from '@/actions/signupAction';

export default function SignUpPage() {
  const [state, formAction] = useFormState(createUserWithProfile, {
    error: null,
    success: false,
  });
  const [softwareExperience, setSoftwareExperience] = useState('');
  const [hardwareAvailable, setHardwareAvailable] = useState('');
  const [roboticsBackground, setRoboticsBackground] = useState('');

  return (
    <div className="min-h-screen flex items-center justify-center bg-gray-50 py-12 px-4 sm:px-6 lg:px-8">
      <div className="max-w-md w-full space-y-8">
        <div>
          <h2 className="mt-6 text-center text-3xl font-extrabold text-gray-900">
            Create your account
          </h2>
        </div>
        <form className="mt-8 space-y-6" action={formAction}>
          <input type="hidden" name="remember" value="true" />
          <div className="rounded-md shadow-sm -space-y-px">
            <div>
              <label htmlFor="email" className="sr-only">
                Email address
              </label>
              <input
                id="email"
                name="email"
                type="email"
                autoComplete="email"
                required
                className="appearance-none rounded-none relative block w-full px-3 py-2 border border-gray-300 placeholder-gray-500 text-gray-900 rounded-t-md focus:outline-none focus:ring-indigo-500 focus:border-indigo-500 focus:z-10 sm:text-sm"
                placeholder="Email address"
              />
            </div>
            <div>
              <label htmlFor="name" className="sr-only">
                Full name
              </label>
              <input
                id="name"
                name="name"
                type="text"
                autoComplete="name"
                required
                className="appearance-none rounded-none relative block w-full px-3 py-2 border border-gray-300 placeholder-gray-500 text-gray-900 focus:outline-none focus:ring-indigo-500 focus:border-indigo-500 focus:z-10 sm:text-sm"
                placeholder="Full name"
              />
            </div>
            <div>
              <label htmlFor="password" className="sr-only">
                Password
              </label>
              <input
                id="password"
                name="password"
                type="password"
                autoComplete="current-password"
                required
                className="appearance-none rounded-none relative block w-full px-3 py-2 border border-gray-300 placeholder-gray-500 text-gray-900 rounded-b-md focus:outline-none focus:ring-indigo-500 focus:border-indigo-500 focus:z-10 sm:text-sm"
                placeholder="Password"
              />
            </div>
          </div>

          {/* Custom Fields Section */}
          <div className="bg-white p-6 rounded-lg border border-gray-200">
            <h3 className="text-lg font-medium text-gray-900 mb-4">Additional Information</h3>

            <div className="mb-4">
              <label htmlFor="softwareExperience" className="block text-sm font-medium text-gray-700 mb-1">
                Software Experience Level
              </label>
              <select
                id="softwareExperience"
                name="softwareExperience"
                value={softwareExperience}
                onChange={(e) => setSoftwareExperience(e.target.value)}
                className="block w-full pl-3 pr-10 py-2 text-base border-gray-300 focus:outline-none focus:ring-indigo-500 focus:border-indigo-500 sm:text-sm rounded-md"
              >
                <option value="">Select your experience level</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
                <option value="expert">Expert</option>
              </select>
            </div>

            <div className="mb-4">
              <label htmlFor="hardwareAvailable" className="block text-sm font-medium text-gray-700 mb-1">
                Hardware Available
              </label>
              <textarea
                id="hardwareAvailable"
                name="hardwareAvailable"
                rows={3}
                value={hardwareAvailable}
                onChange={(e) => setHardwareAvailable(e.target.value)}
                className="shadow-sm focus:ring-indigo-500 focus:border-indigo-500 mt-1 block w-full sm:text-sm border border-gray-300 rounded-md p-2"
                placeholder="Describe your hardware setup (GPUs, Jetson, sensors, etc.)"
              />
            </div>

            <div>
              <label htmlFor="roboticsBackground" className="block text-sm font-medium text-gray-700 mb-1">
                Robotics Background
              </label>
              <textarea
                id="roboticsBackground"
                name="roboticsBackground"
                rows={3}
                value={roboticsBackground}
                onChange={(e) => setRoboticsBackground(e.target.value)}
                className="shadow-sm focus:ring-indigo-500 focus:border-indigo-500 mt-1 block w-full sm:text-sm border border-gray-300 rounded-md p-2"
                placeholder="Tell us about your robotics background and experience"
              />
            </div>
          </div>

          {state.error && (
            <div className="rounded-md bg-red-50 p-4">
              <div className="text-sm text-red-700">{state.error}</div>
            </div>
          )}

          <div>
            <SubmitButton />
          </div>
        </form>
      </div>
    </div>
  );
}

function SubmitButton() {
  const { pending } = useFormStatus();

  return (
    <button
      type="submit"
      disabled={pending}
      className="group relative w-full flex justify-center py-2 px-4 border border-transparent text-sm font-medium rounded-md text-white bg-indigo-600 hover:bg-indigo-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-indigo-500 disabled:opacity-50"
    >
      {pending ? 'Creating account...' : 'Sign up'}
    </button>
  );
}