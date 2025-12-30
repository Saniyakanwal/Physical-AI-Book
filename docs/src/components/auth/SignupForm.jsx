import React, { useState } from 'react';

const SignupForm = ({ onSignup }) => {
  const [formData, setFormData] = useState({
    email: '',
    name: '',
    softwareBackground: '',
    hardwareBackground: ''
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setIsLoading(true);
    setError('');

    try {
      // In a real implementation, this would call the backend API
      // For now, we'll simulate the signup process
      await new Promise(resolve => setTimeout(resolve, 1000));
      
      // Call the onSignup callback with the form data
      if (onSignup) {
        await onSignup(formData);
      }
    } catch (err) {
      setError('Failed to create account. Please try again.');
      console.error('Signup error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="signup-form">
      <h2>Create Your Account</h2>
      {error && <div className="error-message">{error}</div>}
      
      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="email">Email:</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            required
          />
        </div>
        
        <div className="form-group">
          <label htmlFor="name">Full Name:</label>
          <input
            type="text"
            id="name"
            name="name"
            value={formData.name}
            onChange={handleChange}
            required
          />
        </div>
        
        <div className="form-group">
          <label htmlFor="softwareBackground">Software Background:</label>
          <textarea
            id="softwareBackground"
            name="softwareBackground"
            value={formData.softwareBackground}
            onChange={handleChange}
            placeholder="e.g., Experience with Python, C++, ROS, etc."
          />
        </div>
        
        <div className="form-group">
          <label htmlFor="hardwareBackground">Hardware Background:</label>
          <textarea
            id="hardwareBackground"
            name="hardwareBackground"
            value={formData.hardwareBackground}
            onChange={handleChange}
            placeholder="e.g., Experience with Arduino, Raspberry Pi, sensors, etc."
          />
        </div>
        
        <button type="submit" disabled={isLoading}>
          {isLoading ? 'Creating Account...' : 'Sign Up'}
        </button>
      </form>
    </div>
  );
};

export default SignupForm;