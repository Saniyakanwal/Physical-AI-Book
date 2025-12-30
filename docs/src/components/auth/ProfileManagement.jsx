import React, { useState, useEffect } from 'react';

const ProfileManagement = ({ studentId, initialProfile }) => {
  const [profile, setProfile] = useState(initialProfile || {
    email: '',
    name: '',
    software_background: '',
    hardware_background: ''
  });
  const [isEditing, setIsEditing] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [message, setMessage] = useState('');

  useEffect(() => {
    if (initialProfile) {
      setProfile(initialProfile);
    }
  }, [initialProfile]);

  const handleEdit = () => {
    setIsEditing(true);
    setMessage('');
  };

  const handleCancel = () => {
    setIsEditing(false);
    // Reset to initial values if needed
    if (initialProfile) {
      setProfile(initialProfile);
    }
  };

  const handleChange = (e) => {
    const { name, value } = e.target;
    setProfile(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setIsLoading(true);
    setMessage('');

    try {
      // In a real implementation, this would call the backend API to update the profile
      // For now, we'll simulate the update process
      await new Promise(resolve => setTimeout(resolve, 1000));
      
      // Update successful
      setIsEditing(false);
      setMessage('Profile updated successfully!');
    } catch (err) {
      setMessage('Failed to update profile. Please try again.');
      console.error('Update error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="profile-management">
      <h2>Profile Management</h2>
      
      {message && <div className={`message ${message.includes('successfully') ? 'success' : 'error'}`}>{message}</div>}
      
      {!isEditing ? (
        <div className="profile-display">
          <div className="profile-field">
            <strong>Email:</strong> {profile.email}
          </div>
          <div className="profile-field">
            <strong>Name:</strong> {profile.name}
          </div>
          <div className="profile-field">
            <strong>Software Background:</strong> {profile.software_background || 'Not specified'}
          </div>
          <div className="profile-field">
            <strong>Hardware Background:</strong> {profile.hardware_background || 'Not specified'}
          </div>
          
          <button onClick={handleEdit} className="edit-button">
            Edit Profile
          </button>
        </div>
      ) : (
        <form onSubmit={handleSubmit} className="profile-edit-form">
          <div className="form-group">
            <label htmlFor="email">Email:</label>
            <input
              type="email"
              id="email"
              name="email"
              value={profile.email}
              onChange={handleChange}
              required
              disabled // Email might be disabled from editing in some systems
            />
          </div>
          
          <div className="form-group">
            <label htmlFor="name">Full Name:</label>
            <input
              type="text"
              id="name"
              name="name"
              value={profile.name}
              onChange={handleChange}
              required
            />
          </div>
          
          <div className="form-group">
            <label htmlFor="software_background">Software Background:</label>
            <textarea
              id="software_background"
              name="software_background"
              value={profile.software_background}
              onChange={handleChange}
              placeholder="e.g., Experience with Python, C++, ROS, etc."
            />
          </div>
          
          <div className="form-group">
            <label htmlFor="hardware_background">Hardware Background:</label>
            <textarea
              id="hardware_background"
              name="hardware_background"
              value={profile.hardware_background}
              onChange={handleChange}
              placeholder="e.g., Experience with Arduino, Raspberry Pi, sensors, etc."
            />
          </div>
          
          <div className="form-actions">
            <button type="submit" disabled={isLoading}>
              {isLoading ? 'Saving...' : 'Save Changes'}
            </button>
            <button type="button" onClick={handleCancel} disabled={isLoading}>
              Cancel
            </button>
          </div>
        </form>
      )}
    </div>
  );
};

export default ProfileManagement;