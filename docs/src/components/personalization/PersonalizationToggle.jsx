import React, { useState, useEffect } from 'react';

const PersonalizationToggle = ({ onPreferenceChange, initialPreference = 'default' }) => {
  const [preference, setPreference] = useState(initialPreference);

  useEffect(() => {
    // Notify parent component of preference change
    if (onPreferenceChange) {
      onPreferenceChange(preference);
    }
  }, [preference, onPreferenceChange]);

  const handlePreferenceChange = (event) => {
    setPreference(event.target.value);
  };

  return (
    <div className="personalization-toggle">
      <label htmlFor="personalization-select">Personalization Level:</label>
      <select 
        id="personalization-select" 
        value={preference} 
        onChange={handlePreferenceChange}
      >
        <option value="default">Default Content</option>
        <option value="beginner">Beginner-Friendly</option>
        <option value="intermediate">Intermediate</option>
        <option value="advanced">Advanced</option>
      </select>
      <p className="personalization-help">
        {preference === 'beginner' && 'Content will be simplified with additional explanations'}
        {preference === 'intermediate' && 'Content will be presented at standard level'}
        {preference === 'advanced' && 'Content will include additional technical details'}
        {preference === 'default' && 'Content will be presented as originally written'}
      </p>
    </div>
  );
};

export default PersonalizationToggle;