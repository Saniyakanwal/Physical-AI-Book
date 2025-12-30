import React, { useState, useEffect } from 'react';

const TranslationToggle = ({ onLanguageChange, initialLanguage = 'en' }) => {
  const [selectedLanguage, setSelectedLanguage] = useState(() => {
    // Check if language preference is stored in localStorage
    const savedLanguage = localStorage.getItem('preferredLanguage');
    return savedLanguage || initialLanguage;
  });

  useEffect(() => {
    // Store the selected language in localStorage
    localStorage.setItem('preferredLanguage', selectedLanguage);

    // Notify parent component of language change
    if (onLanguageChange) {
      onLanguageChange(selectedLanguage);
    }
  }, [selectedLanguage, onLanguageChange]);

  const handleLanguageChange = (event) => {
    setSelectedLanguage(event.target.value);
  };

  return (
    <div className="translation-toggle">
      <label htmlFor="language-select">Language:</label>
      <select
        id="language-select"
        value={selectedLanguage}
        onChange={handleLanguageChange}
      >
        <option value="en">English</option>
        <option value="ur">Urdu</option>
      </select>
      <p className="translation-help">
        {selectedLanguage === 'ur'
          ? 'متن اردو میں دکھایا جائے گا'
          : 'Content will be displayed in English'}
      </p>
    </div>
  );
};

export default TranslationToggle;