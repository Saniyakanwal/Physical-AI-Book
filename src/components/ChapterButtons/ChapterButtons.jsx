import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

const ChapterButtons = () => {
  const [isProcessing, setIsProcessing] = useState(false);
  const location = useLocation();

  // Function to extract markdown content from the current page
  const extractMarkdownContent = () => {
    // Get the main content element
    const mainContent = document.querySelector('article.markdown');
    if (mainContent) {
      // For now, we'll get the text content
      // In a real implementation, you'd want to get the original markdown
      return mainContent.innerText || '';
    }
    return '';
  };

  // Function to update page content with new markdown
  const updatePageContent = (newContent) => {
    const mainContent = document.querySelector('article.markdown');
    if (mainContent) {
      // In a real implementation, you'd need to convert markdown to HTML
      // and properly replace the content while preserving the structure
      mainContent.innerHTML = `<div class="markdown">${newContent}</div>`;
    }
  };

  const handlePersonalize = async () => {
    setIsProcessing(true);
    try {
      const content = extractMarkdownContent();
      
      const response = await fetch('/api/personalize', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ 
          markdown: content, 
          url: location.pathname 
        }),
      });

      if (response.ok) {
        const data = await response.json();
        updatePageContent(data.personalizedMarkdown);
      } else {
        console.error('Personalization failed:', response.statusText);
        alert('Personalization failed. Please try again.');
      }
    } catch (error) {
      console.error('Error personalizing content:', error);
      alert('Error personalizing content. Please try again.');
    } finally {
      setIsProcessing(false);
    }
  };

  const handleTranslate = async () => {
    setIsProcessing(true);
    try {
      const content = extractMarkdownContent();
      
      const response = await fetch('/api/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ 
          markdown: content, 
          targetLanguage: 'urdu' 
        }),
      });

      if (response.ok) {
        const data = await response.json();
        updatePageContent(data.translatedMarkdown);
      } else {
        console.error('Translation failed:', response.statusText);
        alert('Translation failed. Please try again.');
      }
    } catch (error) {
      console.error('Error translating content:', error);
      alert('Error translating content. Please try again.');
    } finally {
      setIsProcessing(false);
    }
  };

  return (
    <div className="chapter-buttons-container" style={{ 
      display: 'flex', 
      gap: '10px', 
      marginBottom: '20px',
      flexWrap: 'wrap'
    }}>
      <button
        onClick={handlePersonalize}
        disabled={isProcessing}
        style={{
          padding: '10px 16px',
          backgroundColor: '#4CAF50',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: isProcessing ? 'not-allowed' : 'pointer',
          opacity: isProcessing ? 0.6 : 1,
          fontSize: '14px',
          fontWeight: '500'
        }}
      >
        {isProcessing ? 'Processing...' : 'Personalize for Me'}
      </button>
      <button
        onClick={handleTranslate}
        disabled={isProcessing}
        style={{
          padding: '10px 16px',
          backgroundColor: '#2196F3',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: isProcessing ? 'not-allowed' : 'pointer',
          opacity: isProcessing ? 0.6 : 1,
          fontSize: '14px',
          fontWeight: '500'
        }}
      >
        {isProcessing ? 'Processing...' : 'Translate to Urdu'}
      </button>
    </div>
  );
};

export default ChapterButtons;