// Chapter Buttons Script for Docusaurus
(function() {
  'use strict';

  // Create the buttons container
  function createButtonsContainer() {
    const container = document.createElement('div');
    container.className = 'chapter-buttons-container';
    container.style.cssText = `
      display: flex;
      gap: 10px;
      margin-bottom: 20px;
      justify-content: flex-start;
    `;
    return container;
  }

  // Create a button with specified text and color
  function createButton(text, color, onClickHandler) {
    const button = document.createElement('button');
    button.textContent = text;
    button.style.cssText = `
      padding: 8px 16px;
      background-color: ${color};
      color: white;
      border: none;
      border-radius: 4px;
      cursor: pointer;
      font-size: 14px;
    `;
    button.onclick = onClickHandler;
    return button;
  }

  // Function to extract markdown content from the current page
  // In a real implementation, this would get the original markdown
  function extractMarkdownContent() {
    // Try to get content from the main article element
    const article = document.querySelector('article');
    if (article) {
      // For now, we'll get the inner text content
      // In a real implementation, you'd want to get the original markdown
      return article.innerText || '';
    }
    return '';
  }

  // Function to update page content with new markdown
  function updatePageContent(newContent) {
    const article = document.querySelector('article');
    if (article) {
      // This is a simplified version. In practice, you'll need to convert
      // the markdown back to HTML and replace the content appropriately
      article.innerHTML = newContent;
    }
  }

  // Handle personalization request
  async function handlePersonalize() {
    const personalizeBtn = document.getElementById('personalize-btn');
    const originalText = personalizeBtn.textContent;
    
    // Show processing state
    personalizeBtn.textContent = 'Processing...';
    personalizeBtn.disabled = true;
    
    try {
      const content = extractMarkdownContent();
      const response = await fetch('/api/personalize', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ 
          markdown: content,
          url: window.location.pathname
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
      // Restore button state
      personalizeBtn.textContent = originalText;
      personalizeBtn.disabled = false;
    }
  }

  // Handle translation request
  async function handleTranslate() {
    const translateBtn = document.getElementById('translate-btn');
    const originalText = translateBtn.textContent;
    
    // Show processing state
    translateBtn.textContent = 'Processing...';
    translateBtn.disabled = true;
    
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
      // Restore button state
      translateBtn.textContent = originalText;
      translateBtn.disabled = false;
    }
  }

  // Function to inject buttons into the page
  function injectButtons() {
    // Find the main content area
    const mainContent = document.querySelector('main .container');
    if (!mainContent) {
      // Retry after a short delay in case DOM isn't fully loaded
      setTimeout(injectButtons, 500);
      return;
    }

    // Check if buttons are already injected
    if (document.querySelector('.chapter-buttons-container')) {
      return;
    }

    const buttonsContainer = createButtonsContainer();

    const personalizeBtn = createButton('Personalize for Me', '#4CAF50', handlePersonalize);
    personalizeBtn.id = 'personalize-btn';
    
    const translateBtn = createButton('Translate to Urdu', '#2196F3', handleTranslate);
    translateBtn.id = 'translate-btn';

    buttonsContainer.appendChild(personalizeBtn);
    buttonsContainer.appendChild(translateBtn);

    // Insert after the first heading or at the beginning of the main content
    const firstHeading = mainContent.querySelector('h1, h2');
    if (firstHeading) {
      firstHeading.parentNode.insertBefore(buttonsContainer, firstHeading.nextSibling);
    } else {
      mainContent.insertBefore(buttonsContainer, mainContent.firstChild);
    }
  }

  // Wait for the DOM to be fully loaded
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', injectButtons);
  } else {
    // DOM is already loaded, so execute immediately
    injectButtons();
  }

  // For SPAs (like when using Docusaurus's client-side routing)
  let currentPath = location.pathname;
  const observer = new MutationObserver(function() {
    if (location.pathname !== currentPath) {
      currentPath = location.pathname;
      // Wait a bit for new content to load
      setTimeout(injectButtons, 300);
    }
  });

  observer.observe(document, { childList: true, subtree: true });
})();