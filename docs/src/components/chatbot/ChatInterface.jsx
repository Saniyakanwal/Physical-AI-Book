import React, { useState, useEffect } from 'react';
import { useColorMode } from '@docusaurus/theme-common';

// ChatInterface component for the Physical AI & Humanoid Robotics textbook
const ChatInterface = ({ chapterId }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const { colorMode } = useColorMode();

  // Initialize with a welcome message
  useEffect(() => {
    setMessages([
      {
        id: 1,
        role: 'assistant',
        content: `Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook. How can I help you with Chapter: ${chapterId || 'Unknown'}?`,
        timestamp: new Date()
      }
    ]);
  }, [chapterId]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: inputValue,
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputValue,
          selected_text: null, // Could be populated if user selects text
          student_id: 'temp-student-id', // Would come from auth system
          chapter_id: chapterId,
          session_id: sessionId || null
        }),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      let content = data.reply;

      // Add citations if available
      if (data.citations && data.citations.length > 0) {
        content += '\n\n**Sources:**\n';
        data.citations.forEach((citation, index) => {
          content += `- Source ${index + 1}: ${citation}\n`;
        });
      }

      const assistantMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: content,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, assistantMessage]);

      // Update session ID if returned from API
      if (data.session_id) {
        setSessionId(data.session_id);
      }
    } catch (error) {
      console.error('Error in chat API call:', error);
      const errorMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={`chat-interface ${colorMode}`}>
      <div className="chat-header">
        <h3>Textbook Assistant</h3>
      </div>

      <div className="chat-messages">
        {messages.map((message) => (
          <div
            key={message.id}
            className={`message ${message.role}`}
          >
            <div className="message-content">
              {message.content}
            </div>
            <div className="message-timestamp">
              {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
            </div>
          </div>
        ))}

        {isLoading && (
          <div className="message assistant">
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
      </div>

      <form className="chat-input-form" onSubmit={handleSubmit}>
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question about this chapter..."
          disabled={isLoading}
        />
        <button type="submit" disabled={isLoading || !inputValue.trim()}>
          Send
        </button>
      </form>
    </div>
  );
};

export default ChatInterface;