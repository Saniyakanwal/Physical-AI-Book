import React, { useState, useEffect } from 'react';
import './Chatbot.css';
import config from './config.js';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // Function to get selected text from the page
  const getSelectedText = () => {
    return window.getSelection ? window.getSelection().toString() : '';
  };

  // Function to handle sending message to backend
  const sendMessage = async () => {
    if (!inputValue.trim()) return;

    const selectedText = getSelectedText();
    
    // Add user message to chat
    const userMessage = { sender: 'user', text: inputValue };
    setMessages(prev => [...prev, userMessage]);
    const newInputValue = inputValue;
    setInputValue('');
    setIsLoading(true);

    try {
      // Try to connect to the backend server
      const response = await fetch(`${config.BACKEND_URL}${config.CHAT_ENDPOINT}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: newInputValue,
          selected_text: selectedText
        }),
      });

      if (response.ok) {
        const data = await response.json();
        // Add bot response to chat
        const botMessage = { sender: 'bot', text: data.reply };
        setMessages(prev => [...prev, botMessage]);
      } else {
        // Handle different types of errors
        let errorMessage = 'Sorry, there was an error processing your request.';
        
        if (response.status === 404) {
          errorMessage = 'The chat endpoint was not found. Please check if the backend server is running.';
        } else if (response.status >= 500) {
          errorMessage = 'The server encountered an error. Please try again later.';
        } else if (response.status === 422) {
          errorMessage = 'The request was not properly formatted. Please try again.';
        } else {
          errorMessage = `Server responded with status: ${response.status}. Please check the server configuration.`;
        }
        
        const errorBotMessage = { sender: 'bot', text: errorMessage };
        setMessages(prev => [...prev, errorBotMessage]);
      }
    } catch (error) {
      console.error('Connection error:', error);
      let errorMessage = 'Sorry, there was an error connecting to the server.';
      
      // Check the type of error and provide more specific messages
      if (error.name === 'TypeError' && error.message.includes('fetch')) {
        errorMessage = `Unable to connect to the server. Please make sure the backend is running at ${config.BACKEND_URL}.`;
      } else if (error.message.includes('NetworkError')) {
        errorMessage = 'Network error occurred. Please check your internet connection.';
      } else if (error.message.includes('Failed to fetch')) {
        errorMessage = `Failed to fetch from ${config.BACKEND_URL}. Please ensure the backend is running and accessible.`;
      } else {
        errorMessage = error.message;
      }
      
      const errorBotMessage = { sender: 'bot', text: errorMessage };
      setMessages(prev => [...prev, errorBotMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle Enter key press to send message
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Floating Chat Button */}
      {!isOpen && (
        <button 
          className="chat-float-button"
          onClick={() => setIsOpen(true)}
          aria-label="Open chat"
        >
          💬
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div className="chat-container">
          <div className="chat-header">
            <h3>Physical AI Assistant</h3>
            <button 
              className="close-button"
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>
          
          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Hello! I'm your Physical AI Assistant. How can I help you today?</p>
                <p>You can share selected text with me to get more contextual responses.</p>
                <p className="connection-info">
                  <strong>Server Status:</strong> Attempting to connect to {config.BACKEND_URL}
                </p>
                <p className="connection-info">
                  <strong>Instructions:</strong> Make sure the backend server is running and accessible before using the chat.
                </p>
              </div>
            ) : (
              messages.map((msg, index) => (
                <div 
                  key={index} 
                  className={`message ${msg.sender}`}
                >
                  <div className="message-bubble">
                    {msg.text}
                  </div>
                </div>
              ))
            )}
            
            {isLoading && (
              <div className="message bot">
                <div className="message-bubble">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
          </div>
          
          <div className="chat-input-area">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Type your message here..."
              rows="1"
              className="chat-input"
            />
            <button 
              onClick={sendMessage}
              disabled={!inputValue.trim() || isLoading}
              className="send-button"
            >
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default Chatbot;