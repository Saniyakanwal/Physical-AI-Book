// Environment configuration for the Chatbot component
// This version is safe for browser environments

// Get environment configuration from window if available
const getEnvConfig = () => {
  if (typeof window !== 'undefined' && window._env_) {
    return window._env_;
  }
  return {};
};

const envConfig = getEnvConfig();

const config = {
  // Backend API configuration with fallbacks
  BACKEND_URL: envConfig.REACT_APP_BACKEND_URL || 
               (typeof process !== 'undefined' ? process.env.REACT_APP_BACKEND_URL : null) || 
               'http://127.0.0.1:8000',
  CHAT_ENDPOINT: '/chat',
  
  // Connection timeout (in milliseconds)
  REQUEST_TIMEOUT: 30000, // 30 seconds
  
  // Retry settings
  MAX_RETRIES: 3,
  RETRY_DELAY: 1000, // 1 second
};

export default config;