# Physical AI Chatbot Backend

This directory contains the backend server for the Physical AI Chatbot component that appears in the documentation.

## Prerequisites

- Python 3.8 or higher
- pip (Python package installer)

## Installation

1. Install the required Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Make sure you have FastAPI and uvicorn installed:
   ```bash
   pip install "fastapi[all]" uvicorn
   ```

## Running the Server

To run the backend server:

```bash
# Using uvicorn directly
uvicorn main:app --reload --host 127.0.0.1 --port 8000

# Or using the run_server.py script
python run_server.py
```

The server will be available at `http://127.0.0.1:8000`

## Environment Variables

Create a `.env` file in the root directory:

```env
OPENAI_API_KEY=your_openai_api_key_here
DATABASE_URL=postgresql://user:password@localhost/dbname
QDRANT_URL=http://localhost:6333
```

## API Endpoints

- `POST /chat` - Main chat endpoint that the frontend component calls
- `GET /health` - Health check endpoint

## Troubleshooting

### Common Issues

1. **Port already in use**:
   - Check if another process is using port 8000
   - Change the port by modifying the server startup command

2. **Connection refused**:
   - Ensure the server is running on the expected URL
   - Check firewall settings if connecting from a different machine

3. **CORS errors**:
   - Make sure CORS is properly configured in the backend

4. **Backend not responding**:
   - Check the server logs for error messages
   - Ensure all environment variables are set correctly

### Environment Variables for Development

You can set a custom API endpoint for the chatbot component by setting the following environment variable in your React app:

```bash
REACT_APP_BACKEND_URL=http://localhost:8000
```

Or for different environments:

```bash
# Development
REACT_APP_BACKEND_URL=http://127.0.0.1:8000

# Production
REACT_APP_BACKEND_URL=https://yourdomain.com/api
```

## Testing

To test the API endpoints:

```bash
# Test the chat endpoint
curl -X POST http://127.0.0.1:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello", "selected_text": ""}'
```

## Docker Support (Optional)

If you prefer to run with Docker:

```bash
# Build the image
docker build -t physical-ai-chatbot-backend .

# Run the container
docker run -p 8000:8000 physical-ai-chatbot-backend
```

## Shutdown

To stop the server, press `Ctrl+C` in the terminal where it's running.