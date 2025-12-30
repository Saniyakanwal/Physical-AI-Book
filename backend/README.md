# Physical AI Chatbot Backend

This is the backend server for the Physical AI Chatbot component that appears in the documentation. It provides a RAG (Retrieval Augmented Generation) chatbot API that can answer questions based on the physical AI book content.

## Features

- FastAPI-based REST API
- RAG (Retrieval Augmented Generation) for accurate responses
- Qdrant vector database for efficient retrieval
- Neon PostgreSQL database for conversation history
- Integration with OpenAI for natural language processing

## Prerequisites

- Python 3.8 or higher
- pip (Python package installer)
- Git (for cloning the repository)

## Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-directory>
   ```

2. Create a virtual environment:
   ```bash
   python -m venv venv
   ```

3. Activate the virtual environment:
   - On Windows:
     ```bash
     venv\Scripts\activate
     ```
   - On macOS/Linux:
     ```bash
     source venv/bin/activate
     ```

4. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

5. Set up environment variables:
   - Copy `.env.example` to `.env`:
     ```bash
     cp .env.example .env
     ```
   - Edit the `.env` file with your actual configuration

## Environment Variables

You need to set the following environment variables in your `.env` file:

- `NEON_DATABASE_URL`: PostgreSQL connection string for Neon database
- `QDRANT_URL`: URL for Qdrant vector database (e.g., http://localhost:6333)
- `OPENAI_API_KEY`: Your OpenAI API key
- `PORT`: Port number for the server (default: 8000)

## Running the Server

### Method 1: Using the run_server.py script
```bash
python run_server.py
```

### Method 2: Using uvicorn directly
```bash
uvicorn main:app --reload --host 127.0.0.1 --port 8000
```

### Method 3: Using the startup script (Windows)
```bash
start_server.bat
```

The server will start on `http://127.0.0.1:8000`

## API Endpoints

- `GET /`: Root endpoint
- `POST /chat`: Main chat endpoint for the frontend component
  - Request body: `{"message": "your message", "selected_text": "optional selected text"}`
  - Response: `{"reply": "response text"}`
- `GET /health`: Health check endpoint
- `GET /docs`: Interactive API documentation (Swagger UI)

## Testing the API

Once the server is running, you can test it using curl:

```bash
curl -X POST http://127.0.0.1:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello", "selected_text": ""}'
```

## Frontend Integration

The frontend Chatbot component (in the book directory) connects to this backend at `http://127.0.0.1:8000/chat`.
Make sure both the backend server and the frontend are running to use the chat functionality.

## Troubleshooting

### Common Issues

1. **Port already in use**: Change the PORT variable in your .env file
2. **Database connection errors**: Verify your NEON_DATABASE_URL is correct
3. **OpenAI API errors**: Check that your OPENAI_API_KEY is valid and has sufficient credits
4. **Qdrant connection errors**: Ensure Qdrant is running at the specified URL

### Development

For development, the server runs with auto-reload enabled, so changes to the code will automatically restart the server.

## Stopping the Server

To stop the server, press `Ctrl+C` in the terminal where it's running.

## Dependencies

- FastAPI: Web framework
- uvicorn: ASGI server
- OpenAI: Language model API
- Qdrant: Vector database
- Neon: PostgreSQL database
- python-dotenv: Environment variable management