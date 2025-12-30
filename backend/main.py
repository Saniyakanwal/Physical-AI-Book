from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import os
from typing import Optional
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import required libraries
try:
    from db import db_manager
    from vector_db import qdrant_manager
    from services import RagService
    # Import our new modules
    from src.api.chatbot import router as chatbot_router
    from src.api.auth import router as auth_router
    from src.api.translation import router as translation_router
    from src.services.chat_service import ChatService
    from src.services.chapter_service import ChapterService
    from src.services.user_service import UserService
    from src.services.translation_service import TranslationService
except ImportError as e:
    logger.error(f"Import error: {e}")
    raise ImportError("Please install required dependencies: pip install fastapi openai qdrant-client python-dotenv asyncpg")

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    logger.info("Initializing database connection...")
    await db_manager.connect()

    logger.info("Initializing Qdrant collection...")
    qdrant_manager.initialize_collection("physical_ai_book")

    yield

    # Shutdown
    logger.info("Closing database connection...")
    await db_manager.close()


app = FastAPI(
    title="Physical AI Book RAG Chatbot API",
    description="RAG Chatbot endpoint for Physical AI Book. Returns answers based on user query and selected book content.",
    version="1.0.0",
    lifespan=lifespan
)

# Include our new API routes
app.include_router(chatbot_router)
app.include_router(auth_router)
app.include_router(translation_router)

# Initialize services
rag_service = RagService()
chat_service = ChatService()
chapter_service = ChapterService()
user_service = UserService()
translation_service = TranslationService()

# Request model
class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None
    user_email: Optional[str] = None  # For user tracking in database
    session_id: Optional[str] = None  # For continuing previous conversations

# Response model
class ChatResponse(BaseModel):
    reply: str
    session_id: Optional[str] = None
    citations: Optional[list] = None  # List of sources used in the response

@app.post(
    "/chat",
    response_model=ChatResponse,
    summary="RAG Chatbot endpoint",
    description="Returns answers based on user query and selected book content."
)
async def chat_endpoint(request: ChatRequest) -> ChatResponse:
    """
    RAG Chatbot endpoint for Physical AI Book.

    Args:
        request: Contains user message and optional selected text

    Returns:
        ChatResponse: Contains the generated reply
    """
    message = request.message
    selected_text = request.selected_text
    user_email = request.user_email
    session_id = request.session_id

    try:
        # Process the request using the RAG service
        reply, final_session_id = await rag_service.process_chat_request(
            message=message,
            selected_text=selected_text,
            user_email=user_email,
            session_id=session_id
        )

        # For now, we'll return an empty citations list
        # In a full implementation, this would include actual citations
        return ChatResponse(reply=reply, session_id=final_session_id, citations=[])

    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"An error occurred: {str(e)}")

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.getenv("PORT", 8000)))