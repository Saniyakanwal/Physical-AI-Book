from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional
from datetime import datetime
import uuid

from ..services.chat_service import ChatService, Message
from ..services.chapter_service import ChapterService

# Create API router
router = APIRouter(prefix="/chatbot", tags=["chatbot"])

# Request models
class ChatRequest(BaseModel):
    message: str
    student_id: str
    chapter_id: str
    selected_text: Optional[str] = None

# Response models
class ChatResponse(BaseModel):
    reply: str
    session_id: str
    timestamp: datetime

class SessionResponse(BaseModel):
    session_id: str
    student_id: str
    chapter_id: str
    created_at: datetime

# Initialize services
chat_service = ChatService()
chapter_service = ChapterService()

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Initiates a new chat session or continues an existing one with the textbook's intelligent chatbot.
    """
    try:
        # For now, we'll generate a new session ID for each request
        # In a real implementation, we would check if the session already exists
        session_id = str(uuid.uuid4())
        
        # Create a user message
        user_message = Message(role="user", content=request.message)
        
        # Add the message to the session
        # In a real implementation, we would create or retrieve the session
        # For now, we'll just generate a response directly
        
        # Generate response using the chat service
        reply = await chat_service.generate_response(session_id, request.message)
        
        return ChatResponse(
            reply=reply,
            session_id=session_id,
            timestamp=datetime.utcnow()
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/session", response_model=SessionResponse)
async def create_session(student_id: str, chapter_id: str):
    """
    Creates a new chat session for a student with a specific chapter.
    """
    try:
        # Create a new session using the chat service
        session = await chat_service.create_session(student_id, chapter_id)
        
        return SessionResponse(
            session_id=session.id,
            student_id=session.student_id,
            chapter_id=session.chapter_id,
            created_at=session.created_at
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/session/{session_id}", response_model=ChatSession)
async def get_session(session_id: str):
    """
    Retrieves a chat session by its ID.
    """
    try:
        session = await chat_service.get_session_by_id(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")
        
        return session
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))