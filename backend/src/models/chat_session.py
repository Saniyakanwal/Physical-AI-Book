from pydantic import BaseModel
from typing import List, Dict, Any, Optional
from datetime import datetime

class Message(BaseModel):
    """
    Model representing a single message in a chat session
    """
    role: str  # 'user' or 'assistant'
    content: str

class ChatSession(BaseModel):
    """
    Model representing a chat session between a student and the textbook chatbot
    """
    id: str
    student_id: str
    chapter_id: str
    messages: List[Message]
    created_at: datetime
    updated_at: datetime
    
    class Config:
        # Allow ORM mode for database integration
        from_attributes = True