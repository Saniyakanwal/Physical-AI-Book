from pydantic import BaseModel
from typing import Optional
from datetime import datetime

class TranslationResource(BaseModel):
    """
    Model representing a translation resource in the Physical AI & Humanoid Robotics textbook system
    """
    id: str
    resource_type: str  # chapter, ui, error_message
    resource_id: str    # ID of the original resource
    language_code: str  # ISO 639-1 language code (e.g., 'ur' for Urdu)
    translated_content: str
    created_at: datetime
    updated_at: datetime
    
    class Config:
        # Allow ORM mode for database integration
        from_attributes = True