from pydantic import BaseModel
from typing import Optional, Dict, Any
from datetime import datetime

class StudentProfile(BaseModel):
    """
    Model representing a student profile in the Physical AI & Humanoid Robotics textbook system
    """
    id: str
    email: str
    name: str
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None
    created_at: datetime
    updated_at: datetime
    preferences: Optional[Dict[str, Any]] = None  # For storing user preferences
    
    class Config:
        # Allow ORM mode for database integration
        from_attributes = True