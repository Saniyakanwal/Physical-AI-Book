from pydantic import BaseModel
from typing import Optional
from datetime import datetime

class TextbookChapter(BaseModel):
    """
    Model representing a textbook chapter in the Physical AI & Humanoid Robotics textbook
    """
    id: str
    title: str
    module: str  # ROS2, GazeboUnity, NvidiaIsaac, VisionLanguageAction
    week: int    # 1-13
    content: str
    content_ur: Optional[str] = None  # Urdu translation
    created_at: datetime
    updated_at: datetime
    
    class Config:
        # Allow ORM mode for database integration
        from_attributes = True