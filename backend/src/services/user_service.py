from typing import List, Optional
from ..models.student_profile import StudentProfile
import logging
import uuid
from datetime import datetime

logger = logging.getLogger(__name__)

class UserService:
    """
    Service class for handling user operations
    """
    
    def __init__(self):
        # In a real implementation, this would connect to a database
        self.profiles: List[StudentProfile] = []
        logger.info("UserService initialized")

    async def create_profile(self, email: str, name: str, software_background: Optional[str] = None, 
                           hardware_background: Optional[str] = None) -> StudentProfile:
        """
        Create a new student profile
        """
        profile_id = str(uuid.uuid4())
        now = datetime.utcnow()
        
        profile = StudentProfile(
            id=profile_id,
            email=email,
            name=name,
            software_background=software_background,
            hardware_background=hardware_background,
            created_at=now,
            updated_at=now,
            preferences={}
        )
        
        self.profiles.append(profile)
        return profile

    async def get_profile_by_id(self, profile_id: str) -> Optional[StudentProfile]:
        """
        Retrieve a student profile by its ID
        """
        for profile in self.profiles:
            if profile.id == profile_id:
                return profile
        return None

    async def get_profile_by_email(self, email: str) -> Optional[StudentProfile]:
        """
        Retrieve a student profile by email
        """
        for profile in self.profiles:
            if profile.email == email:
                return profile
        return None

    async def update_profile(self, profile_id: str, name: Optional[str] = None, 
                           software_background: Optional[str] = None, 
                           hardware_background: Optional[str] = None,
                           preferences: Optional[dict] = None) -> Optional[StudentProfile]:
        """
        Update an existing student profile
        """
        profile = await self.get_profile_by_id(profile_id)
        if not profile:
            return None
            
        # Update fields if provided
        if name is not None:
            profile.name = name
        if software_background is not None:
            profile.software_background = software_background
        if hardware_background is not None:
            profile.hardware_background = hardware_background
        if preferences is not None:
            profile.preferences = preferences
            
        profile.updated_at = datetime.utcnow()
        return profile

    async def add_preference(self, profile_id: str, key: str, value: any) -> bool:
        """
        Add or update a preference for a user
        """
        profile = await self.get_profile_by_id(profile_id)
        if not profile:
            return False
            
        if profile.preferences is None:
            profile.preferences = {}
            
        profile.preferences[key] = value
        profile.updated_at = datetime.utcnow()
        return True