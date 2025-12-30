from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional
from datetime import datetime
import uuid

from ..services.user_service import UserService
from ..models.student_profile import StudentProfile

# Create API router
router = APIRouter(prefix="/auth", tags=["authentication"])

# Request models
class RegisterRequest(BaseModel):
    email: str
    name: str
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None

class UpdateProfileRequest(BaseModel):
    name: Optional[str] = None
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None

# Response models
class RegisterResponse(BaseModel):
    student_id: str
    email: str
    name: str
    created_at: datetime

class ProfileResponse(BaseModel):
    id: str
    email: str
    name: str
    software_background: Optional[str]
    hardware_background: Optional[str]
    created_at: datetime
    updated_at: datetime

# Initialize service
user_service = UserService()

@router.post("/register", response_model=RegisterResponse)
async def register_student(request: RegisterRequest):
    """
    Register a new student with background information.
    """
    try:
        # Check if user already exists
        existing_user = await user_service.get_profile_by_email(request.email)
        if existing_user:
            raise HTTPException(status_code=400, detail="User with this email already exists")
        
        # Create new profile
        profile = await user_service.create_profile(
            email=request.email,
            name=request.name,
            software_background=request.software_background,
            hardware_background=request.hardware_background
        )
        
        return RegisterResponse(
            student_id=profile.id,
            email=profile.email,
            name=profile.name,
            created_at=profile.created_at
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/profile/{student_id}", response_model=ProfileResponse)
async def get_student_profile(student_id: str):
    """
    Retrieve a student's profile information.
    """
    try:
        profile = await user_service.get_profile_by_id(student_id)
        if not profile:
            raise HTTPException(status_code=404, detail="Student profile not found")
        
        return ProfileResponse(
            id=profile.id,
            email=profile.email,
            name=profile.name,
            software_background=profile.software_background,
            hardware_background=profile.hardware_background,
            created_at=profile.created_at,
            updated_at=profile.updated_at
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.put("/profile/{student_id}", response_model=ProfileResponse)
async def update_student_profile(student_id: str, request: UpdateProfileRequest):
    """
    Update a student's profile information.
    """
    try:
        profile = await user_service.update_profile(
            student_id,
            name=request.name,
            software_background=request.software_background,
            hardware_background=request.hardware_background
        )
        
        if not profile:
            raise HTTPException(status_code=404, detail="Student profile not found")
        
        return ProfileResponse(
            id=profile.id,
            email=profile.email,
            name=profile.name,
            software_background=profile.software_background,
            hardware_background=profile.hardware_background,
            created_at=profile.created_at,
            updated_at=profile.updated_at
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))