from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
import uuid

from ..services.translation_service import TranslationService
from ..models.translation import TranslationResource

# Create API router
router = APIRouter(prefix="/translation", tags=["translation"])

# Request models
class CreateTranslationRequest(BaseModel):
    resource_type: str  # chapter, ui, error_message
    resource_id: str    # ID of the original resource
    language_code: str  # ISO 639-1 language code
    translated_content: str

class UpdateTranslationRequest(BaseModel):
    translated_content: str

# Response models
class TranslationResponse(BaseModel):
    id: str
    resource_type: str
    resource_id: str
    language_code: str
    translated_content: str
    created_at: datetime
    updated_at: datetime

class GetTranslationResponse(BaseModel):
    chapter_id: str
    language: str
    title: str
    content: str
    last_updated: datetime

# Initialize service
translation_service = TranslationService()

@router.post("/", response_model=TranslationResponse)
async def create_translation(request: CreateTranslationRequest):
    """
    Create a new translation resource.
    """
    try:
        translation = await translation_service.create_translation(
            resource_type=request.resource_type,
            resource_id=request.resource_id,
            language_code=request.language_code,
            translated_content=request.translated_content
        )
        
        return TranslationResponse(
            id=translation.id,
            resource_type=translation.resource_type,
            resource_id=translation.resource_id,
            language_code=translation.language_code,
            translated_content=translation.translated_content,
            created_at=translation.created_at,
            updated_at=translation.updated_at
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/{resource_id}/translate", response_model=GetTranslationResponse)
async def get_translation(resource_id: str, lang: str):
    """
    Retrieves translated content for a specific resource in the requested language.
    """
    try:
        translation = await translation_service.get_translation(resource_id, lang)
        if not translation:
            raise HTTPException(status_code=404, detail="Translation not found")
        
        # For now, we'll use the resource_id as the title
        # In a real implementation, we'd look up the original resource to get the title
        return GetTranslationResponse(
            chapter_id=translation.resource_id,
            language=translation.language_code,
            title=f"Translated: {translation.resource_id}",
            content=translation.translated_content,
            last_updated=translation.updated_at
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/{resource_id}/translations", response_model=List[TranslationResponse])
async def get_all_translations_for_resource(resource_id: str):
    """
    Get all translations for a specific resource.
    """
    try:
        translations = await translation_service.get_all_translations_for_resource(resource_id)
        return [
            TranslationResponse(
                id=t.id,
                resource_type=t.resource_type,
                resource_id=t.resource_id,
                language_code=t.language_code,
                translated_content=t.translated_content,
                created_at=t.created_at,
                updated_at=t.updated_at
            ) for t in translations
        ]
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/supported-languages", response_model=List[str])
async def get_supported_languages():
    """
    Get a list of all supported language codes.
    """
    try:
        languages = await translation_service.get_supported_languages()
        return languages
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))