from typing import List, Optional
from ..models.translation import TranslationResource
import logging
import uuid
from datetime import datetime

logger = logging.getLogger(__name__)

class TranslationService:
    """
    Service class for handling translation operations
    """
    
    def __init__(self):
        # In a real implementation, this would connect to a database
        self.translations: List[TranslationResource] = []
        logger.info("TranslationService initialized")
        
        # Add some sample translations for Urdu
        self._add_sample_translations()

    def _add_sample_translations(self):
        """
        Add sample translations for demonstration purposes
        """
        # Add sample translation for ROS2 introduction
        sample_translation = TranslationResource(
            id=str(uuid.uuid4()),
            resource_type="chapter",
            resource_id="ros2-introduction",
            language_code="ur",
            translated_content="# ROS2 کا تعارف\n\nروبوٹ آپریٹنگ سسٹم 2 (ROS2) ایک لچکدار فریم ورک ہے جو روبوٹکس کے سافٹ ویئر کو لکھنے کے لیے استعمال ہوتا ہے...",
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow()
        )
        self.translations.append(sample_translation)

    async def get_translation(self, resource_id: str, language_code: str) -> Optional[TranslationResource]:
        """
        Retrieve a translation for a specific resource and language
        """
        for translation in self.translations:
            if translation.resource_id == resource_id and translation.language_code == language_code:
                return translation
        return None

    async def get_all_translations_for_resource(self, resource_id: str) -> List[TranslationResource]:
        """
        Retrieve all translations for a specific resource
        """
        return [t for t in self.translations if t.resource_id == resource_id]

    async def create_translation(self, resource_type: str, resource_id: str, 
                               language_code: str, translated_content: str) -> TranslationResource:
        """
        Create a new translation resource
        """
        translation_id = str(uuid.uuid4())
        now = datetime.utcnow()
        
        translation = TranslationResource(
            id=translation_id,
            resource_type=resource_type,
            resource_id=resource_id,
            language_code=language_code,
            translated_content=translated_content,
            created_at=now,
            updated_at=now
        )
        
        self.translations.append(translation)
        return translation

    async def update_translation(self, translation_id: str, translated_content: str) -> Optional[TranslationResource]:
        """
        Update an existing translation
        """
        for i, translation in enumerate(self.translations):
            if translation.id == translation_id:
                self.translations[i].translated_content = translated_content
                self.translations[i].updated_at = datetime.utcnow()
                return self.translations[i]
        return None

    async def get_supported_languages(self) -> List[str]:
        """
        Get a list of all supported language codes
        """
        languages = set()
        for translation in self.translations:
            languages.add(translation.language_code)
        return list(languages)