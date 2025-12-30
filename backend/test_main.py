import pytest
import asyncio
from httpx import AsyncClient
from unittest.mock import patch, MagicMock
from main import app
import json

@pytest.mark.asyncio
async def test_health_endpoint():
    """Test the health check endpoint"""
    async with AsyncClient(app=app, base_url="http://test") as ac:
        response = await ac.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "healthy"}

@pytest.mark.asyncio
async def test_chat_endpoint_basic():
    """Test the chat endpoint with basic request"""
    # Mock the external services to avoid making real API calls
    with patch('services.RagService.process_chat_request') as mock_process:
        mock_process.return_value = ("This is a test response", "test-session-id")
        
        async with AsyncClient(app=app, base_url="http://test") as ac:
            payload = {
                "message": "What is Physical AI?",
                "selected_text": "Physical AI is a field..."
            }
            response = await ac.post("/chat", json=payload)
        
        assert response.status_code == 200
        data = response.json()
        assert "reply" in data
        assert "session_id" in data
        assert data["reply"] == "This is a test response"
        assert data["session_id"] == "test-session-id"

@pytest.mark.asyncio
async def test_chat_endpoint_with_user_email():
    """Test the chat endpoint with user email provided"""
    with patch('services.RagService.process_chat_request') as mock_process:
        mock_process.return_value = ("Response with user context", "user-session-id")
        
        async with AsyncClient(app=app, base_url="http://test") as ac:
            payload = {
                "message": "Tell me about chapter 3",
                "selected_text": "",
                "user_email": "test@example.com"
            }
            response = await ac.post("/chat", json=payload)
        
        assert response.status_code == 200
        data = response.json()
        assert data["reply"] == "Response with user context"

@pytest.mark.asyncio
async def test_chat_endpoint_with_session():
    """Test the chat endpoint with existing session"""
    with patch('services.RagService.process_chat_request') as mock_process:
        mock_process.return_value = ("Response in existing session", "existing-session-id")
        
        async with AsyncClient(app=app, base_url="http://test") as ac:
            payload = {
                "message": "Following up on my previous question",
                "selected_text": "Previous context",
                "session_id": "existing-session-id"
            }
            response = await ac.post("/chat", json=payload)
        
        assert response.status_code == 200
        data = response.json()
        assert data["reply"] == "Response in existing session"
        assert data["session_id"] == "existing-session-id"

@pytest.mark.asyncio
async def test_chat_endpoint_error_handling():
    """Test the chat endpoint error handling"""
    with patch('services.RagService.process_chat_request') as mock_process:
        mock_process.side_effect = Exception("Test error")
        
        async with AsyncClient(app=app, base_url="http://test") as ac:
            payload = {
                "message": "This should cause an error",
                "selected_text": ""
            }
            response = await ac.post("/chat", json=payload)
        
        assert response.status_code == 500
        data = response.json()
        assert "detail" in data

def test_main_imports():
    """Test that all necessary modules can be imported without error"""
    try:
        from main import app
        from db import db_manager
        from vector_db import qdrant_manager
        from services import RagService
        assert True
    except ImportError as e:
        pytest.fail(f"Import failed: {e}")

if __name__ == "__main__":
    pytest.main([__file__])