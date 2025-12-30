#!/usr/bin/env python3
"""
Script to run the Physical AI Book RAG Chatbot API
"""

import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def check_environment():
    """Check if required environment variables are set"""
    required_vars = [
        "NEON_DATABASE_URL",
        "QDRANT_URL", 
        "OPENAI_API_KEY"
    ]
    
    missing_vars = []
    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)
    
    if missing_vars:
        print("Warning: The following environment variables are not set:")
        for var in missing_vars:
            print(f"  - {var}")
        print("\nPlease set these variables in your .env file.")
        return False
    
    return True

def run_server():
    """Run the FastAPI server"""
    if not check_environment():
        print("\nStarting server anyway (some features may not work)...")
    else:
        print("All required environment variables are set!")
    
    print("\nStarting Physical AI Book RAG Chatbot API...")
    print("Visit http://localhost:8000/docs for API documentation\n")
    
    import uvicorn
    port = int(os.getenv("PORT", 8000))
    
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=port,
        reload=True,  # Enable auto-reload for development
        log_level="info"
    )

if __name__ == "__main__":
    run_server()