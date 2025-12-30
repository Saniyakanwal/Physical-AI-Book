#!/bin/bash

# Physical AI Chatbot Backend Startup Script
# This script starts the backend server with proper environment

echo "Starting Physical AI Chatbot Backend..."
echo

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    echo "Error: Python3 is not installed or not in PATH"
    echo "Please install Python 3.8 or higher"
    exit 1
fi

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Install dependencies if requirements.txt exists
if [ -f "requirements.txt" ]; then
    echo "Installing dependencies..."
    pip install -r requirements.txt
fi

# Check if .env file exists
if [ ! -f ".env" ]; then
    echo
    echo "Warning: .env file not found"
    echo "Please create a .env file based on .env.example"
    echo "Copying .env.example to .env for reference..."
    cp .env.example .env
    echo
fi

# Start the server
echo "Starting server on http://127.0.0.1:8000..."
echo "Press Ctrl+C to stop the server"
echo
python3 run_server.py