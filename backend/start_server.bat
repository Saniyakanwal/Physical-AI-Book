@echo off
setlocal

REM Physical AI Chatbot Backend Startup Script
REM This script starts the backend server with proper environment

echo Starting Physical AI Chatbot Backend...
echo.

REM Check if Python is available
python --version >nul 2>&1
if errorlevel 1 (
    echo Error: Python is not installed or not in PATH
    echo Please install Python 3.8 or higher
    pause
    exit /b 1
)

REM Check if virtual environment exists
if not exist "venv" (
    echo Creating virtual environment...
    python -m venv venv
)

REM Activate virtual environment
call venv\Scripts\activate.bat

REM Install dependencies if requirements.txt exists
if exist "requirements.txt" (
    echo Installing dependencies...
    pip install -r requirements.txt
)

REM Check if .env file exists
if not exist ".env" (
    echo.
    echo Warning: .env file not found
    echo Please create a .env file based on .env.example
    echo Copying .env.example to .env for reference...
    copy .env.example .env
    echo.
)

REM Start the server
echo Starting server on http://127.0.0.1:8000...
echo Press Ctrl+C to stop the server
echo.
python run_server.py

pause