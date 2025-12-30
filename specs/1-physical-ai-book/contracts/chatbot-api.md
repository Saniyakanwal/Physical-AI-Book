# API Contract: Chatbot Service

## Overview
This document describes the API endpoints for the intelligent chatbot service that powers the Q&A functionality in the Physical AI & Humanoid Robotics textbook.

## Base URL
`https://api.physical-ai-book.com/chatbot`

## Endpoints

### POST /chat
Initiates a new chat session or continues an existing one with the textbook's intelligent chatbot.

#### Request
```json
{
  "message": "What is ROS 2 and how does it differ from ROS 1?",
  "student_id": "student-123",
  "chapter_id": "ros2-introduction",
  "selected_text": "ROS (Robot Operating System) is a flexible framework for writing robot software..."
}
```

#### Response
```json
{
  "reply": "ROS 2 is the next generation of the Robot Operating System...",
  "session_id": "session-456",
  "timestamp": "2025-01-07T10:30:00Z"
}
```

#### Error Response
```json
{
  "error": "Invalid request format",
  "details": "Message field is required"
}
```

### POST /auth/register
Registers a new student with background information.

#### Request
```json
{
  "email": "student@example.com",
  "name": "John Doe",
  "software_background": "Experienced with Python and C++",
  "hardware_background": "Familiar with Arduino and Raspberry Pi"
}
```

#### Response
```json
{
  "student_id": "student-123",
  "email": "student@example.com",
  "name": "John Doe",
  "created_at": "2025-01-07T10:30:00Z"
}
```

### GET /chapters/{chapterId}/translate
Retrieves translated content for a specific chapter in the requested language.

#### Request
```
GET /chapters/ros2-introduction/translate?lang=ur
```

#### Response
```json
{
  "chapter_id": "ros2-introduction",
  "language": "ur",
  "title": "ROS 2 کا تعارف",
  "content": "ROS 2 ایک مرکزی ڈھانچہ ہے جو روبوٹکس کی ترقی کے لیے استعمال ہوتا ہے...",
  "last_updated": "2025-01-07T10:30:00Z"
}
```

## Authentication
All endpoints except registration require a valid JWT token in the Authorization header:
```
Authorization: Bearer <jwt-token>
```

## Rate Limiting
All endpoints are rate-limited to 100 requests per hour per IP address.