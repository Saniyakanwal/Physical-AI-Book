# Data Model: Physical AI & Humanoid Robotics Textbook

## Entity: Student Profile
- **Fields**:
  - id (string, unique, required)
  - email (string, unique, required)
  - name (string, required)
  - software_background (text, optional)
  - hardware_background (text, optional)
  - created_at (datetime, required)
  - updated_at (datetime, required)
  - preferences (json, optional)
- **Relationships**:
  - One-to-many with ChatSession
- **Validation**:
  - Email must be valid format
  - Name must be 2-50 characters
  - Background fields can be null

## Entity: Textbook Chapter
- **Fields**:
  - id (string, unique, required)
  - title (string, required)
  - module (string, required)
  - week (integer, required)
  - content (text, required)
  - content_ur (text, optional)
  - created_at (datetime, required)
  - updated_at (datetime, required)
- **Relationships**:
  - One-to-many with ChatSession (via context)
- **Validation**:
  - Title must be 5-200 characters
  - Module must be one of: "ROS2", "GazeboUnity", "NvidiaIsaac", "VisionLanguageAction"
  - Week must be 1-13

## Entity: Chat Session
- **Fields**:
  - id (string, unique, required)
  - student_id (string, required)
  - chapter_id (string, required)
  - messages (array of objects, required)
  - created_at (datetime, required)
  - updated_at (datetime, required)
- **Relationships**:
  - Many-to-one with Student Profile
  - Many-to-one with Textbook Chapter
- **Validation**:
  - Messages must contain at least one entry
  - Each message must have 'role' and 'content' fields

## Entity: Translation Resource
- **Fields**:
  - id (string, unique, required)
  - resource_type (string, required)
  - resource_id (string, required)
  - language_code (string, required)
  - translated_content (text, required)
  - created_at (datetime, required)
  - updated_at (datetime, required)
- **Relationships**:
  - Links to various resources (chapters, UI elements, etc.)
- **Validation**:
  - Language code must be valid ISO 639-1 code
  - Resource type must be one of: "chapter", "ui", "error_message"