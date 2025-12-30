# Physical AI & Humanoid Robotics Textbook

An interactive educational platform for learning Physical AI and Humanoid Robotics, featuring an intelligent chatbot, personalized learning paths, and multilingual support.

## Features

- Interactive textbook with embedded chatbot
- Personalized learning based on user background
- Multilingual support (English and Urdu)
- Comprehensive coverage of robotics topics
- Integration with ROS2, Gazebo/Unity, NVIDIA Isaac, and VLA models

## Tech Stack

- **Frontend**: Docusaurus (React-based static site generator)
- **Backend**: FastAPI (Python web framework)
- **Database**: Neon Postgres (serverless PostgreSQL)
- **Vector Database**: Qdrant (for RAG functionality)
- **AI Service**: OpenAI API
- **Authentication**: Better Auth
- **Deployment**: GitHub Pages (frontend), Vercel/Render (backend)

## Project Structure

```
physical-ai-book/
├── docs/                 # Docusaurus frontend
│   ├── src/
│   │   ├── components/   # React components (chatbot, auth, etc.)
│   │   ├── context/      # React contexts (language, etc.)
│   │   └── css/          # Custom styles
│   ├── modules/          # Textbook content by module
│   ├── weeks/            # Weekly breakdown content
│   └── ...
├── backend/              # FastAPI backend
│   ├── src/
│   │   ├── models/       # Pydantic models
│   │   ├── services/     # Business logic
│   │   └── api/          # API endpoints
│   ├── db.py             # Database management
│   ├── vector_db.py      # Vector database management
│   └── services.py       # RAG service
├── specs/                # Specification files
└── ...
```

## Setup

### Frontend (Docusaurus)

1. Navigate to the docs directory:
   ```bash
   cd docs
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm start
   ```

4. Open your browser to http://localhost:3000

### Backend (FastAPI)

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your configuration
   ```

5. Start the backend server:
   ```bash
   uvicorn main:app --reload
   ```

## Environment Variables

Create a `.env` file in the backend directory with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
NEON_DATABASE_URL=your_database_url_here
```

## Deployment

### Frontend to GitHub Pages

The frontend is automatically deployed to GitHub Pages when changes are pushed to the main branch. The workflow is defined in `.github/workflows/deploy.yml`.

### Backend

The backend can be deployed to platforms like Vercel or Render using the provided Dockerfile.

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add some amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

If you have any questions or need help, please open an issue in the GitHub repository.