# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Development Setup

### Prerequisites
- Node.js 18+ with npm
- Python 3.8+
- Git

### Frontend Setup (Docusaurus)
1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd physical-ai-book
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

### Backend Setup (FastAPI)
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
   uvicorn src.main:app --reload
   ```

## Adding Content

### Creating a New Chapter
1. Create a new MDX file in the `docs/modules/[module-name]/` directory
2. Add frontmatter with title, module, and week information:
   ```md
   ---
   title: Chapter Title
   module: ROS2
   week: 1
   sidebar_position: 1
   ---
   ```
3. Include interactive elements using Docusaurus components
4. Update the sidebar configuration in `sidebars.js`

### Adding Interactive Elements
Use the custom ChatInterface component to embed chat functionality:
```jsx
import ChatInterface from '@site/src/components/chatbot/ChatInterface';

<ChatInterface chapterId="ros2-introduction" />
```

## Running Tests

### Frontend Tests
```bash
npm test
```

### Backend Tests
```bash
cd backend
python -m pytest
```

## Building for Production

### Frontend
```bash
npm run build
```

### Backend
The backend is deployed separately using Docker:
```bash
docker build -t physical-ai-book-backend .
docker run -p 8000:8000 physical-ai-book-backend
```

## Deployment

### Frontend (GitHub Pages)
1. Ensure your GitHub Pages is enabled in repository settings
2. Run the deployment command:
   ```bash
   npm run deploy
   ```

### Backend (Vercel/Render)
Follow the platform-specific deployment instructions with the provided Dockerfile.

## Troubleshooting

### Common Issues
1. **Port already in use**: Change the PORT variable in your .env file
2. **Dependency conflicts**: Clear node_modules and reinstall: `rm -rf node_modules && npm install`
3. **Python path issues**: Ensure you're using the virtual environment
4. **API connection errors**: Verify backend is running and API URL is correct in frontend config