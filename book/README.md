# Physical AI & Humanoid Robotics Book

A comprehensive guide to building intelligent physical systems with advanced AI capabilities, featuring interactive learning tools and multilingual support.

## 📘 Overview

This interactive book combines cutting-edge robotics knowledge with modern AI tools to make learning about Physical AI and Humanoid Robotics accessible and engaging. The book features a RAG chatbot, intelligent subagents, personalization features, and multilingual capabilities.

## 📚 Book Chapters

Access the complete book at: [Physical AI & Humanoid Robotics](https://physical-ai-book.docusaurus.io) (Coming Soon)

## 🤖 RAG Chatbot Features

- **Intelligent Q&A**: Ask questions about robotics concepts and get context-aware answers from the book content
- **Code Examples**: Get explanations of ROS 2 code snippets with practical examples
- **Real-time Interaction**: Engage in conversations about complex robotics topics
- **Knowledge Graph**: Understand relationships between different robotics concepts

## 🧠 Subagents System

- **Navigation Agent**: Handles path planning and movement in simulation environments
- **Manipulation Agent**: Manages robotic arm control and object manipulation
- **Perception Agent**: Processes sensor data and computer vision tasks
- **Integration Agent**: Coordinates between different system components
- **Autonomous Decision Making**: Each agent can operate independently while collaborating

## 🎯 Personalization & Urdu Features

### Personalization Engine
- **Adaptive Learning**: Content tailored to your experience level and background
- **Custom Paths**: Suggested learning paths based on your interests and goals
- **Progressive Difficulty**: Adjusts complexity based on your understanding

### Urdu Localization
- **Technical Translation**: Formal and technical Urdu translations of all content
- **Cultural Context**: Examples and analogies adapted for Urdu-speaking audiences
- **Dual Language Support**: Toggle between English and Urdu content seamlessly

## 🛠️ Setup Instructions

### Prerequisites
- Node.js (v18 or higher)
- Python (v3.9 or higher)
- Git
- Docker (for local Qdrant setup)

### Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/physical-ai-book.git
cd physical-ai-book
```

2. Install Docusaurus dependencies:
```bash
npm install
```

3. Set up Python environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

4. Install FastAPI dependencies:
```bash
pip install fastapi uvicorn python-multipart python-dotenv
pip install qdrant-client openai langchain-community langchain-core
pip install better-auth drizzle-orm @neondatabase/serverless
```

5. Create environment file:
```bash
cp .env.example .env
```

6. Configure environment variables in `.env`:
```env
DATABASE_URL=your_neon_database_url
AUTH_SECRET=your_auth_secret
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
OPENAI_API_KEY=your_openai_api_key
```

### Running the Application

1. Start Docusaurus (docs interface):
```bash
npm start
```

2. Start FastAPI backend (in separate terminal):
```bash
uvicorn main:app --reload
```

## 🗄️ Neon + Qdrant Setup

### Neon Database Configuration

1. Create a free Neon account at [neon.tech](https://neon.tech)
2. Create a new project and get the connection string
3. The schema includes:
   - User profiles with customization options
   - Authentication tables
   - Learning progress tracking
   - Custom fields for robotics background

### Qdrant Vector Database

1. Deploy Qdrant (cloud or self-hosted):
   ```bash
   # Using Docker
   docker run -d --name qdrant -p 6333:6333 qdrant/qdrant
   
   # Or use Qdrant Cloud
   ```

2. Collection setup for RAG system:
   - `book_content`: Stores book chapters and concepts
   - `code_snippets`: Stores ROS 2 code examples
   - `user_interactions`: Stores personalization data

### Configuration

In your `.env` file:
```env
NEON_DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require
QDRANT_URL=https://your-cluster.qdrant.tech
QDRANT_API_KEY=your_api_key
```

## 🚀 Deployment Instructions

### Docusaurus Frontend

1. **Vercel Deployment** (Recommended):
```bash
vercel --prod
```

2. **GitHub Pages**:
```bash
npm run build
npm run deploy
```

3. **Netlify**:
- Connect your GitHub repository
- Set build command: `npm run build`
- Set publish directory: `build`

### FastAPI Backend

1. **Railway** (Recommended):
```bash
npm install -g @railway/cli
railway login
railway init
railway up
```

2. **Render**:
- Create a new Web Service
- Repository: your repository
- Build command: `pip install -r requirements.txt`
- Start command: `uvicorn main:app:app --host 0.0.0.0 --port $PORT`

3. **Docker Deployment**:
```dockerfile
FROM python:3.10-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt

COPY . .

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Environment Configuration for Production

Set these environment variables in your deployment platform:

```env
DATABASE_URL=your_production_neon_url
AUTH_SECRET=your_production_secret
QDRANT_URL=your_production_qdrant_url
QDRANT_API_KEY=your_production_qdrant_key
OPENAI_API_KEY=your_production_openai_key
NEXTAUTH_URL=your_production_url
```

## 🏗️ Architecture

- **Frontend**: Docusaurus with React components
- **Backend**: FastAPI with async support
- **Database**: Neon (PostgreSQL) for structured data
- **Vector Store**: Qdrant for semantic search
- **Authentication**: Better-Auth with custom fields
- **Deployment**: Vercel (frontend), Railway/Render (backend)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Support

For support, please open an issue in the GitHub repository or contact us at [your-email@example.com].

---

Built with ❤️ using Docusaurus, FastAPI, Neon, Qdrant, and OpenAI.