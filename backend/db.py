import os
import asyncpg
from typing import Optional
from contextlib import asynccontextmanager
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

DATABASE_URL = os.getenv("NEON_DATABASE_URL")

if not DATABASE_URL:
    raise ValueError("NEON_DATABASE_URL environment variable is required")

class DatabaseManager:
    def __init__(self):
        self.pool = None

    async def connect(self):
        """Establish connection to the database"""
        try:
            self.pool = await asyncpg.create_pool(DATABASE_URL)
            print("Successfully connected to Neon database")
            
            # Initialize tables if they don't exist
            await self._initialize_tables()
        except Exception as e:
            print(f"Failed to connect to database: {e}")
            raise

    async def _initialize_tables(self):
        """Initialize necessary tables if they don't exist"""
        async with self.pool.acquire() as conn:
            # Create users table if it doesn't exist
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS users (
                    id SERIAL PRIMARY KEY,
                    email VARCHAR(255) UNIQUE NOT NULL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)
            
            # Create chat_sessions table if it doesn't exist
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS chat_sessions (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    user_id INTEGER REFERENCES users(id),
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    title VARCHAR(255)
                )
            """)
            
            # Create chat_messages table if it doesn't exist
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS chat_messages (
                    id SERIAL PRIMARY KEY,
                    session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
                    role VARCHAR(20) NOT NULL,  -- 'user' or 'assistant'
                    content TEXT NOT NULL,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)
            
            print("Database tables initialized")

    async def close(self):
        """Close the database connection"""
        if self.pool:
            await self.pool.close()

    @asynccontextmanager
    async def get_connection(self):
        """Get a database connection from the pool"""
        async with self.pool.acquire() as conn:
            yield conn

    async def get_user_by_email(self, email: str):
        """Get a user by email"""
        async with self.get_connection() as conn:
            query = "SELECT id, email, created_at FROM users WHERE email = $1"
            row = await conn.fetchrow(query, email)
            return dict(row) if row else None

    async def create_user(self, email: str):
        """Create a new user"""
        async with self.get_connection() as conn:
            query = """
                INSERT INTO users(email)
                VALUES($1)
                RETURNING id, email, created_at
            """
            row = await conn.fetchrow(query, email)
            return dict(row)

    async def create_chat_session(self, user_id: int, title: Optional[str] = None):
        """Create a new chat session"""
        async with self.get_connection() as conn:
            query = """
                INSERT INTO chat_sessions(user_id, title)
                VALUES($1, $2)
                RETURNING id, user_id, created_at
            """
            row = await conn.fetchrow(query, user_id, title)
            return dict(row)

    async def add_message_to_session(self, session_id, role: str, content: str):
        """Add a message to a chat session"""
        async with self.get_connection() as conn:
            query = """
                INSERT INTO chat_messages(session_id, role, content)
                VALUES($1, $2, $3)
                RETURNING id, session_id, role, content, timestamp
            """
            row = await conn.fetchrow(query, session_id, role, content)
            return dict(row)

    async def get_session_messages(self, session_id, limit: int = 50):
        """Get messages from a chat session"""
        async with self.get_connection() as conn:
            query = """
                SELECT role, content, timestamp
                FROM chat_messages
                WHERE session_id = $1
                ORDER BY timestamp ASC
                LIMIT $2
            """
            rows = await conn.fetch(query, session_id, limit)
            return [dict(row) for row in rows]


# Global instance
db_manager = DatabaseManager()