import logging
from typing import List, Dict, Optional
from .vector_db import qdrant_manager
from .db import db_manager
import openai
import os
from uuid import UUID

logger = logging.getLogger(__name__)

class RagService:
    def __init__(self):
        self.qdrant = qdrant_manager
        self.db = db_manager

        # Initialize OpenAI client
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")
        self.openai_client = openai.AsyncOpenAI(api_key=openai_api_key)

    async def process_chat_request(
        self,
        message: str,
        selected_text: Optional[str] = None,
        user_email: Optional[str] = None,
        session_id: Optional[str] = None
    ) -> tuple[str, str]:
        """
        Process a chat request using RAG methodology
        Returns: (reply, session_id)
        """

        # Initialize session if needed
        session_uuid = await self._get_or_create_session(user_email, session_id, message)

        # Store the user's message in DB
        if session_uuid:
            await self.db.add_message_to_session(session_uuid, "user", message)

        # Retrieve relevant context from vector store
        context = await self._retrieve_context(message, selected_text)

        # Generate response using OpenAI
        reply = await self._generate_response(message, context, selected_text)

        # Store the assistant's reply in DB
        if session_uuid:
            await self.db.add_message_to_session(session_uuid, "assistant", reply)

        return reply, str(session_uuid) if session_uuid else session_id

    async def _get_or_create_session(
        self,
        user_email: Optional[str],
        provided_session_id: Optional[str],
        first_message: str
    ) -> Optional[UUID]:
        """Get existing session or create a new one"""

        if provided_session_id:
            # Assuming the session ID is valid if provided
            try:
                return UUID(provided_session_id) if provided_session_id else None
            except ValueError:
                logger.warning(f"Invalid session ID provided: {provided_session_id}")
                # Create a new session instead
                pass

        user_id = None
        if user_email:
            user = await self.db.get_user_by_email(user_email)
            if not user:
                user = await self.db.create_user(user_email)
            user_id = user['id']

        # Create new session
        session_data = await self.db.create_chat_session(
            user_id,
            title=first_message[:50] + "..." if len(first_message) > 50 else first_message
        )

        return session_data['id']

    async def _retrieve_context(self, message: str, selected_text: Optional[str]) -> str:
        """Retrieve relevant context from the vector store"""

        try:
            relevant_chunks = []

            if selected_text:
                # Prioritize content related to selected text
                relevant_chunks = await self.qdrant.search_with_selected_text(
                    selected_text,
                    limit=5
                )
            else:
                # Search based on user message
                relevant_chunks = await self.qdrant.search_relevant_chunks(
                    message,
                    limit=5
                )

            # Combine relevant chunks into a single context string
            context_parts = []
            for chunk in relevant_chunks:
                content = chunk.get("content", "")
                if content.strip():
                    context_parts.append(content)

            context = "\n\n".join(context_parts)

            if not context.strip():
                # If no relevant content found, use a default message
                context = "No relevant content from the Physical AI book was found to answer this question."

            logger.info(f"Retrieved context with {len(context)} characters")
            return context

        except Exception as e:
            logger.error(f"Error retrieving context: {e}")
            return "Error retrieving context. Please try again."

    async def _generate_response(self, message: str, context: str, selected_text: Optional[str] = None) -> str:
        """Generate response using OpenAI based on the context"""

        try:
            # Build system prompt based on whether specific text was selected
            if selected_text:
                system_prompt = f"""
                You are an AI assistant for the Physical AI Book. Your purpose is to answer questions based on the provided context from the book.
                The user has selected specific text: "{selected_text}"
                Focus your response on explaining or expanding upon this selected text.
                If the answer to the user's question is not contained in the provided context, respond with: 'I cannot answer that question based on the Physical AI book content provided.'

                Context:
                {context}
                """
            else:
                system_prompt = f"""
                You are an AI assistant for the Physical AI Book. Your purpose is to answer questions based solely on the provided context from the book.
                If the answer to the user's question is not contained in the provided context, respond with: 'I cannot answer that question based on the Physical AI book content provided.'

                Context:
                {context}
                """

            response = await self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": message}
                ],
                temperature=0.3,
                max_tokens=500
            )

            reply = response.choices[0].message.content.strip()
            logger.info(f"Generated response with {len(reply)} characters")

            return reply

        except openai.APIError as e:
            logger.error(f"OpenAI API error: {e}")
            return "Sorry, I'm having trouble connecting to the AI service. Please try again later."
        except Exception as e:
            logger.error(f"Error generating response: {e}")
            return "Sorry, I encountered an error while generating a response. Please try again."