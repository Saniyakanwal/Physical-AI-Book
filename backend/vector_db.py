import os
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, VectorParams, Distance
from qdrant_client.http.models import Filter, FieldCondition, MatchValue
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

class QdrantManager:
    def __init__(self):
        self.url = os.getenv("QDRANT_URL")
        self.api_key = os.getenv("QDRANT_API_KEY")
        
        if not self.url:
            raise ValueError("QDRANT_URL environment variable is required")
        
        # Initialize Qdrant client
        if self.api_key:
            self.client = QdrantClient(url=self.url, api_key=self.api_key)
        else:
            self.client = QdrantClient(url=self.url)
    
    def initialize_collection(self, collection_name: str = "physical_ai_book"):
        """Initialize the collection if it doesn't exist"""
        try:
            collections = self.client.get_collections().collections
            collection_exists = any(col.name == collection_name for col in collections)
            
            if not collection_exists:
                # Create collection with vector parameters (assuming using text embeddings)
                # Using 1536 dimensions for OpenAI's text-embedding-ada-002
                self.client.create_collection(
                    collection_name=collection_name,
                    vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
                )
                
                # Create payload index for content field to improve filtering
                self.client.create_payload_index(
                    collection_name=collection_name,
                    field_name="source_doc",
                    field_schema="keyword"
                )
                
                logger.info(f"Created collection: {collection_name}")
            else:
                logger.info(f"Collection {collection_name} already exists")
                
        except Exception as e:
            logger.error(f"Error initializing collection: {e}")
            raise
    
    async def add_document_chunks(self, chunks: List[Dict], collection_name: str = "physical_ai_book"):
        """Add document chunks to the collection"""
        try:
            points = []
            for i, chunk in enumerate(chunks):
                point = PointStruct(
                    id=i,
                    vector=chunk.get('vector'),
                    payload={
                        "content": chunk.get('content'),
                        "source_doc": chunk.get('source_doc', ''),
                        "page_number": chunk.get('page_number', 0),
                        "chunk_index": i
                    }
                )
                points.append(point)
            
            # Upload batch of points
            self.client.upsert(
                collection_name=collection_name,
                points=points
            )
            
            logger.info(f"Added {len(chunks)} chunks to collection {collection_name}")
        except Exception as e:
            logger.error(f"Error adding document chunks: {e}")
            raise
    
    async def search_relevant_chunks(
        self, 
        query: str, 
        collection_name: str = "physical_ai_book", 
        limit: int = 5,
        filters: Optional[Dict] = None
    ) -> List[Dict]:
        """Search for relevant chunks based on the query"""
        try:
            # Perform semantic search
            search_result = self.client.search(
                collection_name=collection_name,
                query_text=query,
                limit=limit
            )
            
            # Format results
            relevant_chunks = []
            for hit in search_result:
                chunk = {
                    "content": hit.payload.get("content", ""),
                    "score": hit.score,
                    "source_doc": hit.payload.get("source_doc", ""),
                    "page_number": hit.payload.get("page_number", 0)
                }
                relevant_chunks.append(chunk)
            
            logger.info(f"Found {len(relevant_chunks)} relevant chunks for query: {query[:50]}...")
            return relevant_chunks
            
        except Exception as e:
            logger.error(f"Error searching for relevant chunks: {e}")
            raise
    
    async def search_with_selected_text(
        self, 
        selected_text: str, 
        collection_name: str = "physical_ai_book", 
        limit: int = 5
    ) -> List[Dict]:
        """Search for chunks specifically related to selected text"""
        try:
            # First, search with the selected text as priority
            high_priority_results = await self.search_relevant_chunks(
                selected_text, 
                collection_name, 
                limit=max(limit // 2, 1)  # Get half the results from selected text
            )
            
            # Then search with broader context if needed
            remaining_count = limit - len(high_priority_results)
            if remaining_count > 0:
                additional_results = await self.search_relevant_chunks(
                    f"{selected_text} related concepts", 
                    collection_name, 
                    limit=remaining_count
                )
                high_priority_results.extend(additional_results)
            
            return high_priority_results
            
        except Exception as e:
            logger.error(f"Error in priority search with selected text: {e}")
            # Fall back to regular search
            return await self.search_relevant_chunks(selected_text, collection_name, limit)


# Global instance
qdrant_manager = QdrantManager()

def get_qdrant_manager():
    """Dependency for FastAPI to get Qdrant manager instance"""
    return qdrant_manager