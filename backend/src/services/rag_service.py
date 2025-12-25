from typing import List, Optional
from datetime import datetime
import uuid

from backend.src.services.gemini_service import GeminiService
from backend.src.services.qdrant_service import QdrantService
from backend.src.models.chatbot_response import ChatbotResponse
from backend.src.models.user_query import UserQuery
from backend.src.models.text_chunk import TextChunk # For payload structure from Qdrant
from backend.src.models.textbook_content import TextbookContent # Import for mock data
from backend.src.models.source_document import SourceDocument # Import SourceDocument

class RAGService:
    """
    Service for implementing the Retrieval-Augmented Generation (RAG) core logic.
    Integrates Gemini for embeddings and LLM, and Qdrant for vector retrieval.
    """
    def __init__(
        self,
        gemini_service: GeminiService,
        qdrant_service: QdrantService,
        qdrant_collection_name: str = "textbook_chunks"
    ):
        self.gemini_service = gemini_service
        self.qdrant_service = qdrant_service
        self.qdrant_collection_name = qdrant_collection_name

    # Mock function to simulate fetching source document details from a database
    async def _get_source_document_details(self, original_source_ref: str) -> Optional[SourceDocument]:
        """
        Simulates fetching SourceDocument details based on original_source_ref.
        In a real application, this would query a database.
        """
        # Using mock data similar to what's defined in api/admin.py mock_sources
        mock_db_sources = {
            "source-1": SourceDocument(
                document_id="source-1",
                title="Introduction to ROS 2",
                excerpt="ROS 2 (Robot Operating System 2) is a set of software libraries...",
                url="https://example.com/docs/ros2-intro"
            ),
            "source-2": SourceDocument(
                document_id="source-2",
                title="Physical AI Foundations",
                excerpt="This chapter explores the foundational concepts of physical AI...",
                url="https://example.com/docs/physical-ai"
            ),
            "intro-to-ros2-ch1-sec1": SourceDocument( # Example mapping from TextChunk's original_source_ref
                document_id="intro-to-ros2-ch1-sec1",
                title="ROS 2 Fundamentals - Introduction",
                excerpt="The Robot Operating System 2 (ROS 2) provides libraries and tools to help software developers create robot applications.",
                url="https://your-docusaurus-site.com/docs/week-3/ros2-fundamentals-intro"
            )
        }
        return mock_db_sources.get(original_source_ref)


    async def answer_query(self, user_query: UserQuery) -> ChatbotResponse:
        """
        Processes a user query using RAG:
        1. Generates embedding for the query.
        2. Retrieves relevant chunks from Qdrant.
        3. Constructs a prompt with the query and retrieved context.
        4. Generates a response using the LLM.
        """
        # 1. Generate embedding for the query
        query_embedding = await self.gemini_service.generate_embedding(user_query.text_content)
        if not query_embedding:
            return self._generate_error_response(user_query.id, "Failed to generate query embedding.")

        # 2. Retrieve relevant chunks from Qdrant
        retrieved_points = await self.qdrant_service.search_vectors(
            collection_name=self.qdrant_collection_name,
            query_vector=query_embedding,
            limit=5  # Retrieve top 5 relevant chunks
        )
        
        # Extract text content and source references from retrieved chunks
        context_chunks: List[str] = []
        unique_original_source_refs: set[str] = set()
        for point in retrieved_points:
            # Assuming payload contains 'text_content' and 'original_source_ref'
            if point.payload and 'text_content' in point.payload:
                context_chunks.append(point.payload['text_content'])
                if 'original_source_ref' in point.payload:
                    unique_original_source_refs.add(point.payload['original_source_ref'])
        
        # Fetch source document details for the unique references
        source_documents: List[SourceDocument] = []
        for ref in unique_original_source_refs:
            source_doc = await self._get_source_document_details(ref)
            if source_doc:
                source_documents.append(source_doc)

        if not context_chunks:
            # If no context is found, craft a prompt for a polite out-of-scope response
            out_of_scope_prompt = (
                f"The user asked: '{user_query.text_content}'. "
                "No relevant information was found in the knowledge base. "
                "Politely state that the question is outside the scope of the provided documents "
                "or that you cannot find relevant information within them. "
                "Do not attempt to answer the question using general knowledge."
            )
            llm_response_text = await self.gemini_service.generate_text(out_of_scope_prompt)
            if not llm_response_text:
                llm_response_text = "I'm sorry, I couldn't find relevant information within the available documents to answer your question."
            return ChatbotResponse(
                id=str(uuid.uuid4()),
                query_id=user_query.id,
                text_content=llm_response_text,
                timestamp=datetime.utcnow(),
                retrieved_chunks_refs=[],
                sources=[], # No sources if out of scope
                confidence_score=0.0 # Low confidence without retrieval
            )


        # 3. Construct a prompt with the query and retrieved context
        # This is a simplified prompt construction. More advanced techniques exist.
        context_str = "\n".join(context_chunks)
        llm_prompt = f"Given the following context:\n\n{context_str}\n\nAnswer the following question: {user_query.text_content}"

        # 4. Generate a response using the LLM
        llm_response_text = await self.gemini_service.generate_text(llm_prompt, context=context_chunks)
        if not llm_response_text:
            llm_response_text = "I'm sorry, I encountered an issue generating a response."
        
        # 5. Return the LLM response
        return ChatbotResponse(
            id=str(uuid.uuid4()),
            query_id=user_query.id,
            text_content=llm_response_text,
            timestamp=datetime.utcnow(),
            retrieved_chunks_refs=list(unique_original_source_refs), # Use the unique refs from chunks
            sources=source_documents, # Populate the new sources field
            confidence_score=1.0 # Assuming high confidence if context was found
        )

    def _generate_error_response(self, query_id: str, error_message: str) -> ChatbotResponse:
        """Helper to generate an error chatbot response."""
        return ChatbotResponse(
            id=str(uuid.uuid4()),
            query_id=query_id,
            text_content=f"An error occurred: {error_message}",
            timestamp=datetime.utcnow(),
            retrieved_chunks_refs=[],
            sources=[],
            confidence_score=0.0
        )

