from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel, Field

from backend.src.models.source_document import SourceDocument # Import SourceDocument

class ChatbotResponse(BaseModel):
    """
    The generated natural language answer from the LLM based on retrieved text chunks.
    """
    id: str = Field(..., description="Unique identifier (primary key).")
    query_id: str = Field(..., description="Reference to the User Query it responds to.")
    text_content: str = Field(..., min_length=1, description="The generated answer.")
    timestamp: datetime = Field(..., description="Time of the response.")
    retrieved_chunks_refs: Optional[List[str]] = Field(
        None,
        description="List of references to Text Chunks used for generation (foreign keys to Text Chunk.id)."
    )
    sources: Optional[List[SourceDocument]] = Field( # New field for source documents
        None,
        description="List of source documents linked to this response."
    )
    confidence_score: Optional[float] = Field(
        None,
        ge=0.0,
        le=1.0,
        description="A measure of the response's confidence or relevance (0.0 to 1.0)."
    )

    class Config:
        schema_extra = {
            "example": {
                "id": "response-def-456",
                "query_id": "query-abc-123",
                "text_content": "The main features of ROS 2 include real-time communication, security, and a component-based architecture.",
                "timestamp": "2023-10-27T10:05:30Z",
                "retrieved_chunks_refs": [
                    "intro-to-ros2-ch1-sec1-chunk-0",
                    "intro-to-ros2-ch1-sec2-chunk-1"
                ],
                "sources": [ # Example for new field
                    {
                        "document_id": "textbook-ch1-sec1",
                        "title": "Introduction to ROS 2",
                        "excerpt": "ROS 2 (Robot Operating System 2) is a set of software libraries...",
                        "url": "https://example.com/docs/ros2-intro"
                    }
                ],
                "confidence_score": 0.85
            }
        }
