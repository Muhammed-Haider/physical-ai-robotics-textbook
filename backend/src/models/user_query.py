from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel, Field

class UserQuery(BaseModel):
    """
    A natural language question or input from the user via the chatbot interface.
    """
    id: str = Field(..., description="Unique identifier (primary key).")
    user_id: str = Field(..., description="Identifier of the user.")
    text_content: str = Field(
        ...,
        min_length=1,
        max_length=500,  # Arbitrary limit to prevent overly long queries
        description="The user's question."
    )
    timestamp: datetime = Field(..., description="Time of the query.")
    embedding_vector: List[float] = Field(..., description="Vector representation of text_content.")
    session_id: str = Field(..., description="Identifier for the conversational session.")

    class Config:
        schema_extra = {
            "example": {
                "id": "query-abc-123",
                "user_id": "user-456",
                "text_content": "What are the main features of ROS 2?",
                "timestamp": "2023-10-27T10:05:00Z",
                "embedding_vector": [0.4, 0.5, 0.6, ...], # Truncated for example
                "session_id": "session-xyz-789"
            }
        }
