from typing import List, Dict, Any, Optional
from pydantic import BaseModel, Field

class TextChunk(BaseModel):
    """
    Smaller, embedded segments derived from the Textbook Content.
    These are stored in the Vector Database (Qdrant Cloud).
    """
    id: str = Field(..., description="Unique identifier (primary key).")
    text_content: str = Field(..., min_length=1, description="Segment of text.")
    embedding_vector: List[float] = Field(..., description="Vector representation of text_content.")
    original_source_ref: str = Field(..., description="Reference to the Textbook Content from which it was derived.")
    metadata: Optional[Dict[str, Any]] = Field(
        None,
        description="Inherited or enriched metadata (e.g., page number, paragraph index).",
        example={
            "page_number": 5,
            "paragraph_index": 2
        }
    )

    class Config:
        schema_extra = {
            "example": {
                "id": "intro-to-ros2-ch1-sec1-chunk-0",
                "text_content": "ROS 2 (Robot Operating System 2) is a set of software libraries...",
                "embedding_vector": [0.1, 0.2, 0.3, ...], # Truncated for example
                "original_source_ref": "intro-to-ros2-ch1-sec1",
                "metadata": {
                    "page_number": 5,
                    "paragraph_index": 2
                }
            }
        }
