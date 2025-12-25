from datetime import datetime
from typing import Optional, Dict, Any
from pydantic import BaseModel, Field

class TextbookContent(BaseModel):
    """
    Represents the entire body of knowledge from the Physical AI & Humanoid Robotics textbook.
    """
    id: str = Field(..., description="Unique identifier (primary key).")
    text_content: str = Field(..., min_length=1, description="Full text of the document.")
    source_path: str = Field(..., description="Original file path or URL.")
    metadata: Optional[Dict[str, Any]] = Field(
        None,
        description="Additional information like chapter, section, week, topic.",
        example={
            "chapter": "Physical AI Foundations",
            "section": "Introduction",
            "week": "week-01",
            "topic": "Foundational Concepts"
        }
    )
    last_indexed_at: datetime = Field(..., description="Timestamp of last indexing.")

    class Config:
        schema_extra = {
            "example": {
                "id": "intro-to-ros2-ch1-sec1",
                "text_content": "ROS 2 (Robot Operating System 2) is a set of software libraries...",
                "source_path": "/docs/week-3/ros2-fundamentals-intro.md",
                "metadata": {
                    "chapter": "ROS 2 Fundamentals",
                    "section": "Introduction",
                    "week": "week-03",
                    "topic": "ROS 2"
                },
                "last_indexed_at": "2023-10-27T10:00:00Z"
            }
        }
