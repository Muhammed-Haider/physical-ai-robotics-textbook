from datetime import datetime
from enum import Enum
from pydantic import BaseModel, Field

class ChatSessionState(str, Enum):
    """
    Defines the possible states for a chat session.
    """
    ACTIVE = "active"
    IDLE = "idle"
    CLOSED = "closed"

class ChatSession(BaseModel):
    """
    Represents the sequence of interactions between a user and the chatbot.
    """
    id: str = Field(..., description="Unique identifier (primary key) for the chat session.")
    user_id: str = Field(..., description="Identifier of the user associated with this session.")
    start_time: datetime = Field(..., description="Timestamp when the session started.")
    last_active_time: datetime = Field(..., description="Timestamp of the last activity in the session.")
    status: ChatSessionState = Field(..., description="Current status of the chat session.")

    class Config:
        schema_extra = {
            "example": {
                "id": "session-xyz-789",
                "user_id": "user-456",
                "start_time": "2023-10-27T10:00:00Z",
                "last_active_time": "2023-10-27T10:15:00Z",
                "status": "active"
            }
        }
