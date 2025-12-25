from enum import Enum
from typing import Optional
from pydantic import BaseModel, Field

class UserRole(str, Enum):
    """
    Defines the possible roles for a user.
    """
    END_USER = "end-user"
    ADMIN = "admin"

class User(BaseModel):
    """
    Represents a user of the system (end-user or administrator).
    """
    id: str = Field(..., description="Unique identifier (primary key).")
    role: UserRole = Field(..., description="User role ('end-user' or 'admin').")
    authentication_details: Optional[str] = Field(
        None,
        description="Reference to an external authentication system (e.g., Better Auth user_id)."
    )

    class Config:
        schema_extra = {
            "example": {
                "id": "user-123",
                "role": "end-user",
                "authentication_details": "better-auth-user-id-abc"
            }
        }
