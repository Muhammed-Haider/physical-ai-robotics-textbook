from pydantic import BaseModel, Field

class SourceDocument(BaseModel):
    """
    Represents a source document reference for a chatbot response.
    """
    document_id: str = Field(..., description="Unique identifier of the source document.")
    title: str = Field(..., description="Title of the source document.")
    excerpt: str = Field(..., description="Relevant snippet from the document.")
    url: str = Field(..., description="Link back to the original document.")

    class Config:
        schema_extra = {
            "example": {
                "document_id": "textbook-ch1-sec2",
                "title": "Chapter 1: Introduction to Physical AI",
                "excerpt": "This section provides an overview of physical AI concepts...",
                "url": "https://example.com/docs/ch1#section2"
            }
        }
