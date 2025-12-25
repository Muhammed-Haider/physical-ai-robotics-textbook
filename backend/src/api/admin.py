from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel, Field
from typing import List, Any, Literal, Optional, Dict
from datetime import datetime

from backend.src.services.auth_service import AuthService
from backend.src.services.ingestion_service import IngestionService
from backend.src.services.document_parser import DocumentParser
from backend.src.services.gemini_service import GeminiService
from backend.src.services.qdrant_service import QdrantService
from backend.src.services.indexing_service import IndexingService # Import IndexingService

router = APIRouter()

# Dependency for authentication (mocked for now)
async def get_current_admin_user(auth_service: AuthService = Depends(AuthService)):
    # In a real application, this would verify a token and check user roles.
    # For now, we'll assume a successful authentication leads to an admin user.
    # This is a simplification based on the admin_api.yaml indicating 'AdminAuth'.
    # A proper implementation would use FastAPI's security features.
    
    # Placeholder: if a user is 'authenticated' as admin (e.g., via a header), return admin_user
    # For demonstration, we'll just return a dummy admin user
    # In a real scenario, this would involve token verification
    authenticated_admin = {"user_id": "admin_test_id", "username": "admin_user", "role": "admin"}
    if authenticated_admin and authenticated_admin["role"] == "admin":
        return authenticated_admin
    raise HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Not authenticated as admin",
        headers={"WWW-Authenticate": "Bearer"},
    )

# Dependency for IngestionService
async def get_ingestion_service(
    document_parser: DocumentParser = Depends(DocumentParser),
    gemini_service: GeminiService = Depends(GeminiService),
    qdrant_service: QdrantService = Depends(QdrantService)
) -> IngestionService:
    return IngestionService(
        document_parser=document_parser,
        gemini_service=gemini_service,
        qdrant_service=qdrant_service
    )

# Dependency for IndexingService
async def get_indexing_service(
    ingestion_service: IngestionService = Depends(get_ingestion_service)
) -> IndexingService:
    return IndexingService(ingestion_service=ingestion_service)

class IngestRequest(BaseModel):
    source_ids: List[str]

# Model for Source as defined in admin_api.yaml
class Source(BaseModel):
    id: str
    name: str
    type: Literal["local_directory", "github_repo"]
    path: str
    status: Literal["active", "inactive", "indexing"]

# Model for creating a new source
class CreateSourceRequest(BaseModel):
    name: str
    type: Literal["local_directory", "github_repo"]
    path: str

# Model for updating an existing source
class UpdateSourceRequest(BaseModel):
    name: Optional[str] = None
    type: Optional[Literal["local_directory", "github_repo"]] = None
    path: Optional[str] = None

# Model for a log entry
class LogEntry(BaseModel):
    timestamp: datetime
    level: Literal["INFO", "WARNING", "ERROR", "DEBUG"]
    message: str
    details: Optional[Dict[str, Any]] = None

# Model for updating RAG configuration
class RAGConfigUpdate(BaseModel):
    embedding_model: Optional[str] = None
    chunk_size: Optional[int] = Field(None, ge=100, le=2000)
    chunk_overlap: Optional[int] = Field(None, ge=0)
    llm_model: Optional[str] = None
    llm_temperature: Optional[float] = Field(None, ge=0.0, le=1.0)

# Mock storage for sources
mock_sources = [
    Source(
        id="source-1",
        name="Textbook Chapter 1",
        type="local_directory",
        path="/docs/chapter1.md",
        status="active"
    ),
    Source(
        id="source-2",
        name="GitHub Repo Docs",
        type="github_repo",
        path="https://github.com/example/docs",
        status="indexing"
    )
]

# Mock storage for logs
mock_logs: List[LogEntry] = [
    LogEntry(timestamp=datetime.now(), level="INFO", message="System started."),
    LogEntry(timestamp=datetime.now(), level="DEBUG", message="Debug message.", details={"component": "Auth"}),
    LogEntry(timestamp=datetime.now(), level="WARNING", message="Low disk space.", details={"disk": "/dev/sda1", "available": "10%"}),
    LogEntry(timestamp=datetime.now(), level="ERROR", message="Failed to connect to Qdrant.", details={"error_code": 500}),
]

# Mock RAG configuration (would typically be loaded from a database or config file)
mock_rag_config: Dict[str, Any] = {
    "embedding_model": "models/embedding-001",
    "chunk_size": 500,
    "chunk_overlap": 50,
    "llm_model": "gemini-pro",
    "llm_temperature": 0.7
}


@router.post("/admin/ingest", status_code=status.HTTP_202_ACCEPTED)
async def trigger_ingestion(
    request: IngestRequest,
    admin_user: Any = Depends(get_current_admin_user), # AdminAuth security
    ingestion_service: IngestionService = Depends(get_ingestion_service)
):
    """
    Trigger documentation ingestion from specified sources.
    """
    try:
        # In a real scenario, source_ids would map to actual file paths/URLs from a configuration
        # For now, we'll simulate processing these IDs.
        # This part of the logic needs to be enhanced to fetch actual source data.
        
        # Simulating sources to ingest (replace with actual source fetching logic)
        mock_sources_to_ingest = []
        for source_id in request.source_ids:
            # This is a mock-up. Real implementation needs to retrieve actual file_path and metadata
            # based on the source_id from a persistent store (e.g., a database of configured sources)
            mock_sources_to_ingest.append({
                "file_path": f"/mock/path/to/source/{source_id}.md",
                "metadata": {"source_id": source_id, "type": "mock"}
            })

        # Here, we would ideally pass the actual file paths and metadata to the ingestion service.
        # For this endpoint, we are just triggering the ingestion process.
        # The ingestion service itself would iterate through these and process.
        # For now, we'll just print and acknowledge.
        print(f"Admin '{admin_user['username']}' triggered ingestion for source IDs: {request.source_ids}")
        
        # If we were to actually trigger the full ingestion for each source:
        # for source_data in mock_sources_to_ingest:
        #    await ingestion_service.ingest_document(source_data["file_path"], source_data["metadata"])

        return {"message": "Ingestion process initiated for specified source IDs."}
    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Internal server error during ingestion: {e}"
        )

@router.get("/admin/sources", response_model=List[Source])
async def list_sources(admin_user: Any = Depends(get_current_admin_user)):
    """
    List configured documentation sources.
    """
    # In a real scenario, this would fetch sources from a database.
    # For now, return a mock list of sources.
    return mock_sources

@router.post("/admin/sources", status_code=status.HTTP_201_CREATED)
async def create_source(
    request: CreateSourceRequest,
    admin_user: Any = Depends(get_current_admin_user)
):
    """
    Configure a new documentation source.
    """
    # In a real scenario, this would store the new source configuration in a database.
    # For now, we'll simulate the creation.
    new_source_id = f"source-{len(mock_sources) + 1}"
    new_source = Source(id=new_source_id, name=request.name, type=request.type, path=request.path, status="active")
    mock_sources.append(new_source)
    print(f"Admin '{admin_user['username']}' configured new source: {new_source.name} ({new_source.type}) at {new_source.path}")
    
    return {"message": "Documentation source configured successfully.", "source_id": new_source_id}

@router.put("/admin/sources/{source_id}", response_model=Source)
async def update_source(
    source_id: str,
    request: UpdateSourceRequest,
    admin_user: Any = Depends(get_current_admin_user)
):
    """
    Update an existing documentation source.
    """
    global mock_sources # Declare intent to modify the global list

    for i, source in enumerate(mock_sources):
        if source.id == source_id:
            # Update fields if provided in the request
            if request.name is not None:
                mock_sources[i].name = request.name
            if request.type is not None:
                mock_sources[i].type = request.type
            if request.path is not None:
                mock_sources[i].path = request.path
            
            print(f"Admin '{admin_user['username']}' updated source {source_id}: {mock_sources[i]}")
            return mock_sources[i]
    
    raise HTTPException(
        status_code=status.HTTP_404_NOT_FOUND,
        detail=f"Source with ID '{source_id}' not found."
    )

@router.delete("/admin/sources/{source_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_source(
    source_id: str,
    admin_user: Any = Depends(get_current_admin_user)
):
    """
    Delete a documentation source.
    """
    global mock_sources # Declare intent to modify the global list

    initial_len = len(mock_sources)
    mock_sources[:] = [source for source in mock_sources if source.id != source_id]
    
    if len(mock_sources) < initial_len:
        print(f"Admin '{admin_user['username']}' deleted source {source_id}")
        return # 204 No Content
    else:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Source with ID '{source_id}' not found."
        )

@router.post("/admin/reindex", status_code=status.HTTP_202_ACCEPTED)
async def trigger_reindex(
    admin_user: Any = Depends(get_current_admin_user),
    indexing_service: IndexingService = Depends(get_indexing_service)
):
    """
    Manually trigger re-indexing of all active documentation sources.
    """
    print(f"Admin '{admin_user['username']}' triggered full re-indexing.")
    # In a real scenario, fetch actual sources from DB to pass to reindex_all_sources
    # For now, use the mock_sources list for demonstration
    sources_for_reindex = [{"file_path": s.path, "metadata": {"id": s.id, "name": s.name, "type": s.type}} for s in mock_sources if s.status == "active"]
    await indexing_service.reindex_all_sources(sources_for_reindex)
    return {"message": "Re-indexing process initiated."}

@router.get("/admin/logs", response_model=List[LogEntry])
async def get_logs(
    level: Literal["INFO", "WARNING", "ERROR", "DEBUG"] = "INFO",
    limit: int = 100,
    admin_user: Any = Depends(get_current_admin_user)
):
    """
    Retrieve system logs for the RAG service.
    """
    print(f"Admin '{admin_user['username']}' requested logs with level {level} and limit {limit}.")
    
    filtered_logs = [log for log in mock_logs if _log_level_filter(log.level, level)]
    
    return filtered_logs[:limit]

@router.post("/admin/rag/config")
async def update_rag_config(
    config_update: RAGConfigUpdate,
    admin_user: Any = Depends(get_current_admin_user)
):
    """
    Update RAG-specific configuration parameters.
    """
    global mock_rag_config # Declare intent to modify the global config

    updated_fields = []
    if config_update.embedding_model is not None:
        mock_rag_config["embedding_model"] = config_update.embedding_model
        updated_fields.append("embedding_model")
    if config_update.chunk_size is not None:
        mock_rag_config["chunk_size"] = config_update.chunk_size
        updated_fields.append("chunk_size")
    if config_update.chunk_overlap is not None:
        mock_rag_config["chunk_overlap"] = config_update.chunk_overlap
        updated_fields.append("chunk_overlap")
    if config_update.llm_model is not None:
        mock_rag_config["llm_model"] = config_update.llm_model
        updated_fields.append("llm_model")
    if config_update.llm_temperature is not None:
        mock_rag_config["llm_temperature"] = config_update.llm_temperature
        updated_fields.append("llm_temperature")
    
    if updated_fields:
        print(f"Admin '{admin_user['username']}' updated RAG config fields: {', '.join(updated_fields)}. New config: {mock_rag_config}")
        return {"message": "RAG configuration updated successfully.", "updated_fields": updated_fields, "new_config": mock_rag_config}
    else:
        return {"message": "No RAG configuration fields provided for update."}

def _log_level_filter(log_level: str, filter_level: str) -> bool:
    """Helper function to filter logs by level."""
    levels = {"DEBUG": 0, "INFO": 1, "WARNING": 2, "ERROR": 3}
    return levels.get(log_level, 0) >= levels.get(filter_level, 0)

