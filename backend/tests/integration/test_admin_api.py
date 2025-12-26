import pytest
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch, MagicMock
from datetime import datetime
from typing import List, Dict, Any

# Import the main FastAPI app and the admin router
from main import app as fastapi_app
from src.api.admin import router as admin_router
from src.api.admin import mock_sources, mock_rag_config # Access mock data

# Fixture for the TestClient
@pytest.fixture(scope="module")
def client():
    # Mount the admin router to a FastAPI application for testing
    # In a real app, you'd import your main app and inject dependencies properly
    test_app = MagicMock() # Create a mock FastAPI app
    test_app.include_router = MagicMock() # Mock include_router method
    test_app.include_router(admin_router, prefix="/api/v1")

    # Override dependencies for testing
    with patch("src.api.admin.AuthService", autospec=True) as MockAuthService, \
         patch("src.api.admin.IngestionService", autospec=True) as MockIngestionService, \
         patch("src.api.admin.IndexingService", autospec=True) as MockIndexingService, \
         patch("src.api.admin.DocumentParser", autospec=True) as MockDocumentParser, \
         patch("src.api.admin.GeminiService", autospec=True) as MockGeminiService, \
         patch("src.api.admin.QdrantService", autospec=True) as MockQdrantService:
        
        # Configure the mock AuthService to always authenticate as admin for testing
        mock_auth_instance = MockAuthService.return_value
        mock_auth_instance.authenticate_user = AsyncMock(return_value={"user_id": "admin_test", "username": "admin", "role": "admin"})
        mock_auth_instance.verify_token = AsyncMock(return_value={"user_id": "admin_test", "username": "admin", "role": "admin"})

        # Configure mock IngestionService
        mock_ingestion_instance = MockIngestionService.return_value
        mock_ingestion_instance.ingest_document = AsyncMock(return_value=MagicMock())

        # Configure mock IndexingService
        mock_indexing_instance = MockIndexingService.return_value
        mock_indexing_instance.reindex_all_sources = AsyncMock(return_value=[])

        # Configure other service mocks
        MockDocumentParser.return_value = AsyncMock()
        MockGeminiService.return_value = AsyncMock()
        MockQdrantService.return_value = AsyncMock()
        
        # Create TestClient with the router
        # Note: Directly testing router without a full FastAPI app often requires some dependency overrides
        # For simplicity here, we'll patch the dependencies within the admin.py scope itself
        # This approach is less ideal but avoids needing a full app setup if only testing router
        with TestClient(admin_router) as c:
             yield c

# Fixture to reset mock_sources and mock_rag_config before each test
@pytest.fixture(autouse=True)
def reset_mock_data():
    global mock_sources
    global mock_rag_config
    mock_sources[:] = [
        {"id": "source-1", "name": "Textbook Chapter 1", "type": "local_directory", "path": "/docs/chapter1.md", "status": "active"},
        {"id": "source-2", "name": "GitHub Repo Docs", "type": "github_repo", "path": "https://github.com/example/docs", "status": "indexing"},
    ]
    mock_rag_config.update({
        "embedding_model": "models/embedding-001",
        "chunk_size": 500,
        "chunk_overlap": 50,
        "llm_model": "gemini-pro",
        "llm_temperature": 0.7
    })
    yield

# --- Helper for authorization header ---
def get_auth_header():
    # Since auth is mocked to always succeed for admin, any bearer token will work
    return {"Authorization": "Bearer mock_admin_token"}

# --- Test Authentication ---
async def test_admin_endpoints_require_authentication(client):
    response = client.post("/admin/ingest", json={"source_ids": ["test-id"]})
    assert response.status_code == 401
    assert "Not authenticated as admin" in response.json()["detail"]

    response = client.get("/admin/sources")
    assert response.status_code == 401
    
    response = client.post("/admin/sources", json={"name": "test", "type": "local_directory", "path": "/path"})
    assert response.status_code == 401

    response = client.put("/admin/sources/source-1", json={"name": "updated"})
    assert response.status_code == 401

    response = client.delete("/admin/sources/source-1")
    assert response.status_code == 401

    response = client.post("/admin/reindex")
    assert response.status_code == 401

    response = client.get("/admin/logs")
    assert response.status_code == 401

    response = client.post("/admin/rag/config", json={"chunk_size": 600})
    assert response.status_code == 401

# --- Test POST /admin/ingest ---
async def test_post_admin_ingest_success(client, mock_ingestion_service):
    response = client.post("/admin/ingest", headers=get_auth_header(), json={"source_ids": ["source-new-1", "source-new-2"]})
    assert response.status_code == 202
    assert response.json()["message"] == "Ingestion process initiated for specified source IDs."
    # Note: the mock_ingestion_service.ingest_document is not called in the API endpoint's current mock logic
    # It prints to console. If actual ingestion was desired, we'd mock a different layer.

# --- Test GET /admin/sources ---
async def test_get_admin_sources_success(client):
    response = client.get("/admin/sources", headers=get_auth_header())
    assert response.status_code == 200
    sources = response.json()
    assert len(sources) == 2
    assert sources[0]["id"] == "source-1"
    assert sources[1]["name"] == "GitHub Repo Docs"

# --- Test POST /admin/sources ---
async def test_post_admin_sources_success(client):
    new_source_data = {"name": "New Doc", "type": "local_directory", "path": "/new/doc.md"}
    response = client.post("/admin/sources", headers=get_auth_header(), json=new_source_data)
    assert response.status_code == 201
    assert "source_id" in response.json()
    assert response.json()["message"] == "Documentation source configured successfully."

    # Verify that the mock_sources list was updated
    global mock_sources
    assert len(mock_sources) == 3
    assert mock_sources[2].name == "New Doc"

async def test_post_admin_sources_invalid_data(client):
    invalid_source_data = {"name": "New Doc", "type": "invalid_type", "path": "/new/doc.md"}
    response = client.post("/admin/sources", headers=get_auth_header(), json=invalid_source_data)
    assert response.status_code == 422 # Unprocessable Entity for Pydantic validation error

# --- Test PUT /admin/sources/{source_id} ---
async def test_put_admin_sources_success(client):
    update_data = {"name": "Updated Chapter 1", "status": "inactive"} # status field won't be updated by endpoint current mock
    response = client.put("/admin/sources/source-1", headers=get_auth_header(), json=update_data)
    assert response.status_code == 200
    updated_source = response.json()
    assert updated_source["id"] == "source-1"
    assert updated_source["name"] == "Updated Chapter 1"

async def test_put_admin_sources_not_found(client):
    response = client.put("/admin/sources/non-existent-source", headers=get_auth_header(), json={"name": "new name"})
    assert response.status_code == 404
    assert response.json()["detail"] == "Source with ID 'non-existent-source' not found."

# --- Test DELETE /admin/sources/{source_id} ---
async def test_delete_admin_sources_success(client):
    response = client.delete("/admin/sources/source-1", headers=get_auth_header())
    assert response.status_code == 204
    
    # Verify that the mock_sources list was updated
    global mock_sources
    assert len(mock_sources) == 1
    assert mock_sources[0].id == "source-2"

async def test_delete_admin_sources_not_found(client):
    response = client.delete("/admin/sources/non-existent-source", headers=get_auth_header())
    assert response.status_code == 404
    assert response.json()["detail"] == "Source with ID 'non-existent-source' not found."

# --- Test POST /admin/reindex ---
async def test_post_admin_reindex_success(client, mock_indexing_service):
    response = client.post("/admin/reindex", headers=get_auth_header())
    assert response.status_code == 202
    assert response.json()["message"] == "Re-indexing process initiated."
    # Verify mock_indexing_service.reindex_all_sources was called
    mock_indexing_service.reindex_all_sources.assert_called_once()

# --- Test GET /admin/logs ---
async def test_get_admin_logs_success(client):
    response = client.get("/admin/logs", headers=get_auth_header())
    assert response.status_code == 200
    logs = response.json()
    assert len(logs) > 0
    assert logs[0]["level"] == "INFO"

async def test_get_admin_logs_filter_level_and_limit(client):
    response = client.get("/admin/logs?level=ERROR&limit=1", headers=get_auth_header())
    assert response.status_code == 200
    logs = response.json()
    assert len(logs) == 1
    assert logs[0]["level"] == "ERROR"

# --- Test POST /admin/rag/config ---
async def test_post_admin_rag_config_success(client):
    update_data = {"chunk_size": 700, "llm_temperature": 0.5}
    response = client.post("/admin/rag/config", headers=get_auth_header(), json=update_data)
    assert response.status_code == 200
    assert response.json()["message"] == "RAG configuration updated successfully."
    assert "chunk_size" in response.json()["updated_fields"]
    assert response.json()["new_config"]["chunk_size"] == 700
    assert response.json()["new_config"]["llm_temperature"] == 0.5

    # Verify that the mock_rag_config was updated
    global mock_rag_config
    assert mock_rag_config["chunk_size"] == 700
    assert mock_rag_config["llm_temperature"] == 0.5

async def test_post_admin_rag_config_invalid_data(client):
    invalid_data = {"chunk_size": 50, "llm_temperature": 1.5} # chunk_size too low, llm_temperature too high
    response = client.post("/admin/rag/config", headers=get_auth_header(), json=invalid_data)
    assert response.status_code == 422 # Unprocessable Entity for Pydantic validation error
