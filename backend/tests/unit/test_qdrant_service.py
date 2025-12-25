import pytest
import os
from unittest.mock import AsyncMock, MagicMock, patch

# Need to mock environment variables before QdrantService import
@pytest.fixture(autouse=True)
def mock_env_vars():
    with patch.dict(os.environ, {"QDRANT_URL": "http://mock-qdrant:6333", "QDRANT_API_KEY": "mock-api-key"}):
        yield

from backend.src.services.qdrant_service import QdrantService
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, ScoredPoint, PointStruct, Batch


@pytest.fixture
def mock_qdrant_client():
    with patch("backend.src.services.qdrant_service.QdrantClient", autospec=True) as mock_client_class:
        mock_instance = mock_client_class.return_value
        yield mock_instance

@pytest.fixture
def qdrant_service(mock_qdrant_client):
    return QdrantService()

async def test_qdrant_service_init_success(qdrant_service, mock_qdrant_client):
    assert qdrant_service.qdrant_url == "http://mock-qdrant:6333"
    assert qdrant_service.qdrant_api_key == "mock-api-key"
    mock_qdrant_client.assert_called_once_with(
        url="http://mock-qdrant:6333",
        api_key="mock-api-key",
    )
    assert qdrant_service.client is mock_qdrant_client # Checks if the instance is the mock

async def test_qdrant_service_init_missing_env_vars():
    with patch.dict(os.environ, {}, clear=True): # Clear env vars
        with pytest.raises(ValueError, match="QDRANT_URL and QDRANT_API_KEY environment variables must be set."):
            QdrantService()

async def test_create_collection(qdrant_service, mock_qdrant_client):
    collection_name = "test_collection"
    vector_size = 128
    result = await qdrant_service.create_collection(collection_name, vector_size)
    assert result is True
    mock_qdrant_client.recreate_collection.assert_called_once_with(
        collection_name=collection_name,
        vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
    )

async def test_upsert_vectors(qdrant_service, mock_qdrant_client):
    collection_name = "test_collection"
    ids = ["id1", "id2"]
    vectors = [[0.1, 0.2], [0.3, 0.4]]
    payloads = [{"meta": "data1"}, {"meta": "data2"}]
    
    result = await qdrant_service.upsert_vectors(collection_name, ids, vectors, payloads)
    assert result is True
    
    # Verify upsert was called with the correct arguments.
    # The actual points passed will be a list of PointStruct, so we check the structure.
    args, kwargs = mock_qdrant_client.upsert.call_args
    assert kwargs["collection_name"] == collection_name
    assert kwargs["wait"] is True
    
    points_arg = kwargs["points"]
    assert len(points_arg) == 2
    assert points_arg[0]["id"] == "id1"
    assert points_arg[0]["vector"] == [0.1, 0.2]
    assert points_arg[0]["payload"] == {"meta": "data1"}
    assert points_arg[1]["id"] == "id2"
    assert points_arg[1]["vector"] == [0.3, 0.4]
    assert points_arg[1]["payload"] == {"meta": "data2"}

async def test_search_vectors(qdrant_service, mock_qdrant_client):
    collection_name = "test_collection"
    query_vector = [0.5, 0.6]
    limit = 2
    
    mock_qdrant_client.search.return_value = [
        ScoredPoint(id="res1", score=0.9, payload={"text": "found text"}, vector=None),
        ScoredPoint(id="res2", score=0.8, payload={"text": "another text"}, vector=None),
    ]
    
    results = await qdrant_service.search_vectors(collection_name, query_vector, limit)
    
    assert len(results) == 2
    assert results[0].id == "res1"
    assert results[0].score == 0.9
    mock_qdrant_client.search.assert_called_once_with(
        collection_name=collection_name,
        query_vector=query_vector,
        query_filter=None,
        limit=limit,
    )

async def test_get_client(qdrant_service, mock_qdrant_client):
    client = await qdrant_service.get_client()
    assert client is mock_qdrant_client