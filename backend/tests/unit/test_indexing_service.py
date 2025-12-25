import pytest
from unittest.mock import AsyncMock, patch

from backend.src.services.indexing_service import IndexingService
from backend.src.models.textbook_content import TextbookContent

@pytest.fixture
def mock_ingestion_service():
    service = AsyncMock()
    service.ingest_document.return_value = AsyncMock(spec=TextbookContent) # Mock a TextbookContent object
    return service

@pytest.fixture
def indexing_service(mock_ingestion_service):
    return IndexingService(ingestion_service=mock_ingestion_service)

# --- Test IndexingService Initialization ---
def test_indexing_service_init(indexing_service, mock_ingestion_service):
    assert indexing_service.ingestion_service is mock_ingestion_service

# --- Test reindex_document ---
async def test_reindex_document_success(indexing_service, mock_ingestion_service):
    file_path = "/path/to/doc.md"
    metadata = {"chapter": "test"}
    
    result = await indexing_service.reindex_document(file_path, metadata)
    
    mock_ingestion_service.ingest_document.assert_called_once_with(file_path, metadata)
    assert result is not None
    assert isinstance(result, AsyncMock) # It's a mock object

async def test_reindex_document_ingestion_fails(indexing_service, mock_ingestion_service):
    file_path = "/path/to/fail.pdf"
    metadata = {"chapter": "fail"}
    mock_ingestion_service.ingest_document.return_value = None
    
    result = await indexing_service.reindex_document(file_path, metadata)
    
    mock_ingestion_service.ingest_document.assert_called_once_with(file_path, metadata)
    assert result is None

# --- Test reindex_all_sources ---
async def test_reindex_all_sources_success(indexing_service, mock_ingestion_service):
    sources_list = [
        {"file_path": "/path/to/doc1.md", "metadata": {"id": "doc1"}},
        {"file_path": "/path/to/doc2.md", "metadata": {"id": "doc2"}}
    ]
    
    results = await indexing_service.reindex_all_sources(sources_list)
    
    assert len(results) == 2
    assert mock_ingestion_service.ingest_document.call_count == 2
    mock_ingestion_service.ingest_document.assert_any_call("/path/to/doc1.md", {"id": "doc1"})
    mock_ingestion_service.ingest_document.assert_any_call("/path/to/doc2.md", {"id": "doc2"})

async def test_reindex_all_sources_some_fail(indexing_service, mock_ingestion_service):
    sources_list = [
        {"file_path": "/path/to/doc1.md", "metadata": {"id": "doc1"}},
        {"file_path": "/path/to/fail.pdf", "metadata": {"id": "fail"}},
        {"file_path": "/path/to/doc3.md", "metadata": {"id": "doc3"}}
    ]
    
    # Simulate first and third document succeeding, second failing
    mock_ingestion_service.ingest_document.side_effect = [
        AsyncMock(spec=TextbookContent), # Success for doc1
        None,                             # Failure for fail.pdf
        AsyncMock(spec=TextbookContent)  # Success for doc3
    ]
    
    results = await indexing_service.reindex_all_sources(sources_list)
    
    assert len(results) == 3
    assert results[0] is not None
    assert results[1] is None
    assert results[2] is not None
    assert mock_ingestion_service.ingest_document.call_count == 3

async def test_reindex_all_sources_empty_list(indexing_service, mock_ingestion_service):
    results = await indexing_service.reindex_all_sources([])
    assert len(results) == 0
    mock_ingestion_service.ingest_document.assert_not_called()
