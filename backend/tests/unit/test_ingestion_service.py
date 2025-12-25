import pytest
import asyncio
from unittest.mock import AsyncMock, patch, MagicMock
from datetime import datetime
import uuid

from backend.src.services.ingestion_service import IngestionService
from backend.src.models.textbook_content import TextbookContent
from backend.src.models.text_chunk import TextChunk
from qdrant_client.models import Distance

# Fixtures for mocked dependencies
@pytest.fixture
def mock_document_parser():
    parser = AsyncMock()
    parser.parse_document.return_value = "This is a sample document content. It has multiple sentences. And some more text for chunking."
    return parser

@pytest.fixture
def mock_gemini_service():
    gemini = AsyncMock()
    gemini.generate_embedding.return_value = [0.1] * 768  # Mock embedding vector
    return gemini

@pytest.fixture
def mock_qdrant_service():
    qdrant = AsyncMock()
    qdrant.create_collection.return_value = True
    qdrant.upsert_vectors.return_value = True
    return qdrant

@pytest.fixture
def ingestion_service(mock_document_parser, mock_gemini_service, mock_qdrant_service):
    return IngestionService(
        document_parser=mock_document_parser,
        gemini_service=mock_gemini_service,
        qdrant_service=mock_qdrant_service
    )

@pytest.fixture
def mock_uuid():
    with patch("backend.src.services.ingestion_service.uuid.uuid4") as mock_uuid4:
        mock_uuid4.return_value = uuid.UUID('12345678-1234-5678-1234-567812345678')
        yield mock_uuid4

# --- Test IngestionService Initialization ---
def test_ingestion_service_init(ingestion_service, mock_document_parser, mock_gemini_service, mock_qdrant_service):
    assert ingestion_service.document_parser is mock_document_parser
    assert ingestion_service.gemini_service is mock_gemini_service
    assert ingestion_service.qdrant_service is mock_qdrant_service
    assert ingestion_service.qdrant_collection_name == "textbook_chunks"
    assert ingestion_service.vector_size == 768

# --- Test initialize_qdrant_collection ---
async def test_initialize_qdrant_collection(ingestion_service, mock_qdrant_service):
    await ingestion_service.initialize_qdrant_collection()
    mock_qdrant_service.create_collection.assert_called_once_with(
        collection_name="textbook_chunks",
        vector_size=768,
        distance=Distance.COSINE
    )

# --- Test ingest_document ---
async def test_ingest_document_success(ingestion_service, mock_document_parser, mock_gemini_service, mock_qdrant_service, mock_uuid):
    file_path = "/path/to/doc.md"
    metadata = {"chapter": "test", "week": "week-00"}

    # Mock parse_document to return content
    mock_document_parser.parse_document.return_value = "Content for testing. This is a second sentence."

    # Mock generate_embedding to return a single embedding for all calls
    mock_gemini_service.generate_embedding.side_effect = [[0.1]*768, [0.2]*768] # Two chunks, two embeddings

    textbook_content = await ingestion_service.ingest_document(file_path, metadata)

    assert textbook_content is not None
    assert isinstance(textbook_content, TextbookContent)
    assert textbook_content.id == "12345678-1234-5678-1234-567812345678"
    assert textbook_content.source_path == file_path
    assert textbook_content.metadata == metadata
    assert isinstance(textbook_content.last_indexed_at, datetime)

    mock_document_parser.parse_document.assert_called_once_with(file_path)
    # Check that embedding was called for each chunk
    assert mock_gemini_service.generate_embedding.call_count == 2
    
    mock_qdrant_service.upsert_vectors.assert_called_once()
    upsert_args, upsert_kwargs = mock_qdrant_service.upsert_vectors.call_args
    assert upsert_kwargs["collection_name"] == "textbook_chunks"
    assert len(upsert_kwargs["ids"]) == 2
    assert len(upsert_kwargs["vectors"]) == 2
    assert len(upsert_kwargs["payloads"]) == 2
    assert upsert_kwargs["payloads"][0]["text_content"] == "Content for testing." # First chunk
    assert upsert_kwargs["payloads"][1]["text_content"] == "This is a second sentence." # Second chunk


async def test_ingest_document_parsing_fails(ingestion_service, mock_document_parser):
    file_path = "/path/to/bad_doc.pdf"
    metadata = {"chapter": "fail"}
    mock_document_parser.parse_document.return_value = None

    textbook_content = await ingestion_service.ingest_document(file_path, metadata)
    assert textbook_content is None
    mock_document_parser.parse_document.assert_called_once_with(file_path)
    mock_gemini_service.generate_embedding.assert_not_called()
    mock_qdrant_service.upsert_vectors.assert_not_called()

async def test_ingest_document_embedding_fails(ingestion_service, mock_document_parser, mock_gemini_service, mock_qdrant_service):
    file_path = "/path/to/doc.md"
    metadata = {"chapter": "test"}
    mock_document_parser.parse_document.return_value = "Content for testing."
    mock_gemini_service.generate_embedding.return_value = [] # Simulate embedding failure

    textbook_content = await ingestion_service.ingest_document(file_path, metadata)
    assert textbook_content is None
    mock_document_parser.parse_document.assert_called_once_with(file_path)
    mock_gemini_service.generate_embedding.assert_called_once()
    mock_qdrant_service.upsert_vectors.assert_not_called()

# --- Test _simple_chunking ---
def test_simple_chunking_exact_size(ingestion_service):
    text = "Word1 Word2 Word3 Word4 Word5. Word6 Word7 Word8 Word9 Word10."
    chunks = ingestion_service._simple_chunking(text, max_chunk_size=30, overlap=0)
    assert len(chunks) == 2
    assert chunks[0].strip() == "Word1 Word2 Word3 Word4 Word5."
    assert chunks[1].strip() == "Word6 Word7 Word8 Word9 Word10."

def test_simple_chunking_with_overlap(ingestion_service):
    text = "This is the first part of a very long sentence. This is the second part of the same sentence."
    # With max_chunk_size=30 and overlap=10
    chunks = ingestion_service._simple_chunking(text, max_chunk_size=30, overlap=10)
    assert len(chunks) == 2
    # Check that there's some overlap. The exact word count for overlap might vary slightly
    assert "sentence." in chunks[0] and "sentence." in chunks[1]
