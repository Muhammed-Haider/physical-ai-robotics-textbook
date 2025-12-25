import pytest
import os
from unittest.mock import AsyncMock, MagicMock, patch

# Need to mock environment variables before GeminiService import
@pytest.fixture(autouse=True)
def mock_env_vars():
    with patch.dict(os.environ, {"GEMINI_API_KEY": "mock-api-key"}):
        yield

# Mock the genai module before importing GeminiService
with patch("backend.src.services.gemini_service.genai", autospec=True) as mock_genai:
    mock_genai.get_model.return_value = MagicMock()
    mock_genai.GenerativeModel.return_value = MagicMock()
    from backend.src.services.gemini_service import GeminiService

@pytest.fixture
def gemini_service():
    # Ensure genai is configured before creating service instance
    with patch("backend.src.services.gemini_service.genai.configure") as mock_configure:
        service = GeminiService()
        mock_configure.assert_called_once_with(api_key="mock-api-key")
        return service

async def test_gemini_service_init_success(gemini_service):
    assert gemini_service.gemini_api_key == "mock-api-key"
    gemini_service.embedding_model.assert_called_once_with("models/embedding-001")
    gemini_service.generative_model.assert_called_once_with("gemini-pro")

async def test_gemini_service_init_missing_env_var():
    with patch.dict(os.environ, {}, clear=True): # Clear env vars
        with pytest.raises(ValueError, match="GEMINI_API_KEY environment variable must be set."):
            GeminiService()

async def test_generate_embedding(gemini_service):
    mock_text = "test text for embedding"
    mock_embedding_response = {"embedding": [0.1, 0.2, 0.3]}
    
    with patch("backend.src.services.gemini_service.genai.embed_content", return_value=mock_embedding_response) as mock_embed_content:
        embedding = await gemini_service.generate_embedding(mock_text)
        assert embedding == mock_embedding_response["embedding"]
        mock_embed_content.assert_called_once_with(
            model="models/embedding-001",
            content=mock_text,
            task_type="RETRIEVAL_DOCUMENT"
        )

async def test_generate_embedding_empty_text(gemini_service):
    embedding = await gemini_service.generate_embedding("")
    assert embedding == []

async def test_generate_text(gemini_service):
    mock_prompt = "What is the capital of France?"
    mock_response_text = "Paris"
    
    mock_generate_content_result = MagicMock()
    mock_generate_content_result.text = mock_response_text
    
    gemini_service.generative_model.generate_content.return_value = mock_generate_content_result

    response = await gemini_service.generate_text(mock_prompt)
    assert response == mock_response_text
    gemini_service.generative_model.generate_content.assert_called_once_with(f"Question: {mock_prompt}")

async def test_generate_text_with_context(gemini_service):
    mock_prompt = "What is its main river?"
    mock_context = ["Paris is the capital of France."]
    mock_response_text = "The Seine"

    mock_generate_content_result = MagicMock()
    mock_generate_content_result.text = mock_response_text
    
    gemini_service.generative_model.generate_content.return_value = mock_generate_content_result

    response = await gemini_service.generate_text(mock_prompt, context=mock_context)
    assert response == mock_response_text
    expected_full_prompt = "Context:\nParis is the capital of France.\n\nQuestion: What is its main river?"
    gemini_service.generative_model.generate_content.assert_called_once_with(expected_full_prompt)

async def test_get_client(gemini_service):
    client = await gemini_service.get_client()
    assert client is gemini_service.generative_model
