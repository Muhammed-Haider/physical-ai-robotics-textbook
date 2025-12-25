import pytest
from datetime import datetime
from pydantic import ValidationError

from backend.src.models.textbook_content import TextbookContent
from backend.src.models.text_chunk import TextChunk
from backend.src.models.user import User, UserRole
from backend.src.models.user_query import UserQuery
from backend.src.models.chatbot_response import ChatbotResponse
from backend.src.models.chat_session import ChatSession, ChatSessionState

# Fixtures for common data
@pytest.fixture
def sample_datetime():
    return datetime.now()

@pytest.fixture
def sample_embedding():
    return [0.1] * 768  # Mock embedding vector

# --- Test TextbookContent Model ---
def test_textbook_content_valid_data(sample_datetime):
    content = TextbookContent(
        id="test-book-1",
        text_content="This is some sample text.",
        source_path="/path/to/source.md",
        metadata={"chapter": "Intro"},
        last_indexed_at=sample_datetime
    )
    assert content.id == "test-book-1"
    assert content.text_content == "This is some sample text."
    assert content.source_path == "/path/to/source.md"
    assert content.metadata == {"chapter": "Intro"}
    assert content.last_indexed_at == sample_datetime

def test_textbook_content_missing_required_fields():
    with pytest.raises(ValidationError):
        TextbookContent(id="test-book-2")
    with pytest.raises(ValidationError):
        TextbookContent(text_content="text", source_path="path", last_indexed_at=datetime.now())

def test_textbook_content_empty_text_content():
    with pytest.raises(ValidationError):
        TextbookContent(
            id="test-book-3",
            text_content="",
            source_path="/path/to/source.md",
            last_indexed_at=datetime.now()
        )

# --- Test TextChunk Model ---
def test_text_chunk_valid_data(sample_embedding):
    chunk = TextChunk(
        id="chunk-1",
        text_content="Part of a document.",
        embedding_vector=sample_embedding,
        original_source_ref="test-book-1",
        metadata={"page": 1}
    )
    assert chunk.id == "chunk-1"
    assert chunk.text_content == "Part of a document."
    assert chunk.embedding_vector == sample_embedding
    assert chunk.original_source_ref == "test-book-1"
    assert chunk.metadata == {"page": 1}

def test_text_chunk_missing_required_fields():
    with pytest.raises(ValidationError):
        TextChunk(id="chunk-2")

def test_text_chunk_empty_text_content():
    with pytest.raises(ValidationError):
        TextChunk(
            id="chunk-3",
            text_content="",
            embedding_vector=[0.0],
            original_source_ref="test-book-1"
        )

# --- Test User Model ---
def test_user_valid_data():
    user = User(id="user-1", role=UserRole.ADMIN, authentication_details="auth-det-1")
    assert user.id == "user-1"
    assert user.role == UserRole.ADMIN
    assert user.authentication_details == "auth-det-1"

def test_user_valid_end_user():
    user = User(id="user-2", role=UserRole.END_USER)
    assert user.id == "user-2"
    assert user.role == UserRole.END_USER
    assert user.authentication_details is None

def test_user_invalid_role():
    with pytest.raises(ValidationError):
        User(id="user-3", role="invalid-role")

def test_user_missing_required_fields():
    with pytest.raises(ValidationError):
        User(role=UserRole.ADMIN)

# --- Test UserQuery Model ---
def test_user_query_valid_data(sample_datetime, sample_embedding):
    query = UserQuery(
        id="query-1",
        user_id="user-1",
        text_content="What is Python?",
        timestamp=sample_datetime,
        embedding_vector=sample_embedding,
        session_id="session-1"
    )
    assert query.id == "query-1"
    assert query.user_id == "user-1"
    assert query.text_content == "What is Python?"
    assert query.timestamp == sample_datetime
    assert query.embedding_vector == sample_embedding
    assert query.session_id == "session-1"

def test_user_query_empty_text_content():
    with pytest.raises(ValidationError):
        UserQuery(
            id="query-2",
            user_id="user-1",
            text_content="",
            timestamp=datetime.now(),
            embedding_vector=[0.0],
            session_id="session-1"
        )

def test_user_query_long_text_content():
    with pytest.raises(ValidationError):
        UserQuery(
            id="query-3",
            user_id="user-1",
            text_content="a" * 501, # Exceeds max_length=500
            timestamp=datetime.now(),
            embedding_vector=[0.0],
            session_id="session-1"
        )

# --- Test ChatbotResponse Model ---
def test_chatbot_response_valid_data(sample_datetime):
    response = ChatbotResponse(
        id="resp-1",
        query_id="query-1",
        text_content="Python is a programming language.",
        timestamp=sample_datetime,
        retrieved_chunks_refs=["chunk-1"],
        confidence_score=0.9
    )
    assert response.id == "resp-1"
    assert response.query_id == "query-1"
    assert response.text_content == "Python is a programming language."
    assert response.timestamp == sample_datetime
    assert response.retrieved_chunks_refs == ["chunk-1"]
    assert response.confidence_score == 0.9

def test_chatbot_response_missing_required_fields():
    with pytest.raises(ValidationError):
        ChatbotResponse(id="resp-2")

def test_chatbot_response_invalid_confidence_score():
    with pytest.raises(ValidationError):
        ChatbotResponse(
            id="resp-3",
            query_id="query-1",
            text_content="text",
            timestamp=datetime.now(),
            confidence_score=1.1
        )
    with pytest.raises(ValidationError):
        ChatbotResponse(
            id="resp-4",
            query_id="query-1",
            text_content="text",
            timestamp=datetime.now(),
            confidence_score=-0.1
        )

# --- Test ChatSession Model ---
def test_chat_session_valid_data(sample_datetime):
    session = ChatSession(
        id="sess-1",
        user_id="user-1",
        start_time=sample_datetime,
        last_active_time=sample_datetime,
        status=ChatSessionState.ACTIVE
    )
    assert session.id == "sess-1"
    assert session.user_id == "user-1"
    assert session.start_time == sample_datetime
    assert session.last_active_time == sample_datetime
    assert session.status == ChatSessionState.ACTIVE

def test_chat_session_invalid_status():
    with pytest.raises(ValidationError):
        ChatSession(
            id="sess-2",
            user_id="user-1",
            start_time=datetime.now(),
            last_active_time=datetime.now(),
            status="invalid-status"
        )

def test_chat_session_state_enum():
    assert ChatSessionState.ACTIVE == "active"
    assert ChatSessionState.IDLE == "idle"
    assert ChatSessionState.CLOSED == "closed"