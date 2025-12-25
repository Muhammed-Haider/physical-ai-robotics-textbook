import pytest
import asyncio
from unittest.mock import AsyncMock, patch, mock_open
from pathlib import Path

from backend.src.services.document_parser import DocumentParser

@pytest.fixture
def document_parser():
    return DocumentParser()

@pytest.fixture
async def create_dummy_md_file(tmp_path):
    content = "# Test Markdown\n\nThis is a paragraph.\n\n- List item 1\n- List item 2"
    file_path = tmp_path / "test.md"
    file_path.write_text(content)
    return file_path, content

@pytest.fixture
async def create_dummy_pdf_file(tmp_path):
    file_path = tmp_path / "test.pdf"
    file_path.write_text("dummy pdf content") # Actual content doesn't matter for mock
    return file_path

# --- Test parse_document method ---
async def test_parse_document_markdown(document_parser, create_dummy_md_file):
    file_path, original_content = create_dummy_md_file
    parsed_text = await document_parser.parse_document(str(file_path))
    assert parsed_text is not None
    assert "Test Markdown" in parsed_text
    assert "This is a paragraph" in parsed_text
    assert "List item 1" in parsed_text # Markdown conversion should retain text

async def test_parse_document_pdf_mocked(document_parser, create_dummy_pdf_file):
    file_path = create_dummy_pdf_file
    with patch.object(document_parser, '_parse_pdf', new_callable=AsyncMock) as mock_parse_pdf:
        mock_parse_pdf.return_value = "Mocked PDF content"
        parsed_text = await document_parser.parse_document(str(file_path))
        assert parsed_text == "Mocked PDF content"
        mock_parse_pdf.assert_called_once_with(str(file_path))

async def test_parse_document_unsupported_type(document_parser, tmp_path, capsys):
    file_path = tmp_path / "test.txt"
    file_path.write_text("plain text")
    parsed_text = await document_parser.parse_document(str(file_path))
    assert parsed_text is None
    captured = capsys.readouterr()
    assert "Warning: Unsupported file type for parsing: .txt" in captured.out

# --- Test _parse_markdown method ---
async def test_parse_markdown_success(document_parser, create_dummy_md_file):
    file_path, original_content = create_dummy_md_file
    parsed_text = await document_parser._parse_markdown(str(file_path))
    assert parsed_text is not None
    assert "Test Markdown" in parsed_text # Basic check that content is there

async def test_parse_markdown_file_not_found(document_parser, capsys):
    parsed_text = await document_parser._parse_markdown("non_existent.md")
    assert parsed_text is None
    captured = capsys.readouterr()
    assert "Error: Markdown file not found at non_existent.md" in captured.out

# --- Test _parse_pdf method (mocked due to external dependency) ---
async def test_parse_pdf_placeholder(document_parser, capsys):
    # This tests the placeholder message, as PyPDF2 is not installed by default
    parsed_text = await document_parser._parse_pdf("dummy.pdf")
    assert parsed_text is None
    captured = capsys.readouterr()
    assert "PDF parsing not fully implemented. Please install PyPDF2 to enable. File: dummy.pdf" in captured.out

# --- Test _simple_chunking method ---
def test_simple_chunking_basic(document_parser):
    text = "This is a sentence. This is another sentence. And a third one."
    chunks = document_parser._simple_chunking(text, max_chunk_size=20, overlap=0)
    assert len(chunks) > 1
    assert all(len(chunk.split()) > 0 for chunk in chunks)

def test_simple_chunking_with_overlap(document_parser):
    text = "Word1 Word2 Word3 Word4 Word5 Word6 Word7 Word8 Word9 Word10"
    chunks = document_parser._simple_chunking(text, max_chunk_size=20, overlap=10)
    # Exact chunking depends on algorithm, but we expect more than one chunk and some overlap
    assert len(chunks) > 1
    assert "Word1 Word2 Word3" in chunks[0] # Example expected content
    assert "Word3 Word4 Word5" in chunks[1] # Example expected content showing overlap

def test_simple_chunking_empty_text(document_parser):
    text = ""
    chunks = document_parser._simple_chunking(text)
    assert chunks == []

def test_simple_chunking_single_long_word(document_parser):
    text = "averyveryveryveryveryverylongwordthatexceedschunksize"
    chunks = document_parser._simple_chunking(text, max_chunk_size=20, overlap=0)
    assert len(chunks) == 1
    assert chunks[0] == text
