import os

# from PyPDF2 import PdfReader # Uncomment and install PyPDF2 for PDF parsing
from typing import Any, Dict, List, Optional

import markdown

from src.utils.logger import logger


class DocumentParser:
    """
    Service for parsing different document formats (Markdown, PDF) into extractable text.
    """

    async def parse_document(self, file_path: str) -> Optional[str]:
        """
        Parses a document based on its file extension and returns its text content.
        """
        file_extension = os.path.splitext(file_path)[1].lower()
        logger.info(
            f"Attempting to parse document: {file_path} with extension {file_extension}"
        )

        if file_extension == ".md":
            return await self._parse_markdown(file_path)
        elif file_extension == ".pdf":
            return await self._parse_pdf(file_path)
        else:
            logger.warning(
                f"Unsupported file type for parsing: {file_extension}. File: {file_path}"
            )
            return None

    async def _parse_markdown(self, file_path: str) -> Optional[str]:
        """
        Parses a Markdown file and extracts its text content.
        """
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                md_content = f.read()
                # Convert markdown to plain text, stripping HTML tags that markdown might produce
                # A more robust solution might use a dedicated markdown-to-text converter
                plain_text = "".join(
                    markdown.markdown(md_content, output_format="html").splitlines()
                )
                logger.info(f"Successfully parsed Markdown file: {file_path}")
                return plain_text
        except FileNotFoundError:
            logger.error(f"Markdown file not found at {file_path}")
            return None
        except Exception as e:
            logger.error(f"Error parsing Markdown file {file_path}: {e}")
            return None

    async def _parse_pdf(self, file_path: str) -> Optional[str]:
        """
        Parses a PDF file and extracts its text content.
        NOTE: Requires PyPDF2 to be installed (`pip install PyPDF2`).
        """
        # try:
        #     reader = PdfReader(file_path)
        #     text = ""
        #     for page in reader.pages:
        #         text += page.extract_text() or ""
        #     logger.info(f"Successfully parsed PDF file: {file_path}")
        #     return text
        # except FileNotFoundError:
        #     logger.error(f"PDF file not found at {file_path}")
        #     return None
        # except Exception as e:
        #     logger.error(f"Error parsing PDF file {file_path}: {e}")
        #     return None
        logger.warning(
            f"PDF parsing not fully implemented. Please install PyPDF2 to enable. File: {file_path}"
        )
        return None

    async def _chunk_text(self, text: str) -> List[str]:
        """
        A placeholder for text chunking logic.
        This will be further refined in the ingestion service.
        """
        # Simple chunking by paragraph for demonstration
        return [chunk.strip() for chunk in text.split("\n\n") if chunk.strip()]
