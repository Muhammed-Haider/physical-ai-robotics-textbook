from datetime import datetime
from typing import List, Dict, Any, Optional
import uuid

from backend.src.models.textbook_content import TextbookContent
from backend.src.models.text_chunk import TextChunk
from backend.src.services.document_parser import DocumentParser
from backend.src.services.gemini_service import GeminiService
from backend.src.services.qdrant_service import QdrantService
from qdrant_client.models import Distance # Needed for Qdrant collection creation

class IngestionService:
    """
    Service for ingesting documents, parsing them, creating embeddings,
    and storing them in Qdrant.
    """
    def __init__(
        self,
        document_parser: DocumentParser,
        gemini_service: GeminiService,
        qdrant_service: QdrantService
    ):
        self.document_parser = document_parser
        self.gemini_service = gemini_service
        self.qdrant_service = qdrant_service
        self.qdrant_collection_name = "textbook_chunks"
        self.vector_size = 768 # Assuming Gemini embedding model output size

    async def initialize_qdrant_collection(self):
        """
        Ensures the Qdrant collection exists or creates it.
        """
        # In a real app, you'd check if collection exists first
        # For simplicity, we'll recreate if it doesn't exist or if config changes
        await self.qdrant_service.create_collection(
            collection_name=self.qdrant_collection_name,
            vector_size=self.vector_size,
            distance=Distance.COSINE
        )

    async def ingest_document(self, file_path: str, metadata: Dict[str, Any]) -> Optional[TextbookContent]:
        """
        Ingests a document, parses it, chunks it, generates embeddings,
        and stores them in Qdrant.
        """
        print(f"Starting ingestion for {file_path}")

        # 1. Parse the document
        text = await self.document_parser.parse_document(file_path)
        if not text:
            print(f"Failed to parse document: {file_path}")
            return None

        # 2. Chunk the parsed text (simple chunking for now)
        chunks = self._simple_chunking(text)
        if not chunks:
            print(f"No chunks generated for document: {file_path}")
            return None

        # Create TextbookContent entry (conceptual, actual storage handled elsewhere)
        doc_id = str(uuid.uuid4())
        textbook_content = TextbookContent(
            id=doc_id,
            text_content=text,
            source_path=file_path,
            metadata=metadata,
            last_indexed_at=datetime.utcnow()
        )

        print(f"Generated {len(chunks)} chunks.")

        # 3. Generate embeddings and prepare for Qdrant
        chunk_ids: List[str] = []
        chunk_vectors: List[List[float]] = []
        chunk_payloads: List[Dict[str, Any]] = []

        for i, chunk_text in enumerate(chunks):
            embedding = await self.gemini_service.generate_embedding(chunk_text)
            if not embedding:
                print(f"Failed to generate embedding for chunk {i} of {file_path}. Skipping.")
                continue

            chunk_id = f"{doc_id}-chunk-{i}"
            chunk_ids.append(chunk_id)
            chunk_vectors.append(embedding)
            chunk_payloads.append({
                "original_source_ref": doc_id,
                "chunk_index": i,
                "text_content": chunk_text,
                **metadata # Include document metadata in chunk payload
            })

        if not chunk_ids:
            print(f"No embeddings generated for document: {file_path}")
            return None

        # 4. Store in Qdrant
        print(f"Upserting {len(chunk_ids)} embeddings to Qdrant collection '{self.qdrant_collection_name}'.")
        await self.qdrant_service.upsert_vectors(
            collection_name=self.qdrant_collection_name,
            ids=chunk_ids,
            vectors=chunk_vectors,
            payloads=chunk_payloads
        )

        print(f"Successfully ingested {file_path}. Document ID: {doc_id}")
        return textbook_content

    def _simple_chunking(self, text: str, max_chunk_size: int = 500, overlap: int = 50) -> List[str]:
        """
        Simple text chunking strategy.
        Splits text into chunks of `max_chunk_size` characters with some `overlap`.
        """
        chunks = []
        words = text.split()
        current_chunk_words = []
        current_length = 0

        for word in words:
            if current_length + len(word) + 1 <= max_chunk_size:
                current_chunk_words.append(word)
                current_length += len(word) + 1
            else:
                chunks.append(" ".join(current_chunk_words))
                # Start new chunk with some overlap
                overlap_words = current_chunk_words[max(0, len(current_chunk_words) - (overlap // 5)):] # Approximate
                current_chunk_words = overlap_words + [word]
                current_length = len(" ".join(current_chunk_words)) + 1
        
        if current_chunk_words:
            chunks.append(" ".join(current_chunk_words))

        return chunks
