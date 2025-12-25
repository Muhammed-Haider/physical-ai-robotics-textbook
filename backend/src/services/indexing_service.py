from typing import List, Dict, Any, Optional
from backend.src.services.ingestion_service import IngestionService
from backend.src.models.textbook_content import TextbookContent
from backend.src.utils.logger import logger

class IndexingService:
    """
    Service for managing the re-indexing of documentation.
    """
    def __init__(self, ingestion_service: IngestionService):
        self.ingestion_service = ingestion_service
        logger.info("IndexingService initialized.")

    async def reindex_document(self, file_path: str, metadata: Dict[str, Any]) -> Optional[TextbookContent]:
        """
        Manually triggers re-ingestion of a single document.
        """
        logger.info(f"Re-indexing document: {file_path}")
        # The ingestion service handles the full parse, embed, and store logic.
        return await self.ingestion_service.ingest_document(file_path, metadata)

    async def reindex_all_sources(self, source_list: List[Dict[str, Any]]) -> List[Optional[TextbookContent]]:
        """
        Triggers a re-indexing process for all specified sources.
        `source_list` would typically contain dictionaries with 'file_path' and 'metadata'.
        """
        logger.info(f"Initiating re-indexing for {len(source_list)} sources.")
        indexed_documents: List[Optional[TextbookContent]] = []
        for source in source_list:
            file_path = source.get("file_path")
            metadata = source.get("metadata", {})
            if file_path:
                indexed_documents.append(await self.reindex_document(file_path, metadata))
            else:
                logger.warning(f"Skipping source due to missing 'file_path': {source}")
        logger.info("Re-indexing complete.")
        return indexed_documents
