import os
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
from typing import List, Dict, Any, Optional
from src.utils.logger import logger

class QdrantService:
    """
    Service for interacting with Qdrant Cloud for vector storage and retrieval.
    """
    def __init__(self):
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not self.qdrant_url or not self.qdrant_api_key:
            logger.error("QDRANT_URL and QDRANT_API_KEY environment variables must be set.")
            raise ValueError(
                "QDRANT_URL and QDRANT_API_KEY environment variables must be set."
            )
        try:
            self.client = QdrantClient(
                url=self.qdrant_url,
                api_key=self.qdrant_api_key,
            )
            logger.info("QdrantService initialized successfully.")
        except Exception as e:
            logger.critical(f"Failed to initialize Qdrant client: {e}")
            raise

    async def create_collection(self, collection_name: str, vector_size: int, distance: Distance = Distance.COSINE):
        """
        Creates a new collection in Qdrant.
        """
        logger.info(f"Attempting to create/recreate Qdrant collection: {collection_name}")
        try:
            self.client.recreate_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(size=vector_size, distance=distance),
            )
            logger.info(f"Qdrant collection '{collection_name}' created/recreated successfully.")
            return True
        except Exception as e:
            logger.error(f"Failed to create Qdrant collection '{collection_name}': {e}")
            return False

    async def upsert_vectors(self, collection_name: str, ids: List[str], vectors: List[List[float]], payloads: Optional[List[Dict[str, Any]]] = None):
        """
        Upserts (inserts or updates) vectors and their payloads into a collection.
        """
        logger.info(f"Attempting to upsert {len(ids)} vectors to collection '{collection_name}'.")
        try:
            points = []
            for i, vector in enumerate(vectors):
                point = {"id": ids[i], "vector": vector}
                if payloads:
                    point["payload"] = payloads[i]
                points.append(point)

            self.client.upsert(
                collection_name=collection_name,
                points=points,
                wait=True,
            )
            logger.info(f"Successfully upserted {len(ids)} vectors to collection '{collection_name}'.")
            return True
        except Exception as e:
            logger.error(f"Failed to upsert vectors to collection '{collection_name}': {e}")
            return False

    async def search_vectors(self, collection_name: str, query_vector: List[float], limit: int = 5, query_filter: Optional[Dict[str, Any]] = None):
        """
        Searches for similar vectors in a collection.
        """
        logger.debug(f"Searching vectors in collection '{collection_name}' with limit {limit}.")
        try:
            search_result = self.client.search(
                collection_name=collection_name,
                query_vector=query_vector,
                query_filter=query_filter,
                limit=limit,
            )
            logger.info(f"Found {len(search_result)} results in collection '{collection_name}'.")
            return search_result
        except Exception as e:
            logger.error(f"Failed to search vectors in collection '{collection_name}': {e}")
            return []

    async def get_client(self) -> QdrantClient:
        """
        Returns the initialized Qdrant client.
        """
        return self.client
