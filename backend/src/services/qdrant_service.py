import os
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
from typing import List, Dict, Any, Optional

class QdrantService:
    """
    Service for interacting with Qdrant Cloud for vector storage and retrieval.
    """
    def __init__(self):
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not self.qdrant_url or not self.qdrant_api_key:
            raise ValueError(
                "QDRANT_URL and QDRANT_API_KEY environment variables must be set."
            )

        self.client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
        )

    async def create_collection(self, collection_name: str, vector_size: int, distance: Distance = Distance.COSINE):
        """
        Creates a new collection in Qdrant.
        """
        self.client.recreate_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=vector_size, distance=distance),
        )
        return True

    async def upsert_vectors(self, collection_name: str, ids: List[str], vectors: List[List[float]], payloads: Optional[List[Dict[str, Any]]] = None):
        """
        Upserts (inserts or updates) vectors and their payloads into a collection.
        """
        # This is a simplified example. In a real scenario, you'd handle PointsList or Batch
        # depending on the scale and client version.
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
        return True

    async def search_vectors(self, collection_name: str, query_vector: List[float], limit: int = 5, query_filter: Optional[Dict[str, Any]] = None):
        """
        Searches for similar vectors in a collection.
        """
        search_result = self.client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            query_filter=query_filter,
            limit=limit,
        )
        return search_result

    async def get_client(self) -> QdrantClient:
        """
        Returns the initialized Qdrant client.
        """
        return self.client
