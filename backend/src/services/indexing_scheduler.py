import asyncio
from datetime import datetime, timedelta
from typing import List, Dict, Any

from backend.src.services.indexing_service import IndexingService

class IndexingScheduler:
    """
    Service for scheduling periodic re-indexing of documentation sources.
    """
    def __init__(self, indexing_service: IndexingService, reindex_interval_hours: int = 24):
        self.indexing_service = indexing_service
        self.reindex_interval = timedelta(hours=reindex_interval_hours)
        self._running = False
        self._task = None
        # Placeholder for source configuration - in a real app, this would come from a DB or config
        self.configured_sources: List[Dict[str, Any]] = [
            # Example source (assuming a file-based source for simplicity)
            # In a real app, these would be dynamic and potentially include remote URLs
            {"file_path": "/path/to/your/textbook/chapter1.md", "metadata": {"chapter": "1", "week": "week-01"}},
            {"file_path": "/path/to/your/textbook/chapter2.pdf", "metadata": {"chapter": "2", "week": "week-01"}},
        ]

    async def _run_scheduler(self):
        """
        Internal method to continuously run the re-indexing task.
        """
        while self._running:
            print(f"[{datetime.now()}] Running periodic re-indexing...")
            await self.indexing_service.reindex_all_sources(self.configured_sources)
            print(f"[{datetime.now()}] Periodic re-indexing complete. Next run in {self.reindex_interval}.")
            await asyncio.sleep(self.reindex_interval.total_seconds())

    async def start_scheduler(self):
        """
        Starts the periodic re-indexing scheduler.
        """
        if self._running:
            print("Indexing scheduler is already running.")
            return

        print("Starting indexing scheduler...")
        self._running = True
        self._task = asyncio.create_task(self._run_scheduler())

    async def stop_scheduler(self):
        """
        Stops the periodic re-indexing scheduler.
        """
        if not self._running:
            print("Indexing scheduler is not running.")
            return

        print("Stopping indexing scheduler...")
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        print("Indexing scheduler stopped.")

    async def update_configured_sources(self, new_sources: List[Dict[str, Any]]):
        """
        Updates the list of sources to be re-indexed.
        This would typically be called by an admin endpoint or configuration change.
        """
        self.configured_sources = new_sources
        print("Configured sources updated for scheduler.")
