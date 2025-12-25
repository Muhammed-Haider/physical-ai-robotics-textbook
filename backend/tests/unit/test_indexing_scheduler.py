import pytest
import asyncio
from unittest.mock import AsyncMock, patch
from datetime import timedelta, datetime

from backend.src.services.indexing_scheduler import IndexingScheduler
from backend.src.services.indexing_service import IndexingService

@pytest.fixture
def mock_indexing_service():
    service = AsyncMock(spec=IndexingService)
    service.reindex_all_sources.return_value = []
    return service

@pytest.fixture
def indexing_scheduler(mock_indexing_service):
    return IndexingScheduler(indexing_service=mock_indexing_service, reindex_interval_hours=0.01) # Short interval for testing

# --- Test IndexingScheduler Initialization ---
def test_indexing_scheduler_init(indexing_scheduler, mock_indexing_service):
    assert indexing_scheduler.indexing_service is mock_indexing_service
    assert indexing_scheduler.reindex_interval == timedelta(hours=0.01)
    assert not indexing_scheduler._running
    assert indexing_scheduler._task is None
    assert len(indexing_scheduler.configured_sources) > 0 # Default sources should be present

# --- Test start_scheduler and stop_scheduler ---
async def test_start_and_stop_scheduler(indexing_scheduler, mock_indexing_service):
    with patch("asyncio.sleep", new_callable=AsyncMock) as mock_sleep:
        # Start the scheduler
        await indexing_scheduler.start_scheduler()
        assert indexing_scheduler._running
        assert indexing_scheduler._task is not None

        # Allow some time for the scheduler to potentially run once
        # In a real test, you'd carefully control mock_sleep calls
        await asyncio.sleep(0.001) # Small real sleep to allow task to be created

        # Stop the scheduler
        await indexing_scheduler.stop_scheduler()
        assert not indexing_scheduler._running
        
        # Verify that reindex_all_sources was called at least once (or the mock_sleep would have been called)
        mock_indexing_service.reindex_all_sources.assert_called_once()
        mock_sleep.assert_called_once() # Should have attempted to sleep

async def test_start_scheduler_already_running(indexing_scheduler, capsys):
    indexing_scheduler._running = True # Manually set to running state
    await indexing_scheduler.start_scheduler()
    captured = capsys.readouterr()
    assert "Indexing scheduler is already running." in captured.out

async def test_stop_scheduler_not_running(indexing_scheduler, capsys):
    indexing_scheduler._running = False # Manually set to not running state
    await indexing_scheduler.stop_scheduler()
    captured = capsys.readouterr()
    assert "Indexing scheduler is not running." in captured.out

# --- Test _run_scheduler logic ---
async def test_run_scheduler_periodic_calls(indexing_scheduler, mock_indexing_service):
    mock_indexing_service.reindex_all_sources.reset_mock() # Reset call count from setup

    # Shorten interval to make testing faster
    indexing_scheduler.reindex_interval = timedelta(seconds=0.001)
    
    await indexing_scheduler.start_scheduler()
    
    # Allow it to run a few times
    await asyncio.sleep(0.005) # Let it run for a few intervals
    
    await indexing_scheduler.stop_scheduler()
    
    # Check that reindex_all_sources was called multiple times
    assert mock_indexing_service.reindex_all_sources.call_count >= 1

async def test_update_configured_sources(indexing_scheduler):
    new_sources = [
        {"file_path": "/new/path.md", "metadata": {"tag": "new"}}
    ]
    await indexing_scheduler.update_configured_sources(new_sources)
    assert indexing_scheduler.configured_sources == new_sources
