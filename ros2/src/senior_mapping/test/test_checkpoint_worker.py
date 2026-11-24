"""Only checkpoint requests coalesce; observation ingestion remains ordered."""

import asyncio

from senior_mapping.checkpoint_worker import CheckpointWorker


async def test_busy_worker_has_one_capture_and_flushes_latest_accepted_cut():
    entered, release = asyncio.Event(), asyncio.Event()
    captures = []

    async def save(sequence):
        captures.append(sequence)
        entered.set()
        await release.wait()

    worker = CheckpointWorker(save, idle=lambda: False, max_delay_s=0.02)
    worker.changed(1)
    worker.changed(2)
    worker.changed(3)
    runner = asyncio.create_task(worker.run())
    try:
        # Even when never idle, the oldest dirty request gets a turn.
        await asyncio.wait_for(entered.wait(), 1)
        worker.changed(4)
        worker.changed(5)
        assert captures == [3] and worker.saved_sequence == 0
        release.set()
        await asyncio.wait_for(worker.flush(), 1)
        assert captures == [3, 5] and worker.saved_sequence == 5
        await asyncio.wait_for(worker.close(), 1)
        await asyncio.wait_for(runner, 1)
    finally:
        runner.cancel()
        await asyncio.gather(runner, return_exceptions=True)
