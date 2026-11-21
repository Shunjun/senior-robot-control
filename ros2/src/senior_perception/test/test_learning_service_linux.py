"""Target-only integration: actual kernel capabilities, IPC thread and spawn I/O."""

import asyncio
import sys
import time
from uuid import uuid4

import pytest
from senior_perception.learning_client import LearningClient, request_once
from senior_perception.learning_service import LearningService
from senior_perception.learning_wire import FrameBatch, MotionObservedThrough
from test_learning_compute import recorded
from test_startup_learning import side_observation

pytestmark = pytest.mark.skipif(
    sys.platform != "linux", reason="requires Linux abstract SEQPACKET and SO_PEERCRED"
)


async def until(predicate, client):
    deadline = asyncio.get_running_loop().time() + 8
    while asyncio.get_running_loop().time() < deadline:
        client.poll()
        if predicate():
            return
        await asyncio.sleep(0.01)
    raise AssertionError(f"learning IPC condition timed out: {client.error}")


async def test_native_service_restores_frozen_model_without_replaying_action(tmp_path):
    path = tmp_path / (".startup-learning-" + uuid4().hex)
    service = LearningService(path)
    await service.start()
    clients = []
    try:
        assert service.owner is not None
        first = LearningClient(path)
        clients.append(first)
        await until(lambda: first.ready, first)
        await until(first.reserve_action, first)
        start = time.monotonic() + 1
        start_ns = round(start * 1e9)
        first.phase("moving", "learning_forward_window", "c" * 32)
        first.action(start_ns, 3_000_000_000, finished=True)
        await until(lambda: first.record.action.first_submitted_steady_ns == start_ns, first)
        frames = [recorded(i, start + i / 10) for i in range(1, 25)]
        frames += [recorded(25 + i, start + 3.1 + i / 10) for i in range(5)]
        for offset in range(0, len(frames), 4):
            batch = FrameBatch(
                attempt_id=first.record.attempt_id,
                frames=tuple(frames[offset : offset + 4]),
                through_steady_ns=round((start + 3.6) * 1e9),
            )
            reply = await asyncio.to_thread(request_once, first.run_id, batch)
            assert reply.ok
        await asyncio.to_thread(
            request_once,
            first.run_id,
            MotionObservedThrough(
                stationary=True,
                session=first._session,
                attempt_id=first.record.attempt_id,
                segment_id="c" * 32,
                through_steady_ns=round((start + 3.7) * 1e9),
                validation_after_steady_ns=start_ns + 3_000_000_000,
            ),
        )
        await until(lambda: first.view is not None, first)
        assert first.view.filter(side_observation(0.15)).rays[180].kind == "self"
        # A mismatch never modifies the frozen model.
        assert first.view.filter(side_observation(0.10)).rays[180].kind == "hit"
        first.close()
        restored = LearningClient(path)
        clients.append(restored)
        await until(lambda: restored.view is not None, restored)
        assert restored.record.model == first.record.model
        assert restored._session is None
        assert restored.record.action.consumed
        assert restored.view.filter(side_observation(0.15)).rays[180].kind == "self"
        assert not restored.reserve_action()
    finally:
        for client in clients:
            client.close()
        await service.close()
