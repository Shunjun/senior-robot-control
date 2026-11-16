"""Target-only integration: actual kernel capabilities, IPC thread and spawn I/O."""

import asyncio
import sys
from uuid import uuid4

import pytest
from learning_test_support import geometry
from test_startup_learning import side_observation
from test_startup_learning_feasibility import BOUNDARY

from senior_control.runtime.learning_client import LearningClient
from senior_control.runtime.learning_service import LearningService

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
        manifest = geometry(BOUNDARY)
        first.geometry(manifest)
        await until(lambda: first.record.geometry == manifest, first)
        await until(first.reserve_action, first)
        first.complete(((0.0, 0.15),))
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
