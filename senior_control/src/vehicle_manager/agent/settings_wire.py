"""Settings wire adapter; the service remains the sole policy and persistence owner."""

import time
from typing import TYPE_CHECKING, Any

from senior_contracts.wire import SettingsApplyEnvelope, SettingsEnvelope, SettingsResultEnvelope

from vehicle_manager.agent.state_ports import StatePort as IpcState
from vehicle_manager.agent.state_ports import StateReading as RuntimeState
from vehicle_manager.settings.models import SettingsRequest
from vehicle_manager.settings.service import RuntimeCheck, SettingsService

if TYPE_CHECKING:
    from vehicle_manager.tasks.lifecycle_store import LifecycleStore


class SettingsRuntime:
    def __init__(self, state: IpcState, tasks: "LifecycleStore", *, error: type[Exception]) -> None:
        self._error = error
        self.state, self.tasks = state, tasks
        self.live: RuntimeState | None = None
        self.executing = True

    async def refresh(self) -> None:
        self.live, self.executing = None, True
        try:
            live = await self.state.read()
            executing = False
            for identity in await self.tasks.instances(active_only=True):
                task = await self.tasks.load(identity)
                if task.phase != "paused" or task.pending is not None:
                    executing = True
            self.executing, self.live = executing, live
        except self._error:
            pass

    def check(self) -> RuntimeCheck:
        live = self.live
        now = time.monotonic()
        runtime = live.motion_runtime if live else None
        if (
            runtime is None
            or runtime.evidence_stamp_steady is None
            or runtime.evidence_deadline_steady is None
            or not runtime.evidence_stamp_steady <= now < runtime.evidence_deadline_steady
        ):
            return RuntimeCheck()
        assert live is not None
        authority = live.manual_authority
        return RuntimeCheck(
            stationary=runtime.stationary,
            executing_motion_task=self.executing or runtime.mode == "task",
            manual_active=live.manual_driving
            or bool(authority and authority.get("state") in {"prepared", "active"}),
        )


async def settings_command(
    service: SettingsService, envelope: SettingsEnvelope
) -> dict[str, Any] | None:
    if isinstance(envelope, SettingsApplyEnvelope):
        result = await service.apply(SettingsRequest(**envelope.payload.model_dump()))
        return result.model_dump(mode="json")
    if isinstance(envelope, SettingsResultEnvelope):
        stored = await service.result(envelope.payload.request_id)
        return stored.model_dump(mode="json") if stored else None
    snapshot = await service.get()
    async with service.runtime.lock:
        await service.runtime.refresh()
        runtime = service.runtime.check()
        allowed = (
            runtime.stationary and not runtime.executing_motion_task and not runtime.manual_active
        )
    return {
        "snapshot": snapshot.model_dump(mode="json"),
        "capabilities": service.capabilities.model_dump(mode="json"),
        "can_apply_driving": allowed,
    }
