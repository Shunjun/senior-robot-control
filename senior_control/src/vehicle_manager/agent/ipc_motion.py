"""Management motion requests; no codec, ROS executor or actuator ownership."""

import asyncio
import time
from collections.abc import Callable
from typing import Any

from senior_contracts.errors import ErrorCode
from senior_contracts.wire import Envelope, ManualIntentEnvelope, SimpleEnvelope, serialize_envelope

from vehicle_manager.agent.local_ports import MotionClient, MotionErrors
from vehicle_manager.settings.service import SettingsService
from vehicle_manager.tasks.executor import TaskExecutor


class IpcMotion:
    """``MotionPort`` over the local socket. It never touches rclpy or ``/cmd_vel``."""

    def __init__(
        self,
        client: MotionClient,
        *,
        errors: MotionErrors,
        intent_client: MotionClient,
        disable_client: MotionClient | None = None,
        safety_client: MotionClient | None = None,
    ) -> None:
        self._errors = errors
        self._client = client
        self._learning_available: bool | None = None
        self._intent_client = intent_client
        # //Compatible: isolated callers can still supply one serialized IPC client.
        self._disable_client = disable_client if disable_client is not None else client
        self._safety_client = safety_client if safety_client is not None else client

    async def _ok(
        self, kind: str, *, client: MotionClient | None = None, **fields: Any
    ) -> dict[str, Any]:
        reply = await (client if client is not None else self._client).request(kind, **fields)
        if reply.get("ok") is not True and reply.get("error") in {"unsupported", "unavailable"}:
            raise self._errors.domain(ErrorCode.UNAVAILABLE, f"ROS {kind} is unavailable")
        if reply.get("ok") is not True:
            if reply.get("accepted") is False:
                try:
                    code = ErrorCode(reply.get("error"))
                except (TypeError, ValueError):
                    code = None
                raise self._errors.rejected(kind, code)
            raise self._errors.ipc(f"ros side refused {kind}")
        return reply

    async def verify_robot(self, robot_id: str) -> None:
        await self._ok("identity", robot_id=robot_id)

    async def stop(self, envelope: SimpleEnvelope) -> None:
        await self._ok(
            "stop", client=self._safety_client, envelope=serialize_envelope(envelope).decode()
        )

    async def estop(self, envelope: SimpleEnvelope) -> None:
        await self._ok(
            "estop", client=self._safety_client, envelope=serialize_envelope(envelope).decode()
        )

    async def estop_received(self, envelope: SimpleEnvelope, *, received_steady: float) -> None:
        await self._ok(
            "estop",
            client=self._safety_client,
            envelope=serialize_envelope(envelope).decode(),
            received_at=received_steady,
        )

    async def learning_available(self) -> bool:
        if self._learning_available is None:
            try:
                response = await self._client.request("learning_capability")
                self._learning_available = response.get("learning_protocol_version") == 1
            except (self._errors.ipc, OSError):
                return False
        return self._learning_available

    async def learning(
        self, envelope: Envelope, *, received_steady: float, deadline_steady: float
    ) -> dict[str, Any]:
        try:
            result = await self._ok(
                "learning",
                envelope=serialize_envelope(envelope).decode(),
                received_steady=received_steady,
                deadline_steady=deadline_steady,
            )
        except self._errors.rejected as exc:
            raise self._errors.domain(
                exc.code or ErrorCode.CONFLICT, "learning request was refused"
            ) from exc
        data = result.get("data")
        if not isinstance(data, dict):
            raise self._errors.domain(ErrorCode.UNAVAILABLE, "learning reply unavailable")
        return data

    async def send_intent_frame(
        self,
        envelope: ManualIntentEnvelope,
        *,
        received_steady: float,
        deadline_steady: float,
        driving: dict[str, Any],
        driving_version: int,
    ) -> None:
        await self._intent_client.send(
            "manual_intent",
            envelope=serialize_envelope(envelope).decode(),
            received_steady=received_steady,
            deadline_steady=deadline_steady,
            driving=driving,
            driving_version=driving_version,
        )

    async def manual(
        self,
        envelope: Envelope,
        *,
        received_steady: float | None = None,
        deadline_steady: float | None = None,
        **fields: Any,
    ) -> dict[str, Any]:
        try:
            return await self._ok(
                "manual",
                client=self._safety_client if envelope.type == "control.release" else None,
                envelope=serialize_envelope(envelope).decode(),
                received_steady=received_steady,
                deadline_steady=deadline_steady,
                **fields,
            )
        except self._errors.rejected as exc:
            raise self._errors.domain(
                exc.code or ErrorCode.SAFETY_REJECTED, "manual request was refused"
            ) from exc
        except (self._errors.ipc, OSError) as exc:
            raise self._errors.domain(
                ErrorCode.OPERATION_UNKNOWN, "manual IPC outcome is unknown"
            ) from exc

    async def disable(self, connection_epoch: int) -> None:
        await self._ok(
            "disable_motion", client=self._disable_client, connection_epoch=connection_epoch
        )

    async def exit(self) -> None:
        await self._ok("exit", client=self._disable_client)

    async def call(self, name: str) -> Any:
        return (await self._ok("ros_call", name=name)).get("result")

    async def close(self) -> None:
        for client in {
            self._client,
            self._intent_client,
            self._disable_client,
            self._safety_client,
        }:
            if client is not None:
                await client.close()


class TaskAwareMotion(IpcMotion):
    """Route manual acquisition and explicit resume through the task transaction."""

    def bind_tasks(self, tasks: TaskExecutor) -> None:
        self.tasks = tasks

    settings: SettingsService | None = None
    _manual_clock: Callable[[], float] = time.monotonic

    def bind_clock(self, clock: Callable[[], float]) -> None:
        """Use the same steady domain as Agent ingress/deadline construction."""
        self._manual_clock = clock

    async def send_intent(
        self, envelope: ManualIntentEnvelope, *, received_steady: float, deadline_steady: float
    ) -> None:
        """Select vehicle-owned settings, then hand a complete frame to the transport."""
        if self.settings is None:
            raise self._errors.domain(
                ErrorCode.UNAVAILABLE, "vehicle driving profile is not selected"
            )
        if self._manual_clock() >= deadline_steady:
            return
        snapshot = self.settings.published_snapshot
        await super().send_intent_frame(
            envelope,
            received_steady=received_steady,
            deadline_steady=deadline_steady,
            driving=snapshot.driving.model_dump(),
            driving_version=snapshot.driving_version,
        )

    async def manual(
        self,
        envelope: Envelope,
        *,
        received_steady: float | None = None,
        deadline_steady: float | None = None,
        **fields: Any,
    ) -> dict[str, Any]:
        if envelope.type in {"control.release", "safety.estop.reset", "safety.fault.recover"}:
            return await super().manual(
                envelope,
                received_steady=received_steady,
                deadline_steady=deadline_steady,
            )
        if self.settings is None:
            raise self._errors.domain(
                ErrorCode.UNAVAILABLE, "vehicle driving profile is not selected"
            )
        for attempt in range(5):
            if envelope.type == "control.acquire":
                try:
                    retired = await self.tasks.control_current("takeover", envelope.request_id)
                except self._errors.domain as exc:
                    if exc.code is not ErrorCode.UNAVAILABLE or attempt == 4:
                        raise
                    await asyncio.sleep(0.05)
                    continue
            async with self.settings.runtime.admission():
                if envelope.type == "control.acquire":
                    if not await self.tasks.manual_handoff_matches(retired):
                        continue  # Release the gate before retiring the changed owner.
                snapshot = self.settings.published_snapshot
                return await super().manual(
                    envelope,
                    received_steady=received_steady,
                    deadline_steady=deadline_steady,
                    driving=snapshot.driving.model_dump(),
                    driving_version=snapshot.driving_version,
                )
        raise self._errors.domain(
            ErrorCode.UNAVAILABLE, "task ownership changed before manual handoff"
        )
