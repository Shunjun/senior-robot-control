"""Production capability ports. Missing integrations cannot grant motion."""

from dataclasses import dataclass
from typing import Generic, TypeVar

T = TypeVar("T")


@dataclass(frozen=True)
class CapabilityResult(Generic[T]):
    module: str
    status: str
    reason: str
    value: T | None = None


class CapabilityPort:
    """An explicit absent adapter, never a synthetic production implementation."""

    def __init__(self, module):
        self.module = module

    def read(self):
        return CapabilityResult(self.module, "unavailable", "adapter_not_connected")


class VehicleCapabilities:
    def __init__(self, *, map_operations=None, relocalizer=None, route=None):
        self.mapping = (
            CapabilityPort("mapping")
            if map_operations is None
            else MapManagementAdapter(map_operations)
        )
        self.localization = (
            CapabilityPort("localization")
            if relocalizer is None
            else LocalizationAdapter(relocalizer)
        )
        self.navigation = (
            CapabilityPort("navigation")
            if route is None
            else NavigationAdapter(route, self.localization)
        )
        self.gps = CapabilityPort("gps")

    def navigate(self, goal):
        # No command sink is held here: unavailable localization cannot issue motion.
        return CapabilityResult("navigation", "unavailable", "map_pose_not_confirmed")


class MapManagementAdapter:
    """Expose existing persisted map operations without a second state store."""

    def __init__(self, operations):
        self.operations = operations

    async def query(self, operation_id):
        return await self.operations.query(operation_id)

    async def export_bundle(self, *args, **kwargs):
        return await self.operations.export_bundle(*args, **kwargs)

    async def import_bundle(self, *args, **kwargs):
        return await self.operations.import_bundle(*args, **kwargs)


class LocalizationAdapter:
    """A Home/initial hint never substitutes for the existing scan match."""

    def __init__(self, relocalizer):
        self.relocalizer = relocalizer

    async def begin(self, map_ref, hint=None):
        return await self.relocalizer.begin(map_ref, hint)

    def read(self):
        if not self.relocalizer.ready:
            return CapabilityResult("localization", "unavailable", self.relocalizer.reason)
        return CapabilityResult("localization", "available", "scan_match", self.relocalizer.pose)


class NavigationAdapter:
    """Reuse QualifiedRoute admission; never derive a route from an unconfirmed pose."""

    def __init__(self, route, localization):
        self.route = route
        self.localization = localization

    async def run(self, map_ref, generation, poses):
        if self.localization.read().status != "available":
            return CapabilityResult("navigation", "unavailable", "map_pose_not_confirmed")
        accepted = await self.route.run(map_ref, generation, poses)
        return CapabilityResult("navigation", "available", "route_completed", accepted)
