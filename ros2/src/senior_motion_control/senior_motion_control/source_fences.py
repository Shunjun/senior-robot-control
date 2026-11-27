"""Producer incarnation fences; retired wall-clock deferred queues are removed."""


class RuntimeEpochFence:
    """Existing command-runtime session fence; not a raw sensor admission rule."""

    def __init__(self, *, max_retired: int) -> None:
        if max_retired < 1:
            raise ValueError("epoch capacity must be positive")
        self.max_retired = max_retired
        self.active: str | None = None
        self._retired: set[str] = set()
        self._exhausted = False

    def retire_active(self) -> None:
        if self.active is None:
            return
        if len(self._retired) >= self.max_retired:
            self._exhausted = True
        else:
            self._retired.add(self.active)
        self.active = None

    def is_retired(self, epoch: str) -> bool:
        return epoch in self._retired

    def retire_unadmitted(self, epoch: str) -> None:
        """Fence a replaced deferred source without making it the active source."""
        if not epoch or epoch in self._retired:
            return
        if self.active == epoch:
            self.retire_active()
        elif len(self._retired) >= self.max_retired:
            self._exhausted = True
        else:
            self._retired.add(epoch)

    def accept(self, epoch: str) -> bool:
        if self._exhausted or not epoch or epoch in self._retired:
            return False
        if self.active is not None and self.active != epoch:
            self.retire_active()
            if self._exhausted:
                return False
        self.active = epoch
        return True


class SourceEpochFence:
    """Diagnostic producer label. RawCaptureAdmission owns boot/generation/stamp."""

    def __init__(self, *, max_retired: int) -> None:
        # //Compatible: retain old callers while removing origin blacklists.
        self.active: str | None = None

    def retire_active(self) -> None:
        self.active = None

    def is_retired(self, epoch: str) -> bool:
        return False

    def retire_unadmitted(self, epoch: str) -> None:
        if self.active == epoch:
            self.active = None

    def accept(self, epoch: str) -> bool:
        self.active = epoch
        return True


class ProducerTopology:
    """Require one expected ROS graph endpoint; this is not DDS authentication."""

    def __init__(self, name: str, namespace: str) -> None:
        if not name or not namespace:
            raise ValueError("expected producer identity is required")
        self._name = name
        self._namespace = namespace
        self._gid: bytes | None = None
        self._ready = False

    def observe(self, endpoints: list[tuple[str, str, bytes]]) -> tuple[bool, bool]:
        gid = None
        if len(endpoints) == 1:
            name, namespace, candidate = endpoints[0]
            if name == self._name and namespace == self._namespace:
                gid = candidate
        ready = gid is not None
        changed = ready != self._ready
        self._gid, self._ready = gid, ready
        return ready, changed
