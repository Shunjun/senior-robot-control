"""One consumer-owned ordering frontier per input stream.

The receiving owner confirms process replacement, using its lifecycle or an
immutable first-receipt coordinate from the configured local bridge. Producers
own generation changes; consumers adopt newer batches.
"""

import json


def source_context(epoch: str) -> tuple[str, int]:
    """Read adapter-owned lifetime metadata, keeping legacy opaque epochs readable."""
    try:
        value = json.loads(epoch)
    except (ValueError, TypeError):
        value = None
    if isinstance(value, dict) and set(value) == {"boot_id", "generation"}:
        return value["boot_id"], value["generation"]
    # //Compatible: pure algorithm callers historically supply an opaque lifetime.
    return epoch, 1


class SourceOrder:
    def __init__(self) -> None:
        self._boot_id: str | None = None
        self._generation = 0
        self._latest_stamp_ns: int | None = None
        self._last_receipt_ns: int | None = None
        self.reason = ""

    @property
    def boot_id(self) -> str | None:
        return self._boot_id

    @property
    def generation(self) -> int:
        return self._generation

    @property
    def latest_stamp_ns(self) -> int | None:
        return self._latest_stamp_ns

    def confirm_boot(self, boot_id: str) -> None:
        """Called by the lifecycle owner, never on an arbitrary different frame."""
        if not isinstance(boot_id, str) or not boot_id:
            raise ValueError("source boot must be nonempty")
        if boot_id != self._boot_id:
            self._boot_id = boot_id
            self._generation = 0
            self._latest_stamp_ns = None

    def observe_generation(self, boot_id: str, generation: int) -> bool:
        """Adopt a producer's batch transition without inventing a sample timestamp."""
        if (
            not isinstance(boot_id, str)
            or not boot_id
            or type(generation) is not int
            or not 0 < generation < 1 << 64
        ):
            self.reason = "invalid_source_metadata"
            return False
        if self._boot_id is None:
            self.confirm_boot(boot_id)
        if boot_id != self._boot_id:
            self.reason = "unconfirmed_source_boot"
            return False
        if generation < self._generation:
            self.reason = "old_source_generation"
            return False
        if generation > self._generation:
            self._generation = generation
            self._latest_stamp_ns = None
        self.reason = ""
        return True

    def accept(self, boot_id: str, generation: int, stamp_ns: int) -> bool:
        if type(stamp_ns) is not int or not 0 <= stamp_ns < 1 << 63:
            self.reason = "invalid_source_metadata"
            return False
        if not self.observe_generation(boot_id, generation):
            return False
        if self._latest_stamp_ns is not None and stamp_ns <= self._latest_stamp_ns:
            self.reason = "non_increasing_capture_stamp"
            return False
        self._latest_stamp_ns = stamp_ns
        self.reason = ""
        return True

    def accept_capture(
        self, boot_id: str, generation: int, stamp_ns: int, received_steady_ns: int
    ) -> bool:
        """Receive from the configured single local bridge after structural checks.

        A bridge replacement is confirmed by a newer immutable native first
        receipt, on the same host monotonic clock. Queued old-boot data keeps its
        earlier receipt and cannot change the active lifetime back. This is not
        an age threshold; no timestamp is sampled or rewritten here.
        """
        if (
            not isinstance(boot_id, str)
            or not boot_id
            or type(generation) is not int
            or not 0 < generation < 1 << 64
            or type(stamp_ns) is not int
            or not 0 <= stamp_ns < 1 << 63
            or type(received_steady_ns) is not int
            or not 0 <= received_steady_ns < 1 << 64
        ):
            self.reason = "invalid_source_metadata"
            return False
        if self._boot_id is not None and boot_id != self._boot_id:
            if self._last_receipt_ns is not None and received_steady_ns <= self._last_receipt_ns:
                self.reason = "old_source_boot"
                return False
            self.confirm_boot(boot_id)
        if not self.accept(boot_id, generation, stamp_ns):
            return False
        self._last_receipt_ns = max(received_steady_ns, self._last_receipt_ns or 0)
        return True
