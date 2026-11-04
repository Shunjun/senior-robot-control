"""Small non-blocking ingress queue with an explicit invalidation barrier."""

from collections import deque
from collections.abc import Callable
from threading import Lock
from typing import Any


class BoundedInbox:
    def __init__(self, *, capacity: int) -> None:
        if type(capacity) is not int or not 1 <= capacity <= 16:
            raise ValueError("invalid_inbox_capacity")
        self._capacity = capacity
        self._lock = Lock()
        self._generation = 1
        self._items: deque[tuple[int, Any]] = deque()
        self._barrier = ""

    def revoke(self, reason: str) -> None:
        if type(reason) is not str or not reason or len(reason.encode("utf-8")) > 64:
            raise ValueError("invalid_invalidation_reason")
        with self._lock:
            self._generation += 1
            self._items.clear()
            self._barrier = reason

    def offer(self, value: Any) -> bool:
        with self._lock:
            if len(self._items) >= self._capacity:
                self._generation += 1
                self._items.clear()
                self._barrier = "overflow"
                return False
            self._items.append((self._generation, value))
            return True

    def take(self) -> tuple[int, Any, str] | None:
        with self._lock:
            if self._barrier:
                reason, self._barrier = self._barrier, ""
                return self._generation, None, reason
            if not self._items:
                return None
            generation, value = self._items.popleft()
            return generation, value, ""

    def snapshot(self) -> int:
        with self._lock:
            return self._generation

    def commit_if_current(self, generation: int, publish: Callable[[], None]) -> bool:
        """Order a bounded small-result publish against concurrent invalidations."""
        with self._lock:
            if generation != self._generation or self._barrier or self._items:
                return False
            publish()
            return True

    def current(self, generation: int) -> bool:
        with self._lock:
            return generation == self._generation
