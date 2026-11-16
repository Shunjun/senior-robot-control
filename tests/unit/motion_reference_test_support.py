"""Controlled single-subscriber DDS-ready queue; never an arrival-time claim."""

from collections import deque


class ReadySubscription:
    def __init__(self, message_type, callback, depth=10):
        self.msg_type = message_type
        self.callback = callback
        self.raw = False
        self.handle = self
        self.ready = deque(maxlen=depth)
        self.takes = 0

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        pass

    def take_message(self, _type, _raw):
        self.takes += 1
        return self.ready.popleft() if self.ready else None


def final_command_message(**values):
    """Generated FinalCommand scalar defaults, with no admission behavior."""
    from types import SimpleNamespace

    return SimpleNamespace(
        **dict(
            {"manual_linear": 0.0, "manual_linear_limit": 0.0, "degraded_linear_limit": 0.0},
            **values,
        )
    )
