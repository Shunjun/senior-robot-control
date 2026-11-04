"""Wire validation for local IPC frames."""

import asyncio
import struct

import pytest

from senior_control.runtime.ipc import IpcError, _read


@pytest.mark.parametrize("value", [b"NaN", b"Infinity", b"-Infinity", b"1e999"])
async def test_nonfinite_numbers_are_rejected_on_read(value):
    body = b'{"v":1,"type":"probe","value":' + value + b"}"
    reader = asyncio.StreamReader()
    reader.feed_data(struct.pack(">I", len(body)) + body)
    with pytest.raises(IpcError, match="frame is not json"):
        await _read(reader)
