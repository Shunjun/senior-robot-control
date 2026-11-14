"""Strict same-build guard v2 decoder; no old-frame/default-field fallback."""

import math
import struct
from types import SimpleNamespace

LAYOUT = struct.Struct("!4sBBH24s129s129s129s129sQQQQddd17s129sQd33s")


PAYLOAD_FIELDS = (
    "robot_id",
    "owner_boot_id",
    "session_id",
    "request_id",
    "generation",
    "connection_epoch",
    "sequence",
    "gesture",
    "steady_deadline",
    "vx",
    "wz",
    "source_kind",
    "source_id",
    "source_epoch",
    "received_steady",
    "close_reason",
)


def _text(value: bytes) -> str:
    end = value.find(b"\0")
    if end < 0 or any(value[end:]):
        raise ValueError("noncanonical frame string")
    return value[:end].decode("utf-8", errors="strict")


def decode_frame(packet: bytes) -> tuple[int, bytes, SimpleNamespace]:
    if len(packet) != LAYOUT.size:
        raise ValueError("guard frame length")
    (
        magic,
        version,
        kind,
        length,
        gid,
        robot,
        boot,
        session,
        request,
        generation,
        epoch,
        sequence,
        gesture,
        deadline,
        vx,
        wz,
        source_kind,
        source_id,
        source_epoch,
        received,
        reason,
    ) = LAYOUT.unpack(packet)
    if magic != b"SCF2" or version != 2 or length != LAYOUT.size or kind not in (0, 1, 2):
        raise ValueError("guard frame version/type")
    if not all(math.isfinite(x) for x in (deadline, vx, wz, received)):
        raise ValueError("guard frame number")
    msg = SimpleNamespace(
        robot_id=_text(robot),
        owner_boot_id=_text(boot),
        session_id=_text(session),
        request_id=_text(request),
        generation=generation,
        connection_epoch=epoch,
        sequence=sequence,
        gesture=gesture,
        steady_deadline=deadline,
        vx=vx,
        wz=wz,
        source_kind=_text(source_kind),
        source_id=_text(source_id),
        source_epoch=source_epoch,
        received_steady=received,
        close_reason=_text(reason),
    )
    return kind, gid, msg
