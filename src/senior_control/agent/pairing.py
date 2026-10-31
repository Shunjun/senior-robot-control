"""One-time HTTPS device pairing and private credential storage."""

from __future__ import annotations

import json
import os
import stat
import tempfile
from pathlib import Path
from typing import TYPE_CHECKING
from urllib.parse import urlsplit

if TYPE_CHECKING:
    import httpx
from pydantic import BaseModel, ConfigDict, Field, ValidationError


class Credentials(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, hide_input_in_errors=True)
    robot_id: str = Field(min_length=1, max_length=128)
    device_token: str = Field(min_length=1, max_length=256, repr=False)


def _read_private(path: Path) -> bytes:
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    fd = os.open(path, flags)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_mode & 0o077 or info.st_size > 4096:
            raise ValueError("credential file must be a private regular file")
        with os.fdopen(fd, "rb", closefd=False) as file:
            return file.read(4097)
    finally:
        os.close(fd)


def read_credentials(path: Path) -> Credentials:
    try:
        return Credentials.model_validate_json(_read_private(path))
    except ValidationError as exc:
        raise ValueError("invalid credential file") from exc


def read_token(path: Path) -> str:
    try:
        token = _read_private(path).decode("utf-8").strip()
    except UnicodeError as exc:
        raise ValueError("invalid credential file") from exc
    if not token or len(token) > 256:
        raise ValueError("invalid credential file")
    return token


async def pair_device(
    cloud_url: str,
    code: str,
    name: str,
    path: Path,
    *,
    transport: httpx.AsyncBaseTransport | None = None,
) -> str:
    import httpx

    url = urlsplit(cloud_url)
    if url.scheme != "wss" or not url.hostname or url.username or url.password:
        raise ValueError("cloud URL must use authenticated TLS")
    if not path.is_absolute() or not code.strip() or not name.strip():
        raise ValueError("pairing requires a code, name and absolute credential path")
    if os.path.lexists(path):
        raise FileExistsError("credential file already exists")
    endpoint = f"https://{url.netloc}/api/devices/pair"
    try:
        async with httpx.AsyncClient(transport=transport, timeout=5.0) as client:
            async with client.stream(
                "POST", endpoint, json={"code": code, "name": name}
            ) as response:
                if response.status_code != 200:
                    raise ValueError("pairing failed")
                chunks = bytearray()
                async for chunk in response.aiter_bytes():
                    chunks.extend(chunk)
                    if len(chunks) > 4096:
                        raise ValueError("pairing failed")
        credential = Credentials.model_validate_json(bytes(chunks))
    except (httpx.HTTPError, ValidationError) as exc:
        raise ValueError("pairing failed") from exc
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as file:
            json.dump(credential.model_dump(), file, separators=(",", ":"))
            file.flush()
            os.fsync(file.fileno())
        os.link(temporary, path)
    finally:
        os.unlink(temporary)
    return credential.robot_id
