"""Single-admin authentication: password hash, sessions, CSRF, failure limiter."""

import asyncio
import hashlib
import hmac
import secrets
import time
from collections import OrderedDict, deque
from collections.abc import Callable
from dataclasses import dataclass

from senior_control.cloud.db import fetch_all, write_transaction
from senior_control.storage.database import Database

SESSION_STATEMENTS = (
    """
    CREATE TABLE cloud_session (
        token_hash TEXT PRIMARY KEY,
        csrf_hash TEXT NOT NULL,
        created_at REAL NOT NULL,
        expires_at REAL NOT NULL
    )
    """,
)

_SCRYPT_N = 2**14
_SCRYPT_R = 8
_SCRYPT_P = 1
_SCRYPT_MAXMEM = 64 * 1024 * 1024


def hash_password(password: str) -> str:
    """Salted scrypt hash in `scrypt$n$r$p$salt$hash` form."""
    salt = secrets.token_bytes(16)
    digest = hashlib.scrypt(
        password.encode(),
        salt=salt,
        n=_SCRYPT_N,
        r=_SCRYPT_R,
        p=_SCRYPT_P,
        maxmem=_SCRYPT_MAXMEM,
        dklen=32,
    )
    return f"scrypt${_SCRYPT_N}${_SCRYPT_R}${_SCRYPT_P}${salt.hex()}${digest.hex()}"


def is_password_hash(stored: str) -> bool:
    return _parse_hash(stored) is not None


def _parse_hash(stored: str) -> tuple[int, int, int, bytes, bytes] | None:
    parts = stored.split("$")
    if len(parts) != 6 or parts[0] != "scrypt":
        return None
    try:
        n, r, p = int(parts[1]), int(parts[2]), int(parts[3])
        salt, expected = bytes.fromhex(parts[4]), bytes.fromhex(parts[5])
    except ValueError:
        return None
    if not (2 <= n <= 2**20 and n & (n - 1) == 0 and 1 <= r <= 32 and 1 <= p <= 16):
        return None
    if not salt or not expected:
        return None
    return n, r, p, salt, expected


def verify_password(password: str, stored: str) -> bool:
    parsed = _parse_hash(stored)
    if parsed is None:
        return False
    n, r, p, salt, expected = parsed
    try:
        actual = hashlib.scrypt(
            password.encode(),
            salt=salt,
            n=n,
            r=r,
            p=p,
            maxmem=_SCRYPT_MAXMEM,
            dklen=len(expected),
        )
    except ValueError:
        return False
    return hmac.compare_digest(actual, expected)


async def verify_password_async(password: str, stored: str) -> bool:
    return await asyncio.to_thread(verify_password, password, stored)


class TokenDigest:
    """Keyed digest so a stolen database cannot confirm or replay a secret."""

    def __init__(self, secret: bytes) -> None:
        self._key = hashlib.sha256(b"senior-control/cloud/digest\0" + secret).digest()

    def __call__(self, value: str) -> str:
        return hmac.new(self._key, value.encode(), hashlib.sha256).hexdigest()


class FailureLimiter:
    """In-memory sliding window of failures per client and across all clients.

    The cloud runs as one worker process, so the state lives in the process.
    """

    def __init__(
        self,
        *,
        per_client: int,
        global_limit: int,
        window_seconds: float,
        tracked_clients: int,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._per_client = per_client
        self._global_limit = global_limit
        self._window = window_seconds
        self._tracked = tracked_clients
        self._clock = clock
        self._clients: OrderedDict[str, deque[float]] = OrderedDict()
        self._all: deque[float] = deque()

    def _prune(self, failures: deque[float]) -> None:
        horizon = self._clock() - self._window
        while failures and failures[0] <= horizon:
            failures.popleft()

    def retry_after(self, client: str) -> int:
        """Seconds until a blocked client may try again, or 0 when not blocked."""
        self._prune(self._all)
        mine = self._clients.get(client)
        if mine is not None:
            self._prune(mine)
            if not mine:
                del self._clients[client]
                mine = None
        waits = []
        if mine is not None and len(mine) >= self._per_client:
            waits.append(mine[len(mine) - self._per_client])
        if len(self._all) >= self._global_limit:
            waits.append(self._all[len(self._all) - self._global_limit])
        if not waits:
            return 0
        return max(1, int(max(waits) + self._window - self._clock()) + 1)

    def record_failure(self, client: str) -> None:
        now = self._clock()
        self._all.append(now)
        failures = self._clients.pop(client, None)
        if failures is None:
            failures = deque()
        failures.append(now)
        self._clients[client] = failures
        while len(self._clients) > self._tracked:
            self._clients.popitem(last=False)

    def reset(self, client: str) -> None:
        self._clients.pop(client, None)


@dataclass(frozen=True)
class SessionRecord:
    token_hash: str
    csrf_hash: str
    expires_at: float


class SessionStore:
    def __init__(self, database: Database, digest: TokenDigest, ttl_seconds: float) -> None:
        self._database = database
        self._digest = digest
        self._ttl = ttl_seconds

    async def create(self) -> tuple[str, str]:
        """Return a new (session token, csrf token). Only keyed digests are stored."""
        token = secrets.token_urlsafe(32)
        csrf = secrets.token_urlsafe(32)
        now = time.time()
        async with write_transaction(self._database) as connection:
            await connection.execute("DELETE FROM cloud_session WHERE expires_at <= ?", (now,))
            await connection.execute(
                "INSERT INTO cloud_session (token_hash, csrf_hash, created_at, expires_at)"
                " VALUES (?, ?, ?, ?)",
                (self._digest(token), self._digest(csrf), now, now + self._ttl),
            )
        return token, csrf

    async def lookup(self, token: str | None) -> SessionRecord | None:
        if not token or len(token) > 256:
            return None
        rows = await fetch_all(
            self._database,
            "SELECT token_hash, csrf_hash, expires_at FROM cloud_session"
            " WHERE token_hash = ? AND expires_at > ?",
            (self._digest(token), time.time()),
        )
        if not rows:
            return None
        return SessionRecord(str(rows[0][0]), str(rows[0][1]), float(rows[0][2]))

    async def delete(self, token_hash: str) -> None:
        async with write_transaction(self._database) as connection:
            await connection.execute(
                "DELETE FROM cloud_session WHERE token_hash = ?", (token_hash,)
            )

    def csrf_matches(self, record: SessionRecord, csrf: str | None) -> bool:
        if not csrf or len(csrf) > 256:
            return False
        return hmac.compare_digest(self._digest(csrf), record.csrf_hash)
