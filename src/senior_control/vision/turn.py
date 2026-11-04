"""Short-lived TURN credentials in the coturn REST scheme (`use-auth-secret`).

The shared secret never leaves the cloud and the coturn host. A viewer receives a username
that embeds its own expiry and an HMAC over it, so coturn can check both without a user
table. Nothing here starts a server.
"""

import base64
import hashlib
import hmac
from dataclasses import dataclass, field

MIN_SECRET_BYTES = 32
MAX_TTL_SECONDS = 3600
MAX_SESSION_ID_CHARS = 128
TURN_SCHEMES = ("turn:", "turns:")


@dataclass(frozen=True)
class IceCredentials:
    """What a viewer or the vehicle hands to its ICE agent. The credential stays out of repr."""

    urls: tuple[str, ...]
    username: str
    credential: str = field(repr=False)
    expires_at: int
    relay_only: bool = False


@dataclass(frozen=True)
class TurnConfig:
    urls: tuple[str, ...]
    secret: bytes = field(repr=False)
    ttl_seconds: int = 600

    def __post_init__(self) -> None:
        if len(self.secret) < MIN_SECRET_BYTES:
            raise ValueError(f"TURN secret must be at least {MIN_SECRET_BYTES} bytes")
        if not self.urls or any(not url.startswith(TURN_SCHEMES) for url in self.urls):
            raise ValueError("TURN urls must use the turn: or turns: scheme")
        if not 0 < self.ttl_seconds <= MAX_TTL_SECONDS:
            raise ValueError(f"TURN ttl must be between 1 and {MAX_TTL_SECONDS} seconds")


def _sign(secret: bytes, username: str) -> str:
    # coturn computes HMAC-SHA1 for this scheme, so the digest choice is not ours to change.
    digest = hmac.new(secret, username.encode(), hashlib.sha1).digest()
    return base64.b64encode(digest).decode()


class TurnCredentialIssuer:
    def __init__(self, config: TurnConfig) -> None:
        self._config = config

    def issue_turn_credentials(self, session_id: str, now: int) -> IceCredentials:
        """Credentials that coturn accepts until `now + ttl`, bound to one session id."""
        if not session_id or len(session_id) > MAX_SESSION_ID_CHARS or ":" in session_id:
            raise ValueError("session id must be 1-128 characters without ':'")
        expires_at = int(now) + self._config.ttl_seconds
        username = f"{expires_at}:{session_id}"
        return IceCredentials(
            urls=self._config.urls,
            username=username,
            credential=_sign(self._config.secret, username),
            expires_at=expires_at,
        )


def issue_turn_credentials(session_id: str, now: int, *, config: TurnConfig) -> IceCredentials:
    return TurnCredentialIssuer(config).issue_turn_credentials(session_id, now)


def verify_turn_credentials(username: str, credential: str, now: int, secret: bytes) -> bool:
    """The check coturn performs: a well-formed, unexpired username with a matching HMAC."""
    expiry_text, separator, session_id = username.partition(":")
    if not separator or not session_id or not expiry_text.isdigit():
        return False
    if now >= int(expiry_text):
        return False
    return hmac.compare_digest(_sign(secret, username), credential)
