"""Linux boot identity for persisted local learning-run ownership."""

from pathlib import Path
from uuid import UUID


def monotonic_domain() -> str:
    """Fail explicitly when a shared Linux boot clock cannot be identified."""
    try:
        value = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        return str(UUID(value))
    except (OSError, ValueError) as exc:
        raise RuntimeError(
            "learning ownership requires a same-host Linux monotonic boot domain"
        ) from exc
