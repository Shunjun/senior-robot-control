"""Remove prior run's generated learning files after the startup boundary."""

import fcntl
import logging
import os
import re
import stat
from pathlib import Path
from typing import Literal

_LOG = logging.getLogger(__name__)
_CLAIM = r"\.startup-learning-[0-9a-f]{32}"
_SUFFIXES = ("", ".pending", ".features.json", ".features.json.pending", ".owner.lock")
_GENERATED_FILE = re.compile(
    rf"(?P<claim>{_CLAIM})(?:\.pending|\.owner\.lock|\.features\.json(?:\.pending)?)?"
)


def _remove_old_files(directory: int, root: Path, current_name: str) -> None:
    with os.scandir(directory) as entries:
        for entry in entries:
            match = _GENERATED_FILE.fullmatch(entry.name)
            if match is None or match["claim"] == current_name:
                continue
            name = match["claim"]
            lock_fd = -1
            try:
                try:
                    lock_fd = os.open(
                        name + ".owner.lock",
                        os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK,
                        dir_fd=directory,
                    )
                except FileNotFoundError:
                    pass  # Old fixed-marker runs had no writer lock.
                if lock_fd >= 0:
                    if not stat.S_ISREG(os.fstat(lock_fd).st_mode):
                        raise ValueError("old learning owner lock is not regular")
                    fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                # Lock inode is removed last, after all record/temporary files.
                for suffix in _SUFFIXES:
                    candidate = name + suffix
                    try:
                        info = os.stat(candidate, dir_fd=directory, follow_symlinks=False)
                        if stat.S_ISREG(info.st_mode):
                            os.unlink(candidate, dir_fd=directory)
                    except FileNotFoundError:
                        continue
                    except OSError as exc:
                        _LOG.warning(
                            "Learning cleanup cannot remove %s/%s: %s", root, candidate, exc
                        )
            except (OSError, ValueError) as exc:
                _LOG.warning("Learning cleanup preserving %s/%s: %s", root, name, exc)
            finally:
                if lock_fd >= 0:
                    os.close(lock_fd)


def cleanup_old_learning_files(
    root: Path,
    current_claim: str,
    *,
    peer_lock_name: Literal[".agent-service.lock", ".senior-simulator.lock"],
) -> None:
    """Called once while the start owner's existing service lock is held.

    Also hold the other supervisor's existing root lock non-blockingly: a live
    hardware/simulation peer must keep its claims even when sharing this root.
    No recursion, symlink traversal, current-run deletion, or retries on failure.
    """
    current = Path(os.path.abspath(current_claim))
    if current.parent != Path(os.path.abspath(root)) or not re.fullmatch(_CLAIM, current.name):
        _LOG.warning("Learning cleanup skipped: current claim is not a generated claim in %s", root)
        return
    try:
        directory = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    except OSError as exc:
        _LOG.warning("Learning cleanup cannot open control root %s: %s", root, exc)
        return
    try:
        peer = os.open(
            peer_lock_name,
            os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK,
            0o600,
            dir_fd=directory,
        )
        with os.fdopen(peer, "r+b") as lock:
            if not stat.S_ISREG(os.fstat(lock.fileno()).st_mode):
                _LOG.warning("Learning cleanup skipped: peer service lock is not a regular file")
                return
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            _remove_old_files(directory, root, current.name)
    except OSError as exc:
        _LOG.warning("Learning cleanup skipped or incomplete in %s: %s", root, exc)
    finally:
        try:
            os.close(directory)
        except OSError as exc:
            _LOG.warning("Learning cleanup cannot close control root %s: %s", root, exc)
