"""Single-writer atomic storage. Invoked by the record owner's I/O worker only."""

import fcntl
import os
import stat
from pathlib import Path

from senior_control.motion.learning_record import (
    MAX_RECORD_BYTES,
    LearningRecord,
    ProgramManifest,
    parse_record,
)


def read_record(path: Path) -> LearningRecord:
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, "rb") as file:
        if not stat.S_ISREG(os.fstat(file.fileno()).st_mode):
            raise ValueError("learning_record_not_regular")
        return parse_record(file.read(MAX_RECORD_BYTES + 1))


def write_record(path: Path, record: LearningRecord | ProgramManifest) -> int:
    """Commit complete data and its directory entry before acknowledging revision."""
    payload = record.to_bytes()
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        pending = path.name + ".pending"
        fd = os.open(
            pending,
            os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW | os.O_NONBLOCK,
            0o600,
            dir_fd=directory,
        )
        with os.fdopen(fd, "wb") as file:
            info = os.fstat(file.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_uid != os.geteuid():
                raise ValueError("learning_pending_not_private_regular")
            file.truncate(0)
            file.write(payload)
            file.flush()
            os.fsync(file.fileno())
        os.replace(pending, path.name, src_dir_fd=directory, dst_dir_fd=directory)
        os.fsync(directory)
    finally:
        os.close(directory)
    return record.revision


class RecordWriterLock:
    """Stable lock inode; never lock the JSON inode replaced by a commit."""

    def __init__(self, path: Path) -> None:
        self.path = Path(str(path) + ".owner.lock")
        fd = os.open(self.path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
        try:
            if not stat.S_ISREG(os.fstat(fd).st_mode):
                raise ValueError("learning_owner_lock_not_regular")
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            os.close(fd)
            raise
        self.fd = fd

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


def read_manifest(path: Path) -> ProgramManifest:
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, "rb") as file:
        if not stat.S_ISREG(os.fstat(file.fileno()).st_mode):
            raise ValueError("learning_manifest_not_regular")
        return ProgramManifest.model_validate_json(file.read(MAX_RECORD_BYTES + 1))


def attempt_path(path: Path, attempt_id: str) -> Path:
    import re

    if re.fullmatch(r"[0-9a-f]{32}", attempt_id) is None:
        raise ValueError("learning_attempt_identity")
    return path.with_name(path.name + ".attempt-" + attempt_id)
