import os
import time
from pathlib import Path

from senior_control.vision.simulation_video import LatestFrameFile, publish_image


async def test_source_reads_only_the_latest_new_frame(tmp_path: Path) -> None:
    path = tmp_path / "camera.rgb"
    source = LatestFrameFile(path)
    assert await source.next_frame(0.01) is None

    publish_image(path, 1, 1, 3, "rgb8", b"abc")
    publish_image(path, 1, 1, 3, "bgr8", b"xyz")
    latest = await source.next_frame(0.01)
    assert latest is not None and latest.data == b"xyz" and latest.encoding == "bgr8"
    assert await source.next_frame(0.01) is None
    assert path.stat().st_mode & 0o777 == 0o600


async def test_source_rejects_stale_corrupt_and_nonregular_slots(tmp_path: Path) -> None:
    path = tmp_path / "camera.rgb"
    publish_image(path, 1, 1, 3, "rgb8", b"abc", captured_ns=time.monotonic_ns() - 3_000_000_000)
    assert await LatestFrameFile(path).next_frame(0.01) is None
    path.write_bytes(b"malformed")
    assert await LatestFrameFile(path).next_frame(0.01) is None
    path.unlink()
    os.mkfifo(path, 0o600)
    assert await LatestFrameFile(path).next_frame(0.01) is None


async def test_writer_packs_padded_rows_and_ignores_bad_images(tmp_path: Path) -> None:
    path = tmp_path / "camera.rgb"
    assert publish_image(path, 1, 2, 4, "rgb8", b"abc0def1")
    assert not publish_image(path, 1, 2, 4, "rgba8", b"abc0def1")
    assert not publish_image(path, 1, 2, 4, "rgb8", b"abc0")
    assert not publish_image(path, 641, 1, 1923, "rgb8", b"")
    frame = await LatestFrameFile(path).next_frame(0.01)
    assert frame is not None and frame.data == b"abcdef"
