"""Prepare offline matcher input from recorded filtered scans, never from truth.

Preserves recorded order and source epochs. This isolates matcher behavior; DDS
admission, TF discovery and callback receipt deadlines are not re-simulated.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from collections.abc import Iterable
from pathlib import Path
from typing import Any, TextIO

import yaml

LIMIT_FIELDS = (
    "max_interval", "range_limit", "min_response", "max_residual", "min_inlier_fraction",
    "min_spread", "min_translation_observability", "min_yaw_observability", "max_variance",
    "reanchor_distance", "reanchor_yaw", "min_rays",
)


def write_scan_records(envelopes: Iterable[dict[str, Any]], stream: TextIO) -> list[dict[str, Any]]:
    frames = []
    previous = None
    for ordinal, envelope in enumerate(envelopes):
        body, owner = envelope["body"], envelope["owner"]
        capture = body["raw_capture"]
        if not (owner["valid"] and body["valid"] and capture["valid"]
                and owner["owner_epoch"] == body["source_epoch"]):
            stream.write("R\n")
            previous = None
            continue
        continuity = (owner["owner_gid"], owner["owner_epoch"],
                      capture["raw_gid"], capture["source_epoch"])
        if previous is not None and continuity != previous:
            stream.write("R\n")
        previous = continuity
        stamp = capture["stamp"]
        ns = stamp["sec"] * 1_000_000_000 + stamp["nanosec"]
        header = [ns / 1e9, body["angle_min"], body["angle_increment"],
                  body["range_min"], body["range_max"], len(body["ranges"])]
        # float accepts the explicit nonfinite strings in the lossless bag export.
        stream.write("S " + " ".join(map(str, header)) + " " +
                     " ".join(str(float(value)) for value in body["ranges"]) + "\n")
        frames.append({"filtered_ordinal": ordinal, "stamp_ns": ns,
                       "producer_epoch": body["source_epoch"], "continuity_key": continuity})
    return frames


def prepare(samples: Path, profile: Path, output: Path) -> None:
    parameters = yaml.safe_load(profile.read_text())["scan_motion_node"]["ros__parameters"]
    filtered, expected = [], []
    with gzip.open(samples, "rt") as source:
        for line in source:
            record = json.loads(line)
            if record["topic"] == "/perception/scan_filtered":
                filtered.append(record["message"])
            elif record["topic"] == "/motion/scan":
                expected.append(record["message"])
    output.mkdir(mode=0o700)  # A new evidence directory, never overwrite an earlier run.
    with (output / "input.txt").open("x") as stream:
        stream.write("L " + " ".join(str(parameters[key]) for key in LIMIT_FIELDS) + "\n")
        frames = write_scan_records(filtered, stream)
    manifest = {"samples_sha256": hashlib.sha256(samples.read_bytes()).hexdigest(),
                "profile_sha256": hashlib.sha256(profile.read_bytes()).hexdigest(),
                "input_sha256": hashlib.sha256((output / "input.txt").read_bytes()).hexdigest(),
                "parameters": parameters, "frames": frames,
                "scope": "matcher replay; truth excluded; ROS receipt/TF admission not replayed"}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (output / "recorded-motion.json").write_text(json.dumps(expected, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prepare(args.samples, args.profile, args.output)


if __name__ == "__main__":
    main()
