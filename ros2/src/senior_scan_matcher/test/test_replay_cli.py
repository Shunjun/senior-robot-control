"""The replay stdout contract must remain machine-readable despite Karto logs."""

import json
import math
import subprocess
import sys

ranges = []
for index in range(720):
    angle = -math.pi + index * 2 * math.pi / 720
    dx, dy = math.cos(angle), math.sin(angle)
    hits = []
    for wall in (-2.3, 3.1):
        if abs(dx) > 1e-12:
            t = wall / dx
            if t > 0 and -2 < t * dy < 2.7:
                hits.append(t)
    for wall in (-2.0, 2.7):
        if abs(dy) > 1e-12:
            t = wall / dy
            if t > 0 and -2.3 < t * dx < 3.1:
                hits.append(t)
    ranges.append(min(hits))
header = "L .4 8 .3 .09 .55 .04 .05 .01 .05 .12 .1 100\n"
for stamp in (1.0, 1.1):
    header += f"S {stamp} {-math.pi} {2 * math.pi / 720} .08 8 720 "
    header += " ".join(map(str, ranges)) + "\n"
process = subprocess.run([sys.argv[1]], input=header, text=True, capture_output=True, timeout=5)
assert process.returncode == 0, process.stderr
rows = [json.loads(line) for line in process.stdout.splitlines()]
assert len(rows) == 2
assert rows[0]["reason"] == "initializing"
assert rows[1]["valid"]
