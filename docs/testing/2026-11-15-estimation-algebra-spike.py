"""Throwaway proposal algebra, NOT a production estimator or physical validation.

Only one already-aligned scalar, zero alignment error, and shared-root groups
are exercised. No ROS, input admission, covariance, pose, space or actuator path.
"""

import itertools
import json
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Item:
    name: str
    root: str
    value: float
    observed: float
    quality: float = 1.0


def age_band(age):
    return "invalid" if age >= 1 else "low" if age >= 0.3 else "normal"


def weight(item, now):
    age = now - item.observed
    freshness = 1 if age < 0.3 else 0.1 + 0.9 * (1 - age) / 0.7
    return item.quality * freshness


def newer(items):
    return sorted(items, key=lambda x: (-x.observed, -x.quality, x.name))[0]


def estimate(items, now=10.0, tolerance=0.1):
    roots = {}
    for item in items:
        if not all(math.isfinite(v) for v in (item.value, item.observed, item.quality)):
            raise ValueError("malformed scalar")
        if item.observed > now or not 0 <= item.quality <= 1:
            raise ValueError("invalid scalar metadata")
        if age_band(now - item.observed) == "invalid" or item.quality == 0:
            continue
        roots.setdefault(item.root, []).append(item)
    values = sorted((newer(group) for group in roots.values()), key=lambda i: i.name)
    if len(values) > 8:
        raise ValueError("bounded proposal capacity")
    if not values:
        return None, []
    groups = [
        group
        for count in range(2, len(values) + 1)
        for group in itertools.combinations(values, count)
        if all(abs(a.value - b.value) <= tolerance for a, b in itertools.combinations(group, 2))
    ]
    if not groups:
        chosen = newer(values)
        return chosen.value, [chosen.name]
    group = sorted(
        groups,
        key=lambda g: (
            -len(g),
            -sum(weight(i, now) for i in g),
            -max(i.observed for i in g),
            tuple(i.name for i in g),
        ),
    )[0]
    weights = [weight(i, now) for i in group]
    scale = max(weights)
    weights = [w / scale for w in weights]
    return sum(i.value * w for i, w in zip(group, weights, strict=True)) / sum(weights), [
        i.name for i in group
    ]


def main():
    rows = []
    cases = [
        (
            "close_quality_weight",
            [Item("a", "a", 0.4, 9.9), Item("b", "b", 0.44, 9.9, 0.5)],
            0.41333333333333333,
            ["a", "b"],
        ),
        (
            "close_age_weight",
            [Item("a", "a", 0.4, 9.35), Item("b", "b", 0.42, 9.9)],
            0.4129032258064516,
            ["a", "b"],
        ),
        (
            "two_conflict_raw_newer",
            [Item("a", "a", 0.1, 9.2), Item("b", "b", 0.9, 9.9, 0.1)],
            0.9,
            ["b"],
        ),
        (
            "consensus_beats_newest_outlier",
            [Item("a", "a", 0.4, 9.8), Item("b", "b", 0.44, 9.8), Item("c", "c", 1.2, 9.99)],
            0.42,
            ["a", "b"],
        ),
        (
            "no_consensus_newest",
            [Item("a", "a", 0, 9.8), Item("b", "b", 0.4, 9.85), Item("c", "c", 0.8, 9.9)],
            0.8,
            ["c"],
        ),
        (
            "same_time_quality_tie",
            [Item("a", "a", 0, 9.9, 0.5), Item("b", "b", 0.8, 9.9)],
            0.8,
            ["b"],
        ),
        ("same_time_stable_id_tie", [Item("b", "b", 0.8, 9.9), Item("a", "a", 0, 9.9)], 0, ["a"]),
        (
            "derived_duplicate_one_vote",
            [
                Item("wheel", "wheel", 0.4, 9.9),
                Item("wheel_copy", "wheel", 0.4, 9.8),
                Item("gps", "gps", 0.44, 9.9),
            ],
            0.42,
            ["gps", "wheel"],
        ),
        (
            "expired_contributor_other_still_usable",
            [Item("old", "old", 0.4, 9), Item("live", "live", 0.42, 9.8)],
            0.42,
            ["live"],
        ),
    ]
    for name, inputs, expected, selected in cases:
        value, used = estimate(inputs)
        assert math.isclose(value, expected, abs_tol=1e-12), (name, value, expected)
        assert used == selected, (name, used)
        rows.append(dict(case=name, value=value, selected=used, passed=True))
    bands = [age_band(a) for a in (0.299, 0.3, 0.999, 1)]
    assert bands == ["normal", "low", "low", "invalid"]
    original = Item("held", "held", 0.4, 9.5)
    assert estimate([original], now=10.0)[0] == 0.4
    assert estimate([original], now=10.5)[0] is None
    thresholds = {str(c): max(1, min(32, math.floor(0.75 * c))) for c in (1, 4, 16, 64, 1024)}
    assert thresholds == {"1": 1, "4": 3, "16": 12, "64": 32, "1024": 32}
    print(
        json.dumps(
            dict(
                scope="proposal_scalar_algebra_only",
                cases=rows,
                age_bands=bands,
                original_age_not_refreshed=True,
                queue_thresholds=thresholds,
                assertion_groups=12,
                all_passed=True,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
