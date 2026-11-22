"""Comparable scalar consensus, shared by motion and pose owners, not permissions."""

import itertools
from dataclasses import dataclass


@dataclass(frozen=True)
class Vote:
    value: float
    raw_time: float
    source: str
    weight: float
    quality: float
    confidence: float
    low_age: bool
    alignment_error: float
    error: float | None
    families: frozenset[str] = frozenset()


@dataclass(frozen=True)
class Consensus:
    value: float
    confidence: float
    indices: tuple[int, ...]
    error: float | None
    reason: str
    band: str


def combine(votes: tuple[Vote, ...], tolerance: float) -> Consensus:
    if not votes or len(votes) > 8:
        raise ValueError("bounded scalar consensus needs 1..8 independent votes")

    def newest(i: int) -> tuple[float, float, float, str]:
        v = votes[i]
        return (-v.raw_time, -v.quality, v.alignment_error, v.source)

    groups: list[tuple[set[str], list[int]]] = []
    for i, v in enumerate(votes):
        families = set(v.families) or {v.source}
        members = [i]
        changed = True
        while changed:
            changed = False
            remaining = []
            for old, indices in groups:
                if old & families:
                    families |= old
                    members.extend(indices)
                    changed = True
                else:
                    remaining.append((old, indices))
            groups = remaining
        groups.append((families, members))
    active = tuple(min(indices, key=newest) for _, indices in groups)
    cliques = [
        g
        for n in range(2, len(active) + 1)
        for g in itertools.combinations(active, n)
        if all(
            abs(votes[a].value - votes[b].value)
            <= tolerance + min((votes[a].error or 0.0) + (votes[b].error or 0.0), tolerance)
            for a, b in itertools.combinations(g, 2)
        )
    ]
    chosen = (
        min(
            cliques,
            key=lambda g: (
                -len(g),
                -sum(votes[i].weight for i in g),
                -max(votes[i].raw_time for i in g),
                tuple(sorted(votes[i].source for i in g)),
            ),
        )
        if cliques
        else (min(active, key=newest),)
    )
    scale = max(votes[i].weight for i in chosen)
    if scale <= 0:
        raise ValueError("scalar votes need positive weights")
    weights = [votes[i].weight / scale for i in chosen]
    total = sum(weights)
    value = sum(votes[i].value * (w / total) for i, w in zip(chosen, weights, strict=True))
    confidence = sum(votes[i].confidence * w for i, w in zip(chosen, weights, strict=True)) / total
    low = any(votes[i].low_age for i in chosen)
    if low:
        confidence = min(0.6, confidence)
    if len(chosen) < len(active):
        confidence *= 0.75
    unresolved = not cliques and len(active) > 1
    if unresolved:
        confidence = min(0.2, confidence * 0.5)
    error = (
        None
        if any(votes[i].error is None for i in chosen)
        else max((votes[i].error or 0.0) + abs(votes[i].value - value) for i in chosen)
    )
    return Consensus(
        value,
        confidence,
        chosen,
        error,
        "conflict_unresolved" if unresolved else "weighted" if len(chosen) > 1 else "single",
        "low" if low else "normal",
    )
