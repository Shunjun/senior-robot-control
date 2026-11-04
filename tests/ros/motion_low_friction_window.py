"""Observation predicate for the live low-friction fixture."""


def stopped_with_fresh_request(started, now, outputs, requests, reason_times, motion_times):
    outputs = [(t, x, z) for t, x, z in outputs if t >= started]
    requests = [(t, x) for t, x in requests if t >= started]
    acknowledged = next((t for t, x in requests if abs(x - 0.05) < 1e-9), None)
    if acknowledged is None:
        return False
    requests = [(t, x) for t, x in requests if t >= acknowledged]
    return (
        now - acknowledged >= 2.0
        and any(t >= acknowledged for t in reason_times)
        and len(outputs) >= 5
        and all(x == 0.0 and z == 0.0 for _, x, z in outputs)
        and len(requests) >= 5
        and now - requests[-1][0] < 0.3
        and all(abs(x - 0.05) < 1e-9 for _, x in requests)
        and any(t >= acknowledged for t in motion_times)
    )
