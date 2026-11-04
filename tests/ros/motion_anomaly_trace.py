"""Bounded event capture for the owned live acceptance tests."""

from collections import defaultdict, deque


class AnomalyTrace:
    def __init__(self, *, max_samples=512, post_seconds=2.0):
        if max_samples < 1 or post_seconds < 0:
            raise ValueError("invalid anomaly trace limits")
        self.max_samples = max_samples
        self.post_seconds = post_seconds
        self.before = defaultdict(lambda: deque(maxlen=max_samples))
        self.after = defaultdict(lambda: deque(maxlen=max_samples))
        self.first = None
        self.prefix = None

    def record(self, topic, received, observed_clock_ns, sample, *, anomaly=False):
        item = (received, observed_clock_ns, sample)
        if self.first is None:
            self.before[topic].append(item)
            if anomaly:
                self.first = received
                self.prefix = {key: tuple(values) for key, values in self.before.items()}
        elif self.first <= received <= self.first + self.post_seconds:
            self.after[topic].append(item)

    def document(self, convert):
        if self.first is None:
            return None
        topics = {}
        for topic in sorted(set(self.prefix) | set(self.after)):
            items = (*self.prefix.get(topic, ()), *self.after.get(topic, ()))
            topics[topic] = [
                {
                    "received_steady": received,
                    "observed_clock_ns": observed_clock_ns,
                    "sample": convert(topic, sample),
                }
                for received, observed_clock_ns, sample in items
            ]
        return {
            "first_anomaly_steady": self.first,
            "max_samples_each_side": self.max_samples,
            "post_seconds": self.post_seconds,
            "topics": topics,
        }
