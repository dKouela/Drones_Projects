"""Latency measurement.

"Reacts quickly" is a claim, and a claim needs a number. Every stage of the
loop is timed from the instant the frame was grabbed, and the summary reports
percentiles rather than a mean -- the tail is what makes a drone hit something.
"""

import threading
from collections import deque


class LatencyTracker:
    """Rolling record of a single stage's timings, in milliseconds."""

    def __init__(self, name, window=600):
        self.name = name
        self._samples = deque(maxlen=window)
        self._lock = threading.Lock()

    def record(self, milliseconds):
        with self._lock:
            self._samples.append(milliseconds)

    @property
    def count(self):
        with self._lock:
            return len(self._samples)

    def percentile(self, fraction):
        with self._lock:
            if not self._samples:
                return float("nan")
            ordered = sorted(self._samples)
        index = min(int(fraction * len(ordered)), len(ordered) - 1)
        return ordered[index]

    def summary(self):
        with self._lock:
            if not self._samples:
                return f"{self.name}: no samples"
            ordered = sorted(self._samples)
        mean = sum(ordered) / len(ordered)
        return (
            f"{self.name}: n={len(ordered)} "
            f"mean={mean:6.1f}ms "
            f"p50={self.percentile(0.50):6.1f}ms "
            f"p95={self.percentile(0.95):6.1f}ms "
            f"max={ordered[-1]:6.1f}ms"
        )


class LatencyReport:
    """A named group of trackers, reported together."""

    def __init__(self, *names):
        self._trackers = {name: LatencyTracker(name) for name in names}

    def __getitem__(self, name):
        return self._trackers[name]

    def lines(self):
        return [tracker.summary() for tracker in self._trackers.values()]
