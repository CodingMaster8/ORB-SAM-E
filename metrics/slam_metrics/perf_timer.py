"""
Per-stage latency / throughput measurement.

Use `StageTimer` to accumulate timing samples for a named processing stage
(e.g. "efficientsam3_inference", "mask_apply", "orbslam_track") and get back
latency percentiles and FPS. Designed to be dropped into the filter node and
driver with negligible overhead.

Example:
    timer = StageTimer("efficientsam3_inference")
    with timer.measure():
        run_inference()
    print(timer.summary())          # dict with mean/p50/p95/fps/...
"""

from __future__ import annotations

import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np


@dataclass
class StageTimer:
    """Accumulates wall-clock durations (seconds) for a single named stage."""

    name: str
    samples: List[float] = field(default_factory=list)
    # If a CUDA device is in use, host timing already captures sync if the
    # caller synchronizes; pass synchronize=True to do it here.
    synchronize: bool = False

    @contextmanager
    def measure(self):
        """Context manager that records the elapsed time of the wrapped block."""
        if self.synchronize:
            _cuda_sync()
        start = time.perf_counter()
        try:
            yield
        finally:
            if self.synchronize:
                _cuda_sync()
            self.samples.append(time.perf_counter() - start)

    def add(self, seconds: float) -> None:
        self.samples.append(float(seconds))

    def reset(self) -> None:
        self.samples.clear()

    def summary(self) -> Dict[str, float]:
        """Return latency (ms) percentiles and throughput (fps)."""
        if not self.samples:
            return {
                "name": self.name, "count": 0, "fps": 0.0,
                "mean_ms": 0.0, "median_ms": 0.0, "p95_ms": 0.0,
                "min_ms": 0.0, "max_ms": 0.0, "std_ms": 0.0, "total_s": 0.0,
            }
        arr = np.asarray(self.samples, dtype=np.float64)
        mean = float(arr.mean())
        return {
            "name": self.name,
            "count": int(arr.size),
            "fps": float(1.0 / mean) if mean > 0 else 0.0,
            "mean_ms": mean * 1e3,
            "median_ms": float(np.median(arr)) * 1e3,
            "p95_ms": float(np.percentile(arr, 95)) * 1e3,
            "min_ms": float(arr.min()) * 1e3,
            "max_ms": float(arr.max()) * 1e3,
            "std_ms": float(arr.std()) * 1e3,
            "total_s": float(arr.sum()),
        }


class PerfRegistry:
    """Container for multiple named StageTimers."""

    def __init__(self, synchronize: bool = False):
        self._timers: Dict[str, StageTimer] = {}
        self._synchronize = synchronize

    def timer(self, name: str) -> StageTimer:
        if name not in self._timers:
            self._timers[name] = StageTimer(name, synchronize=self._synchronize)
        return self._timers[name]

    @contextmanager
    def measure(self, name: str):
        with self.timer(name).measure():
            yield

    def add(self, name: str, seconds: float) -> None:
        self.timer(name).add(seconds)

    def summary(self) -> Dict[str, Dict[str, float]]:
        return {name: t.summary() for name, t in self._timers.items()}

    def reset(self) -> None:
        for t in self._timers.values():
            t.reset()


def _cuda_sync() -> None:
    """Synchronize CUDA if torch+CUDA is available (no-op otherwise)."""
    try:
        import torch
        if torch.cuda.is_available():
            torch.cuda.synchronize()
    except Exception:
        pass
