"""
Background resource sampler: GPU utilization/memory/power/temp + CPU + RAM.

Run a `SystemMonitor` for the duration of an experiment to capture the
resource-usage timeseries that backs claims like "X% GPU utilization,
Y MB VRAM, Z W average power" -- the kind of numbers needed to argue an
approach is viable on an edge device such as the Jetson Orin Nano.

GPU stats come from NVML (via `pynvml`) when available, otherwise from parsing
`nvidia-smi`. CPU/RAM come from `psutil` when available. All dependencies are
optional: missing ones simply produce null fields rather than crashing.

Example:
    mon = SystemMonitor(interval=0.5)
    mon.start()
    ... run pipeline ...
    summary = mon.stop()
    mon.save_csv("resources.csv")
"""

from __future__ import annotations

import csv
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class _Sample:
    t: float
    gpu_util: Optional[float]
    gpu_mem_used_mb: Optional[float]
    gpu_power_w: Optional[float]
    gpu_temp_c: Optional[float]
    cpu_percent: Optional[float]
    ram_used_mb: Optional[float]


class SystemMonitor:
    """Samples system resources on a background thread at a fixed interval."""

    def __init__(self, interval: float = 0.5, gpu_index: int = 0):
        self.interval = interval
        self.gpu_index = gpu_index
        self._samples: List[_Sample] = []
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

        self._nvml = None
        self._nvml_handle = None
        self._init_nvml()

        self._psutil = None
        try:
            import psutil
            self._psutil = psutil
            psutil.cpu_percent(interval=None)  # prime the first reading
        except Exception:
            pass

    # -- backends ----------------------------------------------------------

    def _init_nvml(self) -> None:
        try:
            import pynvml
            pynvml.nvmlInit()
            self._nvml = pynvml
            self._nvml_handle = pynvml.nvmlDeviceGetHandleByIndex(self.gpu_index)
        except Exception:
            self._nvml = None
            self._nvml_handle = None

    def _read_gpu_nvml(self):
        p = self._nvml
        h = self._nvml_handle
        util = p.nvmlDeviceGetUtilizationRates(h).gpu
        mem = p.nvmlDeviceGetMemoryInfo(h).used / (1024 ** 2)
        try:
            power = p.nvmlDeviceGetPowerUsage(h) / 1000.0
        except Exception:
            power = None
        try:
            temp = p.nvmlDeviceGetTemperature(h, p.NVML_TEMPERATURE_GPU)
        except Exception:
            temp = None
        return float(util), float(mem), power, temp

    def _read_gpu_smi(self):
        import subprocess
        try:
            out = subprocess.run(
                ["nvidia-smi",
                 f"--query-gpu=utilization.gpu,memory.used,power.draw,temperature.gpu",
                 "--format=csv,noheader,nounits", "-i", str(self.gpu_index)],
                capture_output=True, text=True, timeout=5, check=False,
            )
            if out.returncode != 0:
                return None
            cols = [c.strip() for c in out.stdout.splitlines()[0].split(",")]

            def _f(x):
                try:
                    return float(x)
                except ValueError:
                    return None

            return _f(cols[0]), _f(cols[1]), _f(cols[2]) if len(cols) > 2 else None, \
                _f(cols[3]) if len(cols) > 3 else None
        except Exception:
            return None

    def _read_gpu(self):
        if self._nvml_handle is not None:
            try:
                return self._read_gpu_nvml()
            except Exception:
                pass
        smi = self._read_gpu_smi()
        if smi is not None:
            return smi
        return None, None, None, None

    # -- loop --------------------------------------------------------------

    def _loop(self) -> None:
        while not self._stop.is_set():
            gpu_util, gpu_mem, gpu_power, gpu_temp = self._read_gpu()
            cpu = ram = None
            if self._psutil is not None:
                try:
                    cpu = self._psutil.cpu_percent(interval=None)
                    ram = self._psutil.virtual_memory().used / (1024 ** 2)
                except Exception:
                    pass
            self._samples.append(_Sample(
                t=time.time(), gpu_util=gpu_util, gpu_mem_used_mb=gpu_mem,
                gpu_power_w=gpu_power, gpu_temp_c=gpu_temp,
                cpu_percent=cpu, ram_used_mb=ram,
            ))
            self._stop.wait(self.interval)

    # -- public API --------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._samples.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> Dict[str, Any]:
        if self._thread is not None:
            self._stop.set()
            self._thread.join(timeout=self.interval * 2 + 1)
            self._thread = None
        return self.summary()

    def _agg(self, attr: str) -> Optional[Dict[str, float]]:
        vals = [getattr(s, attr) for s in self._samples if getattr(s, attr) is not None]
        if not vals:
            return None
        import numpy as np
        a = np.asarray(vals, dtype=float)
        return {
            "mean": float(a.mean()),
            "max": float(a.max()),
            "min": float(a.min()),
            "p95": float(np.percentile(a, 95)),
        }

    def summary(self) -> Dict[str, Any]:
        return {
            "num_samples": len(self._samples),
            "interval_s": self.interval,
            "gpu_util_percent": self._agg("gpu_util"),
            "gpu_mem_used_mb": self._agg("gpu_mem_used_mb"),
            "gpu_power_w": self._agg("gpu_power_w"),
            "gpu_temp_c": self._agg("gpu_temp_c"),
            "cpu_percent": self._agg("cpu_percent"),
            "ram_used_mb": self._agg("ram_used_mb"),
        }

    def save_csv(self, path: str) -> None:
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "time", "gpu_util", "gpu_mem_used_mb", "gpu_power_w",
                "gpu_temp_c", "cpu_percent", "ram_used_mb",
            ])
            for s in self._samples:
                w.writerow([
                    s.t, s.gpu_util, s.gpu_mem_used_mb, s.gpu_power_w,
                    s.gpu_temp_c, s.cpu_percent, s.ram_used_mb,
                ])


if __name__ == "__main__":
    import json
    mon = SystemMonitor(interval=0.5)
    mon.start()
    print("Sampling resources for 5 s ...")
    time.sleep(5)
    print(json.dumps(mon.stop(), indent=2))
