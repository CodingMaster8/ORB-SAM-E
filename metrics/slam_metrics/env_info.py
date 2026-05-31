"""
Capture the hardware / software environment for reproducibility.

Every experiment record should embed exactly what machine produced it: GPU
model and architecture, CUDA / driver / PyTorch versions, CPU, RAM, OS, and the
git commit of this repo. This is what reviewers expect in a "Setup" or
"Implementation details" section and lets you reproduce results later.
"""

from __future__ import annotations

import platform
import subprocess
from typing import Any, Dict, Optional


def _run(cmd: list) -> Optional[str]:
    try:
        out = subprocess.run(
            cmd, capture_output=True, text=True, timeout=10, check=False
        )
        if out.returncode == 0:
            return out.stdout.strip()
    except Exception:
        pass
    return None


def _git_commit() -> Optional[str]:
    return _run(["git", "rev-parse", "--short", "HEAD"])


def _git_dirty() -> Optional[bool]:
    status = _run(["git", "status", "--porcelain"])
    if status is None:
        return None
    return len(status.strip()) > 0


def _gpu_info() -> Dict[str, Any]:
    """Best-effort GPU details via pynvml, falling back to nvidia-smi / torch."""
    info: Dict[str, Any] = {"available": False}

    # Prefer torch for arch / capability.
    try:
        import torch
        info["torch_version"] = torch.__version__
        info["cuda_available"] = bool(torch.cuda.is_available())
        info["torch_cuda_version"] = getattr(torch.version, "cuda", None)
        if torch.cuda.is_available():
            info["available"] = True
            idx = torch.cuda.current_device()
            info["name"] = torch.cuda.get_device_name(idx)
            cap = torch.cuda.get_device_capability(idx)
            info["compute_capability"] = f"{cap[0]}.{cap[1]}"
            props = torch.cuda.get_device_properties(idx)
            info["total_memory_mb"] = round(props.total_memory / (1024 ** 2), 1)
            info["multi_processor_count"] = props.multi_processor_count
    except Exception:
        pass

    # nvidia-smi for driver version (and as a fallback for the name).
    smi = _run([
        "nvidia-smi",
        "--query-gpu=name,driver_version,memory.total",
        "--format=csv,noheader",
    ])
    if smi:
        first = smi.splitlines()[0]
        cols = [c.strip() for c in first.split(",")]
        if len(cols) >= 2:
            info.setdefault("name", cols[0])
            info["driver_version"] = cols[1]
            info["available"] = True
            if len(cols) >= 3:
                info.setdefault("memory_total_smi", cols[2])
    return info


def collect() -> Dict[str, Any]:
    """Collect the full environment snapshot as a JSON-serializable dict."""
    info: Dict[str, Any] = {
        "platform": platform.platform(),
        "machine": platform.machine(),          # e.g. x86_64, aarch64
        "processor": platform.processor(),
        "python_version": platform.python_version(),
        "cpu_count": None,
        "ram_total_gb": None,
        "git_commit": _git_commit(),
        "git_dirty": _git_dirty(),
        "gpu": _gpu_info(),
    }

    try:
        import os
        info["cpu_count"] = os.cpu_count()
    except Exception:
        pass

    try:
        import psutil
        info["ram_total_gb"] = round(psutil.virtual_memory().total / (1024 ** 3), 1)
        freq = psutil.cpu_freq()
        if freq:
            info["cpu_max_mhz"] = freq.max
    except Exception:
        pass

    return info


if __name__ == "__main__":
    import json
    print(json.dumps(collect(), indent=2))
