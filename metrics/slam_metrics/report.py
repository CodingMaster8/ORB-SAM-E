"""
Aggregate one experiment run into a structured record + human-readable report.

An "experiment" is a single pipeline run with a known configuration
(sequence, model, filter on/off, device, ...). This module collects:
    - environment snapshot (env_info),
    - per-stage timing (perf_timer),
    - resource usage (system_monitor),
    - trajectory accuracy (trajectory_eval),
    - filtering statistics (from the filter node),
and writes:
    - results.json   machine-readable, one object per run,
    - report.md      a Markdown summary you can paste into a paper/appendix.

It can also combine several runs/results.json files into a single comparison
table (baseline vs filtered, across sequences).
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class ExperimentRecord:
    """One pipeline run."""

    name: str                                   # human label, e.g. "fr3_walking_xyz_filtered"
    config: Dict[str, Any] = field(default_factory=dict)
    env: Dict[str, Any] = field(default_factory=dict)
    timing: Dict[str, Any] = field(default_factory=dict)
    resources: Dict[str, Any] = field(default_factory=dict)
    trajectory: Optional[Dict[str, Any]] = None
    filtering: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "timestamp": self.timestamp,
            "config": self.config,
            "env": self.env,
            "timing": self.timing,
            "resources": self.resources,
            "trajectory": self.trajectory,
            "filtering": self.filtering,
        }

    def save_json(self, path: str) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @staticmethod
    def load_json(path: str) -> "ExperimentRecord":
        with open(path) as f:
            d = json.load(f)
        return ExperimentRecord(
            name=d.get("name", os.path.basename(path)),
            config=d.get("config", {}),
            env=d.get("env", {}),
            timing=d.get("timing", {}),
            resources=d.get("resources", {}),
            trajectory=d.get("trajectory"),
            filtering=d.get("filtering", {}),
            timestamp=d.get("timestamp", 0.0),
        )


# ---------------------------------------------------------------------------
# Markdown rendering
# ---------------------------------------------------------------------------

def _fmt(v: Any, nd: int = 3) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def _get(d: Optional[Dict], *keys, default=None):
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def render_single(rec: ExperimentRecord) -> str:
    """Render a single run as a Markdown section."""
    lines: List[str] = []
    lines.append(f"## {rec.name}\n")

    gpu = _get(rec.env, "gpu") or {}
    lines.append("**Environment**")
    lines.append("")
    lines.append(f"- GPU: {gpu.get('name', '—')} "
                 f"(arch {gpu.get('compute_capability', '—')}, "
                 f"{gpu.get('total_memory_mb', '—')} MB)")
    lines.append(f"- CUDA: {gpu.get('torch_cuda_version', '—')}, "
                 f"driver {gpu.get('driver_version', '—')}, "
                 f"torch {gpu.get('torch_version', '—')}")
    lines.append(f"- Arch: {rec.env.get('machine', '—')}, "
                 f"CPUs: {rec.env.get('cpu_count', '—')}, "
                 f"RAM: {rec.env.get('ram_total_gb', '—')} GB")
    lines.append(f"- Commit: {rec.env.get('git_commit', '—')} "
                 f"(dirty={rec.env.get('git_dirty', '—')})")
    lines.append("")

    if rec.config:
        lines.append("**Config**")
        lines.append("")
        for k, v in rec.config.items():
            lines.append(f"- {k}: {v}")
        lines.append("")

    # Timing table
    if rec.timing:
        lines.append("**Timing (per stage)**")
        lines.append("")
        lines.append("| Stage | FPS | mean (ms) | median (ms) | p95 (ms) | count |")
        lines.append("|---|---|---|---|---|---|")
        for stage, s in rec.timing.items():
            lines.append(
                f"| {stage} | {_fmt(s.get('fps'),1)} | {_fmt(s.get('mean_ms'),2)} "
                f"| {_fmt(s.get('median_ms'),2)} | {_fmt(s.get('p95_ms'),2)} "
                f"| {s.get('count','—')} |"
            )
        lines.append("")

    # Accuracy
    if rec.trajectory:
        ate = _get(rec.trajectory, "ate") or {}
        rpe_t = _get(rec.trajectory, "rpe_trans") or {}
        rpe_r = _get(rec.trajectory, "rpe_rot_deg") or {}
        lines.append("**Accuracy (vs groundtruth)**")
        lines.append("")
        lines.append(f"- ATE RMSE: {_fmt(ate.get('rmse'),4)} m "
                     f"(mean {_fmt(ate.get('mean'),4)}, median {_fmt(ate.get('median'),4)}, "
                     f"std {_fmt(ate.get('std'),4)})")
        lines.append(f"- RPE trans RMSE: {_fmt(rpe_t.get('rmse'),4)} m | "
                     f"RPE rot RMSE: {_fmt(rpe_r.get('rmse'),4)} deg")
        lines.append(f"- Associations: {rec.trajectory.get('num_associations','—')}, "
                     f"alignment scale: {_fmt(rec.trajectory.get('scale'),4)}")
        if rec.trajectory.get("notes"):
            lines.append(f"- NOTE: {rec.trajectory['notes']}")
        lines.append("")

    # Resources
    if rec.resources:
        gu = _get(rec.resources, "gpu_util_percent") or {}
        gm = _get(rec.resources, "gpu_mem_used_mb") or {}
        gp = _get(rec.resources, "gpu_power_w") or {}
        cpu = _get(rec.resources, "cpu_percent") or {}
        lines.append("**Resource usage**")
        lines.append("")
        lines.append(f"- GPU util: mean {_fmt(gu.get('mean'),1)}%, max {_fmt(gu.get('max'),1)}%")
        lines.append(f"- GPU mem: mean {_fmt(gm.get('mean'),0)} MB, max {_fmt(gm.get('max'),0)} MB")
        lines.append(f"- GPU power: mean {_fmt(gp.get('mean'),1)} W, max {_fmt(gp.get('max'),1)} W")
        lines.append(f"- CPU: mean {_fmt(cpu.get('mean'),1)}%")
        lines.append("")

    # Filtering
    if rec.filtering:
        lines.append("**Filtering**")
        lines.append("")
        for k, v in rec.filtering.items():
            lines.append(f"- {k}: {_fmt(v) if isinstance(v, float) else v}")
        lines.append("")

    return "\n".join(lines)


def render_comparison(records: List[ExperimentRecord]) -> str:
    """Render a comparison table across runs (e.g. baseline vs filtered)."""
    lines: List[str] = []
    lines.append("# ORB-SAM-E Experiment Comparison\n")

    # headline accuracy + fps table
    lines.append("## Summary\n")
    lines.append("| Run | Filter | ATE RMSE (m) | RPE trans (m) | RPE rot (deg) "
                 "| Filter FPS | Track FPS | GPU mem (MB) |")
    lines.append("|---|---|---|---|---|---|---|---|")
    for r in records:
        ate = _get(r.trajectory or {}, "ate", "rmse")
        rpe_t = _get(r.trajectory or {}, "rpe_trans", "rmse")
        rpe_r = _get(r.trajectory or {}, "rpe_rot_deg", "rmse")
        filt_fps = _get(r.timing, "efficientsam3_inference", "fps")
        track_fps = _get(r.timing, "orbslam_track", "fps")
        gpu_mem = _get(r.resources, "gpu_mem_used_mb", "max")
        use_filter = r.config.get("use_filter", "—")
        lines.append(
            f"| {r.name} | {use_filter} | {_fmt(ate,4)} | {_fmt(rpe_t,4)} "
            f"| {_fmt(rpe_r,3)} | {_fmt(filt_fps,1)} | {_fmt(track_fps,1)} "
            f"| {_fmt(gpu_mem,0)} |"
        )
    lines.append("")

    for r in records:
        lines.append(render_single(r))
        lines.append("\n---\n")

    return "\n".join(lines)


def write_report(records: List[ExperimentRecord], path: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write(render_comparison(records))


# ---------------------------------------------------------------------------
# CLI: combine multiple results.json into one report
# ---------------------------------------------------------------------------

def main() -> None:
    import argparse
    p = argparse.ArgumentParser(
        description="Combine experiment result JSON files into one Markdown report."
    )
    p.add_argument("results", nargs="+", help="One or more results.json files")
    p.add_argument("-o", "--output", default="report.md", help="Output Markdown path")
    args = p.parse_args()

    records = [ExperimentRecord.load_json(p) for p in args.results]
    write_report(records, args.output)
    print(f"Wrote {args.output} from {len(records)} run(s)")


if __name__ == "__main__":
    main()
