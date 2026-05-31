"""
Trajectory accuracy evaluation: ATE and RPE against a TUM-format groundtruth.

This is the headline metric for dynamic-object-filtering SLAM papers
(cf. DynaSLAM, DS-SLAM): you compare the Absolute Trajectory Error (ATE) of the
baseline (no filter) vs. the filtered pipeline on dynamic sequences such as
TUM fr3/walking_*.

Implementation is self-contained (numpy only) and follows the canonical TUM
RGB-D benchmark methodology:
  - timestamp association between estimate and groundtruth,
  - Umeyama (Horn) least-squares alignment (Sim3 with scale for MONOCULAR,
    SE3 without scale for stereo / RGB-D),
  - ATE = RMS of aligned translational differences,
  - RPE = relative pose error over a fixed time delta.

TUM trajectory file format (one pose per line, '#' comments allowed):
    timestamp tx ty tz qx qy qz qw

References:
  Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012.
  Umeyama, "Least-squares estimation of transformation parameters", PAMI 1991.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def read_tum_trajectory(path: str) -> Dict[float, np.ndarray]:
    """
    Read a TUM-format trajectory file.

    Returns a dict mapping timestamp -> [tx, ty, tz, qx, qy, qz, qw] (float64).
    Lines beginning with '#' and blank lines are skipped.
    """
    traj: Dict[float, np.ndarray] = {}
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 8:
                continue
            try:
                ts = float(parts[0])
                vals = np.array([float(x) for x in parts[1:8]], dtype=np.float64)
            except ValueError:
                continue
            traj[ts] = vals
    if not traj:
        raise ValueError(f"No valid TUM poses parsed from {path}")
    return traj


def associate(
    first: Dict[float, np.ndarray],
    second: Dict[float, np.ndarray],
    max_difference: float = 0.02,
    offset: float = 0.0,
) -> List[Tuple[float, float]]:
    """
    Associate two timestamp-keyed dicts by nearest timestamp (greedy, unique).

    Args:
        first, second: timestamp -> value dicts.
        max_difference: maximum allowed timestamp difference (seconds) to match.
        offset: time offset added to the second set's timestamps.

    Returns:
        Sorted list of (first_ts, second_ts) matched pairs.
    """
    first_keys = list(first.keys())
    second_keys = list(second.keys())
    potential = [
        (abs(a - (b + offset)), a, b)
        for a in first_keys
        for b in second_keys
        if abs(a - (b + offset)) < max_difference
    ]
    potential.sort()
    matched_first: set = set()
    matched_second: set = set()
    matches: List[Tuple[float, float]] = []
    for _, a, b in potential:
        if a in matched_first or b in matched_second:
            continue
        matched_first.add(a)
        matched_second.add(b)
        matches.append((a, b))
    matches.sort()
    return matches


# ---------------------------------------------------------------------------
# Alignment
# ---------------------------------------------------------------------------

def umeyama_alignment(
    src: np.ndarray, dst: np.ndarray, with_scale: bool = True
) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    Least-squares Sim3/SE3 alignment mapping src -> dst (Umeyama 1991).

    Args:
        src: (3, N) source points.
        dst: (3, N) destination points.
        with_scale: estimate scale (True for monocular Sim3, False for SE3).

    Returns:
        (R, t, s) such that  dst ~= s * R @ src + t
    """
    assert src.shape == dst.shape and src.shape[0] == 3, "expect (3, N) arrays"
    n = src.shape[1]
    mu_src = src.mean(axis=1, keepdims=True)
    mu_dst = dst.mean(axis=1, keepdims=True)
    src_c = src - mu_src
    dst_c = dst - mu_dst

    cov = dst_c @ src_c.T / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1.0
    R = U @ S @ Vt

    if with_scale:
        var_src = (src_c ** 2).sum() / n
        s = float(np.trace(np.diag(D) @ S) / var_src) if var_src > 0 else 1.0
    else:
        s = 1.0

    t = mu_dst.flatten() - s * R @ mu_src.flatten()
    return R, t, s


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

@dataclass
class ErrorStats:
    """Summary statistics for an array of per-sample errors."""
    rmse: float
    mean: float
    median: float
    std: float
    min: float
    max: float
    n: int

    @staticmethod
    def from_errors(errors: np.ndarray) -> "ErrorStats":
        errors = np.asarray(errors, dtype=np.float64)
        if errors.size == 0:
            return ErrorStats(0, 0, 0, 0, 0, 0, 0)
        return ErrorStats(
            rmse=float(np.sqrt(np.mean(errors ** 2))),
            mean=float(np.mean(errors)),
            median=float(np.median(errors)),
            std=float(np.std(errors)),
            min=float(np.min(errors)),
            max=float(np.max(errors)),
            n=int(errors.size),
        )


@dataclass
class TrajectoryEvalResult:
    ate: Optional[ErrorStats]
    rpe_trans: Optional[ErrorStats]
    rpe_rot_deg: Optional[ErrorStats]
    scale: float
    num_associations: int
    num_gt_poses: int
    num_est_poses: int
    align_with_scale: bool
    notes: str = ""

    def to_dict(self) -> dict:
        return {
            "ate": asdict(self.ate) if self.ate else None,
            "rpe_trans": asdict(self.rpe_trans) if self.rpe_trans else None,
            "rpe_rot_deg": asdict(self.rpe_rot_deg) if self.rpe_rot_deg else None,
            "scale": self.scale,
            "num_associations": self.num_associations,
            "num_gt_poses": self.num_gt_poses,
            "num_est_poses": self.num_est_poses,
            "align_with_scale": self.align_with_scale,
            "notes": self.notes,
        }


def _quat_to_rot(q: np.ndarray) -> np.ndarray:
    """Quaternion [qx, qy, qz, qw] -> 3x3 rotation matrix."""
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def evaluate(
    est_path: str,
    gt_path: str,
    align_with_scale: bool = True,
    max_difference: float = 0.02,
    rpe_delta: float = 1.0,
    compute_rpe: bool = True,
) -> TrajectoryEvalResult:
    """
    Compute ATE and RPE for an estimated trajectory vs. groundtruth.

    Args:
        est_path: estimated trajectory (TUM format), e.g. KeyFrameTrajectory.txt.
        gt_path: groundtruth trajectory (TUM format), e.g. groundtruth.txt.
        align_with_scale: True for MONOCULAR (Sim3 scale correction). Use False
            for metric (stereo / RGB-D) trajectories.
        max_difference: timestamp association tolerance in seconds.
        rpe_delta: time delta (seconds) for relative pose error.
        compute_rpe: also compute RPE (slightly more expensive).
    """
    est = read_tum_trajectory(est_path)
    gt = read_tum_trajectory(gt_path)

    matches = associate(est, gt, max_difference=max_difference)
    if len(matches) < 3:
        return TrajectoryEvalResult(
            ate=None, rpe_trans=None, rpe_rot_deg=None, scale=1.0,
            num_associations=len(matches), num_gt_poses=len(gt),
            num_est_poses=len(est), align_with_scale=align_with_scale,
            notes="Too few timestamp associations (<3) to evaluate. "
                  "Check timestamp units/overlap and max_difference.",
        )

    est_xyz = np.array([est[a][:3] for a, _ in matches]).T  # (3, N)
    gt_xyz = np.array([gt[b][:3] for _, b in matches]).T     # (3, N)

    # Align estimate -> groundtruth.
    R, t, s = umeyama_alignment(est_xyz, gt_xyz, with_scale=align_with_scale)
    est_aligned = s * (R @ est_xyz) + t.reshape(3, 1)

    # ATE: translational RMSE of aligned positions.
    ate_errors = np.linalg.norm(est_aligned - gt_xyz, axis=0)
    ate_stats = ErrorStats.from_errors(ate_errors)

    rpe_trans_stats = None
    rpe_rot_stats = None
    if compute_rpe:
        rpe_trans_stats, rpe_rot_stats = _compute_rpe(
            matches, est, gt, R, t, s, rpe_delta
        )

    return TrajectoryEvalResult(
        ate=ate_stats,
        rpe_trans=rpe_trans_stats,
        rpe_rot_deg=rpe_rot_stats,
        scale=s,
        num_associations=len(matches),
        num_gt_poses=len(gt),
        num_est_poses=len(est),
        align_with_scale=align_with_scale,
    )


def _pose_matrix(vec: np.ndarray) -> np.ndarray:
    """[tx,ty,tz,qx,qy,qz,qw] -> 4x4 homogeneous transform."""
    T = np.eye(4)
    T[:3, :3] = _quat_to_rot(vec[3:7])
    T[:3, 3] = vec[:3]
    return T


def _compute_rpe(
    matches: List[Tuple[float, float]],
    est: Dict[float, np.ndarray],
    gt: Dict[float, np.ndarray],
    R: np.ndarray,
    t: np.ndarray,
    s: float,
    delta: float,
) -> Tuple[ErrorStats, ErrorStats]:
    """
    Relative Pose Error over time delta `delta` (seconds).

    Estimated poses are first put into the groundtruth frame using the Sim3
    alignment (R, t, s). A similarity transform scales positions but keeps
    orientations proper rotations, so we build aligned poses as
        R_aligned = R @ R_est,   t_aligned = s * R @ t_est + t
    (multiplying the whole 4x4 by [[sR, t],[0,1]] would scale the rotation
    block and corrupt RPE).
    """
    def _align_pose(vec: np.ndarray) -> np.ndarray:
        T = _pose_matrix(vec)
        out = np.eye(4)
        out[:3, :3] = R @ T[:3, :3]
        out[:3, 3] = s * (R @ T[:3, 3]) + t
        return out

    est_ts = [a for a, _ in matches]
    est_T = {a: _align_pose(est[a]) for a, _ in matches}
    gt_T = {a: _pose_matrix(gt[b]) for a, b in matches}

    trans_errors: List[float] = []
    rot_errors_deg: List[float] = []

    for i, a in enumerate(est_ts):
        # find j such that est_ts[j] - a is closest to delta
        target = a + delta
        j = None
        best = float("inf")
        for k in range(i + 1, len(est_ts)):
            d = abs(est_ts[k] - target)
            if d < best:
                best = d
                j = k
            if est_ts[k] > target + delta:
                break
        if j is None:
            continue
        b = est_ts[j]

        rel_est = np.linalg.inv(est_T[a]) @ est_T[b]
        rel_gt = np.linalg.inv(gt_T[a]) @ gt_T[b]
        rel_err = np.linalg.inv(rel_gt) @ rel_est

        trans_errors.append(float(np.linalg.norm(rel_err[:3, 3])))
        cos = (np.trace(rel_err[:3, :3]) - 1.0) / 2.0
        cos = max(-1.0, min(1.0, cos))
        rot_errors_deg.append(float(np.degrees(np.arccos(cos))))

    return (
        ErrorStats.from_errors(np.array(trans_errors)),
        ErrorStats.from_errors(np.array(rot_errors_deg)),
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _print_stats(name: str, stats: Optional[ErrorStats], unit: str) -> None:
    if stats is None:
        print(f"  {name}: n/a")
        return
    print(
        f"  {name} (RMSE={stats.rmse:.4f}{unit}, mean={stats.mean:.4f}, "
        f"median={stats.median:.4f}, std={stats.std:.4f}, "
        f"min={stats.min:.4f}, max={stats.max:.4f}, n={stats.n})"
    )


def main() -> None:
    p = argparse.ArgumentParser(
        description="Evaluate ATE/RPE of an estimated trajectory vs TUM groundtruth."
    )
    p.add_argument("estimate", help="Estimated trajectory (TUM format)")
    p.add_argument("groundtruth", help="Groundtruth trajectory (TUM format)")
    p.add_argument("--no-scale", action="store_true",
                   help="Disable scale alignment (use for stereo/RGB-D; default ON for monocular)")
    p.add_argument("--max-diff", type=float, default=0.02,
                   help="Timestamp association tolerance in seconds (default 0.02)")
    p.add_argument("--rpe-delta", type=float, default=1.0,
                   help="RPE time delta in seconds (default 1.0)")
    p.add_argument("--json", type=str, default=None, help="Write result JSON to this path")
    args = p.parse_args()

    result = evaluate(
        est_path=args.estimate,
        gt_path=args.groundtruth,
        align_with_scale=not args.no_scale,
        max_difference=args.max_diff,
        rpe_delta=args.rpe_delta,
    )

    print(f"Associations: {result.num_associations} "
          f"(est={result.num_est_poses}, gt={result.num_gt_poses})")
    print(f"Alignment scale: {result.scale:.6f} "
          f"(with_scale={result.align_with_scale})")
    _print_stats("ATE", result.ate, " m")
    _print_stats("RPE trans", result.rpe_trans, " m")
    _print_stats("RPE rot ", result.rpe_rot_deg, " deg")
    if result.notes:
        print(f"  NOTE: {result.notes}")

    if args.json:
        import json
        with open(args.json, "w") as f:
            json.dump(result.to_dict(), f, indent=2)
        print(f"Wrote {args.json}")


if __name__ == "__main__":
    main()
