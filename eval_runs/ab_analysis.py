#!/usr/bin/env python3
"""
A/B analysis: ORB-SLAM3 baseline vs EfficientSAM3-filtered, paired per bag.

Inputs (TUM format) per run, in --dir:
  <bag>_baseline_pose.tum   per-frame pose stream, baseline replay
  <bag>_filtered_pose.tum   per-frame pose stream, filtered replay
  <bag>_odom.tum            wheel odometry (approximate reference)

The monocular SLAM resets on heavy dynamics; each reset starts a new map with
a fresh origin, so the pose stream is segmented at time gaps (tracking lost)
and the LONGEST continuously-tracked segment is evaluated:
  - ATE RMSE after Sim(3) (Umeyama with scale) alignment against odometry
  - Tracking coverage: total tracked time / bag span, and #poses / #frames
  - Number of segments (fragmentation proxy, ~resets+1)

Wheel odometry on mecanum wheels drifts (slip), so ATE here is an
approximate reference, valid for *paired* comparison on the same bag.
"""
import argparse
import os

import numpy as np

# total /cam_1/image frames in each source bag (ros2 bag info)
BAG_FRAMES = {"dyn1": 4160, "dyn2": 4621}
BAG_SPAN = {"dyn1": 138.74, "dyn2": 154.02}
# map resets counted in launch logs ("New Map created")
RESETS = {
    ("dyn1", "baseline"): 6, ("dyn1", "filtered"): 3,
    ("dyn2", "baseline"): 12, ("dyn2", "filtered"): 10,
}


def load_tum(path):
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data  # t, x, y, z, qx, qy, qz, qw


def segment(est, max_gap=0.5):
    """Split pose stream at time gaps (tracking lost / map reset)."""
    t = est[:, 0]
    cuts = np.where(np.diff(t) > max_gap)[0] + 1
    return np.split(est, cuts)


def associate(est, ref, max_dt=0.1):
    """Nearest-timestamp association est->ref."""
    ref_t = ref[:, 0]
    idx = np.searchsorted(ref_t, est[:, 0])
    idx = np.clip(idx, 1, len(ref_t) - 1)
    left = ref_t[idx - 1]
    right = ref_t[idx]
    use_left = (est[:, 0] - left) < (right - est[:, 0])
    matched = np.where(use_left, idx - 1, idx)
    dt = np.abs(ref_t[matched] - est[:, 0])
    ok = dt < max_dt
    return est[ok, 1:4], ref[matched[ok], 1:4], ok.sum()


def umeyama(src, dst):
    """Sim(3): returns s, R, t minimizing ||dst - (s R src + t)||^2."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    sc, dc = src - mu_s, dst - mu_d
    cov = dc.T @ sc / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_s = (sc ** 2).sum() / len(src)
    s = np.trace(np.diag(D) @ S) / var_s
    t = mu_d - s * R @ mu_s
    return s, R, t


def path_length(xyz):
    return float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())


def windowed_ate(src, dst, win_len=1.0, min_pts=20):
    """Local ATE: Sim(3) per window of `win_len` meters of reference path.

    Fair across segments of different lengths: long segments are not
    penalized for accumulated monocular scale drift.
    """
    cum = np.concatenate([[0.0], np.cumsum(
        np.linalg.norm(np.diff(dst, axis=0), axis=1))])
    errs = []
    start = 0
    while start < len(dst):
        end = int(np.searchsorted(cum, cum[start] + win_len))
        if end - start >= min_pts:
            s, R, t = umeyama(src[start:end], dst[start:end])
            al = (s * (R @ src[start:end].T)).T + t
            errs.append(np.linalg.norm(al - dst[start:end], axis=1))
        if end >= len(dst):
            break
        start = end
    return np.concatenate(errs) if errs else None


def analyze(pose_path, odom_path, bag, mode,
            min_poses=50, min_ref_len=0.5):
    """Segment-wise Sim(3) ATE, pooled over all moving segments.

    Each continuously-tracked segment is aligned independently (each map
    reset gives a fresh, unrelated origin/scale). Segments where the
    reference barely moves are skipped: Sim(3) on a near-static segment is
    degenerate and yields a meaningless near-zero ATE.
    """
    if not os.path.exists(pose_path):
        return None
    est = load_tum(pose_path)
    ref = load_tum(odom_path)

    segs = segment(est)
    seg_durs = [s[-1, 0] - s[0, 0] for s in segs]
    tracked_time = sum(seg_durs)

    all_err, all_aligned, all_ref, all_werr = [], [], [], []
    used, moving_time, eval_ref_len = 0, 0.0, 0.0
    for sg in segs:
        if len(sg) < min_poses:
            continue
        src, dst, n = associate(sg, ref)
        if n < min_poses:
            continue
        ref_len = path_length(dst)
        if ref_len < min_ref_len:
            continue  # near-static: Sim(3) degenerate
        s, R, t = umeyama(src, dst)
        aligned = (s * (R @ src.T)).T + t
        all_err.append(np.linalg.norm(aligned - dst, axis=1))
        we = windowed_ate(src, dst)
        if we is not None:
            all_werr.append(we)
        all_aligned.append(aligned)
        all_ref.append(dst)
        used += 1
        moving_time += sg[-1, 0] - sg[0, 0]
        eval_ref_len += ref_len

    if not all_err:
        return {"error": "sin segmentos con movimiento evaluables"}
    err = np.concatenate(all_err)
    werr = np.concatenate(all_werr) if all_werr else err

    return {
        "poses": len(est),
        "frames_pct": 100.0 * len(est) / BAG_FRAMES[bag],
        "cobertura_pct": 100.0 * tracked_time / BAG_SPAN[bag],
        "segmentos": len(segs),
        "resets": RESETS.get((bag, mode), -1),
        "seg_mayor_s": float(max(seg_durs)),
        "segs_eval": used,
        "tiempo_eval_s": float(moving_time),
        "dist_eval_m": float(eval_ref_len),
        "ate_rmse_m": float(np.sqrt((err ** 2).mean())),
        "ate_mediana_m": float(np.median(err)),
        "ate_max_m": float(err.max()),
        "ate_local_rmse_m": float(np.sqrt((werr ** 2).mean())),
        "ate_local_med_m": float(np.median(werr)),
        "aligned": np.vstack(all_aligned),
        "ref": np.vstack(all_ref),
        "aligned_segs": [a.tolist() for a in all_aligned],
        "ref_segs": [d.tolist() for d in all_ref],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--bags", nargs="+", default=["dyn1", "dyn2"])
    ap.add_argument("--plot", default="ab_result.png")
    args = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(len(args.bags), 2, figsize=(14, 6 * len(args.bags)))
        axes = np.atleast_2d(axes)
    except ImportError:
        plt = None

    rows = []
    export = {}

    for bi, bag in enumerate(args.bags):
        odom = os.path.join(args.dir, f"{bag}_odom.tum")
        for mi, mode in enumerate(["baseline", "filtered"]):
            pose = os.path.join(args.dir, f"{bag}_{mode}_pose.tum")
            r = analyze(pose, odom, bag, mode)
            label = f"{bag}/{mode}"
            if r is None or "error" in r:
                rows.append((label, None if r is None else r["error"]))
                continue
            rows.append((label, r))
            export[label] = {
                "aligned_segs": [[p[:2] for p in s] for s in r["aligned_segs"]],
                "ref_segs": [[p[:2] for p in s] for s in r["ref_segs"]],
                "metrics": {k: v for k, v in r.items()
                            if k not in ("aligned", "ref",
                                         "aligned_segs", "ref_segs")},
            }
            if plt is None:
                continue

            ax = axes[bi, mi]
            ax.plot(r["ref"][:, 0], r["ref"][:, 1], "k--", lw=1, alpha=0.6, label="odom (ref)")
            ax.plot(r["aligned"][:, 0], r["aligned"][:, 1], lw=1.5,
                    color="tab:red" if mode == "baseline" else "tab:green",
                    label=f"SLAM {mode} (seg. mayor)")
            ax.scatter(*r["ref"][0, :2], c="g", s=50, zorder=5)
            ax.set_title(f"{bag} {mode}  ATE={r['ate_rmse_m']:.3f}m  "
                         f"resets={r['resets']}  seg={r['seg_mayor_s']:.0f}s")
            ax.axis("equal"); ax.grid(alpha=.3); ax.legend(fontsize=8)

    if plt is not None:
        plt.tight_layout()
        plt.savefig(os.path.join(args.dir, args.plot), dpi=120)

    import json
    with open(os.path.join(args.dir, "ab_results.json"), "w") as f:
        json.dump(export, f)

    keys = ["poses", "frames_pct", "cobertura_pct", "resets", "seg_mayor_s",
            "segs_eval", "dist_eval_m", "ate_rmse_m", "ate_local_rmse_m",
            "ate_local_med_m"]
    hdr = f"{'run':<16}" + "".join(f"{k:>15}" for k in keys)
    print(hdr)
    print("-" * len(hdr))
    for label, r in rows:
        if r is None:
            print(f"{label:<16}{'SIN DATOS':>15}")
        elif isinstance(r, str):
            print(f"{label:<16}{r:>15}")
        else:
            vals = "".join(
                f"{r[k]:>15.3f}" if isinstance(r[k], float) else f"{r[k]:>15}"
                for k in keys)
            print(f"{label:<16}{vals}")
    print(f"\nplot: {os.path.join(args.dir, args.plot)}")


if __name__ == "__main__":
    main()
