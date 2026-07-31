# Competitor Comparison Plan — Metrics, Datasets & Reproduction Strategy

*July 30, 2026 · Distilled from `ORB-SAM-E_Literature_Review.md` (§5–8) and
`ROUND2_READINESS_FINDINGS.md` (§3). Defines exactly what numbers the paper
needs, on which sequences, against whom, and which competitors we can re-run
ourselves vs. cite.*

---

## 1. Metrics we must report

| Metric | Definition / protocol | Why | Tooling |
|---|---|---|---|
| **ATE RMSE** (m) | Absolute Trajectory Error after Sim(3) (mono) / SE(3) (RGB-D) alignment, whole trajectory. Report RMSE + SD; median-of-≥5 runs ± IQR (ORB-SLAM3 is non-deterministic). | The headline number in every dynamic-SLAM paper. | `evo` / existing `metrics/` + `ab_analysis.py` |
| **RPE** trans (m) & rot (deg) | Relative Pose Error over fixed intervals, RMSE + SD. | Standard companion to ATE; shows local drift. | `evo` |
| **Improvement %** vs. ORB-SLAM3 baseline | Per-sequence ATE reduction. | Conventional reporting format (e.g., "97.6% on walking_xyz"). | derived |
| **Resets / tracking coverage** | Number of tracking losses + % of sequence with valid pose. | Round 1's clearest win (6→3 resets); first-class metric for mono-with-resets honesty. | `ab_analysis.py` |
| **Tracking time per frame** (ms) / SLAM FPS | On identical hardware, per config. | Real-time claim; where prior SAM-SLAM papers are weakest. | `metrics/` toolkit |
| **Filter latency** (ms) & filter FPS | Segmentation-module latency reported **separately** from SLAM FPS. | Transparency differentiator vs. papers that hide segmentation cost. | `filter_core.get_stats()` |
| **Detection rate & mask coverage** (%) | % of inferred frames with detection; mean masked-pixel ratio. | Explains *why* ATE moves; supports threshold ablation. | filter metrics JSON |
| Mask IoU (optional) | Vs. TUM/Bonn annotations where available. | Secondary; only if we claim mask quality. | VOS tooling |

**Hardware note:** accuracy metrics (ATE/RPE) are hardware-independent → cloud pod OK.
All runtime numbers in the paper must come from **one** hardware config per table
(Orin @ 15 W for the edge story; pod GPU may be reported as a separate column, clearly labeled).

## 2. Datasets & sequences

| Dataset | Sequences | Purpose | Status |
|---|---|---|---|
| **TUM RGB-D** fr3 | `walking_xyz`, `walking_halfsphere`, `walking_rpy`, `walking_static` | High-dynamic core table — every competitor reports these | Downloader exists (`download_tum_dataset.sh`) |
| **TUM RGB-D** fr3 | `sitting_xyz`, `sitting_halfsphere`, `sitting_rpy`, `sitting_static` | Low-dynamic **no-regression** proof (rarely reported; reviewers ask) | same |
| **Bonn RGB-D Dynamic** | `crowd`, `crowd2`, `crowd3`, `person_tracking`, `person_tracking2`, `moving_nonobstructing_box`, `moving_obstructing_box`, `balloon`, `balloon2` (subset OK) | Unlabeled movers → **open-vocabulary prompt experiment** (`box`, `balloon`) — the differentiator vs. closed-set YOLO pipelines | Need download script |
| **Robot (JetAuto/Orin)** | ≥2 dynamic scenarios × ≥5 replays, OptiTrack GT | Edge-deployment + real-robot claim | Blocked on power fix + GT run |
| KITTI (optional, P3) | 1–2 seqs | Outdoor generality claim only if time allows | not planned for round 2 |

**Modality decision:** published baselines are **RGB-D**. Round 2 runs both modes:
RGB-D for the literature-comparable table (issue #16 / PR pending), monocular for
consistency with the robot experiments. Never mix modes in one table.

## 3. Competitors: published numbers & code availability

Reference points on `fr3/walking_xyz` (ATE RMSE, from the literature review §8 —
re-verify exact numbers from each paper when building the table):

| System | Base | Segmentation | ~ATE walking_xyz | Code public? | Our strategy |
|---|---|---|---|---|---|
| ORB-SLAM3 (baseline) | — | none | ~0.75 (ORB-SLAM2 figure; measure ours) | ✅ in-repo | **Run ourselves** (filter-off A/B) — mandatory |
| **DynaSLAM** | ORB-SLAM2 | Mask R-CNN + MVG | ~0.015 | ✅ [BertaBescos/DynaSLAM](https://github.com/BertaBescos/DynaSLAM) | **Run if budget allows** — Python 2.7 + TF1/Keras era; third-party [Docker](https://github.com/git-gfischer/DynaSLAM_Docker) (~15 GB, nvidia-docker) reduces pain. Else cite. |
| **DS-SLAM** | ORB-SLAM2 | SegNet (Caffe) + ROS1 | ~0.025 | ✅ [ivipsourcecode/DS-SLAM](https://github.com/ivipsourcecode/DS-SLAM) | **Cite** — Caffe/ROS1 stack too costly to resurrect for marginal gain. |
| **Panoptic-SLAM** | ORB-SLAM3 | Panoptic seg | (see paper) | ✅ [iit-DLSLab/Panoptic-SLAM](https://github.com/iit-DLSLab/Panoptic-SLAM) + Dockerfile | **Run ourselves** — most modern public competitor; same ORB-SLAM3 base makes it the fairest head-to-head. |
| **YSAM-SLAM** (closest rival) | ORB-SLAM3 | pruned YOLOv11 → SAM2 | ≥65.78% APE reduction claim | ❌ none found | **Cite** — differentiate on open-vocab + edge; note desktop GPU in their setup. |
| YOLO11-seg + ORB-SLAM3 ([Sensors 2026](https://www.mdpi.com/1424-8220/26/5/1487)) | ORB-SLAM3 | YOLO11-seg | −93% vs. baseline | ⚠️ lit review says "code released"; repo not surfaced in search — check paper's Data Availability section | **Run if repo found**, else cite. |
| SamSLAM / DZ-SLAM / DN-SLAM | ORB-SLAM3 | SAM1/FastSAM (+flow/NeRF) | (per papers) | ❌ none found | **Cite.** |

**Recommended reproduction set for the pod:** ORB-SLAM3 baseline (ours, mandatory) +
**Panoptic-SLAM** (docker, fair ORB-SLAM3-based head-to-head) + **DynaSLAM**
(docker, the canonical baseline — attempt once; if the 2018 stack fights back,
cite and note it). Everything else: published numbers with a clear "as reported
in [cite]" footnote — standard practice.

**Fairness rules when re-running competitors:** same sequences, same evaluation
tool (`evo`), same ≥5-run median protocol, same machine; report *their* numbers
from *our* runs alongside their published numbers (both columns) so reviewers
see any reproduction gap.

## 4. The paper's tables (target artifacts)

1. **T1 — TUM high/low-dynamic ATE/RPE** (RGB-D): ours (filter on/off) vs. baseline + all competitors (cited or re-run). Per-sequence, median ± IQR.
2. **T2 — Bonn open-vocab**: `person`-only vs. `person+box+balloon` prompts vs. baseline; the sequences closed-set methods fail on.
3. **T3 — Runtime**: SLAM FPS / filter latency / detection rate, per platform (Orin 15 W; pod GPU as upper bound), vs. competitors' reported runtimes + their hardware.
4. **T4 — Ablations**: threshold/hysteresis sweep, internal-resolution sweep, dilation/propagation variants, (later) motion-verification on/off.
5. **T5 — Robot GT**: ATE vs. OptiTrack, ≥2 scenarios × ≥5 replays, resets/coverage.

## 5. Open items this plan depends on

- [ ] RGB-D input mode (issue #16, PR in progress)
- [ ] Runtime glue merged so pipeline runs from fresh clone (PR #21 interim / issue #1 authoritative)
- [ ] Bonn download + driver support (issue #14)
- [ ] `evo` integrated into `metrics/` for standard ATE/RPE alongside `ab_analysis.py` custom metrics
- [ ] Verify Sensors-2026 YOLO11 paper's code availability (Data Availability section)
- [ ] Decide DynaSLAM reproduction budget (timebox: 1 pod-day; fallback = cite)
