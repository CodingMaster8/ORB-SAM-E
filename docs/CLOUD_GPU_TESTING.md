# ORB-SAM-E: Cloud GPU Testing Guide

Goal: run ORB-SAM-E (ORB-SLAM3 + EfficientSAM3) on a rented cloud GPU to validate
the CUDA pipeline and measure FPS, as a stand-in for a **Jetson Orin Nano**.

You already verified the pipeline works on an Ubuntu VM with **CPU only** (low FPS).
This guide tells you exactly **which instance to rent**, **why**, and gives you a
**results table** to fill in.

---

## TL;DR — What to rent

| If your priority is...                              | Rent this                                  | Provider           | ~Price/hr | Why |
|-----------------------------------------------------|--------------------------------------------|--------------------|-----------|-----|
| **Simplest plug-and-play + cheap (start here)**     | **NVIDIA T4 (16 GB)** or **RTX 4090**      | **RunPod**         | $0.35–0.79| Docker-native templates, GPU-ready in <1 min, per-minute billing |
| Closest to Jetson's **edge / low-power class**      | **NVIDIA T4** or **L4**                    | RunPod / GCP / AWS | $0.35–0.76| Modest GPUs → FPS numbers closer to (still above) Orin Nano |
| Closest to Jetson's **CPU+GPU architecture (ARM64)**| **NVIDIA GH200 (Grace Hopper, aarch64)**   | Lambda / Vast.ai   | $1.5–3.5  | Same `aarch64` CPU + CUDA GPU combo as Jetson (but far more powerful) |
| Don't care about cost, want headroom                | **A10 / L40S / A100**                      | RunPod / Lambda    | $0.8–1.9  | Plenty of VRAM and compute |

**Recommendation: start with RunPod + NVIDIA T4.** It is the simplest plug-and-play
path, costs cents, and confirms the CUDA pipeline end-to-end. Use the results table
below to record FPS.

> ⚠️ **Reality check (important):** *No mainstream cloud rents an actual Jetson Orin
> Nano.* Every cloud GPU above is **much more powerful** than an Orin Nano, so the FPS
> you measure is an **upper bound**, not the real on-device number. Cloud tests prove
> *"the GPU pipeline works and is correct"*; the *final* Jetson FPS must be measured on
> real Jetson hardware. See [Interpreting results](#interpreting-results-vs-jetson-orin-nano).

---

## Why a cloud GPU (and which class)

The Jetson Orin Nano is:
- **CPU:** 6-core ARM Cortex-A78AE (**aarch64**)
- **GPU:** 1024-core **Ampere** GPU + 32 Tensor cores (~40–67 TOPS)
- **Memory:** 8 GB **shared** LPDDR5 (CPU and GPU share RAM)
- **OS:** JetPack (Ubuntu-based, `aarch64`)

Two things matter for ORB-SAM-E:
1. **EfficientSAM3 (PyTorch)** wants a **CUDA GPU**. This is the part that was slow on
   your Mac CPU. Any NVIDIA cloud GPU fixes this.
2. **Architecture (`aarch64`)**. The package is literally named `efficientsam3_arm`.
   x86_64 cloud GPUs run it fine (PyTorch is portable), but if you want to catch
   ARM-specific issues, use an **aarch64 GH200** instance.

Pick based on what you're trying to prove:

- **"Does the GPU path work and roughly how fast?"** → cheapest x86_64 NVIDIA GPU (T4 on RunPod).
- **"Does it build/run on ARM like the Jetson will?"** → GH200 (aarch64).
- **"What FPS will I roughly get on the edge?"** → smallest GPU (T4/L4), treat as upper bound.

---

## Option A (recommended): RunPod + NVIDIA T4 — plug and play

RunPod is the simplest because it boots a container with NVIDIA drivers + CUDA already
installed; you don't touch driver setup at all.

### 1. Create the pod
1. Sign up at [runpod.io](https://www.runpod.io) and add ~$10 credit.
2. **Deploy → GPU Pod → Community Cloud**.
3. GPU: **NVIDIA T4** (cheapest) or **RTX 4090** (faster, still cheap).
4. Template: choose a **Ubuntu 22.04 + CUDA + PyTorch** template
   (e.g. "RunPod PyTorch 2.x"). This maps to **ROS 2 Humble** in the setup guide.
   - If you specifically need **ROS 2 Jazzy**, pick an **Ubuntu 24.04** base image
     instead (search templates for `ubuntu:24.04`) and install ROS 2 Jazzy per `UBUNTU_SETUP.md`.
5. Disk: **40 GB** container + volume (ORB-SLAM3 build + weights + a TUM sequence).
6. Enable **SSH** and **HTTP port** access. Deploy.

### 2. Connect and set up
SSH in (RunPod shows the command), then follow the existing setup guide:

```bash
# Verify the GPU is visible first
nvidia-smi

# Then follow UBUNTU_SETUP.md, but install the CUDA build of PyTorch:
# (match the CUDA version reported by nvidia-smi, e.g. cu121)
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu121

python3 -c "import torch; print('CUDA available:', torch.cuda.is_available())"
# Must print: CUDA available: True
```

Then complete the rest of `UBUNTU_SETUP.md` (ROS 2, Pangolin, ORB-SLAM3, weights, build).

### 3. Run on GPU
Force the filter node onto CUDA:

```bash
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p model_path:=$HOME/weights/efficient_sam3_repvit_s.pt \
    -p device:=cuda
```

### 4. No display? (cloud boxes are headless)
ORB-SLAM3 opens a Pangolin window. On a headless cloud box, either:
- Run with a virtual display: `sudo apt install -y xvfb && xvfb-run -a <your command>`, or
- Build/run ORB-SLAM3 in a mode that skips the viewer (disable Pangolin viewer in config), or
- Use the **TUM dataset** flow from `UBUNTU_SETUP.md` (no live camera needed) and read
  FPS from the terminal / `ros2 topic hz` instead of the GUI.

### 5. Stop billing
**Terminate the pod** when done (or you keep paying). RunPod bills per minute.

---

## Option B: GH200 (aarch64) — true Jetson-like architecture

Use this only if you want to validate the **ARM64** build path (same CPU ISA as Jetson).

- **Provider:** [Lambda](https://lambda.ai) (on-demand GH200) or [Vast.ai](https://vast.ai) (marketplace, cheaper).
- **Instance:** 1× **GH200** (Grace CPU `aarch64` + Hopper GPU).
- **Note:** This is *enormously* more powerful than an Orin Nano, so it's about
  **architecture parity, not FPS parity**. PyTorch wheels for `aarch64`+CUDA exist;
  some pip packages may need source builds — budget extra setup time.
- Follow `UBUNTU_SETUP.md`; the OS is Ubuntu on `aarch64`. Use the CUDA PyTorch build.

---

## Option C: Hyperscalers (AWS / GCP / Azure)

Only if you already have an account / need them for compliance. They are 2–4× more
expensive and require more setup (drivers, security groups, etc.).

| Provider | Instance     | GPU   | Notes |
|----------|--------------|-------|-------|
| AWS      | `g4dn.xlarge`| T4    | Use the **Deep Learning AMI** (drivers preinstalled) |
| GCP      | `g2`         | L4    | Low-power Ada GPU, decent Jetson-class proxy |
| AWS      | `g5.xlarge`  | A10G  | Faster, more VRAM |

---

## Provider quick comparison (May 2026 pricing)

| Provider   | Plug-and-play | Cheapest useful GPU | ~Price/hr | aarch64 option | Best for |
|------------|---------------|---------------------|-----------|----------------|----------|
| **RunPod** | ★★★★★ (templates, <1 min) | T4 / RTX 4090 | $0.35 / $0.39 | No | **Start here** |
| Vast.ai    | ★★★☆☆ (marketplace) | RTX 4090 / T4 | $0.20–0.45 | GH200 (some hosts) | Lowest price |
| Lambda     | ★★★★☆          | A10 / GH200         | $0.75+    | **GH200**      | ARM parity, clean images |
| Paperspace | ★★★★☆ (notebooks)| RTX 4000 / A4000  | $0.51+    | No             | Notebook UX |
| GCP        | ★★★☆☆          | L4 / T4             | $0.35–0.76| No             | Edge-class L4 |
| AWS        | ★★☆☆☆ (DLAMI helps)| T4 (g4dn)        | $0.35–0.76| Graviton (no GPU)| Existing AWS users |

> Prices are approximate on-demand rates and move around; check the provider before you book.

---

## Interpreting results vs. Jetson Orin Nano

When you read your cloud FPS, adjust expectations:

- A **T4** is roughly **3–6×** the GPU throughput of an Orin Nano; a **4090/A10/A100**
  can be **10–30×+**. So cloud FPS is an **optimistic ceiling**.
- The Orin Nano is also constrained by **8 GB shared memory** and a **6-core ARM CPU** —
  ORB-SLAM3's CPU-bound tracking thread may bottleneck on Jetson in ways a beefy cloud
  CPU hides.
- **What cloud testing reliably tells you:**
  - ✅ The CUDA inference path works and produces correct masks.
  - ✅ Relative speedup of GPU vs. your Mac CPU baseline.
  - ✅ Whether the full ROS 2 pipeline runs end-to-end with `device:=cuda`.
- **What it does NOT tell you:** the real on-device Jetson FPS. Save final numbers for
  real hardware (or NVIDIA's cloud Jetson programs if available).

**Tip for a more Jetson-like proxy:** pick the **smallest** GPU (T4 or L4) rather than a
4090/A100, and run the **RepViT-S** model with `process_every_n_frames` tuning. The
absolute numbers will still be high, but the *shape* of the bottlenecks is more realistic.

---

## Gotchas we hit (not bugs — but they cost us hours)

These are the things that *looked* like the pipeline was broken but weren't. Read this
before you go down the same rabbit holes.

### The "0 detections" saga (the big one)

**Symptom:** running the EfficientSAM3 filter on a TUM `walking_xyz` frame returned
`0 detections, 0.0% coverage`, while a notebook with the *same* model and prompts showed
hundreds of detections. We chased four wrong theories before finding the real cause.

False leads (all disproven by controlled tests):

1. **"It's a CUDA bug."** Wrong. A 2×2 test (cpu vs cuda on the same image) gave
   essentially identical results (305 vs 302 detections). GPU and CPU agree.
2. **"It's image resolution (640×480 is too low)."** Wrong. Downscaling the high-res
   image to exactly 640×480 still produced **315** detections, and the `person` prompt
   works on other images even at 320×240. The processor already upscales the long side
   to 1008 internally (`_preprocess_with_padding`), so feeding it bigger frames does
   nothing.
3. **"It's a domain mismatch — the model can't do indoor TUM scenes."** Wrong. Sweeping
   several frames showed the model detects people at **0.63–0.77** confidence on most
   `walking_xyz` frames.
4. **"`filter_core` diverges from the notebook."** Wrong. `filter_core` already
   replicates the notebook (`Sam3Processor`, `preserve_aspect_ratio=True` default,
   multi-prompt body-part union, `reset_all_prompts` between prompts).

**Actual causes (two, compounding):**
- **The diagnostic scripts used a different code path than the real filter.** Early
  probes imported `Sam3ProcessorARM` (the ARM wrapper) and tested single prompts at
  `confidence_threshold=-1.0`, whereas the real `filter_core` uses `Sam3Processor`
  (`efficientsam3_arm.model.sam3_image_processor`) with the body-part union at `0.03`.
  Different processor + settings → different (misleading) numbers.
- **The single test frame we picked was an unlucky one.** Detection is **frame-dependent**:
  across the full sequence the detection rate is ~**80%**, but ~20% of frames (people
  turned away, mid-stride, motion-blurred, or out of view) genuinely yield 0 detections.
  `awk 'NR==350'` happened to land on one of those, which looked like total failure.

**Lesson:** when a result looks catastrophic, (a) test the *real* code path, not a
re-implementation, and (b) sweep multiple frames before concluding anything — one frame
is not a measurement.

### `wlak.png` is not a separate image — it's the TUM walking scene

The notebook variable named `tum_image` actually loads `wlak.png`, a **1804×1352** photo
of the *same* `fr3/walking` scene (two men at desks). It is **not** a 640×480 dataset
frame. This conflated "high resolution" with "different/easier content" and sent the
resolution investigation sideways. Confirm what a variable actually points at before
reasoning from it.

### Wrong checkpoint → silent near-zero scores

The first checkpoint we used (`efficient_sam3_repvit_s.pt`) carries **standard CLIP**
text-encoder weights, but the ARM builder loads them into a **MobileCLIP** text
transformer. The mismatch doesn't error — it just produces near-zero detection scores
for every prompt. Fix: use the matching checkpoint
`efficient_sam3_repvit-m0_9_mobileclip_s1.pth`. The filename encodes the text encoder;
`filter_core` auto-detects it via `MODEL_FILENAME_MAP`.

### Missing model assets that fail late, not at import

- **`ModuleNotFoundError: No module named 'einops'`** — a runtime dependency of
  EfficientSAM3 that isn't pulled in automatically. `pip3 install einops`.
- **`BPE vocabulary file not found: .../assets/bpe_simple_vocab_16e6.txt.gz`** — the text
  tokenizer needs this file present under `efficientsam3_arm/efficientsam3_arm/assets/`.
  Download it there once.

Both are now in `resume_setup.sh` / the runbook.

### FPS scales linearly with the number of prompts

The default filter runs **6 body-part prompts per frame**, each a separate
text-conditioned encoder + 200-query decoder pass → **1.37 FPS** on an A4000. Dropping to
`["human face", "human shirt"]` gives ~74% detection at **3.9 FPS**. This is expected, not
a regression — see `FPS_IMPROVEMENT_PLAN.md`. The image backbone is cheap (~0.04 s); the
per-prompt grounding dominates, and `Sam3Processor` runs in **fp32 with no inference
optimizations** (the sibling `Sam3ProcessorARM` exposes `use_fp16`).

### Dataset / environment friction (quick ones)

- **TUM download 404.** The correct URL has **no** `/tgz/` path segment:
  `https://cvg.cit.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_walking_xyz.tgz`.
- **`tar: Cannot change ownership ...`** on extraction — harmless on the RunPod network
  volume; files extract correctly. Add `--no-same-owner` to silence it.
- **`ros2 --version` → "unrecognized arguments".** Not a real command. Verify ROS with
  `echo $ROS_DISTRO` or `ros2 pkg list`.
- **SSH falls back to a password prompt.** The public key wasn't in the pod's
  `~/.ssh/authorized_keys`; adding it via the web terminal fixed it. Also make sure
  `-i ~/dev/personal/ros_pod` (Mac-side) commands are run **on the Mac**, not inside the
  pod SSH session.
- **`ValueError: Failed to load image`.** A stale sample path that didn't exist on the
  pod — point the test at an actual dataset frame.

### RunPod "Stop" wipes everything outside `/workspace`

Stopping a pod clears the **container disk** (`/`, the `overlay` mount): all `apt`/`pip`
installs (ROS 2, `einops`, …) and anything in `/root` are lost. Only `/workspace`
(the network volume) survives. Keep weights, datasets, and build artifacts under
`/workspace`, and use `resume_setup.sh` to rebuild the environment after a Start.

---

## Automated metrics (recommended for the paper)

Instead of eyeballing FPS from logs, use the **metrics toolkit** in `metrics/`,
which records ATE/RPE accuracy, per-stage FPS/latency, GPU/CPU/RAM usage, and a
full environment snapshot, then emits a paper-ready Markdown comparison
(baseline vs filtered). See `metrics/README.md`. Quick version on the pod:

```bash
pip3 install -r metrics/requirements.txt
cd metrics
SEQ=$HOME/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum/rgbd_dataset_freiburg3_walking_xyz
./run_benchmark.sh fr3_walk_baseline TUM3 "$SEQ" false results/fr3_walk_baseline
./run_benchmark.sh fr3_walk_filtered TUM3 "$SEQ" true  results/fr3_walk_filtered
python3 -m slam_metrics.report results/*/results.json -o results/comparison.md
```

The manual template below is a fallback / for jotting quick notes.

## Results recording template

Duplicate this block for each instance you test. Fill in real numbers.

### Baseline: Mac CPU (already done)
- **Hardware:** _e.g. Apple M_, N cores, Ubuntu VM CPU-only_
- **Model:** efficient_sam3_repvit_s.pt
- **EfficientSAM3 filter FPS:** ______
- **ORB-SLAM3 tracking FPS:** ______
- **End-to-end pipeline FPS:** ______
- **Notes:** ______

### Test 1
- **Date:** ____  **Provider / Instance:** ____  **GPU:** ____  **CPU / arch:** ____ (x86_64 / aarch64)
- **OS / ROS 2:** ____ (Ubuntu 22.04+Humble / 24.04+Jazzy)
- **PyTorch / CUDA:** ____   **`torch.cuda.is_available()`:** ____
- **Model used:** ____ (repvit_s / tinyvit_m / tinyvit_l)
- **Dataset / sequence:** ____ (e.g. TUM fr3/walking_xyz)
- **Masking strategy / `process_every_n_frames`:** ____
- **Cost:** $____/hr × ____ hr = $____

| Metric                              | Value | How measured |
|-------------------------------------|-------|--------------|
| EfficientSAM3 filter FPS (CUDA)     |       | `ros2 topic hz /camera/image_filtered` |
| EfficientSAM3 filter FPS (CPU, ref) |       | run with `device:=cpu` |
| ORB-SLAM3 tracking FPS              |       | node log / `ros2 topic hz` |
| End-to-end pipeline FPS            |       | slowest stage |
| GPU utilization (%)                |       | `nvidia-smi` while running |
| GPU memory used (MB)               |       | `nvidia-smi` |
| Speedup vs. Mac CPU baseline       |       | filter FPS ÷ baseline FPS |

- **Tracking quality / drift notes:** ______
- **Issues / crashes:** ______
- **Verdict (good enough for Jetson? next step?):** ______

### Test 2
_(copy Test 1 block)_

---

## Cost-saving checklist
- [ ] Use **community/spot** tier where available (RunPod Community, Vast.ai spot).
- [ ] **Download weights + dataset to a persistent volume** so you don't re-download each session.
- [ ] **Snapshot/template** the configured environment after first setup to skip rebuilds.
- [ ] **Terminate** (not just stop) the instance when done.
- [ ] Set a **spending limit** / billing alert.

---

## References
- Automated metrics toolkit: `metrics/README.md`
- Existing setup steps: `UBUNTU_SETUP.md`, `efficientsam3_ros2/VM_SETUP.md`
- RunPod: https://www.runpod.io  ·  Lambda: https://lambda.ai  ·  Vast.ai: https://vast.ai
- Jetson Orin Nano specs: https://developer.nvidia.com/embedded/jetson-orin
