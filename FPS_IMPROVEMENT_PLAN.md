# EfficientSAM3 Filter — FPS Improvement Plan

Goal: raise the throughput of the `DynamicObjectFilter` (EfficientSAM3 + MobileCLIP-S1)
so the ORB-SAM-E pipeline is viable on a Jetson Orin Nano. This document records the
measured numbers, the cost model derived from the code, the **bottleneck we were
missing**, and a prioritized list of fixes.

## 1. Measured baseline (RTX A4000, `repvit-m0_9_mobileclip_s1`, TUM `fr3/walking_xyz`)

| Prompts | Detection rate | Avg coverage | FPS |
|---|---|---|---|
| 1 (`human face`) | 73.3% | 3.9% | 6.52 |
| 2 (`human face`, `human shirt`) | 74.4% | 12.0% | 3.90 |
| 6 (default body-part set) | 80.2% | 18.0% | 1.37 |

FPS scales almost perfectly inversely with the number of prompts.

## 2. Cost model (from the FPS vs. prompt-count data)

Fit `total_time = base + k · n_prompts`:

- `base ≈ 0.038 s` → the image backbone (`set_image`, RepViT-m0_9) runs **once per frame**
  and is cheap (~26 fps on its own).
- `k ≈ 0.115 s per prompt` → **each prompt** pays for a text-conditioned encoder pass
  **plus** the 200-query decoder.

So the per-frame cost is dominated by the **per-prompt** work, not the image backbone.

### Why each prompt is expensive (code path)
For every prompt, `Sam3Processor.set_text_prompt()`
(`efficientsam3_arm/model/sam3_image_processor.py`) does:

1. `backbone.forward_text([prompt])` — re-encodes the text **every frame**, even though
   the prompts never change.
2. `_forward_grounding()` → `model.forward_grounding()` (`model/sam3_image.py`), which
   re-runs the **text-conditioned fusion encoder** and the **6-layer / 200-query
   deformable decoder** (`model/decoder.py`, `num_queries=200`).

These are executed in a **sequential Python loop** in `filter_core.process_frame()`
(once per prompt), so 6 prompts = 6 independent encoder+decoder forwards at batch size 1.

## 3. The bottleneck we were missing

**`filter_core` uses `Sam3Processor`, which runs the model in pure FP32 with no inference
optimizations.** Evidence in the repo:

- `Sam3Processor` (`model/sam3_image_processor.py`) never casts the model or inputs —
  it builds the model and runs it as-is (FP32).
- The sibling `Sam3ProcessorARM` (`processor_arm.py`) *does* expose
  `use_fp16=True` and `optimize_for_inference=True` (lines 48-65, 148-149) — but we are
  not on that path.
- `torch.compile` hooks exist in the encoder/decoder/videt/maskformer modules
  (`model/encoder.py`, `model/decoder.py`, etc.) but are **disabled** for this path.

So the model is leaving the two biggest GPU levers on the table: **half precision** and
**batching the prompts**. On top of that, **text features are recomputed every frame**
for static prompts.

## 4. Prioritized fixes

Ordered by (expected gain ÷ effort). Always re-check `detection_rate` after each change —
speed is worthless if masks regress.

### A. Reduce the prompt set (zero code, already measured) — ~3× 
Switch the default from 6 prompts to **`["human face", "human shirt"]`**:
74.4% detection at **3.90 fps** vs. 80.2% at 1.37 fps. Best speed/recall trade-off found
so far. Tune the threshold per prompt if needed.

### B. Half precision (bf16/fp16) — expected ~1.5-2×, low effort
Run grounding under autocast. Lowest-risk version keeps `Sam3Processor` and wraps the
hot calls:

```python
import torch
with torch.autocast(device_type="cuda", dtype=torch.bfloat16):
    state = processor.set_image(pil_image)
    for prompt in prompts:
        processor.reset_all_prompts(state)
        state = processor.set_text_prompt(state=state, prompt=prompt)
```

bf16 is numerically safer than fp16 on Ampere (A4000 / Orin both support it). Validate
that detection scores stay above threshold; if so, push the cast into `filter_core`.

### C. Cache static text embeddings — expected to remove the text-encode share, low effort
The prompts are fixed for the whole run, so `forward_text` should run **once at startup**,
not every frame. Encode all prompts once, store `language_features`/`language_mask`, and
have `process_frame` reuse them (skip step 1 of `set_text_prompt`). Measure how much of
the 0.115 s/prompt is text vs. decoder first (see §5) to size this win.

### D. Batch all prompts into one forward — expected large, higher effort
Instead of looping 6 prompts at batch size 1, tile the cached image features to
batch size = `n_prompts` and run a **single** encoder+decoder pass with
`find_input.text_ids = [0..n-1]`. `_encode_prompt` already treats the text-feature batch
dim as "number of prompts" (`model/sam3_image.py` lines 178-182), so the architecture is
amenable; the work is in adding a batched entry point to the processor. This is the
structural fix that makes multi-prompt nearly as cheap as single-prompt.

### E. `torch.compile` the encoder/decoder — expected ~1.2-1.5× after warmup, medium effort/risk
Enable the existing compile hooks (constant input shapes here, since every frame is
padded to 1008², which suits `torch.compile`). Budget for a one-time warmup compile and
verify numerics.

### F. Decouple filter rate from camera rate — design-level, large effective gain
People move slowly relative to 30 fps. Run the filter every N frames and reuse/propagate
the last mask in between. At N=3 the *effective* masking FPS triples with negligible
accuracy loss on `walking_xyz`. Cleanest place is `dynamic_filter_node.py`.

### Not worth it
- 200 decoder queries is baked into the checkpoint; reducing it needs retraining.
- Upscaling frames does nothing — the processor already resizes the long side to 1008.

## 5. Profile first to confirm the split

Before investing in C/D, measure how `set_image` vs. `set_text_prompt` actually divide the
per-frame time on the pod:

```python
import time, torch, glob, sys
from PIL import Image
sys.path.insert(0, "<efficientsam3_arm path>")
from efficientsam3_arm import build_efficientsam3_image_model
from efficientsam3_arm.model.sam3_image_processor import Sam3Processor

m = build_efficientsam3_image_model(checkpoint_path="<ckpt>", backbone_type="repvit",
    model_name="m0_9", text_encoder_type="MobileCLIP-S1", device="cuda",
    enable_inst_interactivity=False)
p = Sam3Processor(model=m, device="cuda", confidence_threshold=0.03)
img = Image.open("<a walking_xyz frame>").convert("RGB")
prompts = ["human face", "human shirt", "human head", "human hands", "humans pants", "human leg and hands"]

def sync(): torch.cuda.synchronize()
# warmup
st = p.set_image(img); p.set_text_prompt(state=st, prompt="person")

sync(); t=time.perf_counter(); st=p.set_image(img); sync()
print("set_image:", (time.perf_counter()-t)*1e3, "ms")
for pr in prompts:
    p.reset_all_prompts(st)
    sync(); t=time.perf_counter(); st=p.set_text_prompt(state=st, prompt=pr); sync()
    print(f"  set_text_prompt[{pr}]:", (time.perf_counter()-t)*1e3, "ms")
```

If `set_text_prompt` is dominated by the decoder (likely), prioritize **D** (batching) and
**B** (bf16). If text encoding is a big share, **C** (caching) jumps up the list.

## 6. Recommended path

1. Ship **A** (`face`+`shirt`) now → 3.9 fps, used for the benchmark.
2. Add **B** (bf16 autocast) → expect ~6-8 fps; validate detection rate.
3. Run **§5** profiling; if decoder-bound, implement **D** (prompt batching) for the
   headline number.
4. Use **F** (every-N-frames) in the ROS node for the live/real-time demo.

## 7. Reporting for the paper

Report three separate numbers so the edge-deployment cost is transparent:
- ORB-SLAM3 tracking FPS (GPU SLAM throughput),
- EfficientSAM3 filter FPS (the bottleneck, with the chosen prompt set + precision),
- detection_rate (80.2% @ 6 prompts / 74.4% @ 2 prompts) and avg mask coverage.
