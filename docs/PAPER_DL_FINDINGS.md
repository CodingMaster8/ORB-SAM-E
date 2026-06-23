# ORB-SAM-E — Hallazgos de Deep Learning en hardware real (Jetson Orin Nano)

Notas técnicas de la parte de deep learning (EfficientSAM3 como filtro de objetos
dinámicos) medidas **en el robot real**, para construir la sección experimental del
paper. Complementa `CLOUD_GPU_TESTING.md` (mediciones en cloud GPU) y
`ROBOT_INTEGRATION_TODO.md` (bitácora de integración).

Fecha de las mediciones: junio 2026.

---

## 1. Plataforma

| Componente | Detalle |
|---|---|
| Compute | **Jetson Orin Nano Developer Kit 8 GB** (tegra234, aarch64) |
| GPU | 1024-core Ampere + 32 Tensor Cores, **memoria LPDDR5 compartida** (~68 GB/s) |
| Power mode | **15 W = máximo** en esta config (L4T R36.5 solo expone 7W/15W, no hay MAXN) |
| Software | JetPack/L4T R36.5, CUDA 12.6, Python 3.10, ROS 2 Humble |
| PyTorch | **2.11.0 + torchvision 0.26.0** (wheels jetson-ai-lab, índice `jp6/cu126`), CUDA OK |
| Cámara | Orbbec Astra Pro Plus, RGB 640×480 @ 30 fps (`/cam_1/image`) |

### 1.1 Entorno reproducible (Orin)

- venv `~/venvs/esam3` (`--system-site-packages` para convivir con rclpy/Humble);
  numpy 1.26.4 **dentro del venv** (el sistema mantiene 1.21.5 pineado por ROS).
- Deps de sistema que requirió el wheel de torch 2.x para Jetson (fallan en import si
  faltan, no en pip install): `cusparselt-cuda-12`, `cuda-cupti-12-6` (+`ldconfig`),
  **cuDSS 0.6** (`libcudss0-cuda-12` + `/etc/ld.so.conf.d/cudss.conf`).
- El `requires-python >= 3.12` de `efficientsam3_arm/pyproject.toml` es solo metadata:
  los 53 .py parsean y corren en 3.10 (verificado con `ast.parse(feature_version=(3,10))`).
  No se instala el paquete (`pip -e`); se inyecta por `sys.path` (param `efficientsam3_path`).
- Scripts en el Orin: `~/setup_esam3.sh` (instalación completa), `~/test_esam3.sh`
  (import test + 1 frame), pesos en `~/weights/`.

---

## 2. Hallazgo crítico #1 — El checkpoint determina si el text encoder funciona (fallo silencioso)

**Síntoma:** con `efficient_sam3_repvit_s.pt` (stage1, el que sugiere `VM_SETUP.md`) las
detecciones salen con confianza 0.03–0.06 y la máscara cubre ~34% del frame con basura
(botellas, techo) — **sin ningún error**.

**Causa:** ese checkpoint trae los pesos del text encoder **CLIP estándar** (keys
`encoder.transformer.resblocks.*`), pero el builder ARM construye un student
**MobileCLIP** (`encoder.transformer.{i}.pre_norm_mha.*`). `load_state_dict(strict=False)`
deja el text encoder **aleatorio** y los scores quedan en ruido cerca de cero.
El indicador es el dump de `missing_keys=backbone.language_backbone.*` al cargar.

**Regla práctica:** el nombre del archivo codifica el text encoder. Solo usar
checkpoints cuyo nombre incluya el student (`*_mobileclip_s1.pth`, `*_mobileclip_s0_ctx16.pt`);
`filter_core.MODEL_FILENAME_MAP` los autodetecta. Mismo gotcha documentado de forma
independiente en `CLOUD_GPU_TESTING.md` §"Wrong checkpoint" — lo reproducimos en el robot.

---

## 3. Hallazgo crítico #2 — Dónde se va el tiempo (anatomía del costo)

- El backbone de imagen distilado (RepViT-M0.9, 4.7M params) es **barato** (~40 ms clase
  A4000). Lo caro es la **cabeza de grounding tipo DETR**: encoder de fusión + decoder de
  200 queries sobre ~5184 tokens de imagen (resize interno fijo a **1008×1008**, aunque la
  cámara dé 640×480), ejecutada **una vez por prompt**.
- Consecuencia: **la latencia escala ~lineal con el número de prompts**. Confirmado en
  A4000 (`CLOUD_GPU_TESTING.md`) y en el Orin (tabla §5).
- **ctx16 casi no acelera por sí solo** (medido: 6 prompts ctx32 5.47 s vs 6 prompts
  ctx16 6.47 s, ruido térmico incluido): el costo está dominado por los tokens de
  imagen, no por los 16-32 tokens de texto. El valor de los modelos 3.1 está en la
  **calidad** del text encoder (ver §4), no en el contexto corto.
- `Sam3Processor` corre fp32 sin optimizaciones; fp16 es responsabilidad del caller
  (implementado vía autocast en `filter_core`, ver §5).

---

## 4. Hallazgo crítico #3 — Un text encoder bien alineado vale más que cualquier otro tuning

El motivo original de usar 5-6 prompts de partes del cuerpo (`human face`,
`human shirt`, ...) era que el student MobileCLIP-S1 de stage1 (cos-sim 0.854 vs teacher)
responde mal a nombres genéricos como `person`.

Con **EfficientSAM3.1** (`stage1_sam3p1/efficient_sam3p1_repvit_s_mobileclip_s0_ctx16.pt`,
522 MB, text encoder MobileCLIP-S0 de 4 capas bien alineado — línea SA-Co ft, cos-sim ~0.94):

- `person` **funciona directamente**: confianzas 0.68–0.78 en frame real del robot
  (vs 0.47 máx del S1 con union de 6 prompts).
- La máscara con **1 prompt** es más limpia que con 6: cubre exactamente las personas
  (15.6% de píxeles) sin basura.
- El threshold de confianza debe subirse de 0.03 (calibrado para el S1 débil) a **0.3**.
- Bonus de memoria: checkpoint 522 MB vs 2.35 GB (relevante con 8 GB compartidos).

**Implicación para el paper:** la calidad del student de texto es la palanca dominante
del trade-off precisión/latencia, porque controla cuántas pasadas de grounding se
necesitan por frame.

---

## 5. Escalera de performance medida (Orin Nano, warm, mismo frame 640×480)

| # | Config | Latencia mediana | FPS | Calidad |
|---|---|---|---|---|
| 0 | S1 stage1, 6 prompts, fp32 (baseline paper) | 5 469 ms | 0.18 | conf máx 0.47, máscara con spill |
| 1 | sam3p1, 6 prompts, fp32 | 6 471 ms | 0.15 | conf máx 0.78 |
| 2 | sam3p1, 2 prompts, fp32 | 2 357 ms | 0.42 | OK |
| 3 | sam3p1, 1 prompt `person` thr 0.3, fp32 | 1 134 ms | 0.88 | 5 dets (0.73/0.70/0.68), máscara limpia 15.6% |
| 4 | **sam3p1, 1 prompt, fp16 autocast** | **604 ms** | **1.66** | **idéntica a fp32** (mismas confs y máscara, bit a bit en stats) |

- **Total: 9.1× speedup vs baseline con mejor calidad de máscara.**
- fp16 autocast: **1.87×** (604 vs 1127 ms), degradación de calidad **cero** medible
  (mismos 5 dets, mismas confianzas a 2 decimales, mismo mask_ratio 0.156).
  Implementado en `filter_core.DynamicObjectFilter(use_fp16=True)` / CLI `--fp16`.
- Con `process_every_n_frames: 2-3` (los objetos dinámicos no cambian en ~100 ms), la
  cadencia efectiva del filtro es ~0.6 s por frame filtrado mientras ORB-SLAM3 trackea
  a frame-rate completo — suficiente para los runs del paper.

### Reproducir

```bash
# En el Orin, venv esam3:
python filter_core.py --image ~/frame.jpg \
  --model ~/weights/efficient_sam3p1_repvit_s_mobileclip_s0_ctx16.pt \
  --efficientsam3-path ~/ros2_ws/src/ORB-SAM-E/efficientsam3_arm \
  --prompts person --threshold 0.3 --fp16 --no-show --output out.jpg
```

---

## 6. Cambios de código (mínimos, backward-compatible)

| Archivo | Cambio | Motivo |
|---|---|---|
| `efficientsam3_arm/model_builder_arm.py` | Params `text_encoder_context_length` (default 32) y `text_encoder_pos_embed_table_size` (default None → tabla legacy de 77) | Cargar checkpoints 3.1 de contexto fijo (tabla posicional 16) |
| ídem, `_load_checkpoint` | Remap `interactive_convs.` → `sam2_convs.` | SAM3.1 usa nombres de la era TriNeck |
| `efficientsam3_ros2/filter_core.py` | `MODEL_FILENAME_MAP` a 5-tupla (+ctx,+tabla), entradas sam3p1 primero | Autodetección del nuevo checkpoint |
| ídem | `use_fp16` (autocast CUDA) + CLI `--fp16/--prompts/--threshold` | Palanca fp16 en el code path real |

Los defaults preservan el comportamiento legacy al 100% (re-verificado corriendo el
checkpoint S1 viejo tras los cambios).

### Detalle conocido e inofensivo

El ckpt 3.1 deja 4 `missing_keys` (`backbone.vision_backbone.convs.3.*`): es el nivel
0.5× del neck DualNeck, que `SAM3VLBackbone(scalp=1)` **siempre descarta**. Queda con
pesos aleatorios pero su salida nunca se usa. (Optimización futura: quitar ese nivel de
`scale_factors` y ahorrarse su cómputo.)

---

## 7. Protocolo de medición (lecciones que afectan al paper)

1. **Medir siempre el code path real** (`filter_core`), no notebooks ni scripts de
   diagnóstico: procesador/threshold/prompts distintos dan números engañosos
   (lección de `CLOUD_GPU_TESTING.md`, confirmada).
2. **Warm-up obligatorio**: el primer frame en CUDA cuesta ~9-10 s (compilación de
   kernels); con fp16 autocast el segundo frame también compila variantes. Reportar
   mediana de ≥8 frames calientes con `torch.cuda.synchronize()` antes/después.
3. **La detección es dependiente del frame** (~80% de frames con detección en TUM
   walking_xyz): nunca concluir de 1 frame; barrer secuencias.
4. Los números cloud (A4000 etc.) son **cota superior** del Orin (~7-8× más lento
   medido); el número del paper debe salir del robot.
5. GPU compartiendo RAM con el SLAM: monitorear con `monitor_cli` durante los runs
   combinados (pendiente el run integrado filtro+SLAM).

---

## 8. Config recomendada para los experimentos del robot

```yaml
checkpoint: efficient_sam3p1_repvit_s_mobileclip_s0_ctx16.pt   # 522 MB
dynamic_prompts: ["person"]
confidence_threshold: 0.3
use_fp16: true
device: cuda
masking_strategy: grayout
process_every_n_frames: 2   # a ajustar con el run integrado
```

Latencia esperada del filtro: **~0.6 s/frame** (1.66 FPS) standalone; por validar bajo
carga conjunta con ORB-SLAM3.

## 9. Pendientes de la parte DL

- [x] **Wiring integrado validado en vivo** (robot quieto): cámara → `dynamic_filter_node`
  (venv, fp16, `person`, every_n=2) → `/camera/image_filtered` → `mono_node_cpp`.
  Inferencia en vivo **594 ms mediana** (= benchmark standalone; el costo ROS es
  despreciable), ~3.3 dets/frame, salida filtrada a ~3.2 Hz.
- [ ] ⚠️ La inferencia bloquea el callback → solo ~3.2 fps llegan al SLAM. Mejora:
  worker thread de inferencia + passthrough de todos los frames con la última máscara
  (SLAM a ~30 fps, máscara a 1.7 Hz). Hacerlo antes de los runs A/B del paper.
- [ ] Run de eval con movimiento: FPS de las 3 etapas (`topic_fps.py`) y RAM/GPU
  (`monitor_cli`) bajo carga conjunta filtro+SLAM.
- [ ] Barrido multi-frame en secuencia real del robot (tasa de detección, no 1 frame).
- [ ] A/B para el paper: mismo bag, baseline vs filtrado → ATE/RPE pareado.
- [ ] (Plan C de velocidad) export TensorRT — upstream aún no publica export ONNX.
