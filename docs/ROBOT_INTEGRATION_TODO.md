# ORB-SAM-E → JetAuto (robot real) — TODO de integración

**Objetivo:** correr **ORB-SLAM3 monocular** en vivo en el **Orin**, alimentado por la cámara
real (`/cam_1/image`), publicar/grabar su trayectoria y **compararla contra un ground truth**
(OptiTrack o slam_toolbox LiDAR). **Modo experimento/eval** — NO reemplaza TF/EKF/Nav2.

**Decisiones tomadas:**
- Sin EfficientSAM3 al inicio (se añade después; es el cuello de botella).
- Monocular **puro** (escala arbitraria → se alinea con Sim3 en la evaluación).
- Frames propios (`orb_map`/`orb_cam`) → no pelea con `map`/`odom`/Nav2.

**Camino crítico para el primer run en vivo:** Fase 1 (build) → 2 (calib) → 3 (driver vivo)
→ 6 (launch). Las Fases 4–5 (pose + eval) se pueden paralelizar.

---

## Fase 0 — Workspace y despliegue  ✅ (parcial)
- [x] Layout en el Orin: clonado en `~/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/` (coincide con
  la ruta hardcodeada del nodo C++, así que NO hubo que editar rutas). Clonado vía
  `git clone --depth 1 --sparse` desde GitHub por la red del Orin (el enlace laptop↔Orin
  es inestable; el grueso debe bajarse por internet del Orin, no por rsync de la laptop).
- [ ] Añadir ORB-SAM-E al stack del Orin como **submódulo** (patrón `client_ws`) o ws dedicado
  (pendiente formalizar; por ahora es un clone directo en el Orin).
- [ ] Extender `deploy_agv_orin.sh` para empujar + buildear este workspace en el Orin.
- [ ] (Driver Python aún tiene ruta hardcodeada `~/ros2_test/...` → se aborda en Fase 3.)

## Fase 1 — Build de ORB-SLAM3 en el Orin (aarch64)  ✅ HECHO
- [x] Deps presentes: cmake 3.22, gcc 11.4, OpenCV 4.8, Eigen 3.4. **Pangolin v0.8**
  compilado e instalado desde fuente (`/usr/local/lib/libpango_*`).
- [x] Thirdparty compilados en aarch64 sin cambios de flags: `libDBoW2.so`, `libg2o.so`
  (g2o sin flags x86; DBoW2 `-march=native` = ARM). Sophus header-only.
- [x] `ORBvoc.txt.bin` (48.7 MB) descargado del repo upstream Mechazo11 por la red del Orin.
- [x] **Headless:** añadido parámetro `use_viewer` (default `false`) en `common.cpp`;
  `enablePangolinWindow` ahora lo respeta y `enableOpenCVWindow=false`.
- [x] **Fix Humble:** `common.hpp` usaba `cv_bridge/cv_bridge.hpp` (Iron/Jazzy);
  cambiado a `cv_bridge/cv_bridge.h` (lo que trae Humble).
- [x] Compilado OK (`colcon`, RAM controlada con `-j3`; `Optimizer.cc` ya venía a `-O1`).
- [x] Smoke test: `mono_node_cpp` arranca headless, resuelve todas las libs (ldd OK) y
  queda en "Waiting to finish handshake".
- [ ] (Pendiente) Smoke test del **pipeline completo** con dataset de ejemplo (atado a Fase 3,
  porque el driver de dataset tiene ruta hardcodeada).

## Fase 2 — Calibración de cámara → `config/Monocular/JetAuto.yaml`  ✅ (vía factory)
- [x] `JetAuto.yaml` creado con la **calibración de fábrica** publicada en `/cam_1/camera_info`
  (fx=fy=539.13, cx=320.46, cy=240.02; rational_polynomial con k4-k6=0 → plumb_bob k1..k3).
  Validado en vivo: ORB-SLAM3 inicializa y trackea con estos intrínsecos.
- [ ] (Opcional) Calibrar con checkerboard solo si la calidad de reproyección limita.

## Fase 3 — Ingesta de cámara en vivo (reemplaza driver de dataset)
- [x] Escrito `live_camera_driver_node.py` (handshake `settings_name`, suscribe `/cam_1/image`,
  republica a `/mono_py_driver/img_msg` + `timestep` con `header.stamp` real, QoS BEST_EFFORT).
  Instalado en el Orin (`ros2 pkg executables` lo lista).
- [x] **Probado con la cámara real**: frames 640×480 rgb8 @30 Hz fluyen del Nano al nodo;
  ORB-SLAM3 **inicializó el mapa y publicó pose en vivo** ("New Map created with 242 points").
- ⚠️ Operación: TODO comando ROS en el Orin necesita
  `CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml` + `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  (sin eso DDS anuncia la interfaz WiFi equivocada y los topics "no aparecen").
- ⚠️ El tracking se pierde al parar bruscamente / con gente moviéndose en escena (precisamente
  la motivación del filtro EfficientSAM3). Manejo suave y continuo para los runs de eval.

## Fase 4 — Salida de pose/trayectoria  ✅ HECHO (validado en vivo con TUM)
- [x] `common.cpp` (`Img_callback`): publica `geometry_msgs/PoseStamped` de `Twc` en
  `/orbslam3/pose` (`orb_map`→`orb_cam`, `stamp` = stamp de la imagen), solo cuando
  `GetTrackingState()==OK`. Params: `pose_topic`, `map_frame`, `camera_frame`.
  Añadida dep `geometry_msgs` (CMake + package.xml).
- [x] **Validado**: smoke test con TUM fr1/xyz → `/orbslam3/pose` a **~25.8 Hz**, tracking OK.
- [ ] Guardado de trayectoria TUM al cerrar: funciona vía `trajectory_output`, pero hay que
  **dar margen al `Shutdown`** (BA final >6 s) antes de matar el proceso para que escriba
  el archivo (en el smoke lo maté con `-9` muy pronto y salió vacío).

## Fase 5 — Ground truth + evaluación  (infra lista, falta el run)
- [x] GT elegido: **OptiTrack** (`/optitrack/rigid_body`, PoseStamped, sensor QoS). El
  `optitrack_client` ya corre en el Orin (verificar que Motive esté streameando el día del run).
- [x] **Herramientas instaladas**: `evo` v1.31.1 en la laptop; `psutil`+`nvidia-ml-py` en el
  Orin (para `monitor_cli`). ⚠️ NO instalar evo/numpy nuevo en el Orin (rompe Humble).
- [x] **Scripts creados** (en `metrics/`, sincronizados al Orin):
  - `slam_metrics/live_eval_logger.py` — graba est/gt/odom → TUM con un solo reloj.
  - `slam_metrics/topic_fps.py` — FPS cámara vs filtro vs pose SLAM → JSON (probado: cámara 27 fps).
  - `record_eval_run.sh` — orquestador: bag (`/cam_1/image`+poses+GT+tf) + logger + monitor.
  - `~/stop_live.sh` (Orin) — para el pipeline con SIGINT y espera ≥30 s para que
    `trajectory_output` se escriba (el BA final tarda >6 s).
- [ ] **Run de evaluación** (manejo suave 1-2 min con OptiTrack activo):
  `./record_eval_run.sh run1` → `trajectory_eval est.tum gt.tum --rpe-delta 1.0` (Sim3) y/o
  `evo_ape tum gt.tum est.tum -as` en la laptop.

## Fase 6 — Launch e integración operativa  ✅ (base)
- [x] `launch/orbsame_live.launch.py` creado e instalado (args: `settings_name`, `camera_topic`,
  `use_viewer`, `pose_topic`, `trajectory_output`). Corrido en el Orin junto al bringup real
  (`~/live_test.sh` lo lanza con el env DDS correcto).
- [ ] Integrarlo al arranque formal (systemd / deploy_agv_orin.sh) si se vuelve permanente.
- [ ] **Procedimiento de manejo:** traslación + textura para inicializar; evitar rotación pura
  / paredes lisas / poca luz; **batería buena** (ver `ENCODER_ODOMETRY.md`).

---

## Fase 7 — Pipeline completo ORB-SAM-E: EfficientSAM3 en el Orin

**Objetivo:** correr el flow del paper en el robot:
`/cam_1/image` → `dynamic_filter_node` (EfficientSAM3, GPU) → `/camera/image_filtered` →
`mono_node_cpp` (`use_filtered_images:=true`). El nodo C++ **ya soporta** el modo filtrado.

### 7.1 Dependencias  ✅ HECHO (venv funcionando con CUDA)
- [x] **venv** `~/venvs/esam3` (`--system-site-packages` para rclpy) con
  **torch 2.11.0 + torchvision 0.26.0** del índice jetson-ai-lab (`jp6/cu126`) y
  numpy 1.26.4 + timm/ftfy/einops/scipy/open_clip/etc. `torch.cuda.is_available()=True` (Orin).
  - Deps de sistema que hubo que instalar: `cusparselt-cuda-12`, `cuda-cupti-12-6`
    (+ `ldconfig`), y **cuDSS 0.6** (`libcudss0-cuda-12` + `/etc/ld.so.conf.d/cudss.conf`).
  - NO hizo falta `pip install -e`: el código se inyecta vía `efficientsam3_path` (sys.path),
    así que el `requires-python>=3.12` es irrelevante (todo el código parsea en 3.10 ✓).
- [x] Código en el Orin (sparse checkout) + pesos + `assets/bpe_simple_vocab_16e6.txt.gz`
  (el sparse checkout no lo trajo; copiado por scp). Scripts en el Orin:
  `~/setup_esam3.sh` (instalación reproducible) y `~/test_esam3.sh` (import + 1 frame).
- [x] **Import test + inferencia 1 frame en CUDA OK** (`filter_core.py --image frame.jpg`):
  detecta, enmascara y guarda visualización.
- [x] **Checkpoint correcto validado**: `efficient_sam3_repvit_s.pt` (stage1) trae text
  encoder CLIP estándar pero el builder ARM arma un student MobileCLIP-S1 → pesos de texto
  aleatorios, scores casi-cero **sin error** (gotcha ya documentado en
  `CLOUD_GPU_TESTING.md` §"Wrong checkpoint"). Re-descargado el correcto
  **`efficient_sam3_repvit-m0_9_mobileclip_s1.pth`** (2.35 GB) y re-validado en el Orin:
  carga sin `missing_keys`, confianzas reales 0.2-0.47 y máscara sobre las personas ✓.
- ⚠️ **Latencia medida (sin tuning, fp32, 5 prompts)**: ~5.5 s/frame warm (0.18 FPS) en el
  Orin Nano (ya en su modo máximo: 15W; no hay MAXN en esta config). Consistente con el
  hallazgo de `CLOUD_GPU_TESTING.md`: el FPS escala ~lineal con el nº de prompts
  (A4000: 6 prompts→1.37 FPS, 2 prompts→3.9 FPS con ~74% detección) y `Sam3Processor`
  corre fp32 sin optimizaciones (`Sam3ProcessorARM` expone `use_fp16`). El tuning de 7.3
  (2 prompts, fp16, frame skip, `jetson_clocks`) es **obligatorio**, no opcional.
  Estimación con tuning: ~1-1.5 s/frame → usable con `process_every_n_frames: 2-3`.

### 7.1b Upgrade a EfficientSAM3.1 (ctx16 + MobileCLIP-S0)  ✅ VALIDADO — config recomendada
- [x] Checkpoint **`stage1_sam3p1/efficient_sam3p1_repvit_s_mobileclip_s0_ctx16.pt`** (522 MB
  vs 2.35 GB) descargado y validado en el Orin. Su text encoder ft/3.1 está mucho mejor
  alineado: confianzas top **0.78** (vs 0.47 del S1 stage1).
- [x] **Patch aditivo** (no copia: los cambios no fueron drásticos y los defaults preservan
  el comportamiento legacy al 100%):
  - `model_builder_arm.py`: params `text_encoder_context_length` (default 32) y
    `text_encoder_pos_embed_table_size` (default None→tabla 77 legacy) en
    `build_efficientsam3_image_model`/`_create_student_text_encoder`; remap
    `interactive_convs`→`sam2_convs` en `_load_checkpoint` (nombres TriNeck de SAM3.1).
  - `filter_core.py`: `MODEL_FILENAME_MAP` ahora es 5-tupla (+ctx,+table) con entradas
    sam3p1 al inicio (first-match-wins).
- ℹ️ El ckpt 3.1 deja 4 `missing_keys` (`convs.3.*`, el nivel 0.5× del neck): **inofensivo**
  — `SAM3VLBackbone(scalp=1)` descarta ese nivel; queda random pero nunca se usa.
- [x] **🏆 Config ganadora medida en el Orin** (warm, fp32):
  | Config | Latencia | FPS | Detección |
  |---|---|---|---|
  | S1 stage1, 6 prompts (baseline) | 5 469 ms | 0.18 | conf máx 0.47, máscara con spill |
  | sam3p1, 6 prompts | 6 471 ms | 0.15 | conf máx 0.78 |
  | sam3p1, 2 prompts | 2 357 ms | 0.42 | OK |
  | **sam3p1, 1 prompt `person`, thr 0.3** | **1 134 ms** | **0.88** | **5 dets (0.73/0.70/0.68), máscara limpia 15.6%** |
  → **~4.9× speedup** y mejor máscara, antes de fp16. ctx16 por sí solo casi no acelera
  (el costo está en los tokens de imagen, no de texto); la ganancia real es que el text
  encoder 3.1 entiende `person` y permite 1 prompt.
- [x] **fp16 autocast medido**: 604 ms (1.66 FPS) vs 1 134 ms fp32 → **1.87×** con calidad
  idéntica (mismos 5 dets, mismas confianzas, mismo mask_ratio). Implementado en
  `filter_core` (`use_fp16=True` / CLI `--fp16 --prompts --threshold`).
  **Total acumulado: 9.1× vs baseline (5 469 → 604 ms) con mejor máscara.**
- [x] Hallazgos DL documentados para el paper en **`PAPER_DL_FINDINGS.md`**.
- [ ] Ajustar default del `dynamic_filter_node` a `["person"]` + `confidence_threshold:=0.3`
  + `use_fp16:=true` cuando se use el ckpt sam3p1 (el threshold 0.03 era para el S1 débil).
- ℹ️ ft S1 (`repvit_m1.1_mobileclip_s1_ft.pth`) descargado pero descartado: sam3p1 lo
  supera en calidad, velocidad y RAM.
- 📌 Otros gotchas de `CLOUD_GPU_TESTING.md` aplicables a los runs en robot:
  la detección es **dependiente del frame** (~80% de frames con detección en walking_xyz;
  no concluir de 1 frame) y siempre probar el **code path real** (`filter_core`), no
  re-implementaciones de diagnóstico.

### 7.2 Wiring ROS  ✅ HECHO (validado en vivo en el robot)
- [x] `dynamic_filter_node`: ganó param `use_fp16`; corre **dentro del venv** vía
  `ExecuteProcess` con `~/venvs/esam3/bin/python3 -m efficientsam3_ros2.dynamic_filter_node`
  (PYTHONPATH prepende el pkg root sin pisar el de ROS — rclpy sigue viniendo de Humble).
- [x] **Fix de sync** en `Img_callback` (common.cpp): usa `msg.header.stamp` cuando ≠0
  (cámara real y filtro lo preservan) con fallback al timestep latcheado (datasets).
  Con ~0.6 s de latencia del filtro el timestep latcheado ya no correspondía al frame.
- [x] `live_camera_driver_node.py` param `use_filter`: en modo filtro solo hace el
  handshake (la imagen va directo cámara→filtro→SLAM; no se republica nada).
- [x] `orbsame_live.launch.py` con arg `use_filter` + args del filtro (`model_path`,
  `filter_prompts`, `filter_threshold`, `filter_fp16`, `filter_every_n`,
  `filter_metrics_output`). Defaults = config ganadora sam3p1.
  Lanzador en el Orin: `~/live_filtered.sh [trayectoria] [metrics.json]`.
- [x] **Smoke test en vivo OK** (robot quieto, gente en escena): handshake completo,
  `mono_node_cpp` suscrito a `/camera/image_filtered`, filtro a **1.7 FPS de inferencia
  (594 ms mediana, igual que el benchmark standalone)** + reuso de máscara intercalado,
  `/camera/image_filtered` a **~3.2 Hz**, ~3.3 detecciones/frame, metrics JSON al cierre.
- [x] **Primer run con movimiento (2026-06-10, `move2`)**: lazo de ~3.5×2.5 m por joystick,
  200 s. Bag de 5.2 GB en `~/runs/bags/move2` (cámara cruda 30fps + `/orbslam3/pose` +
  odom + tf) → permite A/B offline. SLAM inicializó en t≈85 s, trackeó 115 s con 1 pérdida
  (4.7 s, nuevo mapa en KF 127). Trayectorias extraídas del bag:
  `eval_runs/move2/run_move2_{slam,odom}.tum`.
  - ⚡ **Hallazgo de potencia**: la powerbank no sostiene el pico de la Orin en 15W (brownout
    al cargar el modelo en GPU; ya había matado un run antes y corrompido un log). Fix actual:
    **modo 7W** (`nvpmodel -m 1`) → estable, inferencia a 0.8 FPS (1.3 s/frame). Pendiente:
    alimentar del robot (LiPo 3S 9.9-12.6V → jack 9-19V) o powerbank PD 12V + cable trigger.
  - Fix: `stop_live.sh` timeout 30→120 s (en 7W el BA final tarda >30 s; el kill forzado
    impidió escribir trajectory_output/metrics en este run).
- [x] **Replay offline del bag `move2` (2026-06-10)**: pipeline completo re-ejecutado sobre el
  bag (cámara remapeada a `/replay/image` para no chocar con la cámara viva). Salidas:
  `replay_move2_filtered.txt` (83 KFs), `replay_move2_map.ply` (2334 map points),
  `replay_move2_metrics.json`. Detección de persona verificada cualitativamente con
  `bag_detection_overlays.py`: **57% de los frames de la ventana 60-200s con detección**
  (confidencias 0.32-0.39 con threshold 0.3), overlays en `~/runs/move2_overlays`.
  - Nuevo: export de map points (PLY) vía parámetro `map_points_output`.
  - Nuevo: **guardado on-demand** vía topic `/orbslam3/save_outputs` (el teardown del
    launch no siempre entrega SIGINT al nodo C++ → guardar ANTES de detener).
- [x] **Worker thread (2026-06-10)**: inferencia desacoplada del callback en
  `dynamic_filter_node.py` (thread daemon + reference swap de la máscara). Verificado en vivo:
  `/camera/image_filtered` a **29.2 Hz** (full camera rate), inferencia async a **~1.5 FPS
  (680 ms)**, el SLAM recibe todos los frames con la última máscara (staleness ≤ ~0.7 s).
  Con worker thread `process_every_n_frames` pasa a ser un throttle de GPU opcional (mínimo
  de frames de cámara entre inferencias); el ritmo natural lo marca la GPU.
- [x] **A/B testing completo (2026-06-10/11, bags `dyn1`/`dyn2`)**: 2 bags dinámicos con
  personas (139 s/4160 frames y 154 s/4621 frames) grabados por joystick; 4 replays pareados
  (baseline vs filtrado) a rate 1.0 con `~/run_one_replay.sh`, grabando el stream
  `/orbslam3/pose` a un posebag por replay (robusto a resets del monocular, que vacían la
  trayectoria KF). Análisis en Mac: `eval_runs/extract_posebag.py` (parser CDR directo,
  sin ROS) + `eval_runs/ab_analysis.py` (segmentación por gaps, Sim(3) por segmento + ATE
  local en ventanas de 1 m). **Resultados**:
  - **Resets**: dyn1 6→3 (−50%), dyn2 12→10. Segmento continuo mayor dyn1: 22.1→27.8 s.
  - **ATE global del segmento mayor** dyn2: 0.216→0.053 m. En dyn1 el global del filtrado
    (0.568 m) refleja deriva de escala acumulada en su segmento más largo, no error local:
    el **ATE local (ventanas 1 m)** queda en pocos cm en los 4 runs (0.008-0.050 m).
  - Filtro durante replays (7W): ~1.3 s/inferencia, detección de persona en 23% (dyn1) y
    67% (dyn2) de los frames inferidos.
  - Lección de replay: `ros2 bag play --rate 0.5` rompe la inicialización del mono SLAM
    (resets constantes, trayectorias vacías) → siempre rate 1.0.
  - Datos en Mac: `eval_runs/ab/` (posebags, TUMs, odom, métricas, `ab_results.json`).
    Canvas comparativo: `orbsame-ab-results.canvas.tsx`.
  - **Evidencia cualitativa**: overlays de detección (47+52 paneles, `eval_runs/ab/overlays/`)
    y videos de 3 paneles original|detección|grayout (`eval_runs/ab/dyn{1,2}_filter.mp4`,
    15 fps, inferencia cada ~0.7 s con reuso de máscara, generados con
    `efficientsam3_ros2/bag_filter_video.py`). Personas en movimiento enmascaradas de forma
    fiable; falsos positivos ocasionales en muebles (ventilador/sillas) con umbral 0.3.
  - **SLAM vs odometría**: cobertura en movimiento ~55-67%; en giros el baseline cae a
    55-57% vs 67% en recta, el filtrado en dyn1 mantiene 65.5% en giros. ATE local p95
    de 2.7-10 cm. Gap inicio-fin de odom en dyn1 (lazo cerrado real): 0.44 m = deriva
    de la propia odometría (~4.8% del camino).

### 7.3 Performance en Orin Nano 8GB (de `FPS_IMPROVEMENT_PLAN.md`)
- Arranque: **2 prompts** (`human face`, `human shirt`) → ~3.9 fps medidos en A4000; el Orin será
  más lento. Luego **fp16/bf16 autocast** (~1.5-2×) y **`process_every_n_frames: 2-3`**.
- Presupuesto RAM compartida (8 GB): SLAM ~1-2 GB + torch ~1-2 GB + modelo. Monitorear con
  `monitor_cli` durante el primer run; si hay OOM, bajar resolución del filtro o frames salteados.
- Medir con `topic_fps.py --filtered-topic /camera/image_filtered` (da los 3 números del paper:
  FPS cámara / filtro / SLAM) + `metrics_output` del filtro (latencias + detecciones).

### 7.4 Protocolo de benchmark en robot (paper)
1. **Grabar bag una vez** con `record_eval_run.sh` (cámara + GT OptiTrack) manejando suave
   con gente caminando en escena (el caso dinámico).
2. **Replay offline del MISMO bag** ×2: baseline (`use_filtered_images:=false`) vs filtrado
   → comparación pareada de ATE/RPE con GT idéntico.
3. Complemento: `run_benchmark.sh` con TUM `walking_*` directo en el Orin (números edge
   reproducibles vs literatura).

---

## Riesgos a vigilar
1. **Build aarch64 en Orin** (no documentado upstream) — mayor riesgo de cronograma.
2. **Pérdida de tracking** monocular (rotación pura, poca textura, baja luz).
3. **Escala no métrica** — alinear con Sim3 en eval (ya soportado en `metrics/`).
4. **Rutas hardcodeadas** (`common.hpp`, driver Python) y **Pangolin headless**.
5. **Sync temporal** cámara↔GT, **rolling shutter / decode MJPG** de la Astra.
