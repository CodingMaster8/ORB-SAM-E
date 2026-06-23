# ORB-SAM-E documentation

Long-form guides and research notes for ORB-SAM-E. For the project overview,
architecture, and repository map, see the [top-level README](../README.md).

Some research notes are in Spanish (marked *(ES)*).

## Setup & build
- [UBUNTU_SETUP.md](UBUNTU_SETUP.md) — canonical Ubuntu + ROS 2 install,
  ORB-SLAM3 build, model weights, and first run.
- [../efficientsam3_ros2/VM_SETUP.md](../efficientsam3_ros2/VM_SETUP.md) —
  lighter setup focused on the filter package.

## Running on GPU hardware
- [CLOUD_GPU_TESTING.md](CLOUD_GPU_TESTING.md) — picking and using a cloud GPU
  to validate the CUDA pipeline; includes a detailed "gotchas" section and a
  results-recording template.
- [POD_RUNBOOK.md](POD_RUNBOOK.md) — copy-paste runbook from a fresh RunPod pod
  to recorded benchmark numbers.

## Robot deployment & research findings
- [ROBOT_INTEGRATION_TODO.md](ROBOT_INTEGRATION_TODO.md) *(ES)* — live Jetson /
  JetAuto integration log (phases, A/B results, open items).
- [PAPER_DL_FINDINGS.md](PAPER_DL_FINDINGS.md) *(ES)* — deep-learning findings
  measured on the real robot (checkpoint pitfalls, latency anatomy, the
  measured performance ladder, recommended config).
- [FPS_IMPROVEMENT_PLAN.md](FPS_IMPROVEMENT_PLAN.md) — filter throughput cost
  model and prioritized speedups.

## Related (outside docs/)
- [../metrics/README.md](../metrics/README.md) — evaluation/metrics toolkit.
- [../eval_runs/README.md](../eval_runs/README.md) — recorded runs + A/B analysis.
- [../efficientsam3_ros2/README.md](../efficientsam3_ros2/README.md) — filter node.
