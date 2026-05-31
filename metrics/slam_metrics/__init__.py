"""
slam_metrics: reproducible evaluation toolkit for ORB-SAM-E experiments.

Modules:
    trajectory_eval  ATE / RPE against TUM groundtruth (numpy only)
    perf_timer       per-stage latency / FPS accumulation
    system_monitor   background GPU/CPU/RAM sampler
    env_info         hardware / software environment capture for reproducibility
    report           aggregate everything into results.json + Markdown
"""

__version__ = "0.1.0"
