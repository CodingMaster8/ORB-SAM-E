"""
Run the resource SystemMonitor for the duration of an experiment.

Runs until it receives SIGINT/SIGTERM (or an optional --duration elapses),
then writes a resources.json summary and a resources.csv timeseries.

Usage:
    python -m slam_metrics.monitor_cli --out-json resources.json --out-csv resources.csv
    # ... in another process, run the pipeline ...
    # then send SIGINT (kill -INT <pid>) to stop and flush.
"""

from __future__ import annotations

import argparse
import json
import signal
import time

from .system_monitor import SystemMonitor


def main() -> None:
    p = argparse.ArgumentParser(description="Background resource monitor for experiments.")
    p.add_argument("--out-json", default="resources.json")
    p.add_argument("--out-csv", default="resources.csv")
    p.add_argument("--interval", type=float, default=0.5)
    p.add_argument("--gpu-index", type=int, default=0)
    p.add_argument("--duration", type=float, default=0.0,
                   help="Stop after N seconds (0 = run until signalled)")
    args = p.parse_args()

    mon = SystemMonitor(interval=args.interval, gpu_index=args.gpu_index)
    stop = {"flag": False}

    def _handle(signum, frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)

    mon.start()
    print(f"[monitor] sampling every {args.interval}s "
          f"(gpu {args.gpu_index}); send SIGINT to stop.")
    start = time.time()
    try:
        while not stop["flag"]:
            time.sleep(0.2)
            if args.duration > 0 and (time.time() - start) >= args.duration:
                break
    finally:
        summary = mon.stop()
        mon.save_csv(args.out_csv)
        with open(args.out_json, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"[monitor] wrote {args.out_json} ({summary['num_samples']} samples) "
              f"and {args.out_csv}")


if __name__ == "__main__":
    main()
