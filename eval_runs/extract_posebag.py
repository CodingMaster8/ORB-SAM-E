#!/usr/bin/env python3
"""Extract geometry_msgs/PoseStamped messages from a rosbag2 sqlite3 bag to TUM format.

Parses CDR serialization directly so no ROS installation is needed.
TUM format: timestamp tx ty tz qx qy qz qw
"""
import sqlite3
import struct
import sys
from pathlib import Path


def parse_posestamped_cdr(data: bytes):
    # 4-byte encapsulation header (assume little-endian CDR, id 0x0001)
    off = 4
    sec, nanosec = struct.unpack_from("<iI", data, off)
    off += 8
    (slen,) = struct.unpack_from("<I", data, off)
    off += 4 + slen  # frame_id incl. null terminator
    # CDR alignment is relative to start of payload (after 4-byte encapsulation)
    off = 4 + (((off - 4) + 7) & ~7)
    vals = struct.unpack_from("<7d", data, off)
    t = sec + nanosec * 1e-9
    px, py, pz, qx, qy, qz, qw = vals
    return t, px, py, pz, qx, qy, qz, qw


def main(bag_dir: str, out_path: str):
    db3 = sorted(Path(bag_dir).glob("*.db3"))
    if not db3:
        sys.exit(f"no .db3 in {bag_dir}")
    rows = []
    for db in db3:
        conn = sqlite3.connect(str(db))
        cur = conn.execute(
            "SELECT m.data FROM messages m JOIN topics t ON m.topic_id=t.id "
            "WHERE t.name='/orbslam3/pose' ORDER BY m.timestamp"
        )
        for (blob,) in cur:
            rows.append(parse_posestamped_cdr(blob))
        conn.close()
    with open(out_path, "w") as f:
        for r in rows:
            f.write(" ".join(f"{v:.9f}" for v in r) + "\n")
    print(f"{out_path}: {len(rows)} poses")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
