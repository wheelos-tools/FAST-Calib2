#!/usr/bin/env python3
"""Accumulate PointCloud2 frames from an Apollo Cyber .record into one dense PCD
(x y z intensity). The board + LiDAR are static during the capture, so merging N
frames densifies each scan ring (different azimuthal samples per frame) and lifts
the cloud above FAST-Calib2's dense-LiDAR point-count guards. Runs on the HOST
with the pure-python reader:
    python3 -m pip install --user cyber_record protobuf==3.19.4
No Apollo runtime / sourcing required.
"""
import argparse
import glob

from cyber_record.record import Record

HEADER = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
          "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
          "COUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
          "POINTS {n}\nDATA ascii\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record-glob", required=True)
    ap.add_argument("--channel", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-frames", type=int, default=20,
                    help="cap accumulated frames (static scene; default 20)")
    args = ap.parse_args()

    files = sorted(glob.glob(args.record_glob))
    if not files:
        raise SystemExit("no record files match " + args.record_glob)

    rows = []
    nframes = 0
    done = False
    for f in files:
        if done:
            break
        for _topic, msg, _bt in Record(f).read_messages(args.channel):
            nframes += 1
            for p in msg.point:
                x, y, z = p.x, p.y, p.z
                # drop invalid / zero-return points
                if not (x == x and y == y and z == z):
                    continue
                if abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
                    continue
                rows.append("%.5f %.5f %.5f %.1f" % (x, y, z, p.intensity))
            if nframes >= args.max_frames:
                done = True
                break

    if not rows:
        raise SystemExit("no PointCloud2 messages on %s in %s" %
                         (args.channel, args.record_glob))

    n = len(rows)
    with open(args.out, "w") as fh:
        fh.write(HEADER.format(n=n))
        fh.write("\n".join(rows))
        fh.write("\n")
    print("frames accumulated: %d  total points: %d -> %s" %
          (nframes, n, args.out))


if __name__ == "__main__":
    main()
