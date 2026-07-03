#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interactive board-ROI picker for FAST-Calib2 (Open3D only).

Reads an ASCII PCD (x y z intensity), colors points by intensity so the reflective
annuli/board stand out, lets you Shift+click >=4 points on the board, and prints the
x_min..z_max ROI block. With --yaml it also writes the ROI straight into a FAST-Calib2
camera config (and forces use_auto_lidar_roi: false).

  python3 pick_roi.py <cloud.pcd> [--yaml <config.yaml>] [--pad 0.10] [--out roi.txt]

Needs Open3D + a display (window opens on the machine's monitor).
"""
import argparse
import re
import sys

import numpy as np
import open3d as o3d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcd")
    ap.add_argument("--yaml", default=None,
                    help="FAST-Calib2 camera config to update in place")
    ap.add_argument("--pad", type=float, default=0.10,
                    help="metres added around the picked points (default 0.10)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    out_txt = args.out or (args.pcd.rsplit(".", 1)[0] + "_roi.txt")

    pts, inten = [], []
    with open(args.pcd) as f:
        in_data = False
        for ln in f:
            if in_data:
                v = ln.split()
                if len(v) >= 4:
                    pts.append((float(v[0]), float(v[1]), float(v[2])))
                    inten.append(float(v[3]))
            elif ln.startswith("DATA"):
                in_data = True
    pts = np.asarray(pts)
    inten = np.asarray(inten)
    if len(pts) < 4:
        sys.exit("too few points read from " + args.pcd)

    lo, hi = inten.min(), max(inten.min() + 1e-6, np.percentile(inten, 99))
    g = np.clip((inten - lo) / (hi - lo), 0, 1)
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts)
    pc.colors = o3d.utility.Vector3dVector(np.stack([g, g, g], 1))

    print("Loaded %d points." % len(pts))
    print("In the window: Shift+LeftClick >=4 points on the BOARD, then press Q.")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="pick board ROI: " + args.pcd)
    vis.add_geometry(pc)
    vis.run()
    vis.destroy_window()

    idx = vis.get_picked_points()
    if len(idx) < 4:
        sys.exit("picked %d points (<4); nothing written." % len(idx))

    sel = pts[idx]
    mn, mx, p = sel.min(0), sel.max(0), args.pad
    roi = {
        "x_min": mn[0] - p, "x_max": mx[0] + p,
        "y_min": mn[1] - p, "y_max": mx[1] + p,
        "z_min": mn[2] - p, "z_max": mx[2] + p,
    }
    order = ["x_min", "x_max", "y_min", "y_max", "z_min", "z_max"]
    block = "".join("  %s: %.2f\n" % (k, roi[k]) for k in order)
    with open(out_txt, "w") as fh:
        fh.write(block)
    print("\n=== ROI ===\n" + block + "saved -> " + out_txt)

    if args.yaml:
        y = open(args.yaml).read()
        y = re.sub(r"use_auto_lidar_roi:\s*true", "use_auto_lidar_roi: false", y)
        for k in order:
            y = re.sub(r"(?m)^(\s*%s:).*" % k, r"\g<1> %.2f" % roi[k], y)
        with open(args.yaml, "w") as fh:
            fh.write(y)
        print("updated %s  (use_auto_lidar_roi: false + ROI)" % args.yaml)


if __name__ == "__main__":
    main()
