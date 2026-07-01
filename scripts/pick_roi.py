#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interactive board-ROI picker for FAST-Calib2 (host-native, Open3D only).

Reads an ASCII PCD (x y z intensity) — e.g. the cloud.pcd produced by the
capture pipeline — colors points by intensity so the reflective annuli/board
stand out, lets you Shift+click >=4 points on the board, and prints/writes the
x_min..z_max ROI block to paste into config/cameras/<cam>.yaml.

Needs Open3D + a display (runs on the machine's monitor).
    python3 pick_roi.py <cloud.pcd> [out.txt]
"""
import sys
import numpy as np
import open3d as o3d

if len(sys.argv) < 2:
    sys.exit("usage: pick_roi.py <cloud.pcd> [out.txt]")
pcd_path = sys.argv[1]
out_txt = sys.argv[2] if len(sys.argv) > 2 else pcd_path.rsplit(".", 1)[0] + "_roi.txt"

pts, inten = [], []
with open(pcd_path) as f:
    in_data = False
    for ln in f:
        if in_data:
            v = ln.split()
            if len(v) >= 4:
                pts.append([float(v[0]), float(v[1]), float(v[2])])
                inten.append(float(v[3]))
        elif ln.startswith("DATA"):
            in_data = True
pts = np.asarray(pts)
inten = np.asarray(inten)
if len(pts) < 4:
    sys.exit("ERROR: not enough points read from %s" % pcd_path)

# color by intensity (0..p99) so bright reflective annuli pop
lo, hi = inten.min(), max(inten.min() + 1e-6, np.percentile(inten, 99))
g = np.clip((inten - lo) / (hi - lo), 0, 1)
pc = o3d.geometry.PointCloud()
pc.points = o3d.utility.Vector3dVector(pts)
pc.colors = o3d.utility.Vector3dVector(np.stack([g, g, g], 1))

print("Loaded %d points." % len(pts))
print("In the window: Shift+LeftClick >=4 points on the BOARD (corners), then press Q.")
vis = o3d.visualization.VisualizerWithEditing()
vis.create_window(window_name="pick board ROI")
vis.add_geometry(pc)
vis.run()
vis.destroy_window()

idx = vis.get_picked_points()
if len(idx) < 4:
    sys.exit("ERROR: picked %d points (<4); nothing saved." % len(idx))

sel = pts[idx]
mins = sel.min(0)
maxs = sel.max(0)
pad = 0.10  # tighter than the original tool's 0.2 m to avoid the wall behind the board
x0, x1 = mins[0] - pad, maxs[0] + pad
y0, y1 = mins[1] - pad, maxs[1] + pad
z0, z1 = mins[2] - pad, maxs[2] + pad
block = (
    "  x_min: %.2f\n  x_max: %.2f\n"
    "  y_min: %.2f\n  y_max: %.2f\n"
    "  z_min: %.2f\n  z_max: %.2f\n" % (x0, x1, y0, y1, z0, z1)
)
with open(out_txt, "w") as f:
    f.write(block)
print("\n=== paste into config/cameras/<cam>.yaml (keep use_auto_lidar_roi: false) ===")
print(block)
print("saved -> %s" % out_txt)
