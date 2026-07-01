#!/usr/bin/env python3
"""Project the LiDAR cloud + the 4 detected annulus centers into the camera image
using the FAST-Calib2 extrinsic, for visual confirmation. Points are colored by
intensity (jet: bright reflective annuli -> red) so they should land on the white
rings if the calibration is good. The 4 LiDAR-detected centers are drawn as green
crosshairs.

  python3 overlay_reproj.py <scene_dir> <single_calib_result.txt> <out.png>
"""
import re
import sys
import numpy as np
import cv2

scene = sys.argv[1]
calib = sys.argv[2]
out = sys.argv[3]

txt = open(calib).read()
def g(k):
    return float(re.search(k + r":\s*([-\d.eE]+)", txt).group(1))
fx, fy, cx, cy = g("cam_fx"), g("cam_fy"), g("cam_cx"), g("cam_cy")
k1, k2, p1, p2 = g("cam_d0"), g("cam_d1"), g("cam_d2"), g("cam_d3")
R = np.array(re.search(r"Rcl:\s*\[([^\]]+)\]", txt).group(1).split(","), float).reshape(3, 3)
t = np.array(re.search(r"Pcl:\s*\[([^\]]+)\]", txt).group(1).split(","), float)
K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
D = np.array([k1, k2, p1, p2, 0.0])

img = cv2.imread(scene + "/image.png")
img = cv2.undistort(img, K, D)
H, W = img.shape[:2]

# load lidar cloud (x y z intensity)
P, I = [], []
with open(scene + "/cloud.pcd") as f:
    d = False
    for ln in f:
        if d:
            v = ln.split()
            if len(v) >= 4:
                P.append([float(v[0]), float(v[1]), float(v[2])]); I.append(float(v[3]))
        elif ln.startswith("DATA"):
            d = True
P = np.asarray(P); I = np.asarray(I)

# transform lidar -> camera, project
Pc = (R @ P.T).T + t
m = Pc[:, 2] > 0.05
Pc, Ii = Pc[m], I[m]
u = (fx * Pc[:, 0] / Pc[:, 2] + cx)
v = (fy * Pc[:, 1] / Pc[:, 2] + cy)
inb = (u >= 0) & (u < W) & (v >= 0) & (v < H)
u, v, Ii = u[inb].astype(int), v[inb].astype(int), Ii[inb]

# color by intensity (jet)
g = np.clip(Ii / 255.0, 0, 1)
colors = cv2.applyColorMap((g * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)
overlay = img.copy()
for ui, vi, c in zip(u, v, colors):
    cv2.circle(overlay, (ui, vi), 1, (int(c[0]), int(c[1]), int(c[2])), -1)
# emphasize saturated reflective points
sat = Ii > 200
for ui, vi in zip(u[sat], v[sat]):
    cv2.circle(overlay, (ui, vi), 2, (0, 0, 255), -1)
img = cv2.addWeighted(overlay, 0.85, img, 0.15, 0)

# project the 4 detected lidar centers (green crosshairs).
# circle_center_record.txt lives next to the calib result file (argv[2]).
import os
crec_path = os.path.join(os.path.dirname(os.path.abspath(calib)), "circle_center_record.txt")
centers = []
try:
    block = open(crec_path).read().strip().splitlines()
    line = [l for l in block if l.startswith("lidar_centers")][-1]
    for trip in re.findall(r"\{([^}]+)\}", line):
        centers.append([float(x) for x in trip.split(",")])
except Exception as e:
    print("centers parse skipped:", e)
for C in centers:
    Cc = R @ np.array(C) + t
    if Cc[2] > 0:
        cu = int(fx * Cc[0] / Cc[2] + cx); cv_ = int(fy * Cc[1] / Cc[2] + cy)
        cv2.drawMarker(img, (cu, cv_), (0, 255, 0), cv2.MARKER_CROSS, 40, 3)
        cv2.circle(img, (cu, cv_), 6, (0, 255, 0), 2)

cv2.imwrite(out, img)
print("projected %d in-FOV lidar points (%d saturated), %d centers -> %s" %
      (len(u), int(sat.sum()), len(centers), out))
