#!/usr/bin/env python3
"""Validate camera intrinsics against the physical board:
 (A) reconstruct the 4 ArUco markers INDEPENDENTLY (per-marker solvePnP) -> inter-marker
     distances vs ground truth (tests the metric scale = focal length).
 (B) reconstruct the 4 ring centers by back-projecting the DETECTED ring pixels onto the
     board plane (board pose from all markers) -> inter-ring distances vs ground truth.
Both use only the intrinsics + the image, so distances that match GT confirm the intrinsics.

  intrinsic_board_check.py <image> fx fy cx cy k1 k2 p1 p2
"""
import sys
import itertools
import numpy as np
import cv2

img_path = sys.argv[1]
fx, fy, cx, cy, k1, k2, p1, p2 = map(float, sys.argv[2:10])
K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], float)
D = np.array([k1, k2, p1, p2, 0.0], float)

# board ground truth (metres) from the config
MS, DWq, DHq, DWc, DHc = 0.20, 0.55, 0.35, 0.50, 0.40
mk_gt = {1: (-DWq, DHq), 2: (DWq, DHq), 4: (DWq, -DHq), 3: (-DWq, -DHq)}          # marker centers
ring_gt = np.array([(-DWc / 2, DHc / 2), (DWc / 2, DHc / 2),
                    (DWc / 2, -DHc / 2), (-DWc / 2, -DHc / 2)])                    # ring centers

img = cv2.imread(img_path)
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
try:
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.ArucoDetector(dic, params).detectMarkers(img)
except AttributeError:
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(img, dic, parameters=params)
ids = ids.flatten().tolist()

# ---------- (A) independent per-marker pose ----------
objm = np.array([[-MS / 2, MS / 2, 0], [MS / 2, MS / 2, 0],
                 [MS / 2, -MS / 2, 0], [-MS / 2, -MS / 2, 0]], np.float32)
pos = {}
for c, i in zip(corners, ids):
    ok, rv, tv = cv2.solvePnP(objm, c[0], K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    pos[int(i)] = tv.flatten()

def gt_dist(a, b, table):
    pa, pb = np.array(table[a]), np.array(table[b])
    return np.linalg.norm(pa - pb)

print("=== (A) 4 markers reconstructed INDEPENDENTLY (per-marker solvePnP) ===")
print("  pair    GT(mm)   meas(mm)   diff(mm)   err%")
mk_err = []
for a, b in [(1, 2), (3, 4), (1, 3), (2, 4), (1, 4), (2, 3)]:
    if a in pos and b in pos:
        gt = gt_dist(a, b, mk_gt) * 1000
        me = np.linalg.norm(pos[a] - pos[b]) * 1000
        print("  %d-%d   %8.1f  %8.1f   %+7.1f   %+5.2f%%" % (a, b, gt, me, me - gt, 100 * (me - gt) / gt))
        mk_err.append((me - gt) / gt)
if mk_err:
    print("  mean |err| = %.2f%%   (scale bias %+.2f%%)" %
          (100 * np.mean(np.abs(mk_err)), 100 * np.mean(mk_err)))

# ---------- (B) board pose (all markers) + ring back-projection ----------
objp, imgp = [], []
for c, i in zip(corners, ids):
    if int(i) not in mk_gt:
        continue
    xc, yc = mk_gt[int(i)]
    sq = [(-1, 1), (1, 1), (1, -1), (-1, -1)]
    for j, (sx, sy) in enumerate(sq):
        objp.append([xc + sx * MS / 2, yc + sy * MS / 2, 0.0])
        imgp.append(c[0][j])
objp = np.array(objp, np.float32); imgp = np.array(imgp, np.float32)
ok, rvec, tvec = cv2.solvePnP(objp, imgp, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
R, _ = cv2.Rodrigues(rvec); t = tvec.flatten()
n = R @ np.array([0, 0, 1.0])          # board-plane normal in camera frame
p0 = t                                  # a point on the plane (board origin)

# detect the 4 ring centers in the image: bright blobs INSIDE the marker rectangle
mc_px = {int(i): c[0].mean(0) for c, i in zip(corners, ids)}
hull = np.array([mc_px[k] for k in (1, 2, 4, 3) if k in mc_px], np.float32)
mask = np.zeros(img.shape[:2], np.uint8)
cv2.fillConvexPoly(mask, cv2.convexHull(hull).astype(int), 255)
# board diagonal in px to scale the erosion / ring-size gates
diag_px = np.linalg.norm(mc_px[1] - mc_px[4]) if (1 in mc_px and 4 in mc_px) else 800.0
er = max(5, int(0.10 * diag_px))                        # erode ~10% of board diag to drop markers
mask = cv2.erode(mask, np.ones((er, er), np.uint8))
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
bright = cv2.inRange(gray, 140, 255)
bright = cv2.bitwise_and(bright, mask)
bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
cnts, _ = cv2.findContours(bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
# fit ellipse to each blob; a ring's outer contour -> center = ring center. gate on size.
cand = []
for c in cnts:
    if len(c) < 5 or cv2.contourArea(c) < 500:
        continue
    (u, v), (MA, ma), ang = cv2.fitEllipse(c)
    d = 0.5 * (MA + ma)
    if 0.04 * diag_px < d < 0.30 * diag_px and ma / max(MA, 1) > 0.45:
        cand.append((cv2.contourArea(c), (u, v)))
cand = sorted(cand, reverse=True, key=lambda x: x[0])[:4]
Kinv = np.linalg.inv(K)
dbg = img.copy()
rings3d = []
for _, (u, v) in cand:
    cv2.circle(dbg, (int(u), int(v)), 10, (0, 255, 0), 2)
    upt = cv2.undistortPoints(np.array([[[u, v]]], np.float32), K, D, P=K)[0, 0]
    ray = Kinv @ np.array([upt[0], upt[1], 1.0]); ray /= np.linalg.norm(ray)
    lam = np.dot(n, p0) / np.dot(n, ray)      # ray-plane intersection (camera origin)
    rings3d.append(lam * ray)
rings3d = np.array(rings3d)
cv2.imwrite("/tmp/ring_detect_dbg.png", dbg)

print("\n=== (B) 4 ring centers back-projected from DETECTED pixels onto the board plane ===")
print("  detected %d ring blobs (debug img -> /tmp/ring_detect_dbg.png)" % len(rings3d))
if len(rings3d) == 4:
    ds = sorted(np.linalg.norm(rings3d[i] - rings3d[j]) * 1000
                for i, j in itertools.combinations(range(4), 2))
    gts = sorted([DHc, DHc, DWc, DWc,
                  np.hypot(DWc, DHc), np.hypot(DWc, DHc)])
    gts = [g * 1000 for g in gts]
    print("  sorted pairwise distances:")
    print("  GT(mm)   meas(mm)   diff(mm)   err%")
    errs = []
    for gt, me in zip(gts, ds):
        print("  %7.1f  %8.1f   %+7.1f   %+5.2f%%" % (gt, me, me - gt, 100 * (me - gt) / gt))
        errs.append((me - gt) / gt)
    print("  mean |err| = %.2f%%   (scale bias %+.2f%%)" %
          (100 * np.mean(np.abs(errs)), 100 * np.mean(errs)))
