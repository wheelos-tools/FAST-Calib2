#!/usr/bin/env python3
"""Per-scene QA for a FAST-Calib2 extrinsic: project the LiDAR into the camera image
and produce (1) an overlay PNG (points colored by intensity; reflective=red) and
(2) a colored PCD (LiDAR points painted with the camera pixel color).

Works with the single- OR multi-scene result file. The multi result has no
intrinsics, so pass --config to read fx/fy/cx/cy/k1/k2/p1/p2 from the camera yaml.

  render_scene_qa.py <scene_dir> <result.txt> --config <cam.yaml> \
      --overlay out.png --colored out.pcd
"""
import argparse
import re
import numpy as np
import cv2


def grab(txt, key):
    m = re.search(key + r":\s*([-\d.eE]+)", txt)
    return float(m.group(1)) if m else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene_dir")
    ap.add_argument("result")
    ap.add_argument("--config", required=True)
    ap.add_argument("--overlay", required=True)
    ap.add_argument("--colored", default=None,
                    help="optional colored-PCD output path")
    a = ap.parse_args()

    et = open(a.result).read()
    fx = grab(et, "cam_fx")
    if fx is not None:
        fy, cx, cy = grab(et, "cam_fy"), grab(et, "cam_cx"), grab(et, "cam_cy")
        k1, k2, p1, p2 = grab(et, "cam_d0"), grab(et, "cam_d1"), grab(et, "cam_d2"), grab(et, "cam_d3")
    else:
        cfg = open(a.config).read()
        fx, fy, cx, cy = grab(cfg, "fx"), grab(cfg, "fy"), grab(cfg, "cx"), grab(cfg, "cy")
        k1, k2, p1, p2 = grab(cfg, "k1"), grab(cfg, "k2"), grab(cfg, "p1"), grab(cfg, "p2")
    R = np.array(re.search(r"Rcl:\s*\[([^\]]+)\]", et).group(1).split(","), float).reshape(3, 3)
    t = np.array(re.search(r"Pcl:\s*\[([^\]]+)\]", et).group(1).split(","), float)
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    D = np.array([k1, k2, p1, p2, 0.0])

    img = cv2.undistort(cv2.imread(a.scene_dir + "/image.png"), K, D)
    Him, Wim = img.shape[:2]

    P, I = [], []
    with open(a.scene_dir + "/cloud.pcd") as f:
        d = False
        for ln in f:
            if d:
                v = ln.split()
                if len(v) >= 4:
                    P.append((float(v[0]), float(v[1]), float(v[2]))); I.append(float(v[3]))
            elif ln.startswith("DATA"):
                d = True
    P = np.asarray(P); I = np.asarray(I)

    Pc = (R @ P.T).T + t
    m = Pc[:, 2] > 0.05
    P, Pc, I = P[m], Pc[m], I[m]
    u = fx * Pc[:, 0] / Pc[:, 2] + cx
    v = fy * Pc[:, 1] / Pc[:, 2] + cy
    inb = (u >= 0) & (u < Wim) & (v >= 0) & (v < Him)
    u, v, I, Plid = u[inb].astype(int), v[inb].astype(int), I[inb], P[inb]

    # overlay: points colored by intensity (jet), saturated over-drawn red
    ov = img.copy()
    col = cv2.applyColorMap(np.clip(I, 0, 255).astype(np.uint8).reshape(-1, 1),
                            cv2.COLORMAP_JET).reshape(-1, 3)
    for ui, vi, c in zip(u, v, col):
        cv2.circle(ov, (ui, vi), 1, (int(c[0]), int(c[1]), int(c[2])), -1)
    for ui, vi in zip(u[I > 200], v[I > 200]):
        cv2.circle(ov, (ui, vi), 2, (0, 0, 255), -1)
    cv2.imwrite(a.overlay, cv2.addWeighted(ov, 0.85, img, 0.15, 0))

    # colored PCD in the LiDAR frame, painted with the camera pixel color
    n = len(Plid)
    if a.colored:
        bgr = img[v, u]
        rgb = (bgr[:, 2].astype(np.uint32) << 16) | (bgr[:, 1].astype(np.uint32) << 8) | bgr[:, 0].astype(np.uint32)
        with open(a.colored, "w") as fh:
            fh.write("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\n"
                     "COUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n" % (n, n))
            for (x, y, z), c in zip(Plid, rgb):
                fh.write("%.4f %.4f %.4f %d\n" % (x, y, z, c))
        print("overlay %s (%d pts) + colored %s" % (a.overlay, n, a.colored))
    else:
        print("overlay %s (%d pts)" % (a.overlay, n))


if __name__ == "__main__":
    main()
