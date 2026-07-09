#!/usr/bin/env python3
"""Convert an ASCII PCD (x y z intensity) into a single-message ROS bag holding a
sensor_msgs/PointCloud2, for FAST-Calib2. Runs INSIDE the ROS Noetic container.

For a mechanical LiDAR FAST-Calib2 wants a `ring` field (it then uses the
ring-based mechanical pipeline). Apollo's cloud has no ring, so we reconstruct it
from each point's elevation angle against the sensor's nominal beam table
(Vanjee WLR-720-16: 16 beams, -16 deg .. +14 deg, 2 deg spacing). Pass
--no-ring to emit x/y/z/intensity only (FAST-Calib2 then uses the solid pipeline).
"""
import argparse
import math
import struct

import rosbag
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

# Vanjee WLR-720-16 nominal vertical beam angles (deg), top..bottom.
BEAMS_DEG = [14 - 2 * i for i in range(16)]  # +14 .. -16


def nearest_ring(x, y, z):
    elev = math.degrees(math.atan2(z, math.hypot(x, y)))
    best_i, best_d = 0, 1e9
    for i, a in enumerate(BEAMS_DEG):
        d = abs(elev - a)
        if d < best_d:
            best_d, best_i = d, i
    return best_i  # 0 = top beam (+14), 15 = bottom beam (-16)


def read_pcd(path):
    pts = []
    with open(path) as fh:
        in_data = False
        for line in fh:
            if not in_data:
                if line.startswith("DATA"):
                    in_data = True
                continue
            f = line.split()
            if len(f) < 4:
                continue
            pts.append((float(f[0]), float(f[1]), float(f[2]), float(f[3])))
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pcd", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="/vanjee_points")
    ap.add_argument("--frame", default="vanjeelidar_up")
    ap.add_argument("--no-ring", action="store_true")
    args = ap.parse_args()

    pts = read_pcd(args.pcd)
    if not pts:
        raise SystemExit("no points read from " + args.pcd)

    with_ring = not args.no_ring
    if with_ring:
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
            PointField("ring", 16, PointField.UINT16, 1),
        ]
        point_step = 18
        buf = bytearray()
        for x, y, z, i in pts:
            buf += struct.pack("<ffffH", x, y, z, i, nearest_ring(x, y, z))
    else:
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ]
        point_step = 16
        buf = bytearray()
        for x, y, z, i in pts:
            buf += struct.pack("<ffff", x, y, z, i)

    msg = PointCloud2()
    msg.header = Header(frame_id=args.frame, stamp=rospy.Time(0, 0))
    msg.height = 1
    msg.width = len(pts)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * len(pts)
    msg.is_dense = True
    msg.data = bytes(buf)

    with rosbag.Bag(args.bag, "w") as bag:
        bag.write(args.topic, msg, rospy.Time(1, 0))
    print("wrote %d points (ring=%s) -> %s topic=%s" %
          (len(pts), with_ring, args.bag, args.topic))


if __name__ == "__main__":
    main()
