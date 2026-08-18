/*
Writer for Apollo-format sensor extrinsics YAML.

Follows Apollo's own camera<->LiDAR extrinsics convention (verified against
modules/perception/.../camera/params/front_6mm_extrinsics.yaml and the
velodyne<->novatel examples): the LiDAR is the parent (header.frame_id), the
camera is the child (child_frame_id), and the stored transform is the CAMERA's
pose in the LiDAR frame:
    P_lidar = R_file * P_cam + t_file
The calibration solves T_cam_lidar (P_cam = R * P_lidar + t); this writer
inverts it, so the file drops into Apollo perception params as-is.
*/

#ifndef FAST_CALIB_APOLLO_EXTRINSICS_HPP
#define FAST_CALIB_APOLLO_EXTRINSICS_HPP

#include <fstream>
#include <iomanip>
#include <string>

#include <Eigen/Dense>

#include "log.h"

inline bool writeApolloExtrinsics(const std::string& path,
                                  const Eigen::Matrix4d& T_cam_lidar,
                                  const std::string& lidar_frame,
                                  const std::string& camera_frame) {
  const Eigen::Matrix3d R_cl = T_cam_lidar.block<3, 3>(0, 0);
  const Eigen::Vector3d t_cl = T_cam_lidar.block<3, 1>(0, 3);

  // Apollo convention: transform of the camera (child) in the LiDAR (parent)
  // frame -> invert T_cam_lidar.
  const Eigen::Matrix3d R = R_cl.transpose();
  const Eigen::Vector3d t = -R * t_cl;
  Eigen::Quaterniond q(R);
  q.normalize();

  std::ofstream fh(path);
  if (!fh.is_open()) {
    LOG_ERROR("Failed to open %s for writing.", path.c_str());
    return false;
  }
  fh << std::setprecision(9) << std::fixed;
  fh << "# Apollo sensor extrinsics (camera pose in the LiDAR frame):\n";
  fh << "#   P_" << lidar_frame << " = R * P_" << camera_frame << " + t\n";
  fh << "# Inverse of the calibrated T_cam_lidar (Rcl/Pcl in the result txt).\n";
  fh << "header:\n";
  fh << "  seq: 0\n";
  fh << "  stamp:\n";
  fh << "    secs: 0\n";
  fh << "    nsecs: 0\n";
  fh << "  frame_id: " << lidar_frame << "\n";
  fh << "child_frame_id: " << camera_frame << "\n";
  fh << "transform:\n";
  fh << "  translation:\n";
  fh << "    x: " << t.x() << "\n";
  fh << "    y: " << t.y() << "\n";
  fh << "    z: " << t.z() << "\n";
  fh << "  rotation:\n";
  fh << "    x: " << q.x() << "\n";
  fh << "    y: " << q.y() << "\n";
  fh << "    z: " << q.z() << "\n";
  fh << "    w: " << q.w() << "\n";
  return true;
}

#endif  // FAST_CALIB_APOLLO_EXTRINSICS_HPP
