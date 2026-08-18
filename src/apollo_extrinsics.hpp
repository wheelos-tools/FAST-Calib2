/*
Writer for Apollo-format sensor extrinsics YAML.

The file expresses T_cam_lidar, i.e. the transform that maps a point from the
LiDAR (child_frame_id) frame into the camera (header.frame_id) frame:
    P_cam = R * P_lidar + t
matching Apollo's convention for sensor extrinsics files consumed by the
transform/perception stack.
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
  Eigen::Quaterniond q(T_cam_lidar.block<3, 3>(0, 0));
  q.normalize();
  const Eigen::Vector3d t = T_cam_lidar.block<3, 1>(0, 3);

  std::ofstream fh(path);
  if (!fh.is_open()) {
    LOG_ERROR("Failed to open %s for writing.", path.c_str());
    return false;
  }
  fh << std::setprecision(9) << std::fixed;
  fh << "# Apollo sensor extrinsics: " << lidar_frame << " -> " << camera_frame
     << "\n";
  fh << "# P_" << camera_frame << " = R * P_" << lidar_frame << " + t\n";
  fh << "header:\n";
  fh << "  seq: 0\n";
  fh << "  stamp:\n";
  fh << "    secs: 0\n";
  fh << "    nsecs: 0\n";
  fh << "  frame_id: " << camera_frame << "\n";
  fh << "child_frame_id: " << lidar_frame << "\n";
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
