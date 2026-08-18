/*
ROS-free parameter loading for FAST-Calib2.

Replaces the rosparam-based loadParameters(ros::NodeHandle&): the per-camera
config (config/cameras/<cam>.yaml, or config/qr_params.yaml) is a flat
"key: value" YAML document, parsed here with a small self-contained reader —
no yaml-cpp / roslaunch required. Later occurrences of a key override earlier
ones (same behavior rosparam load had). `$(find fast_calib)` in path values is
substituted with the repository root inferred from the config file location.
*/

#ifndef FAST_CALIB_PARAMS_H
#define FAST_CALIB_PARAMS_H

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "log.h"

struct Params {
  double x_min, x_max, y_min, y_max, z_min, z_max;
  bool use_auto_lidar_roi;
  double fx, fy, cx, cy, k1, k2, p1, p2;
  double marker_size, delta_width_qr_center, delta_height_qr_center;
  double delta_width_circles, delta_height_circles, circle_radius,
      annulus_half_width;
  double board_width, board_height, board_roi_margin, board_roi_depth;
  double auto_roi_voxel_leaf, annulus_voxel_leaf;
  int min_detected_markers;
  std::string image_path;
  std::string cloud_path;   // .pcd file, a cyber .record file, or a directory
                            // of record segments (was: bag_path)
  std::string lidar_topic;  // legacy name; used as the record channel if
                            // lidar_channel is not set
  std::string output_path;

  // Apollo-native input/output additions
  std::string lidar_channel;  // e.g. /apollo/sensor/hesai/main_front/PointCloud2
  int max_fusion_frames = 0;  // frames fused from the record (0 = all; the
                              // scene is static during the short capture)
  std::vector<double> beam_altitudes_deg;  // non-empty => synthesize ring
                                           // (mechanical pipeline)
  std::string lidar_frame = "lidar";
  std::string camera_frame = "camera";

  // LiDAR-detector tuning (was: hard-coded in lidar_detect.hpp; per-rig values
  // now live in the camera config). Defaults keep the committed behavior.
  double board_plane_inlier_threshold = 0.07;     // board plane gate [m]
  double annulus_plane_inlier_threshold = 0.07;   // solid: bright-ring gate [m]
  double boundary_plane_inlier_threshold = 0.03;  // mech: ring-boundary gate [m]
  double annulus_cluster_tolerance = 0.10;        // solid annulus clustering
  int annulus_cluster_min_size = 30;
  double mech_cluster_tolerance = 0.09;           // mech boundary clustering
  int mech_cluster_min_size = 80;
  int auto_roi_cluster_min_size = 200;            // auto-ROI board clustering

  const std::string& channel() const {
    return lidar_channel.empty() ? lidar_topic : lidar_channel;
  }
};

namespace params_detail {

inline std::string trim(const std::string& s) {
  size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  size_t e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

inline std::string stripQuotes(const std::string& s) {
  if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                        (s.front() == '\'' && s.back() == '\'')))
    return s.substr(1, s.size() - 2);
  return s;
}

// Directory part of a path ("" -> ".")
inline std::string dirName(const std::string& path) {
  size_t pos = path.find_last_of('/');
  if (pos == std::string::npos) return ".";
  if (pos == 0) return "/";
  return path.substr(0, pos);
}

// Parse a flat "key: value" YAML file. Comments (#...) are stripped; nested
// structure is not supported (the FAST-Calib2 configs are flat).
inline bool parseFlatYaml(const std::string& path,
                          std::map<std::string, std::string>& kv) {
  std::ifstream fin(path);
  if (!fin.is_open()) return false;
  std::string line;
  while (std::getline(fin, line)) {
    size_t hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    line = trim(line);
    if (line.empty()) continue;
    size_t colon = line.find(':');
    if (colon == std::string::npos) continue;
    std::string key = trim(line.substr(0, colon));
    std::string value = stripQuotes(trim(line.substr(colon + 1)));
    if (key.empty() || value.empty()) continue;
    kv[key] = value;  // later keys override earlier ones
  }
  return true;
}

class KV {
 public:
  KV(std::map<std::string, std::string> kv, std::string repo_root)
      : kv_(std::move(kv)), repo_root_(std::move(repo_root)) {}

  void get(const char* key, double& out, double def) const {
    auto it = kv_.find(key);
    out = (it != kv_.end()) ? std::atof(it->second.c_str()) : def;
  }
  void get(const char* key, int& out, int def) const {
    auto it = kv_.find(key);
    out = (it != kv_.end()) ? std::atoi(it->second.c_str()) : def;
  }
  void get(const char* key, bool& out, bool def) const {
    auto it = kv_.find(key);
    out = (it != kv_.end()) ? (it->second == "true" || it->second == "1")
                            : def;
  }
  void get(const char* key, std::string& out, const std::string& def) const {
    auto it = kv_.find(key);
    out = (it != kv_.end()) ? it->second : def;
    // roslaunch-style package substitution kept for config compatibility
    const std::string tok = "$(find fast_calib)";
    size_t pos;
    while ((pos = out.find(tok)) != std::string::npos)
      out.replace(pos, tok.size(), repo_root_);
  }
  // "[a, b, c]" or "a, b, c"
  void get(const char* key, std::vector<double>& out) const {
    out.clear();
    auto it = kv_.find(key);
    if (it == kv_.end()) return;
    std::string v = it->second;
    v.erase(std::remove(v.begin(), v.end(), '['), v.end());
    v.erase(std::remove(v.begin(), v.end(), ']'), v.end());
    std::stringstream ss(v);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      tok = trim(tok);
      if (!tok.empty()) out.push_back(std::atof(tok.c_str()));
    }
  }

 private:
  std::map<std::string, std::string> kv_;
  std::string repo_root_;
};

}  // namespace params_detail

// Load parameters from a flat YAML config file. Defaults match the previous
// rosparam-based loader.
inline Params loadParameters(const std::string& config_path) {
  std::map<std::string, std::string> raw;
  if (!params_detail::parseFlatYaml(config_path, raw)) {
    LOG_ERROR("Failed to open config file: %s", config_path.c_str());
    std::exit(1);
  }

  // repo root: config/cameras/<cam>.yaml -> ../../, config/<file>.yaml -> ../
  std::string cfg_dir = params_detail::dirName(config_path);
  std::string repo_root =
      (cfg_dir.size() >= 8 &&
       cfg_dir.compare(cfg_dir.size() - 8, 8, "/cameras") == 0)
          ? cfg_dir + "/../.."
          : cfg_dir + "/..";

  params_detail::KV kv(std::move(raw), repo_root);

  Params p;
  kv.get("fx", p.fx, 1215.31801774424);
  kv.get("fy", p.fy, 1214.72961288138);
  kv.get("cx", p.cx, 1047.86571859677);
  kv.get("cy", p.cy, 745.068353101898);
  kv.get("k1", p.k1, -0.33574781188503);
  kv.get("k2", p.k2, 0.10996870793601);
  kv.get("p1", p.p1, 0.000157303079833973);
  kv.get("p2", p.p2, 0.000544930726278493);
  kv.get("marker_size", p.marker_size, 0.2);
  kv.get("delta_width_qr_center", p.delta_width_qr_center, 0.55);
  kv.get("delta_height_qr_center", p.delta_height_qr_center, 0.35);
  kv.get("delta_width_circles", p.delta_width_circles, 0.5);
  kv.get("delta_height_circles", p.delta_height_circles, 0.4);
  kv.get("min_detected_markers", p.min_detected_markers, 3);
  kv.get("circle_radius", p.circle_radius, 0.12);
  kv.get("annulus_half_width", p.annulus_half_width, 0.025);
  kv.get("board_width", p.board_width, 1.4);
  kv.get("board_height", p.board_height, 1.0);
  kv.get("board_roi_margin", p.board_roi_margin, 0.08);
  kv.get("board_roi_depth", p.board_roi_depth, 0.12);
  kv.get("auto_roi_voxel_leaf", p.auto_roi_voxel_leaf, 0.01);
  kv.get("annulus_voxel_leaf", p.annulus_voxel_leaf, 0.005);
  kv.get("image_path", p.image_path, "");
  kv.get("cloud_path", p.cloud_path, "");
  if (p.cloud_path.empty()) kv.get("bag_path", p.cloud_path, "");  // legacy key
  kv.get("lidar_topic", p.lidar_topic, "/livox/lidar");
  kv.get("output_path", p.output_path, "output");
  kv.get("use_auto_lidar_roi", p.use_auto_lidar_roi, false);
  kv.get("x_min", p.x_min, 1.5);
  kv.get("x_max", p.x_max, 3.0);
  kv.get("y_min", p.y_min, -1.5);
  kv.get("y_max", p.y_max, 2.0);
  kv.get("z_min", p.z_min, -0.5);
  kv.get("z_max", p.z_max, 2.0);
  kv.get("lidar_channel", p.lidar_channel, "");
  kv.get("max_fusion_frames", p.max_fusion_frames, 0);
  kv.get("beam_altitudes_deg", p.beam_altitudes_deg);
  kv.get("lidar_frame", p.lidar_frame, "lidar");
  kv.get("camera_frame", p.camera_frame, "camera");
  kv.get("board_plane_inlier_threshold", p.board_plane_inlier_threshold, 0.07);
  kv.get("annulus_plane_inlier_threshold", p.annulus_plane_inlier_threshold,
         0.07);
  kv.get("boundary_plane_inlier_threshold", p.boundary_plane_inlier_threshold,
         0.03);
  kv.get("annulus_cluster_tolerance", p.annulus_cluster_tolerance, 0.10);
  kv.get("annulus_cluster_min_size", p.annulus_cluster_min_size, 30);
  kv.get("mech_cluster_tolerance", p.mech_cluster_tolerance, 0.09);
  kv.get("mech_cluster_min_size", p.mech_cluster_min_size, 80);
  kv.get("auto_roi_cluster_min_size", p.auto_roi_cluster_min_size, 200);
  return p;
}

// Apply a pick_roi.py-style ROI file (flat "x_min: ..." lines) on top of the
// loaded params; forces the manual-ROI path. Returns false if unreadable.
inline bool applyRoiFile(const std::string& roi_path, Params& p) {
  std::map<std::string, std::string> raw;
  if (!params_detail::parseFlatYaml(roi_path, raw)) return false;
  params_detail::KV kv(std::move(raw), ".");
  kv.get("x_min", p.x_min, p.x_min);
  kv.get("x_max", p.x_max, p.x_max);
  kv.get("y_min", p.y_min, p.y_min);
  kv.get("y_max", p.y_max, p.y_max);
  kv.get("z_min", p.z_min, p.z_min);
  kv.get("z_max", p.z_max, p.z_max);
  p.use_auto_lidar_roi = false;
  return true;
}

#endif  // FAST_CALIB_PARAMS_H
