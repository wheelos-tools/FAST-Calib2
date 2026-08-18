/*
ROS-free data loading for FAST-Calib2.

Replaces the rosbag-based DataPreprocess: the point cloud now comes either
straight from Apollo Cyber RT record files (cyber_recorder output, parsed by
cyber_record_reader.hpp) or from a PCD file. Multi-frame fusion of a static
scene happens here (was: scripts/record_to_pcd.py), as does optional scan-ring
synthesis from a nominal beam-altitude table (was: scripts/pcd_to_bag.py).

Cloud source resolution for Params::cloud_path:
  - "*.pcd"                       -> PCD loader (ring column honored if present)
  - a directory                   -> all "rec.*" / "*.record*" files inside
  - any other file                -> treated as a single cyber record file

LiDAR type: mechanical when ring information exists (from the file or
synthesized via beam_altitudes_deg), solid otherwise — same rule the rosbag
path applied.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include "common_lib.h"  // first: defines PCL_NO_PRECOMPILE before PCL headers

#include <algorithm>
#include <cmath>
#include <dirent.h>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cyber_record_reader.hpp"

using namespace std;

enum class LiDARType : int {
  Unknown = 0,
  Solid = 1,  // 固态（如 Livox / AT128 走 solid 管线）
  Mech = 2    // 机械式多线（有 ring 线号）
};

class DataPreprocess {
 public:
  pcl::PointCloud<Common::Point>::Ptr cloud_input_;
  cv::Mat img_input_;
  LiDARType lidar_type_{LiDARType::Unknown};
  LiDARType lidarType() const { return lidar_type_; }
  bool ok() const { return ok_; }

  explicit DataPreprocess(const Params& params, bool need_image = true)
      : cloud_input_(new pcl::PointCloud<Common::Point>) {
    if (need_image) {
      img_input_ = cv::imread(params.image_path, cv::IMREAD_UNCHANGED);
      if (img_input_.empty()) {
        LOG_ERROR("Loading the image %s failed", params.image_path.c_str());
        return;
      }
    }

    if (params.cloud_path.empty()) {
      LOG_ERROR("No cloud_path given (config cloud_path / --cloud / --scene).");
      return;
    }

    bool has_ring = false;
    if (endsWith(params.cloud_path, ".pcd")) {
      if (!loadFromPcd(params.cloud_path, has_ring)) return;
    } else {
      if (!loadFromRecords(params)) return;
    }

    if (!params.beam_altitudes_deg.empty()) {
      synthesizeRings(params.beam_altitudes_deg);
      has_ring = true;
    }
    lidar_type_ = has_ring ? LiDARType::Mech : LiDARType::Solid;

    LOG_INFO("Loaded %zu points from %s (%s pipeline).", cloud_input_->size(),
             params.cloud_path.c_str(),
             lidar_type_ == LiDARType::Mech ? "mechanical" : "solid");
    ok_ = !cloud_input_->empty();
  }

  // Write the fused cloud as an ASCII PCD (x y z intensity) — feeds the
  // pick_roi.py / QA python tooling, replacing scripts/record_to_pcd.py.
  bool dumpFusedPcd(const std::string& path) const {
    std::ofstream fh(path);
    if (!fh.is_open()) return false;
    fh << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
       << "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
       << "COUNT 1 1 1 1\nWIDTH " << cloud_input_->size()
       << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "
       << cloud_input_->size() << "\nDATA ascii\n";
    char line[128];
    for (const auto& p : *cloud_input_) {
      std::snprintf(line, sizeof(line), "%.5f %.5f %.5f %.1f\n", p.x, p.y, p.z,
                    p.intensity);
      fh << line;
    }
    return true;
  }

 private:
  bool ok_ = false;

  static bool endsWith(const std::string& s, const std::string& suf) {
    return s.size() >= suf.size() &&
           s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
  }

  static bool isDirectory(const std::string& path) {
    DIR* d = opendir(path.c_str());
    if (d) closedir(d);
    return d != nullptr;
  }

  static std::vector<std::string> listRecordFiles(const std::string& dir) {
    std::vector<std::string> files;
    DIR* d = opendir(dir.c_str());
    if (!d) return files;
    while (struct dirent* e = readdir(d)) {
      std::string name = e->d_name;
      if (name.rfind("rec.", 0) == 0 ||
          name.find(".record") != std::string::npos)
        files.push_back(dir + "/" + name);
    }
    closedir(d);
    std::sort(files.begin(), files.end());
    return files;
  }

  bool loadFromRecords(const Params& params) {
    std::vector<std::string> files;
    if (isDirectory(params.cloud_path))
      files = listRecordFiles(params.cloud_path);
    else
      files.push_back(params.cloud_path);
    if (files.empty()) {
      LOG_ERROR("No record files (rec.* / *.record*) found in %s",
                params.cloud_path.c_str());
      return false;
    }

    const std::string& channel = params.channel();
    std::set<std::string> channels_seen;
    size_t frames = cyber_record::Reader::ReadPointClouds(
        files, channel,
        params.max_fusion_frames > 0 ? (size_t)params.max_fusion_frames : 0,
        [&](cyber_record::ApolloPointCloud&& frame) {
          cloud_input_->reserve(cloud_input_->size() + frame.points.size());
          for (const auto& q : frame.points) {
            Common::Point p;
            p.x = q.x;
            p.y = q.y;
            p.z = q.z;
            p.intensity = q.intensity;
            p.ring = 0xFFFF;
            cloud_input_->push_back(p);
          }
        },
        &channels_seen);

    if (frames == 0) {
      LOG_ERROR("No PointCloud messages on channel %s in %s", channel.c_str(),
                params.cloud_path.c_str());
      for (const auto& c : channels_seen)
        LOG_INFO("  available channel: %s", c.c_str());
      return false;
    }
    LOG_INFO("Fused %zu frames from %zu record file(s) on %s.", frames,
             files.size(), channel.c_str());
    return true;
  }

  bool loadFromPcd(const std::string& path, bool& has_ring) {
    has_ring = pcdHasField(path, "ring");
    if (has_ring) {
      pcl::PointCloud<Common::Point> cloud;
      if (pcl::io::loadPCDFile(path, cloud) != 0) {
        LOG_ERROR("Loading the PCD %s failed", path.c_str());
        return false;
      }
      *cloud_input_ = cloud;
    } else {
      pcl::PointCloud<pcl::PointXYZI> cloud;
      if (pcl::io::loadPCDFile(path, cloud) != 0) {
        LOG_ERROR("Loading the PCD %s failed", path.c_str());
        return false;
      }
      cloud_input_->reserve(cloud.size());
      for (const auto& q : cloud) {
        Common::Point p;
        p.x = q.x;
        p.y = q.y;
        p.z = q.z;
        p.intensity = q.intensity;
        p.ring = 0xFFFF;
        cloud_input_->push_back(p);
      }
    }
    // drop NaN / zero returns
    auto& pts = cloud_input_->points;
    pts.erase(std::remove_if(pts.begin(), pts.end(),
                             [](const Common::Point& p) {
                               return !(p.x == p.x && p.y == p.y &&
                                        p.z == p.z) ||
                                      (std::fabs(p.x) < 1e-6f &&
                                       std::fabs(p.y) < 1e-6f &&
                                       std::fabs(p.z) < 1e-6f);
                             }),
              pts.end());
    cloud_input_->width = pts.size();
    cloud_input_->height = 1;
    return true;
  }

  static bool pcdHasField(const std::string& path, const std::string& field) {
    std::ifstream fin(path);
    std::string line;
    while (std::getline(fin, line)) {
      if (line.rfind("FIELDS", 0) == 0)
        return (" " + line + " ").find(" " + field + " ") != std::string::npos;
      if (line.rfind("DATA", 0) == 0) break;
    }
    return false;
  }

  // Assign each point the nearest nominal beam (scan-line index) by elevation
  // angle — Apollo clouds carry no ring field (was: pcd_to_bag.py).
  void synthesizeRings(const std::vector<double>& beams_deg) {
    for (auto& p : *cloud_input_) {
      double elev =
          std::atan2(p.z, std::hypot(p.x, p.y)) * 180.0 / M_PI;
      size_t best = 0;
      double best_d = std::numeric_limits<double>::max();
      for (size_t i = 0; i < beams_deg.size(); ++i) {
        double d = std::fabs(elev - beams_deg[i]);
        if (d < best_d) {
          best_d = d;
          best = i;
        }
      }
      p.ring = (std::uint16_t)best;
    }
    LOG_INFO("Synthesized ring indices from %zu nominal beam altitudes.",
             beams_deg.size());
  }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif  // DATA_PREPROCESS_HPP
