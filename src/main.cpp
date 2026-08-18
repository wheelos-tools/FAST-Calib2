/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.

ROS-free single-scene LiDAR-camera calibration.

  fast_calib --config config/cameras/<cam>.yaml --scene calib_data/<cam>/<scene> \
             [--output output/<cam>] [--debug-dir <dir>] [--dump-pcd <path>]

--scene resolves image.png plus the cloud source inside the directory
(record/ with cyber record files preferred, else cloud.pcd) and auto-applies a
pick_roi.py-generated cloud_roi.txt when present. --cloud/--image override the
resolved paths individually.
*/

#include <sys/stat.h>
#include <sys/types.h>

#include "apollo_extrinsics.hpp"
#include "data_preprocess.hpp"
#include "lidar_detect.hpp"
#include "qr_detect.hpp"

namespace {

void usage(const char* prog) {
  std::cout
      << "Usage: " << prog << " --config <cam.yaml> [options]\n"
      << "  --config <yaml>    per-camera config (intrinsics, board, ROI)\n"
      << "  --scene <dir>      scene directory: image.png + record/|cloud.pcd\n"
      << "  --cloud <path>     cloud source: .pcd, .record file, or record dir\n"
      << "  --image <path>     camera image\n"
      << "  --output <dir>     result directory (default: config output_path)\n"
      << "  --debug-dir <dir>  write intermediate clouds as PCDs\n"
      << "  --dump-pcd <path>  write the fused input cloud as ASCII PCD\n"
      << "  --no-roi-file      ignore <scene>/cloud_roi.txt\n";
}

bool fileExists(const std::string& p) {
  struct stat st;
  return ::stat(p.c_str(), &st) == 0;
}

bool isDir(const std::string& p) {
  struct stat st;
  return ::stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

void ensureDir(const std::string& path) {
  std::string partial;
  for (size_t i = 0; i < path.size(); ++i) {
    partial += path[i];
    if (path[i] == '/' || i + 1 == path.size())
      ::mkdir(partial.c_str(), 0755);  // EEXIST is fine
  }
}

template <typename CloudT>
void dumpDebugCloud(const std::string& dir, const std::string& name,
                    const CloudT& cloud) {
  if (!cloud || cloud->empty()) return;
  std::string path = dir + "/" + name + ".pcd";
  if (pcl::io::savePCDFileBinary(path, *cloud) == 0)
    std::cout << "[Debug] Saved " << path << " (" << cloud->size()
              << " points)" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path, scene_dir, cloud_path, image_path, output_path;
  std::string debug_dir, dump_pcd;
  bool use_roi_file = true;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](const char* opt) -> std::string {
      if (i + 1 >= argc) {
        LOG_ERROR("Missing value for %s", opt);
        std::exit(1);
      }
      return argv[++i];
    };
    if (a == "--config") config_path = next("--config");
    else if (a == "--scene") scene_dir = next("--scene");
    else if (a == "--cloud") cloud_path = next("--cloud");
    else if (a == "--image") image_path = next("--image");
    else if (a == "--output") output_path = next("--output");
    else if (a == "--debug-dir") debug_dir = next("--debug-dir");
    else if (a == "--dump-pcd") dump_pcd = next("--dump-pcd");
    else if (a == "--no-roi-file") use_roi_file = false;
    else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
    else { LOG_ERROR("Unknown argument: %s", a.c_str()); usage(argv[0]); return 1; }
  }
  if (config_path.empty()) {
    usage(argv[0]);
    return 1;
  }

  // 读取参数
  Params params = loadParameters(config_path);

  if (!scene_dir.empty()) {
    if (scene_dir.back() == '/') scene_dir.pop_back();
    params.image_path = scene_dir + "/image.png";
    if (isDir(scene_dir + "/record") &&
        !DataPreprocess::listRecordFiles(scene_dir + "/record").empty())
      params.cloud_path = scene_dir + "/record";
    else if (fileExists(scene_dir + "/cloud.pcd"))
      params.cloud_path = scene_dir + "/cloud.pcd";
    else
      params.cloud_path = scene_dir;  // scene dir itself may hold record files
    std::string roi_file = scene_dir + "/cloud_roi.txt";
    if (use_roi_file && fileExists(roi_file) && applyRoiFile(roi_file, params))
      LOG_INFO("Applied scene ROI from %s (manual-ROI mode).",
               roi_file.c_str());
  }
  if (!cloud_path.empty()) params.cloud_path = cloud_path;
  if (!image_path.empty()) params.image_path = image_path;
  if (!output_path.empty()) params.output_path = output_path;
  ensureDir(params.output_path);

  // 读取图像和点云
  DataPreprocessPtr dataPreprocessPtr(new DataPreprocess(params));
  if (!dataPreprocessPtr->ok()) return 1;

  if (!dump_pcd.empty()) {
    if (dataPreprocessPtr->dumpFusedPcd(dump_pcd))
      LOG_INFO("Dumped fused input cloud to %s", dump_pcd.c_str());
    else
      LOG_WARN("Failed to dump fused cloud to %s", dump_pcd.c_str());
  }

  QRDetectPtr qrDetectPtr(new QRDetect(params));
  LidarDetectPtr lidarDetectPtr(new LidarDetect(params));

  cv::Mat img_input = dataPreprocessPtr->img_input_;
  pcl::PointCloud<Common::Point>::Ptr cloud_input =
      dataPreprocessPtr->cloud_input_;

  // 检测 QR 码
  PointCloud<PointXYZ>::Ptr qr_center_cloud(new PointCloud<PointXYZ>);
  qr_center_cloud->reserve(4);
  qrDetectPtr->detect_qr(img_input, qr_center_cloud);

  // 检测 LiDAR 数据
  PointCloud<PointXYZ>::Ptr lidar_center_cloud(new PointCloud<PointXYZ>);
  lidar_center_cloud->reserve(4);

  switch (dataPreprocessPtr->lidar_type_) {
    case LiDARType::Solid:
      lidarDetectPtr->detect_solid_lidar(cloud_input, lidar_center_cloud);
      break;
    case LiDARType::Mech:
      lidarDetectPtr->detect_mech_lidar(cloud_input, lidar_center_cloud);
      break;
    default:
      std::cerr << BOLDYELLOW << "[Main] Unknown LiDAR type." << RESET
                << std::endl;
      break;
  }

  // 调试输出：中间点云落盘（替代原 RViz 发布）
  if (!debug_dir.empty()) {
    ensureDir(debug_dir);
    dumpDebugCloud(debug_dir, "filtered_cloud",
                   lidarDetectPtr->getFilteredCloud());
    dumpDebugCloud(debug_dir, "plane_cloud", lidarDetectPtr->getPlaneCloud());
    dumpDebugCloud(debug_dir, "annulus_cloud",
                   lidarDetectPtr->getAnnulusOriginalCloud());
    dumpDebugCloud(debug_dir, "boundary_cloud",
                   lidarDetectPtr->getBoundaryOriginalCloud());
    dumpDebugCloud(debug_dir, "aligned_cloud",
                   lidarDetectPtr->getAlignedCloud());
    dumpDebugCloud(debug_dir, "edge_cloud", lidarDetectPtr->getEdgeCloud());
    dumpDebugCloud(debug_dir, "center_z0_cloud",
                   lidarDetectPtr->getCenterZ0Cloud());
  }

  if (qr_center_cloud->size() != 4 || lidar_center_cloud->size() != 4) {
    LOG_ERROR(
        "Detection failed: %zu camera centers, %zu lidar centers (need 4+4).",
        qr_center_cloud->size(), lidar_center_cloud->size());
    return 2;
  }

  // 对 QR 和 LiDAR 检测到的圆心进行排序
  PointCloud<PointXYZ>::Ptr qr_centers(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr lidar_centers(new PointCloud<PointXYZ>);
  sortPatternCenters(qr_center_cloud, qr_centers, "camera");
  sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");

  validateTargetGeometry(qr_centers, params.delta_width_circles,
                         params.delta_height_circles, "QR");
  validateTargetGeometry(lidar_centers, params.delta_width_circles,
                         params.delta_height_circles, "LiDAR");

  // 保存中间结果：排序后的 LiDAR 圆心和 QR 圆心
  saveTargetHoleCenters(lidar_centers, qr_centers, params);

  // 计算外参
  Eigen::Matrix4f transformation;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>
      svd;
  svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

  // 将 LiDAR 圆心转换到相机坐标系评估配准残差
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(
      new pcl::PointCloud<pcl::PointXYZ>);
  aligned_lidar_centers->reserve(lidar_centers->size());
  alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);

  double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
  if (rmse > 0) {
    std::cout << BOLDYELLOW << "[Result] RMSE: " << BOLDRED << std::fixed
              << std::setprecision(4) << rmse << " m" << RESET << std::endl;
  }

  std::cout << BOLDYELLOW
            << "[Result] Single-scene calibration: extrinsic parameters "
               "T_cam_lidar = "
            << RESET << std::endl;
  std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation
            << RESET << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  projectPointCloudToImage(cloud_input, transformation,
                           qrDetectPtr->cameraMatrix_, qrDetectPtr->distCoeffs_,
                           img_input, colored_cloud);

  saveCalibrationResults(params, transformation, colored_cloud,
                         qrDetectPtr->imageCopy_);

  // Apollo 格式外参输出
  std::string out_dir = params.output_path;
  if (out_dir.back() != '/') out_dir += '/';
  if (writeApolloExtrinsics(out_dir + "single_calib_extrinsics.yaml",
                            transformation.cast<double>(), params.lidar_frame,
                            params.camera_frame))
    std::cout << BOLDYELLOW << "[Result] Apollo extrinsics saved to "
              << BOLDWHITE << out_dir << "single_calib_extrinsics.yaml"
              << RESET << std::endl;

  if (!debug_dir.empty()) {
    dumpDebugCloud(debug_dir, "aligned_lidar_centers", aligned_lidar_centers);
    dumpDebugCloud(debug_dir, "qr_centers", qr_centers);
    dumpDebugCloud(debug_dir, "lidar_centers", lidar_centers);
    dumpDebugCloud(debug_dir, "colored_cloud", colored_cloud);
  }

  return 0;
}
