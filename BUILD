# Bazel build for the Apollo workspace.
#
# FAST-Calib2 compiles inside the Apollo dev environment using the workspace's
# own third-party modules (pcl 1.15 as @local_config_pcl, opencv 4.13, eigen)
# — no ROS, no system PCL/OpenCV needed. scripts/apollo_build.sh syncs this
# repo to <apollo>/modules/calibration/fast_calib and builds these targets
# there:
#
#   bazel build //modules/calibration/fast_calib:all

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

FAST_CALIB_COPTS = ["-std=c++17"]

cc_library(
    name = "fast_calib_lib",
    hdrs = glob([
        "include/*.h",
        "src/*.hpp",
    ]),
    copts = FAST_CALIB_COPTS,
    # Common::Point is a custom point type: PCL templates must build
    # header-only in every dependent TU.
    defines = ["PCL_NO_PRECOMPILE"],
    includes = [
        "include",
        "src",
    ],
    deps = [
        "@eigen",
        "@local_config_pcl//:common",
        "@local_config_pcl//:features",
        "@local_config_pcl//:filters",
        "@local_config_pcl//:io",
        "@local_config_pcl//:kdtree",
        "@local_config_pcl//:registration",
        "@local_config_pcl//:sample_consensus",
        "@local_config_pcl//:search",
        "@local_config_pcl//:segmentation",
        "@opencv//:opencv",
    ],
)

cc_binary(
    name = "fast_calib",
    srcs = ["src/main.cpp"],
    copts = FAST_CALIB_COPTS,
    deps = [":fast_calib_lib"],
)

cc_binary(
    name = "multi_fast_calib",
    srcs = ["src/multi_scene.cpp"],
    copts = FAST_CALIB_COPTS,
    deps = [":fast_calib_lib"],
)

cc_binary(
    name = "lidar_center_test",
    srcs = ["src/lidar_center_test.cpp"],
    copts = FAST_CALIB_COPTS,
    deps = [":fast_calib_lib"],
)
