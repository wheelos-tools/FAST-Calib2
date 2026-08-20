# 激光雷达 ↔ 相机 外参标定指南（FAST-Calib2）

使用 FAST-Calib2 的反光环形标定板方案，标定**相机**与**激光雷达**之间的刚体变换。
点云**直接读取 Apollo Cyber record**（或 PCD 文件）—— 全流程不使用 ROS。

**产出：** `T_cam_lidar`（`P_cam = R · P_lidar + t`，FAST-LIVO2 txt 格式），以及一份
Apollo 惯例的外参 YAML（相机在雷达坐标系中的位姿），可直接放入 Apollo perception 参数目录。

**原理：** 一个场景 = 标定板的一次摆位，被两个传感器同时观测。相机检测 4 个 ArUco
标记 → 板姿态 → 4 个环心；雷达直接提取 4 个反光环心；SVD 对齐两组环心。
单场景敏感，≥3 个场景联合求解更稳健。

## 1. 部署代码

```bash
cd ~/workspace/01code                # 或任意目录
git clone https://github.com/wheelos-tools/FAST-Calib2.git
cd FAST-Calib2
git checkout feat/ros-free-apollo
```

## 2. 把代码放入 Apollo 工作空间并编译

FAST-Calib2 作为一个普通的 Apollo 模块编译（仅编译本模块，不会编译整个 Apollo）：

```bash
APOLLO=/path/to/apollo    # 你的 Apollo 工作空间

# 把代码复制进工作空间
mkdir -p $APOLLO/modules/calibration
cp -r ~/workspace/01code/FAST-Calib2 $APOLLO/modules/calibration/fast_calib

# 启动并进入 Apollo dev 容器
cd $APOLLO
bash docker/scripts/dev_start.sh      # 若 apollo_dev_* 已在运行则跳过
bash docker/scripts/dev_into.sh

# 标准 Apollo 编译命令（在容器内执行），仅编译本模块
bash apollo.sh build_opt calibration/fast_calib

# 留在容器内，直接使用 bazel-bin 里的可执行文件 —— 无需拷贝
cd /apollo/modules/calibration/fast_calib
FC=/apollo/bazel-bin/modules/calibration/fast_calib
```

首次编译会把 PCL/OpenCV 编译一遍（较慢）；之后为增量编译。**后续步骤都在容器内的
`/apollo/modules/calibration/fast_calib` 目录下进行**，通过 `$FC/...` 调用
可执行文件（`bazel-bin` 软链接在容器内保证可解析）。

> 机器上没有 Apollo？后备方案：
> `sudo apt install cmake libpcl-dev libopencv-dev libeigen3-dev`，然后
> `mkdir -p build && cd build && cmake .. && make -j`，并在下面的命令中使用
> `FC=./build`。

## 3. 运行标定

所有内容都放在模块目录（`/apollo/modules/calibration/fast_calib`）下。
一个相机、三个场景的完整布局如下 —— 数据请用真实复制，**不要用软链接**
（指向主机路径的软链接在容器内会失效）：

```
config/cameras/<cam>.yaml           # 相机配置（见下）
calib_data/<cam>/scene1/image.png   # 摆位 1 的相机帧
calib_data/<cam>/scene1/record/rec.00000   # 雷达通道的 cyber record
                                    #   （也可用 cloud.pcd 代替 record/）
calib_data/<cam>/scene1/cloud_roi.txt      # 可选的手动 ROI（pick_roi.py 生成），
                                    #   本场景自动应用
calib_data/<cam>/scene2/...         # 共 ≥3 个场景，板姿态各不相同
calib_data/<cam>/scene3/...
output/<cam>/                       # 结果输出目录
```

相机配置：复制 `config/cameras/_template.yaml`（带注释的骨架 —— 所有
`# TODO` 都必须替换；`cam0.yaml`–`cam4.yaml` 是我们 5 相机设备上填好的真实
示例）。需要填写：**该相机自身**的内参（焦距错误会直接偏移外参平移）、
实测的板几何、`lidar_channel` + `lidar_frame`/`camera_frame`、ROI。仅低线束
机械雷达需要 `beam_altitudes_deg: [...]`（AT128 / Livox 等稠密/固态雷达省略）。

单场景标定（对 ≥3 个板姿态各运行一次）：

```bash
$FC/fast_calib --config config/cameras/<cam>.yaml --scene calib_data/<cam>/<scene> --output output/<cam>
```

成功标志：相机 `4 centers found`、雷达 `Number of edge clusters: 4`、
`[Result] RMSE: 0.00xx m`（几毫米），退出码 0。

多场景联合标定（使用最近 ≥3 个场景）：

```bash
$FC/multi_fast_calib --config config/cameras/<cam>.yaml --output output/<cam>
```

采信结果前先做重投影检查 —— **每个**场景中红色高反点都必须落在 4 个白环上：

```bash
python3 scripts/render_scene_qa.py calib_data/<cam>/<scene> output/<cam>/multi_calib_result.txt --config config/cameras/<cam>.yaml --overlay output/<cam>/reproj_<scene>.png
```

结果在 `output/<cam>/`：`single/multi_calib_result.txt`（`T_cam_lidar`）与
`single/multi_calib_extrinsics.yaml`（Apollo 惯例，perception 可直接使用）。
加 `--debug-dir <dir>` 可把每个中间点云写成 PCD。

## 4. 独立的雷达环心提取测试

在完整标定前，先单独验证雷达侧的环心提取（无需相机）：

```bash
# 固态/稠密雷达（Livox、AT128 等），PCD 或 cyber record 输入：
$FC/lidar_center_test --config config/cameras/<cam>.yaml calib_data/<cam>/<scene>/cloud.pcd - solid
$FC/lidar_center_test --config config/cameras/<cam>.yaml calib_data/<cam>/<scene>/record /apollo/sensor/<lidar>/PointCloud2 solid

# 机械雷达（配置中需有 beam_altitudes_deg）：
$FC/lidar_center_test --config config/cameras/<cam>.yaml calib_data/<cam>/<scene>/cloud.pcd - mech
```

输出 `*_centers.txt`（4 个环心坐标）和 `*_debug_cloud.pcd`（板点 = 强度伪彩、
环点 = 绿色、边界点 = 红色、环心 = 白色小球）。

## 5. 数据采集与技巧

采集一个场景（RTSP 抓帧 + 录制雷达通道）：

```bash
RTSP=rtsp://user:pass@host:554/live APOLLO_C=<apollo_container> CH=/apollo/sensor/<lidar>/PointCloud2 scripts/capture_scene.sh <cam> <scene> 5
```

或一条命令跑完 ≥3 场景全流程：`scripts/multi_capture.sh <cam> 3 5`（自动 ROI）/
`scripts/pick_multi_roi.sh <cam>`（逐场景手动 ROI）。

- **板的摆放：** 两个传感器都要看得清晰；尽量近；离墙 >0.3 m；倾角适中；
  各场景多变化位置。
- **手动 ROI**（自动失败时：点云稀疏、有其他反光物、板后有墙）：
  `python3 scripts/pick_roi.py calib_data/<cam>/<scene>/cloud.pcd --yaml config/cameras/<cam>.yaml`。
- **难点点云调参**（写配置即可，不需重编译）—— 稀疏 16 线：
  `mech_cluster_min_size: 20`、`mech_cluster_tolerance: 0.13`；稠密非均匀
  （AT128）：`annulus_cluster_min_size: 30`、
  `annulus_cluster_tolerance: 0.06–0.10`；亮环点“离面”被剔除：放宽
  `board/annulus_plane_inlier_threshold`。
- **验收：** 每个场景 4 个标记 + 4 个环且半径正确；RMSE ≤ ~1 cm；
  每个场景叠加图红点落环；`|t|` 与物理安装一致。
