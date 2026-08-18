# 激光雷达 ↔ 相机 外参标定 —— 详细指南（FAST-Calib2）

使用 FAST-Calib2 的反光环形标定板方案，标定**相机**与**激光雷达**之间刚体变换的完整可复现流程。
涵盖：(1) 环境搭建，(2) 数据采集，(3) 数据格式，(4) 标定，(5) 结果评估。

适用于任意相机（RTSP / USB / 录像）与任意激光雷达（机械多线、固态或 Livox）。
点云**直接读取 Apollo Cyber RT record**（`cyber_recorder` 录制结果）或 PCD 文件
—— 全流程不使用 ROS。

**产出：** `T_相机←雷达` = `(R, t)`，满足 `P_相机 = R · P_雷达 + t`（FAST-LIVO2
txt 格式），以及一份 Apollo 惯例的外参 YAML（相机在雷达坐标系中的位姿，可直接
放入 Apollo perception 参数目录）。

---

## 0. FAST-Calib2 工作原理（心智模型）

- **离线**运行，每个*场景*（一次标定板摆位）使用一对 `(image.png, record/ 或 cloud.pcd)`。
- **相机**侧：检测 4 个 ArUco 标记 → 板姿态 → 4 个环心（3D）。
- **雷达**侧：直接从点云提取 4 个反光环心（3D）。
- 求解使两组 4 环心最佳对齐的刚体变换（SVD）。
- **单场景** = 一次摆位（敏感）。**多场景** = 合并 ≥3 次摆位（稳健）。

下文占位符：`<REPO>` = FAST-Calib2 根目录，`<CAM>` = 相机标签，`<SCENE>` = 摆位标签。

---

# 1. 安装与环境搭建

### 1.1 前置条件
- 一台能访问相机和雷达的机器（“主机”）。
- **正在运行的 Apollo dev 容器**（首选编译方式），或系统安装
  `libpcl-dev libopencv-dev libeigen3-dev cmake`（后备编译方式）。
- **ffmpeg**（抓取 RTSP 帧）、**git**、带 `pip` 的 **python3**。
- **反光环形标定板**（FAST-Calib2 板：4 个 ArUco 标记 `DICT_6X6_250` + 4 个回反光环）。
  实测并记录其真实尺寸。

### 1.2 获取 FAST-Calib2 并编译
```bash
git clone https://github.com/wheelos-tools/FAST-Calib2.git <REPO>
cd <REPO>

# 首选：在 Apollo 环境内通过 apollo.sh（bazel）编译，仅编译本模块，
# 使用 Apollo 工作空间自带的 pcl/opencv/eigen 三方模块。
APOLLO_HOST=/path/to/apollo scripts/apollo_build.sh
# 可选环境变量：APOLLO_C=<容器名> APOLLO_USER=nvidia APOLLO_BUILD_CMD=build_opt

# 后备：对系统 PCL/OpenCV/Eigen 直接 CMake 编译
mkdir -p build && cd build && cmake .. && make -j
```
两种方式都产出 `build/{fast_calib, multi_fast_calib, lidar_center_test}`。
任何 C++ 改动后重跑同一条命令（Apollo 首次编译会把 PCL/OpenCV 从源码编一遍；
之后为增量编译，速度很快）。

### 1.3 主机 Python 依赖（辅助脚本用）
```bash
# 读取 Apollo Cyber 记录（仅 Apollo/Cyber 雷达来源需要）：
python3 -m pip install --user cyber_record "protobuf==3.19.4"
# ROI 选点器 + QA 渲染：
python3 -m pip install --user open3d opencv-python numpy
```
> 若 shell 激活了 **conda**，其 `python3` 可能缺少这些包。用 `PYTHON=/usr/bin/python3`
> 让脚本指向系统解释器（脚本支持该环境变量）。

### 1.4 辅助脚本（`<REPO>/scripts/`）
| 脚本 | 作用 |
|---|---|
| `apollo_build.sh` | 在 Apollo dev 容器内编译三个可执行文件（scoped `apollo.sh` bazel 构建） |
| `capture_scene.sh` | 抓相机帧 + 录制雷达 → `image.png`、`record/rec.*`、`cloud.pcd` |
| `record_to_pcd.py` | 融合来源记录的 N 个静止帧 → 稠密 ASCII PCD（供查看/选 ROI；标定程序自己会融合 record） |
| `pick_roi.py` | Open3D 交互式板 ROI 选点器；`--yaml` 直接写入配置，生成的 `cloud_roi.txt` 会被逐场景自动应用 |
| `overlay_reproj.py` / `render_scene_qa.py` | 重投影叠加图（+ 彩色 PCD），用于 QA |
| `multi_capture.sh` / `pick_multi_roi.sh` | 一条命令的多场景流程（自动/手动 ROI） |

### 1.5 启动雷达驱动
采集前雷达必须在发布点云通道。

**Apollo Cyber RT 雷达**：在 Apollo 容器内、**以与其余进程相同的系统用户**
（如 `nvidia`）通过 `cyber_launch` 启动：
```bash
docker exec -d -u nvidia <apollo_container> bash -lc '
  source /apollo/cyber/setup.bash >/dev/null 2>&1
  export CYBER_IP=127.0.0.1 CYBER_DOMAIN_ID=80 CYBER_PATH=/apollo/cyber
  cd /apollo && cyber_launch start /apollo/modules/drivers/lidar/<driver>/launch/<driver>.launch'
```
> **踩坑经验（重要）：**
> - **用户必须一致。** 以 `root` 启动驱动会产生 root 拥有的共享内存，其余（nvidia）进程无法写入
>   → `acquire block failed` / `get shm failed: Permission denied`。遇到时先杀掉残留驱动
>   （`pkill -9 -f <driver>/dag`），并删除孤儿 root 共享内存（`ipcrm -m <id>`）。
> - **dag 用绝对路径。** cyber_launch 会把*相对* `<dag_conf>` 解析到 `/apollo/cyber` 下；
>   在 `.launch` 里用绝对路径（如 `/apollo/modules/.../<driver>.dag`）。
> - **节点名唯一。** 每个 domain 只能有一个驱动实例 —— “duplicated node” 表示已有实例在运行。
> - **匹配网络配置**（设备 IP、主机 IP、MSOP/PTC 端口）于驱动的 `*.pb.txt`。

确认数据在流动（用 `echo`，不用 `list` —— `list` 发现不可靠）：
```bash
docker exec -u nvidia <apollo_container> bash -lc \
  'source /apollo/cyber/setup.bash>/dev/null 2>&1; export CYBER_DOMAIN_ID=80;
   cyber_channel echo /apollo/sensor/<lidar>/PointCloud2 | grep -m1 frame_id'
```

---

# 2. 采集数据

### 2.1 先做相机内参（必须）
内参错误则外参无意义。用棋盘格/ChArUco（OpenCV 或你的内参工具）对**该相机+镜头+分辨率**
标定，记录 `fx, fy, cx, cy, k1, k2, p1, p2`（FAST-Calib2 用这 8 个；舍弃 k3）。
- 目标：**平均重投影误差 < ~0.3 px**，径向单调性通过，角点覆盖全画面。
- **使用该相机自身的标定**，而非“相似”相机的平均 —— 焦距因单机/变焦而异，且直接决定重建深度（见 §5.5）。

### 2.2 标定板物理摆放
使**两个**传感器都能清晰看到标定板：
- **尽量近** —— 低线束机械雷达在板上只落约（命中它的波束数）条扫描线。
- **远离墙面**（>~0.3 m）—— 紧贴墙会干扰平面拟合。
- 对某些雷达上的回反光板**保持倾角适中** —— 过大倾角会把反光回波散布到板面之外（见 §4.2）。
  多改变标定板**位置**，而非只倾斜。

### 2.3 采集一个场景
`capture_scene.sh` 抓一张 RTSP 帧并录制雷达通道：
```bash
RTSP=rtsp://user:pass@host:554/live APOLLO_C=<apollo_container> \
CH=/apollo/sensor/<lidar>/PointCloud2 \
  scripts/capture_scene.sh <CAM> <SCENE> 5     # 5 秒（约数十帧）
```
输出 → `<REPO>/calib_data/<CAM>/<SCENE>/{image.png, record/rec.*, cloud.pcd}`。
原始 cyber record 就是标定输入（程序自己融合其中的帧）；`cloud.pcd` 只供
ROI 选点器和 QA 脚本使用。
**务必目视 `image.png`** —— 整块板（4 标记 + 4 环）必须清晰且不被裁切。

**手动替代：** 用 `cyber_recorder record -c <channel>` 录到 `.../record/`，
再放一张相机帧到 `.../image.png`。也可用 `cloud.pcd`（x y z intensity）作为点云来源。

### 2.4 多场景
对 **≥3 个板姿态**（如正对、右倾、左倾，和/或左/中/右移动）重复 §2.3。§4.4 的编排脚本可自动化。

---

# 3. 数据格式

FAST-Calib2 **直接读取 Apollo Cyber record 文件**（配置通道上的
`apollo.drivers.PointCloud` 消息），或读取 **PCD**（x y z intensity，若含
`ring` 列则一并使用）。不再有任何转换步骤 —— 多帧融合与 ring 处理都在程序内部完成：

### 3.1 多帧融合（自动）
稀疏雷达的单帧常低于 FAST-Calib2 的点数阈值。采集时板与雷达静止，故加载器会
**融合 record 中的多帧**（`max_fusion_frames: 0` = 全部帧，默认值）。
> 融合在**方位角**上加密每条扫描线，无法增加扫描线数（由波束数决定）。若板仍显稀疏，放近些。

### 3.2 `--scene` 期望的数据格式
```
<REPO>/calib_data/<CAM>/<SCENE>/image.png
<REPO>/calib_data/<CAM>/<SCENE>/record/rec.*   # 首选；或 cloud.pcd
<REPO>/calib_data/<CAM>/<SCENE>/cloud_roi.txt  # 可选，自动应用的手动 ROI
<REPO>/config/cameras/<CAM>.yaml          # 内参 + 板几何 + 通道 + ROI
<REPO>/output/<CAM>/                        # 结果
```

### 3.3 相机配置文件
复制模板到 `config/cameras/<CAM>.yaml` 并填写：
```yaml
# 内参（来自 §2.1）
  fx: ...  fy: ...  cx: ...  cy: ...
  k1: ...  k2: ...  p1: ...  p2: ...
# 板几何 —— 实测你的板
  marker_size: 0.20             # ArUco 边长 [m]
  delta_width_qr_center: 0.55   # 标记中心水平间距的一半
  delta_height_qr_center: 0.35  # 标记中心垂直间距的一半
  delta_width_circles: 0.5      # 环心水平间距
  delta_height_circles: 0.4     # 环心垂直间距
  circle_radius: 0.12           # 环中心线半径 [m]
  annulus_half_width: 0.025
  min_detected_markers: 3
# 雷达来源
  lidar_channel: "/apollo/sensor/<lidar>/PointCloud2"
  lidar_frame: "<lidar_frame>"   # Apollo 外参 YAML 中使用的坐标系名
  camera_frame: "<CAM>"
  max_fusion_frames: 0           # 0 = 融合 record 中全部帧
  # beam_altitudes_deg: [...]    # 仅低线束机械雷达需要（§3.2）
  use_auto_lidar_roi: true      # 先试 true；失败则 false + 手动框（§4.1）
  x_min: ...  x_max: ...  y_min: ...  y_max: ...  z_min: ...  z_max: ...
# 检测器调参（可选；下为默认值 —— 每台设备的调参写在这里，不再改源码；见 §4.2）
  # board_plane_inlier_threshold: 0.07
  # annulus_plane_inlier_threshold: 0.07
  # boundary_plane_inlier_threshold: 0.03
  # annulus_cluster_tolerance: 0.10
  # annulus_cluster_min_size: 30
  # mech_cluster_tolerance: 0.09
  # mech_cluster_min_size: 80
```

---

# 4. 标定

### 4.1 设置板 ROI
FAST-Calib2 需先框出板点。
- **自动 ROI**（`use_auto_lidar_roi: true`）：当板的反光回波在高强度点中占主导时有效 —— **稠密**点云常见。
- **手动 ROI**：自动失败时（点云稀疏、其他反光物、板后有墙）。在板面上选紧凑框：
  ```bash
  # 直接把 use_auto_lidar_roi:false + 框写入配置
  PYTHON=/usr/bin/python3 python3 scripts/pick_roi.py \
      calib_data/<CAM>/<SCENE>/cloud.pcd --yaml config/cameras/<CAM>.yaml
  ```
  Shift+点选 ≥4 个板上点（按强度着色，环清晰可见），按 Q。紧凑框迫使 RANSAC 拟合板平面而非背景/墙面。

### 4.2 运行单场景
```bash
./build/fast_calib --config config/cameras/<CAM>.yaml \
                   --scene calib_data/<CAM>/<SCENE> --output output/<CAM> \
                   [--debug-dir output/<CAM>/debug_<SCENE>]
```
程序打印结果后**自行退出**（0 = 成功，2 = 检测失败）；`--debug-dir` 会把每个中间
点云写成 PCD（取代原先的 RViz 话题）。结果 →
`output/<CAM>/single_calib_result.txt` + `single_calib_extrinsics.yaml`。

**成功标志：** 相机 `4 centers found`；雷达 `4 edge/annulus clusters`，同心圆以你板的内/外半径拟合；
`[Result] RMSE: 0.00xx m`。

**难点点云调参**（写在相机配置里 —— 不改源码、不重编译）：
- *稀疏 16 线、`boundary clusters: 0`* → `mech_cluster_min_size: 20`、
  `mech_cluster_tolerance: 0.13`。
- *稠密非均匀（AT128）、只有 2/4 簇* → `annulus_cluster_min_size: 30`、
  `annulus_cluster_tolerance: 0.06–0.10`。
- *亮环点“离面”被剔除* → 收紧 ROI 使 RANSAC 拟合板（最佳），或放宽
  `board_plane_inlier_threshold` / `annulus_plane_inlier_threshold`
  （饱和的反光膜回波可能偏离板面数厘米）。

### 4.3 运行多场景（推荐）
每次单场景运行把 4 对环心追加到 `output/<CAM>/circle_center_record.txt`；联合步骤读取最近 ≥3 个：
```bash
# 完成 ≥3 次成功的单场景运行后：
./build/multi_fast_calib --config config/cameras/<CAM>.yaml --output output/<CAM>
# -> output/<CAM>/multi_calib_result.txt + multi_calib_extrinsics.yaml
```

### 4.4 一条命令的编排脚本
```bash
# 自动 ROI：每个角度暂停让你重新摆板，采集、标定、联合拟合、渲染 QA
scripts/multi_capture.sh <CAM_BASE> 3 5

# 大倾角/自动 ROI 失败：逐场景手动选紧凑 ROI（数据已采集）
scripts/pick_multi_roi.sh <CAM>
```
两者都会（以正确用户）自动启动驱动，并由**联合**外参生成 `reproj_scene{1..3}.png` +
`colored_scene{1..3}.pcd`。

---

# 5. 评估结果

### 5.1 数值残差（读运行日志）
- **`[Result] RMSE`** —— 跨传感器配准残差（4 雷达 vs 4 相机环心）。通常几毫米到约 1 cm；
  单场景约 3–7 mm，雷达越稠密越紧。
- **`[Geometry][LiDAR] max error / RMSE`** —— 4 个*雷达测得*环心与板已知几何的吻合度
  （雷达自身测量误差，约 mm–cm）。
- **`[Geometry][QR]`** —— 相机几何；≈1e-5 mm，因为相机环心由板姿态构造（见 §5.5）。
  这**不是**内参检验。
- **同心拟合** 应报告你板的半径（如 `0.095 / 0.145`）。

### 5.2 重投影叠加图（主要目视检查）
```bash
# 单场景结果：
python3 scripts/overlay_reproj.py calib_data/<CAM>/<SCENE> \
        output/<CAM>/single_calib_result.txt output/<CAM>/reproj.png
# 由多场景结果（该文件不含内参 → 传 --config）：
python3 scripts/render_scene_qa.py calib_data/<CAM>/<SCENE> \
        output/<CAM>/multi_calib_result.txt --config config/cameras/<CAM>.yaml \
        --overlay output/<CAM>/reproj_<SCENE>.png --colored output/<CAM>/colored_<SCENE>.pcd
```
雷达投影到图像上，按强度着色（JET）。**红 = 高反射点落在 4 个白环上**且扫描线正确贴合
墙面/桌面/地面，即标定良好。多场景时**逐个场景**核对，而非只看一个。

### 5.3 彩色点云
`colored_*.pcd` = 用相机像素颜色重着色的雷达点。用 CloudCompare/pcl_viewer 打开；
板上图案（黑板 + 白环）应清晰不发糊。
> 多场景运行后，忽略单场景的 `colored_cloud.pcd` / `single_calib_result.txt` —— 它们只是最后运行的
> 那个场景（可能是退化的单场景 SVD）。以 `multi_calib_result.txt` + 逐场景 `reproj_*`/`colored_*` 为准。

### 5.4 物理合理性检查
`t` 是雷达原点在相机坐标系中的位置（OpenCV：+X 右、+Y 下、+Z 前）。读出左右/上下/前后及
`|t|`（直线间距）。交叉验证：某板点到雷达与到相机的距离之差应约等于 `t` 在视线方向的分量。

### 5.5 内参↔板 验证（可选，`intrinsic_board_check.py`）
用内参从一张图像独立重建板，并与真值比较距离：
```bash
python3 intrinsic_board_check.py <image.png> fx fy cx cy k1 k2 p1 p2
```
- **(A) 标记，逐标记 solvePnP** —— 稳健探针；好的内参把板重建到 **≈0.5%**（几毫米）。
- **(B) 环心，由检测像素反投影** —— 水平吻合良好；**倾斜**板上可能出现小的*垂直*偏小
  （倾斜圆的投影椭圆中心 ≠ 圆心 —— 是测量伪影，非内参问题）。
- **注意：** 近正对板上，特征间*距离*对焦距**基本不敏感**（横向 X,Y ∝ 像素偏移，与 fx 无关）。
  焦距决定重建**深度**，故修正内参时改变的是*外参平移* —— 应以重投影误差/深度验证内参，而非板上距离。

### 5.6 验收清单
- [ ] 每个场景相机都检出 4 个标记；QR 几何误差约 1e-5 mm。
- [ ] 每个场景雷达都以正确半径提取 4 个同心环。
- [ ] 跨传感器 RMSE 在容差内（单场景 ≤ ~1 cm；多场景更紧）。
- [ ] 重投影叠加图：**所有**场景红点都落在环上。
- [ ] `|t|` 与姿态符合物理装配。

---

## 附录 —— 快速全流程（Apollo/Cyber 示例）

```bash
# 0. 编译 + 依赖（一次）
cd <REPO> && APOLLO_HOST=/path/to/apollo scripts/apollo_build.sh
python3 -m pip install --user cyber_record protobuf==3.19.4 open3d opencv-python numpy

# 1. 内参 -> config/cameras/<CAM>.yaml   （棋盘格，§2.1）；同一配置里填好
#    lidar_channel / lidar_frame / camera_frame（§3.4）

# 2-4. 采集 3 个角度 + 标定 + QA，一条命令：
CH=/apollo/sensor/<lidar>/PointCloud2 \
LIDAR_LAUNCH=/apollo/modules/drivers/lidar/<drv>/launch/<drv>.launch \
  scripts/multi_capture.sh <CAM> 3 5
#   （倾斜板改用 scripts/pick_multi_roi.sh <CAM>_<date>）

# 5. 查看 output/<CAM>_<date>/: multi_calib_result.txt,
#    multi_calib_extrinsics.yaml（Apollo 惯例：相机在雷达坐标系中的位姿，
#    可直接放入 perception 参数目录）, reproj_scene{1,2,3}.png,
#    colored_scene{1,2,3}.pcd
```
