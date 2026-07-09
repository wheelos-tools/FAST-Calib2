# 激光雷达 ↔ 相机 外参标定 —— 详细指南（FAST-Calib2）

使用 FAST-Calib2 的反光环形标定板方案，标定**相机**与**激光雷达**之间刚体变换的完整可复现流程。
涵盖：(1) 环境搭建，(2) 数据采集，(3) 数据格式转换，(4) 标定，(5) 结果评估。

适用于任意相机（RTSP / USB / 录像）与任意激光雷达（机械多线、固态或 Livox），
无论点云来自原生 ROS 驱动还是 **Apollo Cyber RT**。

**产出：** `T_相机←雷达` = `(R, t)`，满足 `P_相机 = R · P_雷达 + t`。

---

## 0. FAST-Calib2 工作原理（心智模型）

- **离线**运行，每个*场景*（一次标定板摆位）使用一对 `(image.png, cloud.bag)`。
- **相机**侧：检测 4 个 ArUco 标记 → 板姿态 → 4 个环心（3D）。
- **雷达**侧：直接从点云提取 4 个反光环心（3D）。
- 求解使两组 4 环心最佳对齐的刚体变换（SVD）。
- **单场景** = 一次摆位（敏感）。**多场景** = 合并 ≥3 次摆位（稳健）。

下文占位符：`<REPO>` = FAST-Calib2 根目录，`<CAM>` = 相机标签，`<SCENE>` = 摆位标签，
`<IMG>` = ROS 容器镜像 `fast-calib2:noetic`。

---

# 1. 安装与环境搭建

### 1.1 前置条件
- 一台能访问相机和雷达的机器（“主机”）。
- **Docker**（用于 ROS Noetic 编译容器 —— 主机无需装 ROS）。
- **ffmpeg**（抓取 RTSP 帧）、**git**、带 `pip` 的 **python3**。
- **反光环形标定板**（FAST-Calib2 板：4 个 ArUco 标记 `DICT_6X6_250` + 4 个回反光环）。
  实测并记录其真实尺寸。

### 1.2 获取 FAST-Calib2 并构建容器
```bash
git clone https://github.com/wheelos-tools/FAST-Calib2.git <REPO>
cd <REPO>
docker/build.sh        # 构建 fast-calib2:noetic 并把工作空间编译到 docker/.ws_devel
```
容器内含 PCL + OpenCV + ROS Noetic；编译产物保存在 `docker/.ws_devel/`，在 `docker run --rm`
间保留。任何 C++ 改动后重跑 `docker/build.sh`（或在容器内 `catkin_make`）。

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
| `capture_scene.sh` | 抓相机帧 + 录制/融合雷达 → `image.png`、`cloud.pcd`、`cloud.bag` |
| `record_to_pcd.py` | 融合来源记录的 N 个静止帧 → 稠密 ASCII PCD |
| `pcd_to_bag.py` | PCD → ROS bag；为机械雷达合成 `ring` 字段 |
| `pick_roi.py` | Open3D 交互式板 ROI 选点器；`--yaml` 直接写入配置 |
| `overlay_reproj.py` / `render_scene_qa.py` | 重投影叠加图（+ 彩色 PCD），用于 QA |
| `multi_capture.sh` / `pick_multi_roi.sh` | 一条命令的多场景流程（自动/手动 ROI） |

### 1.5 启动雷达驱动
采集前雷达必须发布点云通道/话题。

**原生 ROS 雷达：** 启动厂商 ROS 驱动，记下 `PointCloud2` 话题。

**Apollo Cyber RT 雷达**（本项目情况）：在 Apollo 容器内、**以与其余进程相同的系统用户**
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
`capture_scene.sh` 抓一张 RTSP 帧并录制+融合雷达为 bag：
```bash
RTSP=rtsp://user:pass@host:554/live APOLLO_C=<apollo_container> \
CH=/apollo/sensor/<lidar>/PointCloud2 TOPIC=/lidar_points FRAME=<lidar_frame> \
RING_FLAG=<稠密/固态用 --no-ring | 留空则合成 ring> \
  scripts/capture_scene.sh <CAM> <SCENE> 5     # 5 秒（融合数十帧）
```
输出 → `<REPO>/calib_data/<CAM>/<SCENE>/{image.png, cloud.pcd, cloud.bag}`。
**务必目视 `image.png`** —— 整块板（4 标记 + 4 环）必须清晰且不被裁切。

**原生 ROS 替代：** `rosbag record -O .../cloud.bag <TOPIC> --duration=5`（原生保留真实 `ring`），
再放一张帧到 `.../image.png`。

### 2.4 多场景
对 **≥3 个板姿态**（如正对、右倾、左倾，和/或左/中/右移动）重复 §2.3。§4.4 的编排脚本可自动化。

---

# 3. 转换为标定格式

FAST-Calib2 读取带 `sensor_msgs/PointCloud2`（或 Livox `CustomMsg`）的 **ROS bag**。原生 ROS 来源
已具备；**Apollo/Cyber 等非 ROS 来源** 由 `capture_scene.sh` 完成以下两步，这里显式说明：

### 3.1 融合多帧 → 稠密 PCD
稀疏雷达的单帧常低于 FAST-Calib2 的点数阈值。采集时板与雷达静止，故**融合多帧**：
```bash
python3 scripts/record_to_pcd.py --record-glob '<record_dir>/*' \
        --channel /apollo/sensor/<lidar>/PointCloud2 --out cloud.pcd --max-frames 20
```
> 融合在**方位角**上加密每条扫描线，无法增加扫描线数（由波束数决定）。若板仍显稀疏，放近些。

### 3.2 PCD → ROS bag（机械雷达 + ring）
```bash
python3 scripts/pcd_to_bag.py --pcd cloud.pcd --bag cloud.bag \
        --topic /lidar_points --frame <lidar_frame> [--no-ring]
```
- **`ring`** = 雷达*扫描线序号*（**不是**板上的环）。bag 含 `ring` 字段时，FAST-Calib2 用其
  **机械**流程（沿每条扫描线找强度跳变）；不含时用**固态**流程（对高反环点聚类 → 拟合圆）。
- Apollo 点云不带 `ring`。对**低线束机械**雷达（如 16 线），`pcd_to_bag` 依据各点俯仰角与标称
  波束表**合成** `ring` → 提取更好。对**稠密**雷达（如 128 线、非均匀波束），用 **`--no-ring`**（固态流程）。

### 3.3 launch 期望的数据布局
```
<REPO>/calib_data/<CAM>/<SCENE>/image.png
<REPO>/calib_data/<CAM>/<SCENE>/cloud.bag
<REPO>/config/cameras/<CAM>.yaml          # 内参 + 板几何 + 话题 + ROI
<REPO>/output/<CAM>/                        # 结果
```

### 3.4 相机配置文件
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
# 雷达
  lidar_topic: "/lidar_points"
  use_auto_lidar_roi: true      # 先试 true；失败则 false + 手动框（§4.1）
  x_min: ...  x_max: ...  y_min: ...  y_max: ...  z_min: ...  z_max: ...
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
docker run --rm --net=host \
  -v "<REPO>:/root/calib_ws/src/fast_calib" \
  -v "<REPO>/docker/.ws_build:/root/calib_ws/build" \
  -v "<REPO>/docker/.ws_devel:/root/calib_ws/devel" \
  <IMG> bash -lc "roslaunch fast_calib calib_cam.launch cam:=<CAM> scene:=<SCENE> rviz:=false"
```
（节点打印结果后进入 RViz 循环 —— 用 `timeout 90` 包裹，或在出现 `[Result] RMSE` 与
`Saved four pairs of target centers` 后 Ctrl-C。）结果 → `output/<CAM>/single_calib_result.txt`。

**成功标志：** 相机 `4 centers found`；雷达 `4 edge/annulus clusters`，同心圆以你板的内/外半径拟合；
`[Result] RMSE: 0.00xx m`。

**难点点云调参**（改 `src/lidar_detect.hpp` 后重编译）：
- *稀疏 16 线、`boundary clusters: 0`* → 降低 `clusterMechanicalAnnulusBoundaryCloud` 的
  `setMinClusterSize`（80→~20），提高 `setClusterTolerance`（0.09→~0.13）。
- *稠密非均匀（AT128）、只有 2/4 簇* → 降低 `clusterAnnulusCloud` 的 `setMinClusterSize`
  （200→~30），提高 tolerance（0.02→~0.06）。
- *亮环点“离面”被剔除* → 收紧 ROI 使 RANSAC 拟合板（最佳），或放宽平面内点阈值（`0.015`/`0.03`）。

### 4.3 运行多场景（推荐）
每次单场景运行把 4 对环心追加到 `output/<CAM>/circle_center_record.txt`；联合步骤读取最近 ≥3 个：
```bash
# 完成 ≥3 次成功的单场景运行后：
docker run --rm --net=host -v ... <IMG> \
  bash -lc "roslaunch fast_calib multi_calib_cam.launch cam:=<CAM>"
# -> output/<CAM>/multi_calib_result.txt
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
# 0. 构建 + 依赖（一次）
cd <REPO> && docker/build.sh
python3 -m pip install --user cyber_record protobuf==3.19.4 open3d opencv-python numpy

# 1. 内参 -> config/cameras/<CAM>.yaml   （棋盘格，§2.1）

# 2-4. 采集 3 个角度 + 标定 + QA，一条命令：
CH=/apollo/sensor/<lidar>/PointCloud2 TOPIC=/lidar_points FRAME=<frame> RING_FLAG=--no-ring \
LIDAR_LAUNCH=/apollo/modules/drivers/lidar/<drv>/launch/<drv>.launch \
  scripts/multi_capture.sh <CAM> 3 5
#   （倾斜板改用 scripts/pick_multi_roi.sh <CAM>_<date>）

# 5. 查看 output/<CAM>_<date>/: multi_calib_result.txt, reproj_scene{1,2,3}.png,
#    colored_scene{1,2,3}.pcd
```
