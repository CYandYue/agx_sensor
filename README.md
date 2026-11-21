# RealSense RGB + MID360 稀疏深度采集

独立 Git 项目（`my_ws` 下每个项目子目录各自管理 Git），部署位置为 `/home/agilex/my_ws/lidar_rgbd_capture`。
复用 `/home/agilex/agilex_ws/lio_ws` 的双 MID360、HI226 和 FAST-LIO-MULTI，
在线生成与 RealSense 彩色图像像素模型对应的雷达深度。RealSense 自带深度不参与输出。

## 数据约定

- RGB 保留相机原像素、原时间戳和畸变模型。
- 深度为同尺寸 `32FC1`，数值是彩色光学坐标系的 **Z，单位米**，空洞为 NaN。
- 每个有效像素来自时间窗口内的原始雷达测量。一个像素有多个点时取最近 Z。
  不扩散点、不补洞；像素栅格化本身仍有舍入误差。
- `/lidar_rgbd/valid_mask` 为在线查看用的 `mono8` 有效掩码。最终 bag 中掩码可由深度有限值重建。
- 相机位姿为 `T_lio_map_camera_color_optical_frame`，即把相机点变换到 LIO 世界系。
  使用 `geometry_msgs/PoseStamped` 保存每组图像的相机位姿；原始 `/lio/odometry` 同时保留。
  `body` 是当前 LIO 选用的 IMU 系。没有从尚未提供的相机速度构造 Odometry。
- RGB、投影深度、相机位姿的关联时刻为 RGB header。它们并非物理同时采样；
  每帧 JSON 保存真实点的采样时间范围和所用运动模型。

## 使用方法

本机短测的**完整 RGB/雷达深度帧组约 5.96 Hz**，默认相机与投影上限均为 6 Hz。
这是完整帧组速率；投影平均输出延迟约 0.46 s。实际速率、延迟和深度覆盖率随场景与负载变化。
每次采集后，从 `.bag.report.json` 的 `actual_rate_hz` 查看实际帧率、从 `validation` 查看校验结果；
每帧投影延迟保存在 `/dataset/frame` JSON 的 `projection.output_age_s` 中。

### 1. 准备环境

本机已经编译，可以直接启动。新拉取代码或修改 C++ 后，先执行：

```bash
cd /home/agilex/my_ws/lidar_rgbd_capture
./build.sh
```

依赖 ROS Noetic、已构建的 `/home/agilex/agilex_ws/lio_ws`、Eigen、OpenCV、标准 ROS Python 消息和 NumPy。
厂家驱动和 LIO 源码没有复制进本仓库。迁移机器时需要先准备这些依赖；可用
`AGILEX_LIO_WS=/path/to/lio_ws ./build.sh` 指定另一份 LIO overlay。入口脚本自动加载所需 ROS 环境。

确认双 MID360、D435 和 HI226 已连接。当前雷达配置地址为 `192.168.1.113` / `192.168.1.154`，
主机雷达网口为 `192.168.1.4`，默认 HI226 串口为 `/dev/ttyUSB0`。
先结束原厂家 GMapping/雷达 launch 和原 `rgbd_capture/start_sensors.sh`；新入口已包含驱动，
同时启动会重复占用设备或发布 TF。

### 2. 终端 A：启动传感器、LIO 和在线投影

```bash
cd /home/agilex/my_ws/lidar_rgbd_capture
./start_sensors.sh
```

启动后让机器人静止数秒，等待 IMU 初始化，并确认投影日志中的 `frames=...` 持续增长，再开始录制。
默认相机 640×480、6 FPS，投影上限 6 Hz；默认不开 RViz，不启动底盘控制。
RealSense 原深度流仍开启以维持驱动原有的 camera_link 基准，但关闭深度对齐和 RGB-D 帧组同步；
输出深度全部来自 MID360，投影节点不读取 RealSense 深度。

需要改变启动选项时，在终端 A 选用对应命令：

```bash
# 同时显示 RViz
./start_sensors.sh rviz:=true

# 改用 MID360 A 的内置 IMU
./start_sensors.sh imu_source:=mid360
```

### 3. 终端 B：开始录制

```bash
cd /home/agilex/my_ws/lidar_rgbd_capture
./run.sh record datasets/scene01.bag
```

`datasets/` 会自动创建。每次使用新文件名，程序拒绝覆盖已有 bag、报告或恢复文件。
等待终端 B 的 `frames=...` 持续增加，表示已经录到有效完整帧组；仅看到启动日志不代表采集成功。

默认会把 RGB、雷达投影深度、图像时刻的相机位姿、原始双雷达、已发布的原始 IMU、
LIO odometry、TF、标定与会话信息一起录入最终 bag，不需要额外加 `--raw-sensors`。
录制器上限默认 10 Hz，用于接收投影的约 6 Hz 输出，不会生成或补出 10 Hz 帧。

也可以指定时长：

```bash
./run.sh record datasets/scene02.bag --duration 60
```

60 秒从采集进程启动时算起，包含准备时间；等待 LIO 尾部、合并和校验还会额外用时。

### 4. 按顺序停止并检查结果

1. 在**终端 B** 按 Ctrl-C 停止录制；定时录制会自动进入收尾。
2. 保持终端 A 运行，等待 LIO 尾部、原始点云合并与校验。收尾可能需要几十秒。
3. 看到 **`Validated bag: ...`** 后，再在**终端 A** 按 Ctrl-C 停止传感器。

成功会得到：

```text
datasets/scene01.bag                  # 完整数据文件
datasets/scene01.bag.report.json      # 实际帧率、帧数、原始话题计数和校验结果
```

报告中的 `status` 应为 `PASS`；`frames` 是完整帧组数，`actual_rate_hz` 是实际完成帧率。
`PASS` 表示结构、扫描来源和位姿计算校验通过，不代表达到某个最低帧率或通过物理标定。

原始录制器会先等待订阅就绪；使用就绪前扫描的帧会跳过。校验逐帧核对所引用扫描确实存在于最终 bag。
两路雷达及所选 IMU 缺失时拒绝生成成功文件。`.active`、`.partial`、`.raw.bag`、
`.raw.bag.active`、`.merge.active` 是中间或失败文件，不作为完成数据使用。
合并阶段需额外约一份数据空间，程序默认留存至少 2 GiB 空闲空间。

### 5. 离线校验和导出

以下命令无需启动传感器或 ROS master：

```bash
cd /home/agilex/my_ws/lidar_rgbd_capture
./run.sh check datasets/scene01.bag

# 保留原始 RGB 像素模型
./run.sh export datasets/scene01.bag datasets/scene01_export

# 或同步对 RGB 和雷达深度去畸变
./run.sh export datasets/scene01.bag datasets/scene01_rectified --rectify
```

导出目录必须不存在。输出包含 `rgb/*.png`、浮点米制 `depth/*.npy`、`poses_tum.txt`、
`frames.jsonl` 和 `dataset.json`。深度空洞为 NaN；按 `frames.jsonl` 的编号、路径和时间戳关联数据。
服务器接收最终 `.bag` 即可；会话与录制报告也保存在 bag 内。样例和实测预览见文末。

## 在线发布与 bag

在线话题：`/lidar_rgbd/rgb/image_raw`、`/lidar_rgbd/depth/image_raw`、
`/lidar_rgbd/rgb/camera_info`、`/lidar_rgbd/depth/camera_info`、
`/lidar_rgbd/camera_pose`、`/lidar_rgbd/valid_mask`、`/lidar_rgbd/projection`。
可用 rqt_image_view 查看 RGB、深度及掩码。深度图的空洞是正常输出。

最终每组 `/dataset/*` 有 7 条消息，bag 时间一致、frame_index 连续：

| 话题 | 内容 |
|---|---|
| `/dataset/rgb/image_raw` | 原 RGB |
| `/dataset/depth/image_raw` | MID360 投影深度 |
| `/dataset/rgb/camera_info`、`/dataset/depth/camera_info` | 相同的 RGB 像素模型 |
| `/dataset/camera_pose` | RGB 时刻的相机光学坐标系位姿 |
| `/dataset/body_to_camera` | 实际使用的相机安装变换 |
| `/dataset/frame` | JSON：原始时间、位姿插值、雷达来源及点时间窗口 |

同时保留 `/lio/odometry`、`/tf`、`/tf_static`、`/dataset/session`、`/dataset/report`、
两路 `/livox/lidar_192_168_1_113` 和 `...154`，以及发布中的原始 IMU、相机 metadata。
原始雷达为 driver2 CustomMsg，完整保留逐点时间；服务器只读 `/dataset/*` 时不需要 Livox 包。
schema_version=2，新增投影来源校验，不与旧 RealSense 深度 bag 混淆。

## 运动、标定和延迟边界

默认使用 RGB 查询时刻前后各 50 ms 的点。记 `T_A_B` 把 B 系变换到 A 系，逐点计算：

`p_C(t_rgb) = inverse(T_W_C(t_rgb + offset)) × T_W_B(t_point) × T_B_L × p_L`

`T_W_B` 来自 LIO 前后位姿的平移线性插值和四元数 SLERP，最大允许间隔 0.25 s；
不外推。默认相机时间偏移为 0，定义 `t_lio = t_rgb + offset`。修改时必须在启动
`camera_time_offset:=...` 和录制 `--camera-time-offset ...` 中使用同一个值，校验会检查。
投影支持 CameraInfo 的 plumb_bob、rational_polynomial 和四参数 equidistant。
原始 RGB 不去畸变，投影时应用其畸变；可在导出时同步去畸变。

必须等待点云窗口完整和 LIO 前后位姿，因而存在输出延迟。默认等候最多 3 秒，缺数据整组跳过；
队列、内存和输入时间单调性受限，时钟倒退或外参变化会终止会话。
报表记录实际帧率、有效深度比例和投影输出年龄。相机 6 FPS 不是完成帧组速率的保证。

本版用的是车辆运动估计，并不能消除行人等独立物体的运动。雷达与相机有基线和不同遮挡；
Z-buffer 只能处理已采到的点之间的遮挡，不能证明相机视线上的未采样遮挡物不存在。
稀疏图覆盖率由视场、安装角度、距离和场景共同决定。未做长时间地图累积或未来帧补全。

现有 mounts.yaml 与 LIO 外参仍为厂家/名义安装初值，HI226 使用主机收包时间。
因此软件测试通过不代表外参精度、硬件同步或运动精度已通过标定。
本项目不把 `/lio/cloud_registered` 作为深度输入：现有 LIO 对输入每 6 点取 1 点，
并使用 0.2 m 体素；本节点保留原始点密度，但仍依赖该轨迹做运动估计。

## 验证

```bash
bash -c 'source /opt/ros/noetic/setup.bash; /usr/bin/python3 -m unittest discover -s tests -p test_recorder.py -v'
bash -c 'source devel/setup.bash; /usr/bin/python3 tests/smoke_pipeline.py'
# 接上设备后，静态实机测试（会自行启停私有 ROS master、传感器和录制，不控制底盘）
bash -c 'source devel/setup.bash; /usr/bin/python3 tests/smoke_pipeline.py --hardware --duration 20'
```

合成测试用已知旋转和平移轨迹、静止三维点和非零镜头畸变驱动真实 C++ 节点，
扫描内点顺序故意打乱，逐像素对照解析投影和相机位姿，并验证原始数据录制、合并和完整帧组校验。
测试 bag 和日志保留于输出的临时目录。版本化结果放在 verification/，大 bag 不进 Git。

来源：recorder 三个 Python 文件及录制测试基于用户已有
`agilex_ws/rgbd_capture`，基线提交 `77a9cd2`；本项目修改了输入、schema、投影来源验证及默认原始录制。
新增 C++ 节点和合成运动场景为本项目实现。

参考：[OpenCV 投影模型](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)、
[ROS 深度约定 REP 118](https://github.com/ros-infrastructure/rep/blob/master/rep-0118.rst)、
[MID360 官方参数](https://www.livoxtech.com/mid-360/specs)。

## 本机首次实测（2026-11-21）

17 项离线测试、乱序逐点时间的合成运动测试，以及双 MID360 + HI226 + D435 实机短测通过。
实机完成 81 组 640×480 RGB/雷达深度，约 5.96 Hz，投影平均输出年龄 0.46 s、最大 0.74 s。
两路原始雷达各 178 / 177 条，原始 IMU 均有数据；原始扫描来源核对和相机位姿复算通过。

此场景平均有效深度覆盖约 0.60%，最小约 0.064%。画面下部存在大片雷达视场空白，
不能把这一版当成稠密深度相机。投影叠加图只用于观察，尚无物理外参残差或运动精度认证。
相机位置各轴变化范围约 4 / 5 / 7 mm，是这段短测的轨迹变化量，不是定位精度指标。

- 样例 bag：`datasets/static_trial_20261121_2138.bag`（约 132 MiB，不进 Git）。
- 对应报告：同名 `.bag.report.json`；副本哈希见 `verification/sample.json`。
- 实机预览：`verification/hardware_rgb.png`、`hardware_projection_overlay.png`、`hardware_depth_preview.png`。
- 录制结束后，测试所启动的全部 ROS/硬件进程已经退出；没有启动底盘控制。
