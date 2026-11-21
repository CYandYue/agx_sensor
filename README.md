# RealSense RGB + MID360 稀疏深度采集

独立 Git 项目，部署位置为 `/home/agilex/my_ws/lidar_rgbd_capture`。
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

## 构建和使用

本机依赖：ROS Noetic、已构建的原 `agilex_ws/lio_ws`、Eigen、OpenCV、标准 ROS Python 消息、NumPy。
厂家驱动和 LIO 源码没有复制进新项目。迁移机器时先准备这些依赖；可用
`AGILEX_LIO_WS=/path/to/lio_ws ./build.sh` 指定另一份 LIO overlay。

```bash
cd /home/agilex/my_ws/lidar_rgbd_capture
./build.sh

# 终端 A：双雷达、HI226、相机、LIO 和在线投影；不启动底盘控制
./start_sensors.sh

# 终端 B：等待 IMU 初始化及投影 frames 持续增加，再录制
./run.sh record /home/agilex/my_ws/lidar_rgbd_capture/datasets/scene01.bag
```

默认相机 640×480、6 FPS，投影上限 6 Hz。录制上限默认 10 Hz，避免对 6 Hz 输出再次降频。默认不开 RViz，可传 `rviz:=true`；
`imu_source:=mid360` 可切换至 A 雷达内置 IMU。原深度流仍开启以维持相机驱动原有
camera_link 基准，但关闭深度对齐和 RGB-D 帧组同步，投影不读取原深度。

在录制终端按 Ctrl-C，继续等待 LIO 尾部和原始 bag 合并。看到 `Validated bag:`
后再停止终端 A。原始录制器先等待订阅就绪；使用就绪前扫描的帧会跳过。校验逐帧核对所引用扫描确实存在于最终 bag。
原始双雷达、两路内置 IMU 和外置 IMU 默认录制，必需的两路雷达及
所选 IMU 缺失时拒绝生成成功文件。最终只需上传 `.bag`；旁边的 `.report.json` 方便快速检查。
`.active`、`.partial`、`.raw.bag` 和 `.merge.active` 为中间或失败文件，不能当作完成的数据。
输出拒绝覆盖，合并阶段需额外约一份数据空间。默认留存至少 2 GiB 空闲空间。

```bash
./run.sh check datasets/scene01.bag
./run.sh export datasets/scene01.bag datasets/scene01_export
# 输出 RGB PNG、浮点深度 NPY、poses_tum.txt、frames.jsonl、dataset.json
./run.sh export datasets/scene01.bag datasets/scene01_rectified --rectify
```

原厂家 GMapping/雷达 launch 和原 rgbd_capture/start_sensors.sh 不要同时启动，
它们会重复占用设备或发布 TF。新项目的传感器入口已包含所需驱动。

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
