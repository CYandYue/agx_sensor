# 验证结果

- build.log：最终路径下编译。
- tests.log：离线测试。
- synthetic.json：真实 C++ 投影与录制器的合成运动场景测试，包含逐像素解析误差。
- hardware_preflight.json：设备与网络检查；若满足条件，另有 hardware.json 与静态实机预览。
- *_artifacts.txt：完整 bag 与日志所在目录。

所有测试均不启动底盘控制。合成测试不证明物理标定或硬件同步精度。

- legacy_export.json：81 帧实机 bag 与旧 depth_camera PNG 格式对照，通过单位、无效值、RGB 同名和内参核对；源 bag 哈希保持不变。
- legacy_export_tests.log：21 项离线测试结果。

## LIO 底盘位姿更新（2026-11-23）

27 项离线测试通过。合成录制校验底盘位姿、RGB-D 与 bag 保留的新话题；已有实机 bag 的 169 条 LIO 位姿回放全部转换成功并保留原时间戳。结果见 [base_odometry.json](base_odometry.json)。未启动真实底盘或进行实车运动测试，安装参数仍为名义初值。
