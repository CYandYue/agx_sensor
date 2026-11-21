# 验证结果

- build.log：最终路径下编译。
- tests.log：离线测试。
- synthetic.json：真实 C++ 投影与录制器的合成运动场景测试，包含逐像素解析误差。
- hardware_preflight.json：设备与网络检查；若满足条件，另有 hardware.json 与静态实机预览。
- *_artifacts.txt：完整 bag 与日志所在目录。

所有测试均不启动底盘控制。合成测试不证明物理标定或硬件同步精度。
