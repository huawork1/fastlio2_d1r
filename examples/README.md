# D1R weak100 完整室内示例数据

主示例 bag 不直接提交到源码仓库，以 GitHub Release 附件 `d1r_weak100_indoor_65s.tar.gz` 发布，避免 `.mcap` 数据进入 Git 历史。

## 数据概况

- 来源：既有独立室内段 `d1r_split_20261012/bags/indoor/raw`，完整保留，不截断末尾。
- 时长：64.951 s；前约 5 s 静止用于 IMU 初始化，随后覆盖完整室内运动路线。
- 话题：`/iv_points`（650 帧 PointCloud2）和 `/iv_imu`（13040 条 Imu）。
- 格式：ROS2 MCAP，原生 Zstandard chunk 压缩，可直接只读播放。
- MCAP 约 495.6 MiB，Release `tar.gz` 约 431 MiB；全部消息内容和时间戳与原室内包逐字节一致。

## 下载与检查

从仓库 Releases 下载并解压：

```bash
tar -xzf d1r_weak100_indoor_65s.tar.gz
cd d1r_weak100_indoor_65s
sha256sum -c SHA256SUMS.txt
ros2 bag info .
```

## 运行

先按根目录 README 构建代码并 `source install/setup.bash`，然后在仓库根目录执行：

```bash
bash examples/run_example.sh /absolute/path/to/d1r_weak100_indoor_65s
```

默认 1× 播放、无 RViz。D1R 点云订阅使用 reliable QoS；播放器延迟 2 秒开始发布，并在结束前等待可靠订阅确认，防止首帧、中间帧和末帧静默丢失。脚本随后等待 PGO 排空并调用 `/pgo/save_maps`。

结果默认写入当前目录的 `weak100_example_output/<时间戳>/map/`；可通过 `OUTPUT_ROOT` 修改输出位置。也可调整播放参数：

```bash
RATE=0.5 RVIZ=true PLAY_DELAY=3 DRAIN_SECONDS=10 \
  bash examples/run_example.sh /absolute/path/to/d1r_weak100_indoor_65s
```

`WAIT_FOR_ALL_ACKED_MS` 默认值为 10000，设为 0 表示无限等待所有可靠订阅确认。

## 已验证结果

正式 x86_64 ROS2 Humble 上传副本在完整回放中收到并处理 648 组 LIO 输出，PGO backlog 为 0，保存地图包含 112 个关键帧和 1618233 点。原生 weak100 同包参照为 648 次 LIO 输出、113 个关键帧和 1632526 点。

两份地图的双向采样最近邻距离中位数约 8.73 mm、P95 约 24.23 mm。该结果用于检查同包地图几何一致性，不代表绝对精度，也不承诺不同运行时关键帧数量或 PCD 逐字节一致。详细根因和对照见 [`docs/VALIDATION.md`](../docs/VALIDATION.md)。
