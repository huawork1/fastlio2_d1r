# weak100 算法说明

版本标识：`D1R-Liang / normal1500 / auto3m / weak100`，认可日期为 2026-10-28。

## 算法组成

- **LIO**：保留 FAST-LIO2 的 IMU 传播、逐点去畸变、IESKF、ikd-Tree 和 5NN 点面关联；对有效观测按 42 个无向法向桶均衡选择，每次迭代最多 1500 个观测。
- **自动回环**：历史关键帧中心搜索半径 3 m，时间间隔大于 60 s；ICP 失败后继续尝试后续候选。D1R 前向候选门使用 140°×100° 名义视场，点数比例至少 0.5，前向视轴夹角不超过 90°。
- **weak100**：从 ICP 对齐后的局部点面支持构造平移几何矩阵 `Htt`。特征值小于最大特征值 0.1 倍的方向，信息权重乘 0.01，等价于方差乘 100；旋转块和旋转—平移交叉块沿用原模型。
- **PGO**：起点先验、相邻里程计边和已接受回环边输入 iSAM2；不向 LIO 反馈状态，不重建前端增量树。

运行入口显式设置：

```bash
LIO_SAMPLE_MODE=normal
LIO_SAMPLE_BUDGET=1500
PGO_RETRY_CANDIDATES=1
PGO_DIRECTION_MODE=weak100
```

`weak100` 模式要求可写的 `PGO_DIRECTION_AUDIT_PATH`。`ros2 launch pgo weak100.launch.py` 在该变量未设置时，把审计文件写到当前目录的 `weak100_runs/launch_default/direction_audit.jsonl`。`scripts/run_weak100.sh` 会为每次运行创建独立目录并改指向那里。

## 已验证结果

原始约 600 秒参考包的历史完整回放得到 5993 帧点云、5991 个 LIO 输出、769 个关键帧和 6 条自动回环。当时保留版本的输入、前端轨迹、关键帧、回环测量和最终地图完成了一致性核验。

这些验证说明指定数据上的重复观测一致性，不是外部位姿真值或全图绝对精度。部分非 ROI 尾部指标仍存在取舍；运动建图、持续实时吞吐和跨平台行为应分别验收。

## 上传副本边界

- 保留 ROS2 的 `interface`、`fastlio2`、`pgo` 三个必要包。
- LIO 节点采用 D1R 原始 `/iv_points` PointCloud2 和 `/iv_imu` 输入。
- 算法核心文件保持 2026-10-28 接受版本的 SHA256；配置只调整 ROS 话题与坐标帧，算法参数采用该版本的值。
- 不包含 bag、PCD、地图、checkpoints、实验结果、预编译库、Docker 镜像、厂商 SDK、HBA、localizer、GLIM 或后续优化实验。
- 原生 Python 回放使用的预编译 `libweak100_live.so` 不在本库中；本库提供 ROS2 源码构建路径，不宣称可从历史对象逐字节重建该预编译库。
