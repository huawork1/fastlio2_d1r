# fastlio2_d1r

Seyond Hummingbird D1-R 上的 FAST-LIO2 建图工程。前端沿用 Liang 的 ROS 2 移植（[`liangheming/FASTLIO2_ROS2`](https://github.com/liangheming/FASTLIO2_ROS2) @ `7baa59f`），输入改为 D1-R 点云和内置 IMU；后端保留 `simple_pgo` 的回环与位姿图。

本仓库先放说明。源码、配置和回放入口随后再提交。

## 传感器与输入

- 雷达：Seyond Hummingbird D1-R，100BASE-TX，默认地址 `172.168.1.10`，端口 `8010`
- IMU：内置 ICM-45686
- 名义前向视场：140° × 100°
- 点云：`/iv_points`（`sensor_msgs/PointCloud2`，带逐点时间戳）
- IMU：`/iv_imu`（m/s²、rad/s）
- 外参：本机核对过的旋转为 `r_il = diag(1, -1, -1)`；平移 `t_il = [0, 0, 0]` 仍是待标定初值

驱动把点云转到前-左-上。点云和 IMU 的物理轴并不因为相同的 `frame_id` 就已经对齐。

## 当前基线

本地基线名称：**D1R–Liang / normal1500 / auto 3 m / weak100**。

2026-10-28 经完整 SLAM 回放认可，2026-11-03 在本机回退并复核到这一版。后续改动以它为比较基线。Orin 上的部署需要单独核对，不能用本机回放结果代替。

相对 Liang 原流程，当前基线多了这些步骤：

| 环节 | 当前设置 |
|---|---|
| 抽点 | 每 3 点取 1，在扫描体素之前 |
| 距离 | 0.5–50 m |
| 扫描体素 / 地图体素 | 0.12 m / 0.15 m |
| 前端法向均衡 | 42 个无向法向桶，每次迭代最多 1500 个观测 |
| 回环候选视野 | 名义视野内点数比例 ≥ 0.5，前向视轴夹角 ≤ 90° |
| 候选搜索 | 历史关键帧中心 3 m；某个候选 ICP 失败后继续下一个 |
| 回环权重 | 按该次回环的平移方向支持，只降低弱方向的信息（weak100） |

数据流是：D1-R 驱动 → FAST-LIO2 → PGO。前端 ikd-Tree 地图和后端关键帧位姿图分开维护，PGO 不把优化结果写回 LIO 状态。

TF：`map → odom → d1r_imu`。`d1r_imu` 是雷达 IMU 坐标系。

## 上游

- [liangheming/FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2) @ `7baa59f7339ba88ce3b646fe35b23f61031c5d3f`
- [Seyond-Inc/seyond_ros_driver](https://github.com/Seyond-Inc/seyond_ros_driver)

## 状态

已认可的是本机回放基线。它不是全图绝对精度，也不能据此认为跨平台结果已经对齐。源码尚未放入本仓库。
