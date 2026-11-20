# fastlio2_d1r

基于 Liang [`FASTLIO2_ROS2`](https://github.com/liangheming/FASTLIO2_ROS2) @ `7baa59f` 修改，适配 Seyond Hummingbird D1-R。前端按 42 个法向桶均衡采样，每次迭代最多 1500 个观测；后端做 3 m 自动回环，并把回环平移的弱方向协方差交给 iSAM2。仓库只放 ROS2 编译和运行需要的代码、配置和说明。

## 特点

- D1R `/iv_points` PointCloud2 与 `/iv_imu` 直接输入，保留逐点时间用于去畸变；点云默认 reliable QoS，避免大数据帧静默丢失。
- FAST-LIO2 前端按 42 个法向桶均衡选择观测，每次迭代最多 1500 个。
- 3 m 自动回环搜索、候选失败重试和 D1R 前向视野候选门。
- 回环平移的弱方向协方差，以及 iSAM2 位姿图优化。
- 关键帧地图保存服务及可选 RViz2 显示。

## 仓库结构

```text
src/
  interface/   ROS2 服务定义
  fastlio2/    D1R 输入与 LIO
  pgo/         自动回环与 PGO
scripts/
  run_mapping.sh
  verify_baseline.py
docs/BASELINE.md
baseline_manifest.json
```

## 依赖

- Ubuntu 22.04、ROS2 Humble
- C++17、PCL、Eigen3、yaml-cpp、OpenMP
- Sophus（历史验证环境为 1.22.10）
- GTSAM（历史验证环境为 4.2.0）
- 可选 RViz2

请确保 Sophus 和 GTSAM 的 CMake 包可通过 `CMAKE_PREFIX_PATH` 找到。

## 编译

在仓库根目录执行。Sophus 和 GTSAM 的 CMake 包要能被找到；本机若装在独立前缀，把它们加进 `CMAKE_PREFIX_PATH`。

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/path/to/sophus:/path/to/gtsam:${CMAKE_PREFIX_PATH:-}
colcon build --symlink-install --packages-select interface fastlio2 pgo
source install/setup.bash
```

编译产物在 `build/`、`install/`、`log/`，不进入版本库。GTSAM 装在独立前缀时，CMake 通常会把库目录写进 `pgo_node` 的运行路径；若启动仍提示找不到 `libgtsam.so.4`，把该库目录加入 `LD_LIBRARY_PATH`。

## 使用

启动入口是 `ros2 launch pgo mapping.launch.py`。它同时拉起 LIO 和 PGO，并在节点起来前写入下表。调用前已经导出的同名变量不会被改掉。

| 变量 | 缺省 |
|---|---|
| `LIO_SAMPLE_MODE` | `normal` |
| `LIO_SAMPLE_BUDGET` | `1500` |
| `PGO_RETRY_CANDIDATES` | `1` |
| `PGO_DIRECTION_MODE` | `weak100` |
| `PGO_DIRECTION_AUDIT_PATH` | `./mapping_runs/launch_default/direction_audit.jsonl` |

D1-R 驱动需要发布 `/iv_points`（`sensor_msgs/PointCloud2`，带逐点时间）和 `/iv_imu`（线加速度 m/s²，角速度 rad/s）。点云订阅默认 `lidar_reliable: true`，与当前驱动和示例 bag 的 reliable 发布端一致。发布端是 best-effort 时，把 `src/fastlio2/config/lio.yaml` 里的该项改成 `false`。`t_il = [0, 0, 0]` 是壳内杆臂的零初值，厂家手册没有给出这个距离。

### 实车

先编译并 `source install/setup.bash`。启动后保持雷达静止，等 IMU 初始化结束再移动。

```bash
ros2 launch pgo mapping.launch.py
```

需要画面时加上 RViz。配置在 `src/pgo/rviz/pgo.rviz`。

```bash
ros2 launch pgo mapping.launch.py rviz:=true
```

建图结束后保存。`file_path` 必须是已经存在的目录，成功后里面有 `map.pcd`、`poses.txt` 和 `patches/`。

```bash
mkdir -p /path/to/map
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: /path/to/map, save_patches: true}"
```

### 回放 bag

另开一个终端，等上面的节点起来后再播放。reliable 点云需要启动发现延时和结束确认，否则首尾帧会丢掉。

```bash
ros2 bag play /path/to/bag --rate 1 --delay 2 \
  --wait-for-all-acked 10000 --disable-keyboard-controls
```

播放结束后等几秒，再调用上面的保存服务。

约 65 秒室内示例的校验、解压和一键脚本见 [`examples/README.md`](examples/README.md)：

```bash
bash examples/run_example.sh /absolute/path/to/d1r_weak100_indoor_65s
```

### 单独的输出目录

直接 launch 时，方向审计写在当前目录的 `mapping_runs/launch_default/direction_audit.jsonl`。要按次分目录，并额外记录视野门，用启动脚本。它会建立 `mapping_runs/<时间戳>/`，再启动同一个 launch 文件。

```bash
bash scripts/run_mapping.sh
RVIZ=true bash scripts/run_mapping.sh
OUTPUT_ROOT=/path/to/results bash scripts/run_mapping.sh
```

数值配置在 `src/fastlio2/config/lio.yaml` 和 `src/pgo/config/pgo.yaml`。

## 源码核对

```bash
python3 scripts/verify_baseline.py
```

该命令按 SHA256 核对算法源文件和 D1-R 接入文件。算法说明见 [`docs/BASELINE.md`](docs/BASELINE.md)，编译与运行记录见 [`docs/VALIDATION.md`](docs/VALIDATION.md)。

## 不包含的内容

原始 bag、PCD/地图、checkpoints、实验结果、日志、预编译库、Docker 镜像、厂商 SDK、HBA、localizer、GLIM 和后续运行优化实验均未纳入。原生预编译库回放不在本仓库中；这里提供的是 ROS2 源码构建路径。

## 来源

上游项目：[`liangheming/FASTLIO2_ROS2`](https://github.com/liangheming/FASTLIO2_ROS2) @ `7baa59f7339ba88ce3b646fe35b23f61031c5d3f`。MIT 许可文本保留在各包及根目录；发布前请阅读 `NOTICE.md` 并确认新增修改的版权署名。

## TODO

- 退化检测
- 退化场景优化

## Contributors

- Codex
- Cursor
- Gemini
- DeepSeek
- Doubao
