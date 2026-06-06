# Height Measurements for elmap RL Controller

本包提供四足机器人 elmap RL controller 使用的高程观测工具链：

- 离线：把室内 `.pcd` 点云地图转换为规则二维全局高程图。
- 在线：C++ ROS2 节点读取 `.npy + metadata.yaml`，根据机器人全局 odometry 发布 12x11=132 维 `/height_measurements`。
- Goal：C++ ROS2 节点读取有序 map-frame goal 点，发布 parkour policy 需要的 `/parkour/goal_yaw`。
- 验证：Python 采样器和单点查询脚本用于检查坐标系、采样顺序和公式。

## 1. 安装依赖

ROS2/C++ 依赖：

```bash
sudo apt update
sudo apt install -y ros-humble-nav-msgs libyaml-cpp-dev
```

Python 离线工具依赖：

```bash
python3 -m pip install open3d numpy scipy matplotlib pyyaml
```

也可以使用：

```bash
python3 -m pip install -r requirements.txt
```

## 2. 离线生成高程图

```bash
conda activate elmap_heightmeasure


python3 tools/pcd_to_global_heightmap.py \
  /path/to/room_map.pcd \
  --output_dir /path/to/room_heightmap \
  --resolution 0.025 \
  --voxel_size 0.025 \
  --z_mode min \
  --frame_id map
```

默认会在体素降采样前先删除 `z > 1.0m` 的点，以去掉天花板、墙面高处和杂散高点。需要保留 1 米以上点云时，加上 `--z_max none`；需要调整阈值时用 `--z_max 0.8` 这类数值。

如果只需要给在线节点使用的 `.npy + metadata.yaml`，可以先跳过 PNG 预览来减少耗时和内存：

```bash
python3 tools/pcd_to_global_heightmap.py \
  all_raw_points.pcd \
  --output_dir ./room_heightmap \
  --resolution 0.025 \
  --voxel_size 0.025 \
  --z_mode min \
  --frame_id map \
  --skip_png
```

大范围稀疏地图的 NaN 填补也会耗时；调试时可先用 `--fill_iterations 0` 或较小迭代数快速生成结果。不要把 `--output_dir` 设成 `/`，否则会尝试把输出文件写到系统根目录。

生成文件：

- `global_heightmap_raw.npy`：未填补 NaN 的二维高程矩阵。
- `global_heightmap_filled.npy`：局部均值填补后的二维高程矩阵，在线节点默认使用它。
- `metadata.yaml`：数组索引和世界坐标的映射信息。
- `global_heightmap_raw.png`：raw 地图可视化。
- `global_heightmap_filled.png`：filled 地图可视化。

`global_heightmap_filled.npy` 只是二维 NumPy 数组，本身不自带世界坐标。必须同时使用 `metadata.yaml` 中的 `origin_x`、`origin_y`、`resolution`、`width`、`height`、`frame_id` 解释它：

```text
world_x = origin_x + col * resolution
world_y = origin_y + row * resolution
terrain_z = height_map[row, col]
```

反向查询：

```text
col = int((world_x - origin_x) / resolution)
row = int((world_y - origin_y) / resolution)
terrain_z = height_map[row, col]
```

## 3. 编译 ROS2 C++ 节点

在包含本包的 ROS2 workspace 根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select height_measurements --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

当前目录直接测试时也可以使用独立 build/install 目录：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/height_measurements_log build \
  --packages-select height_measurements \
  --build-base /tmp/height_measurements_build \
  --install-base /tmp/height_measurements_install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source /tmp/height_measurements_install/setup.bash
```

## 4. 在线运行节点

使用 `ros2 run`：

```bash
ros2 run height_measurements height_measurement_node \
  --ros-args \
  -p heightmap_path:=/path/to/room_heightmap/global_heightmap_filled.npy \
  -p metadata_path:=/path/to/room_heightmap/metadata.yaml \
  -p odom_topic:=/aft_mapped_in_map \
  -p map_frame:=map \
  -p base_frame:=base_link \
  -p publish_topic:=/height_measurements \
  -p publish_rate:=50.0 \
  -p height_formula:=legged_gym \
  -p measured_height_offset:=0.3 \
  -p base_to_odom_x:=0.16266 \
  -p base_to_odom_y:=0.0 \
  -p base_to_odom_z:=0.11703
```

使用 launch：

```bash
ros2 launch height_measurements height_measurement.launch.py \
  heightmap_path:=/home/getting/humble/Quadruped/Height_Measurements/room_heightmap/global_heightmap_filled.npy \
  metadata_path:=/home/getting/humble/Quadruped/Height_Measurements/room_heightmap/metadata.yaml \
  odom_topic:=/aft_mapped_in_map \
  map_frame:=map \
  base_frame:=base_link \
  publish_topic:=/height_measurements \
  publish_rate:=50.0 \
  height_formula:=legged_gym \
  measured_height_offset:=0.3 \
  base_to_odom_x:=0.16266 \
  base_to_odom_y:=0.0 \
  base_to_odom_z:=0.11703
```

如果不通过 `ros2 run`，编译后可直接运行二进制：

```bash
/tmp/height_measurements_build/height_measurements/height_measurement_node \
  --ros-args \
  -p heightmap_path:=/path/to/room_heightmap/global_heightmap_filled.npy \
  -p metadata_path:=/path/to/room_heightmap/metadata.yaml
```

节点默认订阅 `/aft_mapped_in_map` 的 `nav_msgs/msg/Odometry`。如果该 odometry 是雷达/SLAM 原点而不是机身 `base_link` 原点，节点会先用 `base_to_odom_x/y/z` 把雷达位姿换算成 base 位姿，再做高程采样。

当前默认外参按本机设置为：

```text
base_to_odom_x = 0.16266 m   # 雷达在 base 原点前方 162.66 mm
base_to_odom_y = 0.0 m
base_to_odom_z = 0.11703 m   # 雷达在 base 原点上方 117.03 mm
```

换算公式为 yaw-only：

```text
base_x = odom_x - (cos(yaw) * base_to_odom_x - sin(yaw) * base_to_odom_y)
base_y = odom_y - (sin(yaw) * base_to_odom_x + cos(yaw) * base_to_odom_y)
base_z = odom_z - base_to_odom_z
```

如果你的 odometry 已经是 `base_link` 原点，启动时把这三个参数都设成 `0.0`。采样网格只跟随 yaw，不跟随 roll/pitch 倾斜。

## 5. Goal yaw manager

`parkour_goal_manager_node` 从 YAML 读取有序 goal 点，订阅雷达/SLAM odometry，按当前 base 位姿逐个追踪 goal，并发布 deploy 端直接可用的：

```text
/parkour/goal_yaw = [0.0, delta_yaw_to_current_goal, delta_yaw_to_next_goal]
```

Goal 点固定在 `map` 坐标系下，不是启动时雷达局部坐标。YAML 格式：

```yaml
frame_id: map
goals:
  - [1.0, 0.0]
  - [2.0, 0.3]
  - [3.0, 0.0]
```

只使用每个 goal 的 `x/y`；如果写了第三个 `z`，节点会忽略。示例文件在 `config/parkour_goals_example.yaml`。

使用 `ros2 run`：

```bash
ros2 run height_measurements parkour_goal_manager_node \
  --ros-args \
  -p goal_points_path:=/path/to/parkour_goals.yaml \
  -p odom_topic:=/aft_mapped_in_map \
  -p goal_yaw_topic:=/parkour/goal_yaw \
  -p map_frame:=map \
  -p publish_rate:=50.0 \
  -p goal_reach_threshold:=0.2 \
  -p reach_goal_delay:=0.1 \
  -p base_to_odom_x:=0.16266 \
  -p base_to_odom_y:=0.0 \
  -p base_to_odom_z:=0.11703
```

使用 launch：

```bash
ros2 launch height_measurements parkour_goal_manager.launch.py \
  goal_points_path:=/path/to/parkour_goals.yaml \
  odom_topic:=/aft_mapped_in_map \
  goal_yaw_topic:=/parkour/goal_yaw \
  map_frame:=map
```

节点使用和高程节点相同的 yaw-only 雷达到 base 换算。当前 goal 距 base 小于 `goal_reach_threshold` 并持续 `reach_goal_delay` 后，会切换到下一个 goal；最后一个 goal 抵达后继续发布 `[0.0, 0.0, 0.0]`，避免 deploy fallback。

检查输出：

```bash
ros2 topic echo /parkour/goal_yaw --once
```

## 6. 高度公式

`height_formula` 支持：

- `terrain_minus_base`：`h = terrain_z - robot_z`
- `base_minus_terrain`：`h = robot_z - terrain_z`
- `legged_gym`：`h = robot_z - measured_height_offset - terrain_z`

默认使用训练时的高程观测：

```text
h = clip(base_z - 0.3 - terrain_z, -1.0, 1.0)
```

平地上如果 `base_z` 约 `0.45m`，高程 observation 大概是 `0.15`。如果 controller 的 observation 定义与训练不同，修改启动参数即可：

```bash
-p height_formula:=terrain_minus_base
```

输出默认裁剪到 `[-1.0, 1.0]`，可用 `clip_min` / `clip_max` 调整。

## 7. 坐标系检查

PCD 地图坐标系必须和 SLAM / 重定位输出的 `map` 坐标系一致。理想情况：

```text
PCD point cloud frame = SLAM map frame = heightmap metadata frame
```

如果 PCD 曾被 CloudCompare 或其他工具手动平移、旋转、裁剪，需要额外外参 `T_heightmap_from_slam_map`。当前版本不实现这个外参变换，因此应先保证输入 PCD 与 odometry 所在 `map` 对齐。

检查机器人位姿：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

检查输出：

```bash
ros2 topic echo /height_measurements --once
ros2 topic hz /height_measurements
```

查询单点高度：

```bash
python3 tools/query_height_at.py \
  --heightmap /path/to/room_heightmap/global_heightmap_filled.npy \
  --metadata /path/to/room_heightmap/metadata.yaml \
  --x 1.0 \
  --y 2.0
```

## 8. 为什么在线不用 PCD 查询

在线阶段不读取原始 PCD。PCD 查询需要点云搜索，复杂度高，也容易引入动态内存和延迟抖动。高程图查询是 O(1) 数组访问，每帧只查 132 个格点，更适合 50Hz / 100Hz 的 RL controller 实时运行。

## 9. 采样顺序

默认采样点：

```python
x_points = [-0.45, -0.3, -0.15, 0.0, 0.15, 0.3, 0.45, 0.6, 0.75, 0.9, 1.05, 1.2]  # 12
y_points = [-0.75, -0.6, -0.45, -0.3, -0.15, 0.0, 0.15, 0.3, 0.45, 0.6, 0.75]  # 11
xx, yy = np.meshgrid(x_points, y_points, indexing="ij")
local_points = np.stack([xx.reshape(-1), yy.reshape(-1)], axis=1)
```

因此输出顺序是：每个 `x` 从 `-0.45` 到 `1.2`，在每个 `x` 下 `y` 从 `-0.75` 到 `0.75`，总长度 132。也就是先固定第一个 `x`，遍历所有 `y`，再进入第二个 `x`。

## 10. 自检

```bash
python3 -m unittest discover -s tests
```

自检覆盖：

- `x_points=12`，`y_points=11`，`local_points=132`
- `world_to_grid` / `grid_to_world`
- `sample()` 输出 `(132,)`
- `sample_as_grid` 输出 `(12, 11)`
- `yaw=0` 和 `yaw=pi/2` 的采样方向

## 11. controller 维度兼容

本包发布 132 维高度。`elmap-rl-controller/deploy_cpp` 也需要编译期维度、YAML 配置和 JIT policy 输入维度一致。

如果 controller 或 JIT policy 是按其他高度网格训练的，不能直接混用 132 维高度观测，否则 CSE observation 维度会变化。请使用按 12x11 高度网格训练并导出的 `adaptation_module_latest.jit` / `body_latest.jit`。
