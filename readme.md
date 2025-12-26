# 小车/无人机里程计可视化（ROS1 / RViz）

这是一个独立的 `catkin` 工作空间（`catkin_ws`），包含两个轻量可视化功能包：

- `car_visualization`：订阅小车里程计 `nav_msgs/Odometry`，发布 RViz 可用的 `Marker/Path/Pose` 等话题，并显示小车 mesh。
- `drone_visualization`：同样订阅 `nav_msgs/Odometry`，发布可视化话题，并显示无人机 mesh（默认 `yunque-M.dae`）。

## 环境依赖

- Ubuntu + ROS Noetic（ROS1）
- `catkin_make`
- RViz（`rviz`）

## 编译

```bash
cd <this_repo>
source /opt/ros/noetic/setup.bash   # 或 source /opt/ros/$ROS_DISTRO/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## 快速使用（RViz）

### 小车

启动（默认参数见 launch 文件）：

```bash
source devel/setup.bash
roslaunch car_visualization car_visualization.launch
```

推荐显式指定你的 odom 话题与坐标系（更通用）：

```bash
roslaunch car_visualization car_visualization.launch odom:=/your_odom_topic output_frame:=world
```

### 无人机

```bash
source devel/setup.bash
roslaunch drone_visualization drone_visualization.launch
```

同样可 remap：

```bash
roslaunch drone_visualization drone_visualization.launch odom:=/your_odom_topic output_frame:=world
```

### RViz 显示

1. 打开 RViz：`rviz`
2. `Fixed Frame` 设为 `world`（或你设置的 `output_frame`）
3. 添加 Display：
   - `Marker`：订阅 `/<node_name>/robot`（例如 `/car_visualization/robot`、`/drone_visualization/robot`）
   - `Path`：订阅 `/<node_name>/path`
   - 可选：`/<node_name>/trajectory`、`/<node_name>/velocity`

> 注意：如果 RViz 的 `Fixed Frame` 与 marker 的 `header.frame_id` 没有 TF 关系，会显示不出来。最简单的办法是把 `output_frame` 设成 RViz 的 `Fixed Frame`（例如 `world`）。

## 话题（节点命名空间下）

两个包发布的话题一致（以下以 `<ns>` 表示节点名，例如 `car_visualization`）：

- `/<ns>/pose`：`geometry_msgs/PoseStamped`
- `/<ns>/path`：`nav_msgs/Path`
- `/<ns>/velocity`：`visualization_msgs/Marker`（箭头，长度=速度模）
- `/<ns>/trajectory`：`visualization_msgs/Marker`（LINE_STRIP）
- `/<ns>/robot`：`visualization_msgs/Marker`（MESH_RESOURCE）
- `/<ns>/covariance`：`visualization_msgs/Marker`（可选）
- `/<ns>/covariance_velocity`：`visualization_msgs/Marker`（可选）

## 参数（ROS private params）

通用参数（两个包相同）：

- `~output_frame`：为空则使用 `odom.header.frame_id`；建议设为 `world`
- `~mesh_resource`：mesh URI，支持 `package://...` 或 `file://...`
- `~resolve_mesh_resource`：默认 `true`，会把 `package://` 自动解析成 `file://` 绝对路径，降低 RViz 找包失败概率
- `~robot_scale`：mesh 缩放
- `~rotate_yaw_deg`：绕 Z 轴额外旋转（用于修正模型前向轴）
- `~color/r|g|b|a`：颜色
- `~robot_id`：marker id（默认 `-1` 会回退到 `0`）
- `~origin`：`true` 时，以第一次收到的 odom 作为原点（位置与姿态都会变成相对量）
- `~path_append_dt` / `~path_max_points`：路径追加周期/最大点数（0=不限制）
- `~traj_append_dt` / `~traj_max_points`：轨迹追加周期/最大点数（0=不限制）
- `~covariance_position` / `~covariance_velocity` / `~covariance_scale`：协方差球显示

订阅的 odom 话题通过 remap：

- `odom:=/your_odom_topic`（launch 里已做了 `odom` 和 `~odom` 的兼容 remap）

## 常见问题排查

### 1) `/.../robot` 没有任何输出

通常是没订阅到 odom：

```bash
rosnode info /car_visualization   # 或 /drone_visualization
rostopic info /your_odom_topic    # 确认话题存在且类型是 nav_msgs/Odometry
```

启动时显式指定：

```bash
roslaunch car_visualization car_visualization.launch odom:=/your_odom_topic
```

### 2) RViz 能看到 Marker 但模型加载失败（`could not load package://...`）

- 确保节点启动时已 `source devel/setup.bash`
- `~resolve_mesh_resource` 默认开启，会将 `package://` 转成 `file://`；也可直接把 `mesh_resource` 设成 `file:///绝对路径/...`

### 3) 车头方向不对

这是模型本身前向轴与 ROS 约定（+X 前）不一致导致。用 `rotate_yaw_deg` 修正：

```bash
roslaunch car_visualization car_visualization.launch rotate_yaw_deg:=90
```

## 模型（meshes）

- 小车默认：`src/car_visualization/meshes/car.dae`
- 无人机默认：`src/drone_visualization/meshes/yunque-M.dae`
- `src/drone_visualization/meshes/yunque-M.dae` 文件较大，仅作为可选替换模型（启动时用 `mesh_resource:=...` 指定）

## 目录结构

```
src/
  car_visualization/
  drone_visualization/
```
