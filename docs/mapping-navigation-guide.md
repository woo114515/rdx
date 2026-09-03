# 任务二：建图与自主导航操作手册

本手册适用于 RDK X5 + ROSMASTER X3 + MS200P。厂商驱动、Nav2 和 SLAM Toolbox 使用机器人已有的 Humble 安装，不需要把所有包下载到仓库。

## 0. 上电前检查

1. 给电池充足电，清空场地并安排一名操作员随时可断电。
2. 测量车体和固定叉形结构从 `base_footprint` 中心到最远点的距离。把实测值加至少 0.05 m，分别写入 `config/nav2.yaml` 的 local/global costmap `robot_radius`；未测量前严禁解锁运动。
3. 建图和导航都先使用 `emergency_stop_on_start:=true`。急停解除前确认车轮架空或周围无人。

## 1. 连接与部署

机器人通过热点或有线 DHCP 获取地址。不要重启 NetworkManager/wpa_supplicant；在机器人串口终端运行 `ip -br addr`，取 `wlan0` 或 `usb0` 的 IPv4 地址，然后从开发机连接：

```bash
ssh sunrise@<当前IP>
```

开发机和机器人必须使用相同的 `ROS_DOMAIN_ID`（项目约定为 99）。在机器人上：

```bash
source /opt/tros/humble/setup.bash
source ~/yahboomcar_ws/install/setup.bash
cd ~/rdx
colcon build --symlink-install --packages-select rdx_safety rdx_bringup rdx_navigation rdx_mission
source install/setup.bash
```

## 2. 硬件与安全冒烟测试

先保持锁定状态检查传感器和 TF：

```bash
ros2 launch rdx_bringup hardware.launch.py \
  footprint_verified:=false emergency_stop_on_start:=true
ros2 topic hz /scan
ros2 topic echo /rdx_safety/state
ros2 run tf2_ros tf2_echo odom base_footprint
```

确认 `/scan` 持续发布、`odom -> base_footprint` 存在且安全状态为锁定。架空车轮后，在有人守着急停的情况下解除急停：

```bash
ros2 topic pub --rate 2 /emergency_stop std_msgs/msg/Bool "{data: false}"
```

验证无指令和断开遥控后 `/cmd_vel` 回到零；再发布 `true`，确认立即停车。测试结束按 `Ctrl-C`。只有完成真实尺寸测量并更新参数后，才把 `footprint_verified:=true` 用于运动。

## 3. 低速建图

在机器人终端启动：

```bash
ros2 launch rdx_navigation mapping.launch.py \
  footprint_verified:=true emergency_stop_on_start:=true \
  max_linear_speed:=0.12
```

另开终端解除急停，并使用开发机或机器人上的遥控节点，让它把速度发布到安全仲裁输入：

```bash
ros2 topic pub --rate 2 /emergency_stop std_msgs/msg/Bool "{data: false}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/cmd_vel_teleop
```

以不超过 0.12 m/s 的速度沿场地边界和通道缓慢往返，避免急转和遮挡雷达。开发机有图形桌面时运行 `rviz2`，设置 Fixed Frame 为 `map`，添加 `Map`、`LaserScan`、`TF`；SSH 纯终端不能直接显示 RViz。

建图完成且机器人回到起点附近后保存：

```bash
mkdir -p ~/rdx_maps
ros2 run nav2_map_server map_saver_cli -f ~/rdx_maps/task2
```

应得到 `task2.yaml` 和 `task2.pgm`（或 `.png`）。备份这两个文件，不要把未经审核的实地图提交为默认地图。

## 4. 记录航点

在 RViz 用 `2D Goal Pose` 读取四个位姿（顺序：任务点 1、2、3、起点）。也可执行 `ros2 topic echo /goal_pose --once` 获取位置和四元数，再换算 yaw。编辑 `src/rdx_navigation/config/waypoints.yaml`：保留精确名称和顺序，填入实测 `x`、`y`、`yaw`，确认 `frame_id: map`，最后把 `mission.ready` 改为 `true`，然后重新构建。

## 5. 静态地图导航与任务

```bash
ros2 launch rdx_navigation navigation.launch.py \
  map:=/home/sunrise/rdx_maps/task2.yaml \
  footprint_verified:=true emergency_stop_on_start:=true
ros2 run rdx_mission rdx_mission_node \
  --ros-args -p waypoint_file:=/home/sunrise/rdx/src/rdx_navigation/config/waypoints.yaml
```

在 RViz 设置 `2D Pose Estimate` 初始化 AMCL，确认机器人位姿与地图重合，再解除急停并启动任务：

```bash
ros2 topic pub --rate 2 /emergency_stop std_msgs/msg/Bool "{data: false}"
ros2 service call /mission/start std_srvs/srv/Trigger {}
```

任务严格按 `task_1 → task_2 → task_3 → start` 执行；目标失败不会跳过。临时障碍由局部 costmap 和激光层处理，持续阻塞时保持安全停车并由操作员决定取消或清场。

取消任务或执行急停：

```bash
ros2 service call /mission/cancel std_srvs/srv/Trigger {}
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
```

## 6. 故障排查

- `/scan` 无数据：检查 MS200P 电源、串口权限和 `ros2 topic list`；确认厂商 launch 名称仍为 `ms200_scan.launch.py`。
- 安全节点显示 `footprint_unverified`、`emergency_stop`、`scan_stale` 或 `command_stale`：这是预期的失效保护，先修复原因，不要绕过 `/cmd_vel`。
- Nav2 报地图不存在：`map:=` 必须指向机器人上实际存在的 `.yaml`，并能读取同目录图像文件。
- 任务节点提示配置未就绪：检查 `mission.ready: true`、四个名称及顺序，且坐标均为有限数。
- RViz 无法从 SSH 打开：在有桌面的开发机运行 RViz，并设置相同 `ROS_DOMAIN_ID`；SSH 只用于启动节点和查看话题。

当前仓库未包含实机场地地图和航点，默认 `ready: false` 是有意的安全锁。完成一次实机建图、尺寸测量和航点复核后再解锁。
