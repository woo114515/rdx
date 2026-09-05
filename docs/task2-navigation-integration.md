# 任务二导航与任务三分拣接口

> **非比赛默认方案：** 当前任务二使用小车厂家自带导航，任务三默认使用 `/odom_raw`
> 和相对分类区域独立运行。本页只记录可选地图返回接口；在验证厂家导航的话题、动作、TF
> 和速度重映射完全兼容之前，不得按本页启用实车输出。

## 目标

任务三保留视觉识别、最终对准、靠近和推送。任务二提供地图、AMCL 定位和 Nav2；
任务三完成一次投放后，通过 `NavigateToPose` 返回启动时记录的地图起点。

## 任务二必须提供的接口

- `/map`：已保存地图；
- `/amcl_pose`：机器人在 `map` 坐标系中的定位；
- `/odom_raw`：底盘高频里程计；
- `/scan`：MS200P 激光数据；
- `/navigate_to_pose`：Nav2 动作服务；
- `map -> odom -> base_footprint`：连续 TF 链。

Nav2 的控制器输出必须从 `/cmd_vel` 重映射到：

```text
/rdx_sorting/nav_cmd_vel_request
```

Nav2 不得直接发布 `/cmd_vel`。只有 `rdx_sorting_safety` 可以向底盘发布最终速度，
否则输出冲突保护会锁定停车。

## 任务三地图模式参数

任务二完成后，在 `color_sorting.yaml` 中设置：

```yaml
pose_source: amcl
zones_relative_to_origin: false
nav2_return_enabled: true

zones.green.x: 0.0
zones.green.y: 0.0
zones.blue.x: 0.0
zones.blue.y: 0.0
zones.orange.x: 0.0
zones.orange.y: 0.0

navigation_request_enabled: true
lidar_stop_enabled: true
```

上面的区域坐标必须替换为现场测得的 `map` 坐标，不能保留为零。启动时记录的
`/amcl_pose` 是本轮任务的返回点，不需要把小车机械地放在完全相同的位置。

## 启动前检查

任务二启动后运行：

```bash
./scripts/check_task2_nav_ready.sh
```

五个接口均显示 `OK` 后，检查 `/cmd_vel` 没有 Nav2 或遥控器发布者。首次联调必须
保持 `motion_enabled:=false` 和 `output_enabled:=false`，确认状态、定位与动作服务后，
再按架空车轮、低速单物块、落地单物块、完整流程的顺序测试。

## 地图模式启动参数

任务二的 Nav2 已正确重映射后，任务三可使用：

```bash
ros2 launch rdx_color_sorting color_sorting_motion.launch.py \
  pose_source:=amcl zones_relative_to_origin:=false \
  nav2_return_enabled:=true navigation_request_enabled:=true \
  lidar_stop_enabled:=true
```

上述命令默认仍不允许运动。实车验证阶段才分别显式设置
`motion_enabled:=true` 与 `output_enabled:=true`。
