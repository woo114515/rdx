# 2026-09-03 MS200P 与 RViz 现场测试记录

## 测试环境

- 小车：RDK X5，Ubuntu 22.04，ROS 2/TROS Humble
- 开发机：Ubuntu 24.04，ROS 2 Jazzy
- 小车地址：`192.168.43.109`（热点 DHCP 地址，仅代表本次测试）
- 开发机热点地址：`192.168.43.23`
- 隔离测试域：`ROS_DOMAIN_ID=98`，后续最小化测试曾改用 67
- 实际雷达包：`oradar_lidar`
- 驱动工作区：`/home/sunrise/software/library_ws`
- 雷达设备：`/dev/oradar -> /dev/ttyACM0`
- 项目部署目录：`/home/sunrise/rdx`

## 已验证结果

1. `rdx_safety`、`rdx_bringup`、`rdx_navigation`、`rdx_mission` 已在小车的 Humble 环境构建成功。
2. `mapping.launch.py` 在 `start_hardware:=false` 时成功加载 SLAM Toolbox 和 Ceres solver。
3. `navigation.launch.py` 在 `start_hardware:=false` 时成功加载已有 `test_map.yaml`，并配置 AMCL、DWB、全局/局部 costmap 和自定义行为树。因为故意关闭硬件，最终等待缺失的 `odom -> base_footprint`，符合预期。
4. 传感器测试使用 `start_driver:=false`，未启动底盘和电机。MS200P 成功打开 `/dev/oradar`，日志出现：

   ```text
   lidar device connect succuss.
   get lidar scan data
   ROS topic:scan
   ```

5. URDF 中确认存在 `base_footprint`、`base_link`、`lidar_link` 和 `imu_link` 等 frame。
6. 安全节点保持 `emergency_stop` 和 `footprint_unverified` 双重锁定。本次测试没有解除急停、没有发布运动命令，小车没有行驶。
7. 开发机 RViz 曾短暂显示真实扫描点，证明雷达数据内容可用。

## 发现的问题

### 1. 厂商雷达驱动稳定性

- 一次运行约 5 分钟后，`oradar_scan` 以 `exit code -11`（段错误）退出，随后 `/scan` 的 publisher 数量变为 0。
- 一次接收 SIGINT 退出时触发 `buffer overflow detected`，以 `exit code -6` 结束。
- 项目已改为直接启动 `oradar_lidar/oradar_scan`，并设置 `respawn=True`、`respawn_delay=2.0`。相关回归测试已加入，但自动恢复尚未完成长时间实机验收。

### 2. Humble 与 Jazzy 直接 DDS 通信不稳定

开发机 Jazzy RViz 和小车 Humble 节点进入同一 ROS Domain 后，小车多节点出现：

```text
Fast CDR exception deserializing message of type
rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_
'Bad alloc' exception deserializing message
```

ROS 2 不保证不同发行版之间的节点能够正确通信。本次表现为 RViz 偶尔短暂显示扫描点，随后无数据或界面卡死。因此不能把 Jazzy 直接连接 Humble DDS 作为正式演示方案。

### 3. 热点与 SSH

- 手机热点曾掉线，开发机临时切换到 `10.195.0.0/16`，导致无法访问小车的 `192.168.43.109`。
- 重新接入同一热点后 ping 恢复，但小车已有较多 SSH 会话，偶尔出现 SSH banner exchange 超时。
- 测试期间创建过多个空的 `rdx_lidar` screen 会话，后续已全部关闭。

### 4. 蜂鸣告警

最后一次测试中小车出现持续蜂鸣。停止 ROS 雷达和临时 TF 进程后蜂鸣仍未停止，关闭小车电源后才停止。告警来源尚未确认，可能与电池欠压、供电或其他硬件状态有关，不能把它归因于 RViz 或雷达软件。

### 5. 存储空间

小车根分区在部署构建后约为 92% 使用率，只剩约 2.3 GB。后续录制 rosbag、重复构建或保存地图前必须先检查空间。

## 测试结束状态

- 本次启动的 `rdx_bringup`、`rdx_safety`、`oradar_scan` 和临时 TF 进程均已停止。
- 三个 `rdx_lidar` screen 会话已关闭。
- `/dev/oradar` 已释放。
- 小车已由现场人员断电。
- 未修改或停止队友的相机、网页服务及其他非本项目进程。

## 下次测试门槛

1. 先完成充电并检查电池电压、供电连接和蜂鸣器含义；只上电不启动 ROS，观察是否仍蜂鸣。
2. 若上电即蜂鸣，停止测试并处理硬件/电源问题。
3. 检查 `df -h /`，为地图和日志预留空间。
4. 避免让 Jazzy RViz 直接加入 Humble 的 ROS Domain。优先使用小车已有 `rosbridge_server`，通过 WebSocket 在开发机浏览器可视化；或准备与小车一致的 Humble 可视化环境。
5. 恢复测试时先只启动 MS200P，确认 `/scan` publisher 和频率；再验证自动重启，最后才进入锁定状态下的底盘/SLAM 检查。
6. 在实测 footprint 并通过架空车轮测试前，不得解除项目安全锁或发送非零速度。
