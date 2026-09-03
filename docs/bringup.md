# 首次开机与接入检查

## 目的

先确认平台事实，再启动任何硬件驱动。静态盘点不安装软件、不修改厂商服务、不启动电机；结果用于确定工作空间、设备路径、ROS 接口和后续验证顺序。

## 安全准备

1. 将机器人放在宽阔平整区域，固定叉形结构远离人员和物品。
2. 保持底盘电机未使能；若无法独立断开电机，则架空四个车轮。
3. 确认操作人员能够立即切断底盘电源。
4. 不要在 RDK X5 通电时插拔 IMX219 排线。

## 只读采集

从开发机仓库根目录运行：

```bash
robot_ip=192.168.49.25  # 替换为当前 DHCP 地址
ssh -o IdentitiesOnly=yes \
  -i ~/.ssh/rdx_ed25519 \
  sunrise@"${robot_ip}" 'bash -s' \
  < scripts/collect_system_info.sh
```

直连或热点 IP 由 DHCP 分配。先从热点已连接设备查看，或在开发机运行
`ip neigh show dev <interface>`。上午曾观察到小车地址从 `192.168.49.8`
变为 `192.168.49.25`，不要把租约地址写死。输出可能包含主机名、设备路径和软件版本，不应提交原始诊断日志。

## 已确认事实

- RDK X5 V1.0，Ubuntu 22.04.5，RDK OS 3.0.0，aarch64
- ROS 2 Humble 与 TROS Humble；系统环境为 `/opt/tros/humble/setup.bash`
- Yahboom 工作区为 `/home/sunrise/yahboomcar_ws`，并由 `.bashrc` 自动叠加
- MS200P 为 `/dev/oradar -> /dev/ttyACM0`
- 底盘控制板使用 `/dev/myserial`；USB 枚举曾在 `ttyUSB0` 和 `ttyUSB1` 间变化
- 根分区 27 GiB，2026-09-03 已使用约 87%，剩余约 3.4 GiB
- 初始盘点时无业务 ROS 节点；启动测试前仍需检查和清理重复节点
- `yahboom_oled.service` 已在故障排查期间禁用

完整接口和风险记录见 [实机平台基线](platform-baseline.md)。厂商底盘驱动的本地安全修改、备份和回滚过程见 [底盘驱动安全补丁](vendor-driver-safety-patch.md)。

## 后续验证顺序

每次只增加一个变量，并记录启动命令、话题类型、频率、QoS、帧名和异常：

1. 架空车轮后启动底盘状态链。最小启动顺序是先运行 `yahboomcar_base_node/base_node`，再运行 `yahboomcar_bringup/Mcnamu_driver`；不要使用厂商完整 bringup。
2. 验证 `/driver_node` 订阅 `/cmd_vel`、发布 `/vel_raw`，且 `/base_node` 订阅 `/vel_raw`、发布 `/odom_raw`。同时检查 `/voltage` 是否能返回控制器读数。
3. 单独启动 MS200P，验证 `/scan` 与 `lidar_link`。
4. 单独启动 IMX219，验证 `/csi/image_raw/compressed`。
5. 核对 `odom -> base_footprint` 和机器人描述中的传感器静态 TF。
6. 确认停车路径后，才做低速运动测试。

不要直接启动厂商完整 bringup 后立即遥控；该 launch 同时包含底盘、状态估计、手柄和机器人描述，需先排除意外输入。

## 最小底盘链路与故障边界

厂商链路的实测数据流为：

```text
/cmd_vel -> /driver_node (Mcnamu_driver) -> /vel_raw -> /base_node -> /odom_raw
```

`base_node` 本身不订阅 `/cmd_vel`，因此单独启动它不会让小车响应速度命令。`Mcnamu_driver` 直接将 `/cmd_vel` 交给厂商 `SunriseRobot` 库；它没有实现命令超时停车，且源码中声明的默认限速参数未实际约束回调输入。

一次架空轮验证已确认串口打开、话题链路和电压读取可用，但 `linear.x = 0.08` 的短时请求未让车轮转动。继续前先检查电机电源/使能、蓄电池和机械连接；不要用重复速度命令掩盖硬件故障。

## MS200P 与 SLAM Toolbox 冒烟测试（2026-09-03）

在停止所有键盘、手柄和完整 bringup 后，将 `Mcnamu_driver` 的速度订阅重映射
到 `/cmd_vel_disabled`，并用 `base_node` 的 `pub_odom_tf:=true` 提供最小里程
TF。测试期间 ROS 图中不存在 `/cmd_vel`。

已确认：

- `/scan` 为 `sensor_msgs/msg/LaserScan`，坐标系为 `lidar_link`，频率约 9.9 Hz
- 每帧约 450 个采样点，覆盖约 360°，配置量程为 0.15–20 m
- `/odom_raw` 频率约 10 Hz
- TF 链为 `odom -> base_footprint -> lidar_link`
- SLAM Toolbox 发布了 `/map` 和 `map -> odom`
- 静止地图分辨率为 0.05 m，尺寸为 337 × 154，共 51,898 个栅格

该结果只证明雷达、里程计、TF 和 SLAM 数据链能连通；机器人没有移动，所以不是
可用于导航的完整场地地图。停止测试时 MS200P 进程曾以缓冲区溢出退出，但没有
发现内核崩溃，后续须继续验证雷达驱动的关闭稳定性。完整设计和验收门槛见
[固定航点巡逻与动态避障设计](superpowers/specs/2026-09-03-task-2-navigation-design.md)。

## RViz 查看方式

不要在普通 SSH 终端内直接运行 `rviz2`。RDK 会因为没有可用的 X11/Wayland 显示
而报 `could not connect to display` 或 XCB 错误。应在有图形桌面的开发机启动
RViz2，并确保开发机与小车处于同一网络、使用相同的 `ROS_DOMAIN_ID=99`，再查看
小车发布的 `/scan`、TF 和 `/map`。SSH 只负责在板端启动和检查驱动。

如果开发机看不到话题，先核对两端 IP、`ROS_DOMAIN_ID` 和 ROS 发行版，再排查
组播或防火墙；不要为了恢复发现而重启小车的网络服务，以免中断队友会话。

## 官方入口

- [D-Robotics RDK X3/X5 用户手册](https://developer.d-robotics.cc/rdk_x_doc/RDK)
- [RDK X5 硬件与 IMX219 接口说明](https://developer.d-robotics.cc/rdk_doc/Quick_start/hardware_introduction/rdk_x5/)
- [TROS 文档](https://developer.d-robotics.cc/tros_doc/tros)
- [TROS SLAM 示例](https://developer.d-robotics.cc/tros_doc/apps/slam)
- [TROS Navigation2 示例](https://developer.d-robotics.cc/tros_doc/apps/navigation2)
- [TROS ROS 包快速入门](https://developer.d-robotics.cc/tros_doc/quick_start/ros_pkg)
- [Yahboom RDK X5 Robot 资料](https://www.yahboom.com/study/RDK-X5-ROBOT)
- [Yahboom MS200 雷达课程资料](https://www.yahboom.com/study/MS200)
