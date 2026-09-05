# 实机平台基线

> 初始采集日期：2026-09-02；最近更新：2026-09-03。静态盘点之外，已在架空车轮且有人可立即断电的条件下完成一次最小底盘链路验证，并完成 MS200P 静态 SLAM 冒烟测试。

## 系统与资源

| 项目 | 实机值 |
| --- | --- |
| 板卡 | D-Robotics RDK X5 V1.0，Board ID 302 |
| 系统 | Ubuntu 22.04.5 LTS，RDK OS 3.0.0 |
| 内核 | Linux 6.1.83，aarch64，PREEMPT |
| CPU | 8 × Cortex-A55，最高 1.5 GHz |
| 内存 | 6.5 GiB；空闲约 4.9 GiB |
| Swap | 6.0 GiB；当前未使用 |
| 根分区 | 27 GiB，2026-09-03 已使用约 87%，剩余约 3.4 GiB |
| ROS | ROS 2 Humble，TROS Humble |
| 主工作区 | `/home/sunrise/yahboomcar_ws` |

根分区空间偏紧。构建、地图和 rosbag 录制前需要清理缓存或扩容，但不得在未审计目录占用前直接删除文件。

## 网络与远程访问

初始直连测试中，开发机曾使用 `10.42.0.1/24`，机器人曾获得
`10.42.0.72/24`；手机热点测试中，机器人又从 `192.168.49.8` 变为
`192.168.49.25`。这些均为历史 DHCP 租约，不是固定连接地址。USB 网络接口配置为
`192.168.128.10/24`，但上午检查时链路为 `DOWN`。连接前应重新发现当前地址。

SSH 使用 `sunrise` 用户和本机专用 Ed25519 密钥。私钥不在仓库中；公钥和使用方式见 [SSH Access](../config/ssh/README.md)。

## 硬件设备映射

| 设备 | 稳定路径 | 内核设备 | USB 身份 |
| --- | --- | --- | --- |
| MS200P 雷达 | `/dev/oradar` | `/dev/ttyACM0` | QinHeng 1a86:55d4 |
| 底盘控制板 | `/dev/myserial` | `/dev/ttyUSB0` 或 `/dev/ttyUSB1` | CH340 1a86:7523 |

两者权限当前均为 `0666`。开发代码应使用稳定 udev 链接，不应写死易变化的 `ttyACM0` 或 `ttyUSB0`。

IMX219 使用 RDK MIPI CSI 管线，静态检查时不存在 `/dev/video*` 或 `/dev/media*`。厂商 Python 节点通过 `hobot_vio.libsrcampy` 打开 camera pipe 1，并发布 JPEG 压缩图像；因此不能用普通 USB/V4L2 检查方式判断相机故障。

## 厂商 ROS 环境

Shell 启动顺序：

```bash
source /opt/tros/humble/setup.bash
source /home/sunrise/yahboomcar_ws/install/setup.bash
```

工作区已包含底盘、雷达、相机、导航、颜色追踪和机器人描述包。系统还安装了 Nav2、SLAM Toolbox、Cartographer、robot_localization 和 IMU Madgwick filter。

### 已识别接口

| 功能 | ROS 接口/帧 |
| --- | --- |
| 底盘速度命令 | 安全链路最终发布 `/cmd_vel_safe`（`geometry_msgs/Twist`） |
| 原始底盘速度 | 发布 `/vel_raw` |
| 原始 IMU | 发布 `/imu/data_raw`，帧 `imu_link` |
| 磁力计 | 发布 `/imu/mag` |
| 电池与版本 | 发布 `/voltage`、`/edition` |
| 原始里程计 | 发布 `/odom_raw`，`odom -> base_footprint` 可选 |
| MS200P | 发布 `/scan`，帧 `lidar_link` |
| IMX219 压缩图像 | 发布 `/csi/image_raw/compressed` |

MS200P 厂商配置使用 `/dev/oradar`、230400 baud、10 Hz、0.15–20 m。普通 scan launch 为逆时针全周扫描；gmapping launch 另设 90°–270°、顺时针，使用前必须实测坐标方向。

## 当前运行状态与风险

- 初始盘点时没有业务 ROS 节点运行；后续已验证底盘数据链、MS200P、TF 和静态 SLAM，正式启动前仍需检查重复节点。
- `yahboom_oled.service` 已在故障排查期间禁用；恢复前需先修复其缺失脚本并确认不会干扰业务节点。
- 厂商完整 bringup 同时启动底盘、里程计、IMU filter、EKF、手柄和机器人描述。首次验证应拆分启动。
- 厂商底盘驱动的默认速度上限是 x/y 各 1.0 m/s、角速度 1.0 rad/s；本项目必须覆盖为更保守的值。
- 厂商相机 Python 节点请求 640×480，但 NV12 转换代码硬编码了 1920×1080 缓冲区形状。复用前必须实机验证并可能修正。
- 厂商旧驱动直接接受 `/cmd_vel`，未在已读代码中发现命令超时停车保护。本项目改为由安全节点发布 `/cmd_vel_safe`；驱动切换必须和安全节点原子部署，不能只更新其中一侧。

## 底盘链路实测

本节记录一次 SSH 实机验证。测试时四轮架空，且现场有人可切断底盘电源；不应把本节的临时命令用于车轮落地、无人看管或正式运行。

### 已验证

- `ros2 run yahboomcar_base_node base_node --ros-args -p pub_odom_tf:=false` 成功启动 `/base_node`。该节点订阅 `/vel_raw`，并发布 `/odom_raw`。
- `ros2 run yahboomcar_bringup Mcnamu_driver` 成功启动 `/driver_node`。日志确认 `Sunrise Robot Serial Opened! Baudrate=115200`。
- 旧版实测 `/driver_node` 订阅 `/cmd_vel`，发布 `/vel_raw`、`/voltage`、`/edition`、IMU 和磁力计数据；`/base_node` 订阅 `/vel_raw`。硬化版目标链路是 `/cmd_vel_nav` 或 `/cmd_vel_teleop` -> `rdx_safety` -> `/cmd_vel_safe` -> `/driver_node`。
- `/voltage` 返回 `8.0`（单位以厂商驱动为准），证明驱动可从底盘控制器读取电压。
- 对 `/cmd_vel` 发布一次零速度消息已成功完成 ROS 发现与发布。

### 未通过及待排查

- 在上述条件下，对 `/cmd_vel` 发布 `linear.x = 0.08` 的短时前进请求，车轮没有可见转动。
- 串口、ROS 图和控制器电压读取均正常，故 SSH、ROS 节点发现和 USB 串口不是当前首要嫌疑。优先检查主电源是否完成启动、电机功率级/使能、蓄电池电量与电机连接；必要时按厂商电机调试流程排查。
- `Mcnamu_driver.py` 的 `cmd_vel_callback()` 会直接调用 `SunriseRobot.set_car_motion()`；其声明的 x/y/角速度限制参数没有在回调中使用，也没有命令超时或退出归零处理。因此在实现独立安全仲裁节点前，不能认为远程命令具备失联停车保障。

### 横移实测与使用决策

2026-09-03 记录了两组 `/cmd_vel`、`/vel_raw`、`/motor_encoder` 和
`/wheel_speed` 数据。第二组先左转再横移，bag 位于机器人：

```text
/home/sunrise/rosbag2_2026_09_03-14_06_53
```

左转时四轮编码器速度绝对值差异不到 0.1%；横移命令为
`linear.y = 0.44 m/s` 时，四轮速度符号关系正确，累计计数最大差异约 1.2%，
`/vel_raw.linear.y` 平均约为 `0.420 m/s`。这表明编码器、底层 PID 和麦克纳姆
轮运动学分解基本正常。

但车体横移时仍可观察到明显的实际前后窜动，而 `/vel_raw.linear.x` 平均仅约
`0.0024 m/s`。轮式编码器没有反映实际漂移，问题更符合滚子与地面打滑、四轮
承重、地面摩擦或麦轮机械特性，而不是单轮 PID 明显失调。

项目决定不再使用麦克纳姆横移能力：

- 所有业务控制将 `linear.y` 固定为零；
- 横向位置调整改为“转向—前进/后退—再转向”；
- 路径规划和演示验收按近似差速/非完整约束设计；
- `/motor_encoder` 和 `/wheel_speed` 继续保留用于诊断，但不用于恢复横移功能。

## 下一次安全验证

先排除电机供电/使能和电池问题，再在架空车轮且有人靠近电源的条件下复测底盘动作。每次只启动一个硬件链路；确认零速度和退出停车行为之前，不运行巡逻、避障、手柄或导航 launch。
