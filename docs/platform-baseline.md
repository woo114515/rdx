# 实机平台基线

> 采集日期：2026-09-02。以下内容来自 SSH 只读检查；未启动底盘、雷达或相机 ROS 驱动，也未发送运动指令。

## 系统与资源

| 项目 | 实机值 |
| --- | --- |
| 板卡 | D-Robotics RDK X5 V1.0，Board ID 302 |
| 系统 | Ubuntu 22.04.5 LTS，RDK OS 3.0.0 |
| 内核 | Linux 6.1.83，aarch64，PREEMPT |
| CPU | 8 × Cortex-A55，最高 1.5 GHz |
| 内存 | 6.5 GiB；空闲约 4.9 GiB |
| Swap | 6.0 GiB；当前未使用 |
| 根分区 | 27 GiB，已用 21 GiB（82%），剩余约 4.8 GiB |
| ROS | ROS 2 Humble，TROS Humble |
| 主工作区 | `/home/sunrise/yahboomcar_ws` |

根分区空间偏紧。构建、地图和 rosbag 录制前需要清理缓存或扩容，但不得在未审计目录占用前直接删除文件。

## 网络与远程访问

开发机直连地址为 `10.42.0.1/24`，机器人当前为 `10.42.0.72/24`。机器人同时启用热点 `192.168.8.88/24`，USB 网络接口静态地址为 `192.168.128.10/24`。直连地址来自 DHCP，不能写死为永久配置。

SSH 使用 `sunrise` 用户和本机专用 Ed25519 密钥。私钥不在仓库中；公钥和使用方式见 [SSH Access](../config/ssh/README.md)。

## 硬件设备映射

| 设备 | 稳定路径 | 内核设备 | USB 身份 |
| --- | --- | --- | --- |
| MS200P 雷达 | `/dev/oradar` | `/dev/ttyACM0` | QinHeng 1a86:55d4 |
| 底盘控制板 | `/dev/myserial` | `/dev/ttyUSB0` | CH340 1a86:7523 |

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
| 底盘速度命令 | 订阅 `/cmd_vel`（`geometry_msgs/Twist`） |
| 原始底盘速度 | 发布 `/vel_raw` |
| 原始 IMU | 发布 `/imu/data_raw`，帧 `imu_link` |
| 磁力计 | 发布 `/imu/mag` |
| 电池与版本 | 发布 `/voltage`、`/edition` |
| 原始里程计 | 发布 `/odom_raw`，`odom -> base_footprint` 可选 |
| MS200P | 发布 `/scan`，帧 `lidar_link` |
| IMX219 压缩图像 | 发布 `/csi/image_raw/compressed` |

MS200P 厂商配置使用 `/dev/oradar`、230400 baud、10 Hz、0.15–20 m。普通 scan launch 为逆时针全周扫描；gmapping launch 另设 90°–270°、顺时针，使用前必须实测坐标方向。

## 当前运行状态与风险

- 没有业务 ROS 节点运行，尚不能证明传感器数据或底盘反馈有效。
- `yahboom_oled.service` 已启用；桌面运行厂商 OLED 与管理程序，但未发现其启动 ROS 驱动。
- 厂商完整 bringup 同时启动底盘、里程计、IMU filter、EKF、手柄和机器人描述。首次验证应拆分启动。
- 厂商底盘驱动的默认速度上限是 x/y 各 1.0 m/s、角速度 1.0 rad/s；本项目必须覆盖为更保守的值。
- 厂商相机 Python 节点请求 640×480，但 NV12 转换代码硬编码了 1920×1080 缓冲区形状。复用前必须实机验证并可能修正。
- 厂商驱动直接接受 `/cmd_vel`，未在已读代码中发现命令超时停车保护。本项目需要独立安全仲裁节点。

## 下一次安全验证

架空车轮并安排人员靠近电源后，依次验证底盘状态链、MS200P、IMX219 和 TF。每次只启动一个硬件链路；确认零速度和退出停车行为之前，不运行巡逻、避障、手柄或导航 launch。
