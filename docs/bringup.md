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
ssh -o IdentitiesOnly=yes \
  -i ~/.ssh/rdx_ed25519 \
  sunrise@10.42.0.72 'bash -s' \
  < scripts/collect_system_info.sh
```

直连 IP 由 DHCP 分配，变化时先运行 `ip neigh show dev enp129s0`。输出可能包含主机名、设备路径和软件版本，不应提交原始诊断日志。

## 已确认事实

- RDK X5 V1.0，Ubuntu 22.04.5，RDK OS 3.0.0，aarch64
- ROS 2 Humble 与 TROS Humble；系统环境为 `/opt/tros/humble/setup.bash`
- Yahboom 工作区为 `/home/sunrise/yahboomcar_ws`，并由 `.bashrc` 自动叠加
- MS200P 为 `/dev/oradar -> /dev/ttyACM0`
- 底盘控制板为 `/dev/myserial -> /dev/ttyUSB0`
- 根分区 27 GiB，已使用 82%，剩余约 4.8 GiB
- 当前无业务 ROS 节点；仅 `/parameter_events` 与 `/rosout`
- `yahboom_oled.service` 已启用，桌面同时运行厂商 OLED/管理程序

完整接口和风险记录见 [实机平台基线](platform-baseline.md)。

## 后续验证顺序

每次只增加一个变量，并记录启动命令、话题类型、频率、QoS、帧名和异常：

1. 架空车轮后启动底盘状态链，先不发布 `/cmd_vel`。
2. 验证 `/vel_raw`、`/imu/data_raw`、`/imu/mag`、`/odom_raw` 和 TF。
3. 单独启动 MS200P，验证 `/scan` 与 `lidar_link`。
4. 单独启动 IMX219，验证 `/csi/image_raw/compressed`。
5. 核对 `odom -> base_footprint` 和机器人描述中的传感器静态 TF。
6. 确认停车路径后，才做低速运动测试。

不要直接启动厂商完整 bringup 后立即遥控；该 launch 同时包含底盘、状态估计、手柄和机器人描述，需先排除意外输入。

## 官方入口

- [D-Robotics RDK X3/X5 用户手册](https://developer.d-robotics.cc/rdk_x_doc/RDK)
- [RDK X5 硬件与 IMX219 接口说明](https://developer.d-robotics.cc/rdk_doc/Quick_start/hardware_introduction/rdk_x5/)
- [Yahboom RDK X5 Robot 资料](https://www.yahboom.com/study/RDK-X5-ROBOT)
- [Yahboom MS200 雷达课程资料](https://www.yahboom.com/study/MS200)
