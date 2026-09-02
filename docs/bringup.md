# 首次开机检查

## 目的

首次 SSH 连接只采集事实，不安装软件、不修改厂商服务、不启动电机。检查结果用于确定 ROS 发行版、工作空间格式、驱动来源、话题名称和 TF 结构。

## 安全准备

1. 将机器人放在宽阔平整区域，固定叉形结构远离人员和物品。
2. 保持底盘电机未使能；若无法独立断开电机，则架空四个车轮。
3. 确认操作人员能够立即切断底盘电源。
4. 不要在 RDK X5 通电时插拔 IMX219 排线。

## 自动采集

在仓库根目录运行：

```bash
mkdir -p /tmp/rdx-diagnostics
./scripts/collect_system_info.sh \
  | tee /tmp/rdx-diagnostics/system-info.txt
```

输出可能包含主机名、设备路径和软件版本，只用于团队调试，不应提交到公开仓库。

## 需要确认的事实

- RDK OS、Ubuntu、内核与 CPU 架构
- ROS/TogetheROS.Bot 发行版及环境脚本路径
- Yahboom 底盘驱动包、串口设备与控制话题
- MS200P 驱动包、设备路径、`LaserScan` 话题与 `frame_id`
- IMX219 设备、相机驱动、图像与相机信息话题
- 里程计、IMU、机器人模型及完整 TF 树
- 厂商自启动服务是否占用底盘或传感器

## 后续验证顺序

环境确认后按以下顺序逐项启动，每次只增加一个变量：底盘状态（不运动）、IMU/里程计、雷达、相机、TF，然后才进行架空车轮的低速运动测试。每项记录启动命令、输入输出话题、频率、QoS、坐标帧和已知异常。

## 官方入口

- [D-Robotics RDK X3/X5 用户手册](https://developer.d-robotics.cc/rdk_x_doc/RDK)
- [RDK X5 硬件与 IMX219 接口说明](https://developer.d-robotics.cc/rdk_doc/Quick_start/hardware_introduction/rdk_x5/)
- [Yahboom MS200 雷达课程资料](https://www.yahboom.com/study/MS200)
- [Yahboom ROSMASTER X3 产品资料](https://category.yahboom.net/products/rosmaster-x3)
