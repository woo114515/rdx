# RDX Robot Demos

本仓库用于开发和演示 ROSMASTER X3 实体机器人的三项能力：底盘基本控制、二维 SLAM 与自主导航、彩色柱体自主分类推动。

## 已确认平台

- ROSMASTER X3 四麦克纳姆轮底盘
- 地平线 RDK X5 V1.0 主控
- Orbbec/Oradar MS200P 二维激光雷达
- Sony IMX219 CSI 相机
- 无活动关节的固定叉形推动结构
- Ubuntu 22.04.5、RDK OS 3.0.0、ROS 2/TROS Humble

2026-09-02 已通过 SSH 完成只读盘点，确认厂商工作区、串口映射和 ROS 软件环境。详见 [实机平台基线](docs/platform-baseline.md)。

## Demo 范围

1. **基本控制**：以低速 8 字轨迹证明底盘运动控制可用，轨迹尺寸与速度可配置。
2. **SLAM 与导航**：使用 MS200P 建图、保存和加载地图，按固定顺序访问任务点 1、2、3 并返回起点；巡逻中应绕开固定障碍和移动人员。
3. **柱体分类推动**：自主搜索易拉罐大小的彩色柱体，将暖色和冷色柱体整理为两堆。该任务位于独立场地，不复用导航 Demo 的地图。

三个 Demo 分别启动和验收。Python 是首选语言，仅在性能或底层接口需要时使用 C++。

## 仓库结构

```text
config/      跨节点参数模板与现场配置
docs/        架构、路线图和实机操作文档
reference/   官方资料链接与可再分发参考材料
scripts/     环境检查和开发辅助脚本
src/         本项目 ROS 功能包
tests/       脱离硬件运行的自动化测试
```

## 当前阶段

SSH、底盘数据链和 MS200P 静态 SLAM 冒烟测试已完成。SLAM Toolbox 已成功从
`/scan`、里程计和 TF 生成二维栅格地图，但尚未进行安全的移动采图。下一步先
实现速度安全仲裁和最小 bringup，再在实体场地低速建图。

任务二的需求、技术路线、实测数据和验证门槛见
[固定航点巡逻与动态避障设计](docs/superpowers/specs/2026-09-03-task-2-navigation-design.md)。

只读检查可从本机运行：

```bash
robot_ip=192.168.49.25  # 替换为当前 DHCP 地址
ssh -o IdentitiesOnly=yes \
  -i ~/.ssh/rdx_ed25519 \
  sunrise@"${robot_ip}" 'bash -s' \
  < scripts/collect_system_info.sh
```

有线直连或热点地址均由 DHCP 分配，可能变化。连接方式见 [SSH 说明](config/ssh/README.md)，实机步骤见 [首次开机检查](docs/bringup.md)，里程碑见 [开发路线图](docs/roadmap.md)。

## 安全原则

首次运动测试必须清空场地或架空车轮，使用保守速度，并安排人员靠近电源控制。所有运动节点必须在命令超时、传感器失效和正常退出时发布零速度。
