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
2. **SLAM 与导航**：使用 MS200P 建图、保存和加载地图，并自主导航到目标点；动态障碍物策略待现场验收规则确定。
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

SSH 和静态环境盘点已完成。下一步是在确保车轮架空或场地清空后，分别启动底盘、雷达和相机驱动，验证话题、频率和 TF；在此之前不发送运动指令。

只读检查可从本机运行：

```bash
ssh -o IdentitiesOnly=yes \
  -i ~/.ssh/rdx_ed25519 \
  sunrise@10.42.0.72 'bash -s' \
  < scripts/collect_system_info.sh
```

直连地址由 DHCP 分配，可能变化。连接方式见 [SSH 说明](config/ssh/README.md)，实机步骤见 [首次开机检查](docs/bringup.md)，里程碑见 [开发路线图](docs/roadmap.md)。

## 安全原则

首次运动测试必须清空场地或架空车轮，使用保守速度，并安排人员靠近电源控制。所有运动节点必须在命令超时、传感器失效和正常退出时发布零速度。
