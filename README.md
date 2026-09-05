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
3. **柱体分类推动**：使用 IMX219 自主搜索绿色、蓝色和橙色瓶状目标，并将三种颜色分别推向可配置区域。该任务位于独立场地，默认不复用导航 Demo 的地图。

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

SSH、底盘数据链和 MS200P 静态 SLAM 冒烟测试已完成。比赛导航采用小车厂家自带导航
方案，仓库内原有的 `rdx_navigation`、`rdx_mission` 和 `rdx_safety` 暂时保留作历史参考，
不再作为比赛默认运行链路。比赛地图保存在 `src/rdx_navigation/maps/`，现场操作记录见
[建图与导航操作手册](docs/mapping-navigation-guide.md)。

2026-09-03 已把源码部署到小车 `/home/sunrise/rdx`，四个自有包在 TROS Humble 上构建
成功。隔离 ROS 域内的无硬件测试已确认 SLAM Toolbox、现有测试地图、AMCL、DWB、
costmap 和行为树能够加载；真实底盘、雷达数据和运动仍须按操作手册分阶段验证。
随后在关闭底盘驱动的条件下验证了 `/dev/oradar` 连接、MS200P 扫描数据获取、
`lidar_link` 模型链路，以及安全节点的急停和 footprint 双重锁定。底盘里程计和运动测试
仍未执行。

同日进一步测试发现厂商雷达驱动会异常退出，且 Jazzy RViz 与 Humble 小车直接 DDS 通信
会触发类型反序列化错误。测试末尾出现来源未明的持续蜂鸣，已停止全部项目测试并断电。
恢复上电前必须先排查供电告警。详见
[MS200P 与 RViz 现场测试记录](docs/field-test-2026-09-03-lidar-rviz.md)。

任务三已导入队友开发的 `rdx_color_sorting` 包，包括 HSV 三色识别、瓶形过滤、目标测距、
`SEARCH → ALIGN → APPROACH → PUSH → RELEASE → RETURN` 状态机和任务三专用安全仲裁。
纯 Python 测试能够脱离机器人运行；相机链路、现场 HSV、外参和真实推瓶仍需实车标定。
使用方法见 [三色目标分类推动](docs/color-sorting.md)。

任务二原自研方案的需求和实验记录仍保留在
[固定航点巡逻与动态避障设计](docs/superpowers/specs/2026-09-03-task-2-navigation-design.md)，
但不代表当前比赛启动方案。

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
