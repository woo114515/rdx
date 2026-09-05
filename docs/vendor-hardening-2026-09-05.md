# 厂商底盘接口硬化变更记录

## 目的

把厂商底盘从“任意节点可直接写 `/cmd_vel`”改成单一、可失效保护的运动出口，并记录这项改动对既有功能的影响。该文档对应 2026-09-05 的仓库变更。

## 话题契约

```text
Nav2 / 八字节点       -> /cmd_vel_nav
键盘 / 手柄 / 标定     -> /cmd_vel_teleop
                              |
                         rdx_safety
                              |
                         /cmd_vel_safe
                              |
                         Mcnamu_driver
```

`/cmd_vel_safe` 是唯一允许进入底盘驱动的速度话题。驱动应同时具备命令超时停车、串口写失败报告和退出时重复发送零速度的行为。旧版驱动仍订阅 `/cmd_vel`，所以安全节点和驱动必须成套部署；只部署其中一侧会导致安全节点有输出但小车不动。

## 已记录的源码/配置变化

- 安全节点输出改为 `/cmd_vel_safe`，保留 `/cmd_vel_nav` 与 `/cmd_vel_teleop` 两个输入。
- bringup 不再启动厂商手柄或键盘节点，并为底盘驱动配置安全话题和崩溃重启。
- Nav2 使用融合后的 `/odom`，禁止横向速度，角速度上限暂为 `0.20 rad/s`。
- costmap 使用实测矩形 footprint：长 `0.30 m`、宽 `0.20 m`，不再重复增加 padding。
- 八字节点只发布前进速度和角速度，采用服务显式启动；TF 丢失、超时、取消和退出均发布零速度。

## 对既有功能的影响

- 建图仍使用 `/scan`、`odom -> base_footprint` 和 SLAM Toolbox；不改变地图消息接口。
- 导航依赖 EKF 持续发布 `/odom`。若 EKF 未启动，Nav2 会等待里程计，不能把问题误判为地图故障。
- 键盘、手柄和标定程序必须改发 `/cmd_vel_teleop`；直接向旧 `/cmd_vel` 发布不再是受支持的控制方式。
- 禁止横移会牺牲麦轮侧移能力，但避免已实测的横移打滑和前后窜动；轮速/编码器话题仍保留用于诊断。
- 降低角速度和 inflation 半径会改变导航速度与贴障距离，必须在架空轮/低速、有人断电的条件下重新验收。

## 部署与回滚

部署前必须保留厂商备份：

```text
/home/sunrise/vendor-backups/2026-09-04-before-vendor-hardening.tar.gz
SHA256: b776b71f8bc260c67fa1f804522b305ffbc1415ce40c6fb1bc64f48b518db6fa
```

当前提交只记录仓库侧契约和配置；厂商驱动硬化尚未部署到机器人。部署时应先停止所有运动节点，再同步驱动、安全节点和启动文件，编译后检查：

```bash
ros2 topic info /cmd_vel_safe -v
ros2 topic info /cmd_vel_nav -v
ros2 topic info /cmd_vel_teleop -v
ros2 topic hz /odom
ros2 topic echo /rdx_safety/state
```

若驱动异常，使用备份恢复厂商文件，再把安全输出临时恢复到旧驱动实际订阅的话题，恢复过程必须保持急停生效并由现场人员控制电源。

## 验证记录

- 本地 `git diff --check` 通过。
- 使用包路径运行测试：`45 passed`。
- 本次没有启动机器人、没有发送运动命令；实机部署和低速验收仍待完成。
