# 厂商底盘接口硬化变更记录

## 目的

在保持雅博 6710/6712 官方节点、命令和 `/cmd_vel` 接口不变的前提下，加固底盘驱动，并记录这项改动对既有功能的影响。该文档对应 2026-09-05 的仓库变更。

## 官方 Task 2 话题契约

```text
厂商键盘 / 手柄 / 标定 ─┐
厂商 Nav2 (DWB) ────────┴─> /cmd_vel ─> 加固后的 Mcnamu_driver ─> 底盘
```

该模式只使用官方节点。RDX 安全、八字和任务节点是可选实验组件，不是复现
6710/6712 的依赖。任何时刻只允许启动一个 `/cmd_vel` 控制源。

官方复现模式继续使用 `/cmd_vel`：厂商键盘、手柄、标定节点和 Nav2 均保持官方接口，硬化版驱动默认订阅 `/cmd_vel`。驱动补丁另外实现命令超时停车、串口写失败报告和退出时重复发送零速度。

## 已记录的源码/配置变化

- 安全节点输出改为 `/cmd_vel_safe`，保留 `/cmd_vel_nav` 与 `/cmd_vel_teleop` 两个输入。
- bringup 不再启动厂商手柄或键盘节点，并为底盘驱动配置安全话题和崩溃重启。
- Nav2 使用融合后的 `/odom`，禁止横向速度，角速度上限暂为 `0.20 rad/s`。
- costmap 使用实测矩形 footprint：长 `0.30 m`、宽 `0.20 m`，不再重复增加 padding。
- 八字节点只发布前进速度和角速度，采用服务显式启动；TF 丢失、超时、取消和退出均发布零速度。

## 对既有功能的影响

- 建图仍使用 `/scan`、`odom -> base_footprint` 和 SLAM Toolbox；不改变地图消息接口。
- 导航依赖 EKF 持续发布 `/odom`。若 EKF 未启动，Nav2 会等待里程计，不能把问题误判为地图故障。
- 键盘、手柄、标定程序和 Nav2 保持发布 `/cmd_vel`，因此 6710/6712 的官方命令无需修改。多控制源仍不得同时启动。
- 禁止横移会牺牲麦轮侧移能力，但避免已实测的横移打滑和前后窜动；轮速/编码器话题仍保留用于诊断。
- 降低角速度和 inflation 半径会改变导航速度与贴障距离，必须在架空轮/低速、有人断电的条件下重新验收。

## 部署与回滚

部署前必须保留厂商备份：

```text
/home/sunrise/vendor-backups/2026-09-04-before-vendor-hardening.tar.gz
SHA256: b776b71f8bc260c67fa1f804522b305ffbc1415ce40c6fb1bc64f48b518db6fa
```

本次部署前备份：`/home/sunrise/vendor-backups/2026-09-05-before-official-compatible-hardening.tar.gz`，SHA256 `8421284951dab7347cfd7b698e6b2100305c8f0a2c0ff87865ffcacb423f5aec`。

厂商硬化源码已重建为 `reference/vendor-patches/2026-09-05/` 下的两份统一补丁，已于 2026-09-05 部署到机器人。部署时先停止所有运动节点，成套更新 SunriseRobotLib、驱动、里程计和厂商控制节点，编译后检查：

```bash
ros2 topic info /cmd_vel -v
ros2 topic hz /odom
ros2 topic echo /driver_health
```

官方复现模式不启动 `rdx_safety`，也不要求急停心跳；失效保护由加固驱动的命令超时和串口健康检查承担。

## 验证记录

- 本地 `git diff --check` 通过。
- 使用包路径运行测试：`45 passed`。
- 仓库测试现为 `57 passed`，两份厂商补丁对备份基线 `dry-run` 成功。
- SunriseRobotLib 离线假串口测试通过读超时、报告新鲜度和短写失败场景。
- 已部署 SunriseRobotLib 与六个厂商源码文件，三个受影响 ROS 包在 aarch64/TROS Humble 构建成功；官方三个 launch 通过无启动解析。未发送运动命令，低速实车验收仍待完成。
- 首次启动发现 `0.30 s` 反馈健康阈值会把约 `0.33 s` 的正常抖动误判为故障；已改为 `1.00 s`，命令停车超时仍为 `0.25 s`，并捕获串口关闭竞态的 `TypeError`。驱动已重新构建，待重新启动验证。

## 2026-09-07 编码器重复样本故障

Task 3 全栈启动时，`Mcnamu_driver` 在重复读取同一编码器反馈后，以
`encoder sample interval must be positive` 退出。根因是驱动根据反馈年龄
反推采样时刻，而定时器可能在新串口帧到达前再次读取同一帧，使推算间隔
等于零、略微倒退，或仅有浮点抖动。

修复保存在
`reference/vendor-patches/2026-09-07/yahboomcar-encoder-sample-guard.patch`：
只在推算采样时刻至少前进 1 ms 时更新编码器基准并发布 `/wheel_speed`；
重复、倒退或非有限样本跳过本次轮速发布，不再让异常穿透 ROS 定时器。
`/motor_encoder` 及同一批次中的其他新鲜反馈接口保持不变。

该补丁已部署到机器人并构建 `yahboomcar_bringup`。部署前文件保存在
`/home/sunrise/vendor-backups/2026-09-07-before-encoder-sample-guard/`。
底盘安全测试结果为 `22 passed`；不发送 `/cmd_vel` 的 20 秒冒烟测试中，
`/vel_raw_stamped`、`/motor_encoder` 和 `/wheel_speed` 均约为 10 Hz，驱动
持续运行至外层 timeout 正常结束；随后 12 秒复测同样没有异常退出。
Python 运行时解析到 build 中的新模块并确认包含 `encoder_sample_interval`。
短时窗口内 `/driver_health` 未被 ROS CLI 发现，因此该话题仍需随完整硬件
bringup 单独复核，不能把本次结果解释为整个导航栈已经通过实车验收。

重启机器人后又执行了 25 秒 `laser_bringup_launch.py` 无运动冒烟测试：
`/scan` 与 `/odom` 均约为 10 Hz，`Mcnamu_driver` 没有在运行期间再次出现
编码器间隔异常。`base_node` 同期记录过一次近零反馈间隔并按既有逻辑丢弃，
也印证“重复/非递增反馈应跳过而非杀死节点”的处理方向。测试结束时外层
`timeout` 与 launch 同时传递 SIGINT，产生退出阶段的 `KeyboardInterrupt`
和 publisher context 日志；它发生在测试主动停止之后，不是运行期复发。
