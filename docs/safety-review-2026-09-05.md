# 安全链路与厂商硬化源码审查

> 审查日期：2026-09-05。范围：仓库 `df650bd`（厂商硬化仓库侧改动）、已部署的驱动安全补丁
> `Mcnamu_driver.py`、`rdx_safety` 安全仲裁、`rdx_base_control` 八字节点。审查结论基于
> 对实车 `/home/sunrise/yahboomcar_ws` 与本地仓库的逐行核对，未进行实车运动验证。

## 背景

原先丢失的厂商硬化修改已在 `reference/vendor-patches/2026-09-05/` 重建并完成离线验证。2026-09-05 部署前重新备份整个厂商源码和 SunriseRobotLib：
`/home/sunrise/vendor-backups/2026-09-05-before-official-compatible-hardening.tar.gz`，
SHA256 `8421284951dab7347cfd7b698e6b2100305c8f0a2c0ff87865ffcacb423f5aec`。两份补丁随后已部署，三个受影响 ROS 包已在小车 aarch64/TROS Humble 环境构建成功；尚未进行实车运动验证。

## 一、确定的 Bug

### B1【严重，源码已修复】安全链断裂：`motion_command_topic` 是死参数

- 位置：`src/rdx_bringup/launch/hardware.launch.py:44` 传参
  `motion_command_topic: /cmd_vel_safe`；补丁驱动 `Mcnamu_driver.py:98` 硬编码
  `create_subscription(Twist, "cmd_vel", ...)`，从不读该参数，launch 也没有 remap。
- `rdx_safety` 输出已改为 `/cmd_vel_safe`（`src/rdx_safety/config/safety.yaml:19`）。
- 后果：部署 `df650bd` 后 `/cmd_vel_safe` 无消费者、驱动仍听 `/cmd_vel`，Nav/八字/遥控
  全部失效，小车不响应速度命令。这是唯一「部署即故障」的确定性 bug。

### B2【中】急停不是失效安全（发布侧掉线不触发停车）

- 位置：`src/rdx_safety/rdx_safety/safety_node.py:170-173`。
- 仅 latch 最新 `/emergency_stop` 的 Bool 值。急停发布者在发出 `false` 后崩溃/断线，节点
  会一直停留在「已解除」，Nav 持续命令时小车照常行驶。缺少心跳/看门狗语义。

### B3【中】驱动看门狗与串口 I/O 共享单线程 executor

- 位置：补丁驱动 `watchdog_callback`（50ms 定时器）与 `set_car_motion` 串口写入同线程。
- `set_car_motion` 阻塞（串口卡死）时看门狗无法调度，超时归零失效。
- 已记录于 `docs/vendor-driver-safety-patch.md`「已知限制」。

### B4【低】近距离障碍可能被安全逻辑漏掉

- 位置：`src/rdx_safety/rdx_safety/safety_logic.py:_normalize_range`。
- 把 `<= 0` 和 `NaN` 测距跳过；激光对「过近（< range_min 0.15m）」物体常返回 0/NaN，
  贴到 0.15m 内的障碍不会触发 `obstacle_stop`。

### B5【低】八字节点超时/容差未标定

- 位置：`src/rdx_base_control/rdx_base_control/figure_eight_node.py:14`
  `TASK_TIMEOUT=30s`，轨迹在 0.21–0.66 m/s 下耗时未校验，可能误报超时；
  `cross_track_limit=0.5m` 脱轨阈值未经实车标定。

## 二、安全风险（失效模式）

| 编号 | 风险 | 说明 | 现状 |
| --- | --- | --- | --- |
| R1 | 多源争抢速度话题 | 驱动无所有权仲裁，任何节点可直写；曾发生 joy+keyboard 交替写致间歇运动（见 `JOYSTICK_AUTOSTART_CHANGE.md`） | `use_joy` 已缓解，根因未除 |
| R2 | 非正常退出不归零 | SIGKILL/内核崩溃/掉电走不到 `finally`/看门狗；USB 断开无法下发零速 | 软件无法兜底，依赖硬件急停 |
| R3 | 串口写异常被静默吞掉 | 厂商 `SunriseRobotLib` 吞写异常，无法确认零速帧到达 MCU，无 ack/健康反馈 | 已部署写失败上报；尚无 MCU ack |
| R4 | 串口接收线程死亡但节点存活 | 接收线程断后节点继续运行、旧数据重打当前时间戳、里程计用消息到达时间积分 | 已部署线程/反馈健康检查 |
| R5 | 标定节点缺陷 | `calibrate_linear`/`calibrate_angular` TF 崩溃、边转边走、难停车 | 已部署重写版本，待实车验证 |

## 三、源码修复与部署状态

- 厂商驱动保留官方 `/cmd_vel`，并已部署命令超时、限速、非法输入拒绝、多线程 executor、串口与反馈健康检查、`/driver_health` 和 `/vel_raw_stamped`。
- 厂商 `base_node` 已改用带时间戳反馈积分，拒绝异常时间间隔。
- 两个厂商标定节点已重写为单自由度运动；TF 丢失、超时和退出均重复归零。
- 厂商键盘和手柄保留官方 `/cmd_vel`；手柄未显式激活时不发布运动。
- RDX 安全、八字和导航修改仍仅在仓库中，不属于本次官方 Task 2 部署。

补丁位于 `reference/vendor-patches/2026-09-05/`，已完整部署并通过反向补丁干跑确认；三个 ROS 包构建和三个官方 launch 无启动解析通过，尚未进行实车运动验证。
- 首次启动暴露反馈健康阈值过紧和串口关闭竞态；已部署 `1.00 s` 反馈阈值与 `TypeError` 处理，运动命令超时仍保持 `0.25 s`。

## 四、剩余结论

1. 官方 Task 2 保持厂商 `/cmd_vel` 契约，不依赖任何 `rdx_*` 节点；同一时刻必须只有一个控制源。
2. 串口、驱动、里程计和标定修复已部署并构建，但仍须完成架空轮和低速实车验证后才能宣称验收完成。

## 五、下一步验证

1. 架空轮验证官方 `/cmd_vel` 链路、命令超时停车、串口断开和驱动重启；
2. 完成低速正反 90°录包；
3. 再按雅博 6710/6712 顺序验证建图、地图保存、定位和导航。
