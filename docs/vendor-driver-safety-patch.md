# Yahboom 底盘驱动安全补丁

> 修改日期：2026-09-03。目标设备：RDK X5 ROSMASTER X3，用户 `sunrise`。

## 背景

厂商 `Mcnamu_driver` 将每条 `/cmd_vel` 直接传给底盘控制板，没有实际执行
已声明的速度限制，也没有命令超时和退出归零。一次按时间发布的前进测试中，
后续临时零速度发布器初始化失败，控制板继续保持最后一条非零命令，最终由
现场操作员强制断电。临时创建另一个 ROS 发布器不能作为可靠停车路径。

本补丁保留原节点名称和启动入口，直接修改厂商工作区中的源文件，让停车逻辑
与串口控制处于同一常驻进程中。

## 文件位置

修改的源文件：

```text
/home/sunrise/yahboomcar_ws/src/yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver.py
```

新增测试：

```text
/home/sunrise/yahboomcar_ws/src/yahboomcar_bringup/test/test_motion_safety.py
```

仓库保存的可审查补丁：

```text
reference/patches/yahboomcar-bringup-safety.patch
```

## 原始备份

修改前已停止厂商完整 bringup，并确认 `/dev/ttyUSB0` 无进程占用。只读备份位于：

```text
/home/sunrise/vendor-backups/2026-09-03-before-driver-safety
```

备份包含源码包、安装副本、厂商 `SunriseRobotLib` egg 和 `SHA256SUMS`。三个
关键原始文件的 SHA-256 为：

```text
c427d992a1801e11e872db9e72b6e986220af2ac6d55c9120daa65c49ad00e32  source-yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver.py
cb1b05dd269a3be6a30047f31f7daa143cadce85fa06432fa22ec46affc5f094  install-yahboomcar_bringup/lib/python3.10/site-packages/yahboomcar_bringup/Mcnamu_driver.py
5633960ef39ff49384dbc44196418c03c794b46bd7ce34d2149a05b5d9abce34  SunriseRobotLib-3.3.9-py3.10.egg
```

备份目录已移除写权限，防止误改。

## 修改内容

- 启动串口后连续发送三次零速度。
- 使用单调时钟记录最后一条有效 `/cmd_vel`。
- 新增 50 ms 安全定时器；250 ms 没有新命令时持续下发零速度。
- 真正执行 X、Y 和角速度限制，默认分别为 `1.0 m/s`、`1.0 m/s` 和
  `1.0 rad/s`。
- 拒绝 `NaN`、正无穷和负无穷，拒绝后连续发送三次零速度。
- 覆盖节点销毁路径，在退出前连续发送五次零速度。
- 将 `SIGTERM` 转换为正常清理路径。
- 使用 `/tmp/yahboomcar-mcnamu-driver.lock` 防止补丁版本重复启动。
- 将 `/dev/i2c-0` 改为仅在收到 RGB 控制命令时打开，避免底盘启动无条件依赖
  I2C。
- 发布 `/motor_encoder`（`Int32MultiArray`），数据依次为 M1～M4 的累计
  编码器计数。
- 发布 `/wheel_speed`（`Float32MultiArray`），数据依次为 M1～M4 按相邻
  样本计算的 `ticks/s`，并正确处理有符号 32 位计数器回绕。
- 在修改后的节点文件头记录日期、备份和本文档位置。

厂商压缩库 `SunriseRobotLib-3.3.9` 未修改。

## 编码器反馈

驱动运行后可监听四轮累计计数：

```bash
ros2 topic echo /motor_encoder
```

监听四轮计数变化率：

```bash
ros2 topic echo /wheel_speed
```

两个数组的顺序都是 `[M1, M2, M3, M4]`。`/wheel_speed` 的单位明确为
`ticks/s`，不是 RPM 或 m/s；首次采样没有前一帧可比较，因此固定发布四个
零值。控制板自动上报、驱动按 10 Hz 发布。

横移诊断时建议记录：

```bash
ros2 bag record /cmd_vel /vel_raw /motor_encoder /wheel_speed
```

在确认 M1～M4 的物理轮位、正方向以及每个车轮一圈对应的编码器计数之前，
不得把 `ticks/s` 直接换算为轮速、RPM 或车体速度。

## 构建与验证

构建命令：

```bash
cd /home/sunrise/yahboomcar_ws
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select yahboomcar_bringup
```

构建成功。setuptools 输出了 `easy_install` 已弃用警告，但没有构建错误。
`--symlink-install` 生成的 egg-link 指向：

```text
/home/sunrise/yahboomcar_ws/build/yahboomcar_bringup
```

纯软件测试命令：

```bash
cd /home/sunrise/yahboomcar_ws/src/yahboomcar_bringup
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
PYTHONPATH=.:"$PYTHONPATH" python3 -m pytest -q test/test_motion_safety.py
```

结果：`15 passed`。测试覆盖限幅、合法值保持、非有限值拒绝、超时边界、回调
实际限幅、非法回调归零、看门狗超时归零、编码器正反向差值、32 位回绕、
采样时间换算和首帧归零。测试没有实例化驱动、打开串口或
发送实车运动命令。

截至本文档记录时，尚未进行通电运行、架空车轮或落地测试。

## 回滚

保持电机断电，停止所有相关节点后执行：

```bash
backup=/home/sunrise/vendor-backups/2026-09-03-before-driver-safety
cp -a "$backup/source-yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver.py" \
  /home/sunrise/yahboomcar_ws/src/yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver.py
rm -f /home/sunrise/yahboomcar_ws/src/yahboomcar_bringup/test/test_motion_safety.py
cd /home/sunrise/yahboomcar_ws
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select yahboomcar_bringup
```

回滚后使用备份中的 `SHA256SUMS` 复核源码。不要同时运行补丁版和旧版驱动。

## 已知限制

该补丁明显改善 ROS 命令中断和正常进程退出时的停车行为，但不是硬实时或
硬件级急停：

- `SIGKILL`、内核崩溃、主控断电或 Python 解释器崩溃无法执行 `finally`。
- USB 串口物理断开时，主控无法把零速度帧送到底盘控制板。
- 厂商库会吞掉部分串口写入异常，补丁目前无法确认每个停车帧已被 MCU 接收。
- 看门狗与状态发布仍共享单线程 ROS executor；回调永久阻塞会影响看门狗调度。
- 文件锁只能阻止补丁版节点的重复实例，不能阻止不遵守该锁的其他程序直接
  打开串口。
- M1～M4 的物理轮位、计数正方向、编码器每圈计数和减速比尚未标定。
- MCU 是否具备通信丢失自动停车仍需向厂商确认或通过架空车轮测试验证。

因此现场电源控制或硬件急停仍是最终保护。再次落地运行前，必须依次验证启动
归零、输入超时、`Ctrl+C`、`SIGTERM`、串口异常和重复实例拒绝；所有测试先
在四轮架空状态完成。
