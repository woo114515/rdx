# 三色柱体分类推动 Demo

## 范围与隔离

该 Demo 识别绿色、蓝色和橙色目标，并将它们分别移动到三个配置区域。它由独立的
`rdx_color_sorting` ROS 2 包提供，不修改或启动基本控制与导航 Demo。

节点默认只发布零速度，且速度请求发布到
`/rdx_sorting/cmd_vel_request`，不会直接写入厂商底盘使用的 `/cmd_vel`。只有在任务三
专用安全仲裁完成超时、急停、限速和退出归零测试后，才能把该请求接入底盘。

`rdx_sorting_safety` 是独立的安全边界。它默认不向 `/cmd_vel` 发布；显式启用后会
限制平移和旋转速度，请求超过 0.3 秒未更新时归零，并在收到
`/rdx_sorting/emergency_stop=true` 后锁定停车，必须重启才能解除。
当安全节点已启用输出，并发现还有其他节点同时发布 `/cmd_vel` 时，也会锁定停车，
避免两个控制器互相覆盖命令。
可选雷达保护会按运动方向检查近距离障碍，并在雷达数据超时后停车。接近和推动阶段
会忽略车头中央很窄的目标通道，避免把待推动物块当成普通障碍；通道两侧仍会停车。
该功能默认关闭，必须先在机器人上确认 `/scan` 的零度方向和有效量程。
状态机还限制单次搜索和推动的最长时间，超时会进入故障状态并请求零速度。
`/rdx_sorting/status` 的 `fault_reason` 字段会给出具体原因；故障不会自动解除，处理
原因并重新放置小车后，需要重启分类节点。

可选地图模式仅作为后续实验接口保留，不属于当前比赛默认方案。当前任务二使用厂家导航，
尚未验证其 AMCL、动作名称和速度重映射是否满足该接口。完成兼容性验证后，任务三才可
使用 AMCL 修正后的地图位姿控制推送，并在释放后调用 Nav2 返回本轮启动时的地图起点。
Nav2 的速度必须重映射到
`/rdx_sorting/nav_cmd_vel_request`，由同一个安全节点统一选择、限速和输出；详细接口
见 `docs/task2-navigation-integration.md`。

## 数据流

```text
IMX219 -> mipi_cam -> /image_raw -> 三色检测
                                      |
/odom_raw --+-------------------> 分类推动状态机
            |
/amcl_pose -+ (可选地图定位修正)
                                      |
                           /rdx_sorting/cmd_vel_request
                                      |
                         超时 / 急停 / 限速安全层
                                      |
                                  /cmd_vel
```

检测使用 OpenCV HSV 分割，不依赖神经网络。搜索时会从尚未完成数量的颜色中选择
画面面积最大的目标；开始对准后锁定该颜色，直到本次分类完成。节点通过图像中心
误差低速对准和接近。目标进入叉形结构后，根据 `/odom_raw` 将目标移动到
相对初始位姿配置的区域中心，然后后退释放。只要还有待分类物块，小车就低速返回
本次启动记录的起点附近，再搜索下一种颜色。返回不直接斜穿中央物块区，而是先移动
到分类区外侧通道，再沿通道退到起点横向位置，最后回到起点。

可选的雷达测距会把图像横向误差换算为相机视场内的目标方位，在对应激光小扇区内
取有效距离中位数。靠近目标时根据距离逐渐减速，到达 `contact_distance` 后切换到
推动；激光样本不足或数据过期时自动回退到图像面积判断。该功能必须标定后才能开启。

## 配置

参数文件位于 `src/rdx_color_sorting/config/color_sorting.yaml`。现场必须重新测量并
设置以下参数：

- `zones.<color>.x/y`：三个区域相对机器人初始位姿的位置，单位为米；
- `targets_per_color.<color>`：每种颜色的目标数量；
- `hsv.<color>.lower/upper`：现场光照下的 HSV 阈值；
- `minimum_width_height_ratio` / `maximum_width_height_ratio`：目标外接框宽高比；
- `minimum_extent` / `minimum_solidity`：过滤稀疏、破碎或不规则色块；
- `bottle_detection_enabled`：是否同时接受竖直瓶子；
- `bottle_minimum_width_height_ratio` / `bottle_maximum_width_height_ratio`：
  瓶子外接框宽高比；
- `bottle_maximum_neck_body_width_ratio`：瓶颈与瓶身宽度的最大比例；数值越小，
  对“瓶颈明显变窄”的要求越严格；
- `bottle_minimum_extent` / `bottle_minimum_solidity`：瓶子轮廓完整度阈值；
- `maximum_area_ratio`：过滤占据画面过大的背景色块；
- `minimum_detection_duration`：目标连续稳定出现多久后才允许开始对准；
- `maximum_search_duration`：单轮扫描时长；默认到时开始下一轮扫描，不停车；
- `search_timeout_is_fault`：仅设为 `true` 时，扫描超时才进入故障停车；
- `contact_area_ratio`：目标进入推动位置时在图像中占据的面积比例；
- `lidar_target_range_enabled`：是否使用雷达辅助目标测距；
- `camera_horizontal_fov_degrees` / `camera_lidar_yaw_offset_degrees`：相机视场与
  雷达相对朝向；
- `target_range_half_angle_degrees` / `target_range_minimum_samples`：目标测距扇区和
  最少有效激光点数；
- `contact_distance` / `approach_range_kp`：雷达接触距离和接近减速比例；
- 速度、超时和区域容差。
- `return_speed` / `return_tolerance`：释放物块后返回起点的速度和到达容差。
- `return_corridor_offset` / `return_corridor_margin`：侧边返回通道与分类区的间距。
- `push_target_lost_timeout`：大于零时，推动途中目标丢失多久后停车报错；
- `delivery_confirmation_timeout`：大于零时，释放后必须在时限内重新看到面积变小的
  同色物块，才计为完成。
- `delivery_maximum_area_scale`：释放后的目标面积最多为接触时面积的比例。

若物块数量未知，可将对应的 `targets_per_color.<color>` 设置为 `-1`，并把
`finish_when_no_target_duration` 设置为大于零的秒数。节点会持续处理该颜色，并在
连续指定时间看不到任何可处理目标后正常完成。状态中的 `processed` 会记录各颜色
已经成功投放的数量。固定数量模式下应保持 `finish_when_no_target_duration: 0.0`，
防止暂时遮挡导致提前结束。

推动跟踪和释放确认默认关闭，因为相机安装角度可能存在近距离盲区。确认物块在推动和
后退后仍可见，再将对应超时参数设置为正数。

瓶子与长方体使用两套独立形状规则，长方体阈值不会因启用瓶子识别而放宽。瓶子需要
竖直放置，并且分割轮廓上部瓶颈明显窄于中下部瓶身。调试画面分别标记为
`<color>/bottle` 和 `<color>/cuboid`。透明瓶、无明显瓶颈的圆柱瓶以及横放瓶不保证
通过当前轮廓规则；首次使用必须根据现场瓶子图像调整瓶子参数和 HSV 阈值。

默认情况下，三个区域是相对每次启动时初始位姿的虚拟区域。复用任务二地图时，将
`pose_source` 设为 `amcl`、`zones_relative_to_origin` 设为 `false`，并把三个区域填写
为地图坐标。节点利用 `/amcl_pose` 计算地图到原始里程计的修正，再继续以高频
`/odom_raw` 更新控制位姿，从而兼顾地图定位和短距离控制的连续性。AMCL 尚未给出
定位结果时节点保持停车。

相机距地约 10 cm 时，应调整俯仰角，使画面同时覆盖叉子前端和前方搜索区域。地面
反光明显，调参时应覆盖场地内最亮和最暗的位置。

## 构建与仅感知验证

在机器人上创建独立工作区覆盖层，不修改厂商工作区：

```bash
source /opt/tros/humble/setup.bash
source /home/sunrise/yahboomcar_ws/install/setup.bash
colcon build --symlink-install --packages-select rdx_color_sorting
source install/setup.bash
```

先启动官方相机驱动，再以默认配置启动分类节点：

```bash
ros2 launch mipi_cam mipi_cam_640x480_bgr8.launch.py
ros2 launch rdx_color_sorting color_sorting.launch.py
```

如果改用雅博压缩图像节点，则启动 `yahboomcar_csi_cam_py`，并使用
`color_sorting_motion.launch.py` 的默认 `/csi/image_raw/compressed` 输入；两套相机驱动
不能同时启动。

默认 `motion_enabled: false`，节点只检测并持续请求零速度。检查：

```bash
ros2 topic echo /rdx_sorting/status
ros2 topic hz /rdx_sorting/debug/compressed
ros2 topic echo /rdx_sorting/cmd_vel_request
```

联调状态机和安全仲裁但不接管底盘：

```bash
ros2 launch rdx_color_sorting color_sorting_motion.launch.py
```

不得在其他电脑仍发布 `/cmd_vel` 时启用输出。只有完成架空车轮测试、现场清场并由
操作员守在断电控制旁后，才可同时设置 `motion_enabled:=true` 和
`output_enabled:=true`。

## 实机验证顺序

1. 静态画面验证三种颜色在不同光照和距离下均无误检。
2. 架空车轮，验证图像或里程计未到、超时时请求速度立即归零。
3. 每次只放置一个目标，验证搜索、对准、接近、推动和释放。
4. 分别完成三种颜色后，再增加到每种两个目标。
5. 清空场地、保留现场断电人员，最后进行完整流程。

不得将 `/rdx_sorting/cmd_vel_request` 直接重映射到 `/cmd_vel`。厂商驱动没有命令
超时停车能力，必须由独立安全仲裁节点接入。
