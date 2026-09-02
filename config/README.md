# Configuration

该目录保存跨节点的参数模板和按场地划分的 Demo 配置。ROS 包内部仍应保留可直接启动的安全默认值。

建议命名：

- `base_control.yaml`：速度、加速度、超时和 8 字轨迹参数
- `navigation.yaml`：雷达、定位、规划和动态障碍物参数
- `perception.yaml`：相机标定、颜色阈值和检测参数
- `sorting.yaml`：搜索边界、堆放点、对准和推动参数

不要提交 IP、用户名、密码、访问令牌或只对某台机器有效的绝对路径。
