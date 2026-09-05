# Configuration

该目录保存跨节点的参数模板和按场地划分的 Demo 配置。ROS 包内部仍应保留可直接启动的安全默认值。

建议命名：

- `base_control.yaml`：速度、加速度、超时和 8 字轨迹参数
- `navigation.yaml`：雷达、定位、规划和动态障碍物参数
- `perception.yaml`：跨包共享的相机标定和感知参数（需要时再创建）
- `sorting.yaml`：跨包共享的场地分类参数（需要时再创建）

任务三当前可直接启动的安全默认参数位于
`src/rdx_color_sorting/config/color_sorting.yaml`。现场 HSV、区域坐标、相机/雷达外参和接触距离
应先在该文件的比赛副本中标定，禁止把未经验证的数值当作完成配置。

不要提交 IP、用户名、密码、访问令牌或只对某台机器有效的绝对路径。
