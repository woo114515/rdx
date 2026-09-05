# 任务三颜色分类代码整合设计

## 背景

队友仓库 `tghr183811/task-3` 已实现 `rdx_color_sorting` ROS 2 包，包含三色 HSV 感知、瓶形与长方体过滤、目标测距、分类推动状态机、任务三专用安全仲裁以及纯 Python 单元测试。主仓库已经保存比赛地图和近期实机记录，但其中自研 `rdx_navigation`、`rdx_mission` 与 `rdx_safety` 方案不再作为比赛导航方案；任务二继续使用小车厂家自带导航。

本次整合的目标是在不覆盖主仓库地图和近期记录的前提下，将队友的任务三实现纳入主仓库，并让任务三可以独立构建、识别和测试。

## 整合范围

导入以下内容：

- 完整的 `src/rdx_color_sorting` 包；
- `docs/color-sorting.md` 任务三说明；
- `docs/task2-navigation-integration.md`，作为可选接口参考，不默认启用；
- `scripts/check_task2_nav_ready.sh`，作为可选诊断脚本；
- 根 README、架构文档、路线图和配置说明中与任务三直接相关的增量更新。

不整体复制队友仓库的 README、AGENTS、平台基线、SSH 配置、故障记录或其他共享文件。这些文件来自较早的仓库快照，整体覆盖会破坏主仓库现有地图、导航记录和项目状态。

## 运行架构

任务二和任务三保持独立：

```text
任务二：厂家导航栈 -> 厂家底盘控制

任务三：IMX219 -> rdx_color_sorting -> /rdx_sorting/cmd_vel_request
                                      -> rdx_sorting_safety -> /cmd_vel
                                      -> 厂家底盘驱动
           MS200P --------------------^（可选测距与停车）
           /odom_raw -----------------> 状态机定位与返回
```

`rdx_sorting_safety` 是任务三运行时唯一允许发布 `/cmd_vel` 的节点。主仓库旧 `rdx_safety` 不参与任务三启动，也不与任务三同时运行。任务三默认使用 `/odom_raw`，不依赖任务二地图、AMCL 或 Nav2。

队友代码中可选的 Nav2 返回接口予以保留，但默认参数继续设置为 `nav2_return_enabled: false` 和 `navigation_request_enabled: false`。只有以后确认厂家导航接口兼容时才单独启用和验证。

## 安全默认值

- `motion_enabled` 默认保持 `false`；
- `output_enabled` 默认保持 `false`；
- `lidar_stop_enabled` 默认保持 `false`，直到雷达方向与阈值完成标定；
- 仅识别模式不得产生非零速度请求；
- 实车运动前必须先验证图像、里程计、急停、命令超时、限速和退出归零；
- 不自动在小车上启动节点或发送运动命令。

## 文档与项目状态

根 README 将增加任务三包入口、识别测试命令和安全提示。架构文档将明确厂家导航与任务三专用控制链彼此独立。路线图将只勾选已经由代码和单元测试证明的开发项；相机实机链路、HSV 参数、外参和真实推瓶仍保持未完成，不能因代码合并而标记为实车完成。

原 `rdx_navigation`、`rdx_mission` 和 `rdx_safety` 文件暂时保留，避免在本次功能导入中混入大范围删除。文档会把它们标记为历史/非比赛默认方案，后续可在独立提交中归档或删除。

## 验证

整合后执行：

1. `rdx_color_sorting` 的全部纯 Python 单元测试；
2. 主仓库现有测试，确认没有回归；
3. Python 语法编译检查；
4. `git diff --check`；
5. 检查安装数据，确认 YAML 和 launch 文件会随包安装；
6. 若当前 Windows 环境没有 ROS 2，不声称完成 `colcon build`，而在 Ubuntu/小车上提供后续构建命令。

## 成功标准

- 主仓库包含可独立构建的 `rdx_color_sorting` 包；
- 队友仓库的单元测试在主仓库结构下全部通过；
- 比赛地图与近期实机文档保持不变；
- 默认启动不会驱动车轮；
- 文档明确任务二使用厂家导航，任务三使用独立颜色分类控制链；
- 未经实车验证的参数和能力不会被标记为完成。
