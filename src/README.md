# Source Packages

ROS 发行版与厂商驱动接口确认后，在此创建工作空间功能包。计划按职责拆分为 bringup、base control、navigation、perception 和 sorting，并统一使用 `rdx_` 包名前缀。

不要复制完整厂商工作空间。优先把厂商驱动作为已安装依赖；必须修补上游代码时，记录来源、版本、许可证和修改原因。
