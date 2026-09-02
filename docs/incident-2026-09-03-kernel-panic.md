# RDK X5 黑屏与内核崩溃记录（2026-09-03）

## 结论

本次“屏幕黑掉、键鼠无响应、SSH 断开”的直接原因已由 UART 日志确认：
`Open_AP` 桌面自启动脚本停止并重启无线网络组件时，触发 AIC8800
厂商 Wi-Fi 驱动 `aic8800_fdrv` 的内核空指针异常，最终导致 kernel panic。

这不是单纯的 HDMI、LightDM、OLED 或 SSH 故障。SSH 也会因 `Open_AP`
切换无线网络而提前断开，但整机失效来自随后的内核崩溃。

本项目此前编写的 Wi-Fi 客户端自动连接脚本也会停止厂商热点、启动
`wpa_supplicant` 并重启 NetworkManager。该脚本不是内核缺陷的来源，但会
触发与 `Open_AP` 相同的危险切换流程，因此是故障的直接诱因之一。该实现
已经从仓库清除，不得部署到机器人。

## 为什么有时能够长期运行

这是驱动内部的竞态问题，并非每次切换 Wi-Fi 都必然崩溃。无线驱动的后台
任务和网络服务关闭设备、释放数据的操作会争抢执行时序：

- 如果后台任务先结束，切换可以完成；通过启动阶段的危险窗口后，只要不再
  重启网络服务，系统可能长期稳定运行。
- 如果网络切换先释放了数据，而后台任务随后继续访问，驱动就会触发空指针
  异常并导致 kernel panic。

CPU 调度、无线信号、设备枚举和服务启动速度都会改变这个时序。因此同一套
配置可能一次开机后长期可用，另一次却在约 51 秒时崩溃。偶尔成功不能证明
切换流程是安全的。

## 关键证据

故障稳定发生在开机约 51 秒。崩溃前系统正在执行：

```text
Stopping wpa_supplicant.service...
Stopping Network Manager...
Starting Network Manager...
```

UART 捕获到的崩溃信息：

```text
[   51.063258] Unable to handle kernel NULL pointer dereference at virtual address 00000000000000b8
[   51.118181] Internal error: Oops: 0000000096000004 [#1] PREEMPT SMP
[   51.198485] CPU: 5 PID: 86 Comm: kworker/5:1 Tainted: P O 6.1.83 #11
[   51.211383] Workqueue: events rwnx_rc_stat_work [aic8800_fdrv]
[   51.306394] Call trace:
[   51.308846]  down_write+0x48/0xf0
[   51.312169]  simple_recursive_removal+0x48/0x254
[   51.316799]  debugfs_remove+0x58/0x80
[   51.320472]  rwnx_rc_stat_work+0x160/0x480 [aic8800_fdrv]
[   51.361459] Kernel panic - not syncing: Oops: Fatal exception
[   51.392787] Rebooting in 5 seconds..
```

调用栈表明 `aic8800_fdrv` 的 `rwnx_rc_stat_work` 工作队列在删除
debugfs 项目时访问无效地址。它与 `Open_AP` 脚本停止/重启 NetworkManager
和 wpa_supplicant 的时间点直接吻合。

## 临时修复

关闭 `Open_AP` 的桌面自启动。系统无法稳定保持 SSH 时，应关机后将旧 SD
卡插入维护电脑，在离线根文件系统中执行：

```bash
sudo mv \
  /media/woo/rootfs/home/sunrise/.config/autostart/Open_AP.desktop \
  /media/woo/rootfs/home/sunrise/.config/autostart/Open_AP.desktop.disabled
sync
```

此操作可逆。需要恢复时把 `.disabled` 后缀移除，但在 Wi-Fi 驱动修复前，
重新启用可能再次触发 kernel panic。

## 当前状态与限制

截至 2026-09-03，系统处于“已规避触发条件”，而不是“根因已修复”：

- `Open_AP` 桌面自启动已禁用；
- 不安装或启用本项目此前的 `rdx-wifi-client.service`；
- ext4 和 FAT 文件系统已离线检查并修复；
- 丢失的 `libaom.so.3` 和 ROS 包文件已经恢复；
- 系统已恢复显示、SSH 和 ROS 基础功能；
- AIC8800 厂商内核驱动缺陷仍然存在；
- miniboot/BL2 校验失败是另一个尚未解决的独立风险。

在获得确认修复该问题的 Wi-Fi 驱动或系统镜像前，禁止通过自动化脚本停止、
启动或重启 `NetworkManager`、`wpa_supplicant`、`hobot-wifi` 和相关热点服务。
如需调整无线网络，应保留有线管理链路、现场串口和可恢复的 SD 卡镜像，并先
在非生产介质上验证。

此前还发现 `yahboom_oled.service` 的 `oled.py` 卡在 `i2c_dw_xfer` 的 D
状态，因此已禁用该服务。OLED 问题会造成 I2C 进程卡死和 CPU 消耗，但禁用
后 kernel panic 仍然复现，所以它不是本次整机崩溃的主因。

## 文件系统影响

多次 kernel panic 和强制断电造成了附带的文件系统损坏：

- ext4 根分区出现 orphan linked list 损坏、`EFSCORRUPTED` 和
  `clean with errors`；
- FAT 启动分区出现 dirty bit；
- systemd journal 多次报告未正常关闭或损坏。

因此 `fsck` 修复的是异常复位造成的后果，无法消除 Wi-Fi 驱动崩溃源。
每次非正常复位后，应在分区未挂载时重新检查文件系统。

## 第二个独立问题：miniboot

内核 panic 自动复位后，BootROM 多次报告板载 SPI NAND 中的启动镜像校验
失败，随后进入 USB DFU：

```text
ERROR: Failed to checksum image id=1 (-97)
ERROR: boot mode 5 main bl2 fail and switch to bakeup1...
NOTICE: Currtenly use dfu-util to download file
NOTICE: DFU Start...
```

这说明板载 miniboot/BL2 还存在独立的损坏或兼容性问题。它解释了异常复位
后连续输出 `C`、进入 DFU，以及克隆到新 SD 卡后不能正常启动的现象。处理
顺序应为：先禁用 `Open_AP`、恢复旧卡稳定启动，再使用匹配当前 RDK 系统版本
的官方流程修复或升级 miniboot。不要在供电不稳定或系统仍会崩溃时更新
miniboot。

## 数据备份

旧卡已有完整块设备镜像：

```text
/home/woo/rdx-sd-source-2026-09-02.img
```

镜像大小为 `62534975488` 字节，SHA-256：

```text
6a42fff3bd5bcea7fb62e555eb321edf6e3450f6017e6092c35d6ae9077dba93
```
