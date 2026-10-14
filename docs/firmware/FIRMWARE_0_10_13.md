# ESP-EXO 0.10.13 固件下载与烧录

适用硬件：ESP32-S3-N16R8，16 MB Flash；CAN TX GPIO4、RX GPIO1、1 Mbit/s；
两台RS01，执行器A=0x10、B=0x20。出厂方向须以设备标定读回为准。
构建环境为ESP-IDF 5.5 / esptool 4.12.0，DIO / 40 MHz；本构建未启用PSRAM。

## 下载与版本范围

从 [GitHub Release v0.10.13](https://github.com/xiepm/ESP-EXO/releases/tag/v0.10.13)
下载 `ESP-EXO-v0.10.13-esp32s3-n16r8.zip` 和 `SHA256SUMS.txt`。
Release的 `Source code` 是源码归档；上面的命名ZIP才是已编译烧录包。

这是**开发预发布版**。包内二进制直接来自2026-10-13冻结并烧录过的0.10.13候选，
不是新功能升级。233个冻结文件已逐项核对哈希；48个固件源码/配置文件与上传源码相符。
2026-10-14重建app的哈希也一致。版本标签对应源码、网页及说明，具体提交见包内清单。

已知限制：运动中发生过 `CAN_ERROR_PASSIVE`，物理根因尚未解决；阶段10整体、
真人分级助力、高峰值机械验证和阶段11仍未完成。见
[助力中断诊断](../phase10/PHASE_10_ASSIST_INTERRUPTION_20261013.md)和
[阶段10验收](../phase10/PHASE_10_ACCEPTANCE.md)。不能将下载发布视为新增运动验收通过。

## 文件与地址

| 文件（解压目录内） | 地址 | 大小 | 用途 |
| --- | --- | --- | --- |
| `bootloader/bootloader.bin` | `0x0000` | 21088 B | 引导程序 |
| `partition_table/partition-table.bin` | `0x8000` | 3072 B | 分区表 |
| `twai_network_master.bin` | `0x10000` | 683328 B | 0.10.13应用 |
| `debug/twai_network_master.elf` | 不烧录 | 7922728 B | 崩溃地址符号解析 |
| `flash_args` / `flasher_args.json` | 不烧录 | — | 原构建的烧录参数 |
| `config/`、`docs/`、`evidence/` | 不烧录 | — | 配置、操作说明、历史证据 |
| `release-manifest.json`、`SHA256SUMS.txt` | 不烧录 | — | 来源、验收边界及文件校验 |

三个二进制的SHA256：

```text
aaff4219954934f6d7add9f55d017837029529dc9bd6bf28e384be3a3b4f3877  bootloader/bootloader.bin
7f00b6c042a89b15b0cac534f82ed988caf29278ff5700b0c511eb1b5bb7c820  partition_table/partition-table.bin
e4002a0a2945009cb96402b88f1c17f94c8d326d7aff2b0e177afaec8c97532a  twai_network_master.bin
```

本包采用分段烧录，不含整片Flash备份和从0地址填充的合并镜像。
分区表中NVS为 `0x9000`、24 KiB，PHY为 `0xF000`、4 KiB，应用为 `0x10000`、1 MiB。
按下面命令只写三个指定区域，保留现有NVS内的方向与零位候选；仅适用于相同分区布局。
不要执行整片擦除，也不要把app写到0地址。升级不会保留运行中的RAM零位。

## 校验下载包

在下载目录运行PowerShell，对照Release的 `SHA256SUMS.txt`：

```powershell
Get-FileHash -Algorithm SHA256 -LiteralPath '.\ESP-EXO-v0.10.13-esp32s3-n16r8.zip'
```

解压后，在包含 `flash_args` 的目录验证所有包内文件；以下命令不访问硬件：

```powershell
Get-Content -LiteralPath '.\SHA256SUMS.txt' | ForEach-Object {
    if ($_ -notmatch '^([0-9a-f]{64})  (.+)$') { throw '校验清单格式错误' }
    $expectedHash = $Matches[1]
    $packageFile = $Matches[2]
    $actualHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $packageFile).Hash
    if ($actualHash -ine $expectedHash) { throw "校验失败: $packageFile" }
}
'全部文件校验通过'
```

## 烧录步骤（仅在需要升级或恢复时执行）

1. 确认两台电机动力已断开，保留主控USB数据连接。烧录会复位主控。
2. 关闭占用串口的监视器。设备管理器核对主控实际COM口，不能假设一直是COM6。
3. 打开ESP-IDF 5.5 PowerShell，进入固件包解压目录，运行 `python -m esptool version`；
   本说明验证的工具版本为4.12.0。普通PowerShell需要先进入相应IDF Python环境。
4. 将下面 `COMx` 换成实际端口，再执行。保持串口稳定连接直到三个区域均通过写入校验。

```powershell
$flashPort = 'COMx' # 必须改成实际主控端口
python -m esptool --chip esp32s3 --port $flashPort --baud 460800 `
    --before default_reset --after hard_reset write_flash `
    --flash_mode dio --flash_freq 40m --flash_size 16MB `
    0x0 bootloader/bootloader.bin `
    0x8000 partition_table/partition-table.bin `
    0x10000 twai_network_master.bin
```

这些文件、地址和参数已按冻结产物及历史烧录记录核对；本次发布没有再次烧录硬件。
执行烧录前请重新确认当前动力条件，不能沿用历史断电状态。

## 烧录后核对

1. 按[网页操作说明](../web/WEB_OPERATION_GUIDE.md)启动同一标签的网页，重新连接ESP-EXO。
2. 读回固件 **0.10.13**；主控应为待机、输出门关闭。不要以浏览器缓存版本判定成功。
3. 恢复电机/CAN供电后，先检查双侧持续新反馈、CAN诊断及故障状态。
4. 读回出厂方向；本次穿戴选择重新校准，或在满足当前中立位及预览门禁后沿用保存候选。
   候选不会在重启后自动成为有效RAM零位。
5. 助力设置由设备读回确认。启动默认关闭助力、双侧0%、峰值0.5 N·m、固定时长600 ms。
   最大可配置15 N·m、固定脉冲100–3000 ms、速率20 N·m/s；实际峰值仍受时长和退出条件限制。

发现ERROR、反馈过期或CAN错误时，先保存诊断记录并查明原因；反馈恢复不等于故障锁存解除。
具体校准、脉冲参数、示波器、记录导出和启停门禁见网页操作说明。
