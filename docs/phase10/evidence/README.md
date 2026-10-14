# 阶段10精选原始证据

2026-10-14为GitHub源码归档，按明确文件列表复制以下证据，并逐项校验SHA256一致。
原始文件仍保留在本机`output/phase10/`；没有改写、清理或替换原始实验目录。
来源、字节数及哈希见[ARCHIVE_MANIFEST.json](ARCHIVE_MANIFEST.json)。

| 目录 | 内容 | 结论边界 |
| --- | --- | --- |
| 20261013_deployment | 0.10.13烧录日志、BLE身份/参数查询及恢复默认、串口快照 | 烧录/参数读回通过；不是硬件15 N·m或人体助力通过 |
| 20261013_software | 当次C/网页/IDF测试日志、冻结源文件及产物哈希manifest | 软件测试证据；manifest中的完整冻结源码/二进制仍在本机 |
| 20261013_assist_stops | 两轮原始只读串口日志、一次状态摘录、采集脚本 | 当前Boot CAN_ERROR_PASSIVE停机；物理根因未定位 |
| 20261012_scope | 真实BLE待机示波器及暂停行为的摘要证据 | 不代表行走相位或ACTIVE验证通过 |

`.log`特定例外允许这些证据进入Git。完整冻结候选、重复源码、ELF/BIN/MAP、原始
高速遥测和其他实验产物不在此次Git提交内。本目录不是所有历史实验的完整备份。

`capture_readonly.py`只读取当前产品控制台，open前DTR/RTS=false；它绑定当前已知
FTDI序列号，输出新时间戳文件。先阅读脚本边界，不把它当成烧录或运动验收脚本。

参见[本次归档/验证记录](../GITHUB_BACKUP_20261014.md)及
[助力中断诊断](../PHASE_10_ASSIST_INTERRUPTION_20261013.md)。
