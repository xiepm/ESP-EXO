# 2026-10-14 GitHub源码与网页操作说明归档

版本：**0.10.13**。目标远程：`origin = https://github.com/xiepm/ESP-EXO.git`，分支`main`。
状态：`SOFTWARE CHECKS PASS; CAN PHYSICAL ROOT CAUSE UNRESOLVED; PHASE 10/11 INCOMPLETE`。
固件/网页源码提交：`c40c600a43558f44e58dc7be6e47b406fc4aadd3`；配套操作说明与证据
作为下一次文档提交归档。当前检查摘要见[VALIDATION_20261014.json](evidence/VALIDATION_20261014.json)。

## Git核对与提交范围

本轮开始HEAD为9225bb3，抓取远端后origin/main为2ee7e98；本地领先6个提交，远端没有
本地缺少的提交。保留既有历史和未提交工作，不reset、不清理、不强推，不向upstream推送。

归档内容：

- 阶段10固件：相位曲线/唯一命令合成、事件脉冲、CAN反馈恢复与故障停机、
  出厂方向及成对零位/可选沿用、产品BLE协议和遥测。
- 配套五栏目网页：连接、设置、运动、实时示波器、日期记录与导出。
- 对应主机夹具、回放/协议测试，所有阶段10设计/部署/验收与失败记录。
- [中文网页操作说明](../web/WEB_OPERATION_GUIDE.md)，已按0.10.13源码和实际按钮核对。
- [精选原始证据](evidence/README.md)，复制前后SHA256逐项相同；含0.10.13部署与
  2026-10-13真实CAN停机日志。

完整本机`output/`不进入Git：其中包括重复冻结源码、烧录产物和实验缓存，新增ignore只
影响Git收集，不删除文件。`research/`的基金申请/调研/演示文稿等独立工作保留未提交，
没有混入固件仓库上传。浏览器IndexedDB日期记录不在仓库，重要行走记录仍需网页导出备份。

GitHub保存可构建源码及文档，不表示网站已经部署到互联网或GitHub Pages。
网页仍按操作说明在本机启动`http://localhost:4173/exo`。

## 本轮重新执行的验证

所有检查为软件构建/主机回放；未打开硬件串口、连接BLE、烧录、归零、使能或START。

| 检查 | 结果 |
| --- | --- |
| npm.cmd test（含生产构建） | PASS，123/123 |
| npx.cmd tsc --noEmit | PASS |
| npm.cmd run lint | PASS，0错误/警告 |
| phase10_swing_pulse_replay.ps1 | PASS，17/17，原相位集成4/4 |
| phase10_runtime_fault_replay.ps1 | PASS，8/8 |
| phase10_pair_zero_replay.ps1 | PASS，成对8/8、原沿用10/10、方向11/11 |
| ESP-IDF 5.5 idf.py build | PASS；app 683328 B，分区剩余35%；未烧录 |

本次build哈希与最近部署候选一致（app SHA256=
`e4002a0a2945009cb96402b88f1c17f94c8d326d7aff2b0e177afaec8c97532a`）。
历史O2 ASan/UBSan等日志另保留在精选证据中，没有把历史测试重新执行状态混写。
本次Git暂存检查与推送结果以实际Git
提交及远端分支读取为准；这份文档记录上传内容和验收边界。

## 尚未完成

2026-10-13最近目标Boot 0x7C9317A5：主控ERROR，最近故障CAN_ERROR_PASSIVE，锁存
TEC/REC=0/133，输出门已关闭；反馈和10 Hz遥测继续。后续两个只读窗口中累计CAN错误855
不增长，不能视为运动链路已修复。本轮上传没有重新检查当前硬件现场。

CAN供电/共地/线束/端接的物理根因仍待确认；真人主动SHADOW、分级ACTIVE、新速率及
高峰值机械验证、全7通道20 Hz耐久、完整网页和重连E2E仍未完成。保留历史工程SHADOW
通过记录与后续运动失败记录，阶段10整体及阶段11都不能标记完成。
