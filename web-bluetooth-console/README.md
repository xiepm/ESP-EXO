# ESP-EXO 网页上位机

配套当前仓库固件 **0.10.13**。左侧五栏目为连接、设置、运动、实时数据、数据记录。
网页负责低频配置、启停、显示和记录；ESP32本地执行唯一100 Hz控制链。

**完整中文操作说明：** [网页上位机操作说明](../docs/web/WEB_OPERATION_GUIDE.md)。

## 启动

Windows，Node.js >=22.13.0，Chrome或Edge及可用蓝牙。

```powershell
Set-Location 'D:\ESP32\esp_can\web-bluetooth-console'
npm.cmd ci
npm.cmd run local
```

已有依赖时只需运行最后一条。也可双击本目录的 `start-local.cmd`。
保持服务窗口运行，打开 **http://localhost:4173/exo**；Ctrl+C停止网页服务。
根路径 `/` 是原通用终端，当前产品二进制BLE接口使用 `/exo`。
本地服务只监听127.0.0.1；没有发布到手机或GitHub Pages。

## 使用顺序

| 栏目 | 操作 |
| --- | --- |
| 连接 | 选择ESP-EXO、等待服务/订阅/初始读回；检查固件、主控与双电机CAN反馈 |
| 设置 | 首次装配标定前抬正方向并保存NVS；穿戴后选择重新校准或确认沿用上次零位 |
| 运动 | 选策略/执行方式、设置左右等级与参数，保存并读回，单独START/STOP |
| 实时数据 | 默认10 Hz六通道示波器，时间窗、左右曲线、游标及暂停查看 |
| 数据记录 | 按北京时间日期查看本机记录，导出CSV/JSON、原始遥测和BLE事务日志 |

校准把左右当前位置成对写为本次0°；沿用恢复旧偏置，不能代替当前姿态确认。
方向断电保留，本次零位在RAM，已保存候选在NVS，重启不会自动激活候选。

策略包含TRANSPARENT、PHASE_PROFILE和ANGLE_THRESHOLD事件脉冲，AAN尚未实现。
0.10.13事件脉冲支持FIXED100–3000 ms、申请/锁存力矩0–15 N·m、20 N·m/s速率约束，
左右独立、每次真实新事件触发一次，结束后须完成复位。范围是工程能力，不是人体已验收参数。
SHADOW计算建议但新增助力为0；ACTIVE受基础模式、方向、零位、反馈及实时健康门禁约束。

STOP需等待IDLE/输出关闭。关闭网页、BLE断开、停止遥测或结束记录都不等于STOP。
日期记录保存在当前浏览器IndexedDB，内存遥测/日志刷新前需要导出；上传GitHub不会备份浏览器记录。

## 开发与测试

```powershell
npm.cmd test
npx.cmd tsc --noEmit
npm.cmd run lint
```

`npm test`包含生产构建及123项网页/协议测试（2026-10-14重新执行通过）。
项目沿用vinext/Vite与Cloudflare本地运行结构；BLE控制无需D1/R2。

主要文件：`app/exo/page.tsx`页面与BLE事务；`lib/exo-ble-protocol.ts`协议；
`app/exo/telemetry-scope.tsx`和`lib/exo-scope.ts`示波器；`app/exo/gait-history.tsx`
及`lib/exo-gait-history.ts`日期记录；`lib/exo-zero-calibration.ts`成对零位读回验证。

## 当前限制与证据

2026-10-13运动中CAN接收错误REC=133触发ERROR，助力关闭而反馈继续。
物理根因尚未证实，详见[助力中断诊断](../docs/phase10/PHASE_10_ASSIST_INTERRUPTION_20261013.md)。
不能把历史通信恢复称为已修复运动稳定性。真人主动SHADOW/分级ACTIVE整体验收、
新速率/高峰值机械测试、全7通道20 Hz耐久与完整重连E2E仍未完成。

[阶段10验收](../docs/phase10/PHASE_10_ACCEPTANCE.md) ·
[0.10.13部署](../docs/phase10/PHASE_10_0_10_13_DEPLOYMENT_20261013.md) ·
[20 N·m/s软件边界](../docs/phase10/PHASE_10_RISE_20_20261013.md) ·
[本次GitHub上传范围](../docs/phase10/GITHUB_BACKUP_20261014.md)
