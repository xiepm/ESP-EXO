# ESP-EXO 阶段 9 本地真机 BLE 验证页

本目录包含基于 [vinext](https://github.com/cloudflare/vinext) 的本地网页，用于连接
ESP-EXO 固件 `0.9.0`，执行产品 BLE v1 的连接、双髋 RAM 软件归零、基础模式与
左右五元参数静态配置、步态/高层助力 SHADOW 状态显示、START/RUNNING/STOP 状态
确认以及验收数据导出。

## 页面入口

- `http://localhost:4173/exo`：ESP-EXO 阶段 9 真机 BLE 验证页，应使用此入口。
- `http://localhost:4173/`：仓库原有的通用 BLE/USB 串口终端，仍使用旧的
  RobStride/Nordic UART Service 约定，不是 ESP-EXO 产品 BLE 接口。

不要在根路径向当前固件发送 `start`、`stop`、`status` 等 ASCII 文本；ESP-EXO
产品接口使用 `/exo` 页面生成的二进制协议帧。

## 环境要求

- Windows 10/11。
- 当前版本的 Google Chrome 或 Microsoft Edge。
- Node.js `>=22.13.0`。
- Windows 蓝牙已开启，ESP32 已烧录待验收的 ESP-EXO `0.9.0` 固件。

Web Bluetooth 要求安全上下文。在本机 Windows Chrome/Edge 中，
`http://localhost:4173/exo` 可作为本地可信入口使用。

Android Chrome 不能直接使用电脑的 `http://<局域网 IP>:4173/exo`：手机端必须从
具有受信任证书的 HTTPS 地址打开 `/exo`，并授予浏览器蓝牙权限。本地启动脚本只监听
`127.0.0.1`，不会向手机或局域网发布网页。

## 快速启动

在 Windows 资源管理器中双击：

```text
start-local.cmd
```

脚本会在首次运行时安装 npm 依赖，启动本地 vinext 服务，并打开：

```text
http://localhost:4173/exo
```

如果默认浏览器不是 Chrome/Edge，请复制该地址到 Chrome 或 Edge。使用期间保持命令行
窗口打开；按 `Ctrl+C` 停止本地服务。

也可以在 PowerShell 中手动启动：

```powershell
cd D:\ESP32\esp_can\web-bluetooth-console
npm install
npm run local
```

## 真机操作流程

1. 打开 `/exo`，确认页面显示 Web Bluetooth 和安全上下文可用。
2. 点击“选择 ESP-EXO”，在浏览器设备选择器中连接设备。页面会先订阅 Response
   Notify，再读取固件、协议、系统状态和左右关节状态。
3. 页面会核对固件 `0.9.0`、协议 v1、capabilities 包含 `0x003F` 和最大帧长
   `20 B`。
   任一项不匹配时会锁定归零与 START，但保留 STOP；同时记录 `boot_id`，确认系统处于
   `IDLE`。
4. 将双髋保持在中立位，勾选中立位确认，然后点击“双髋依次归零”。该操作只捕获
   ESP32 RAM 软件零位，不修改 RS01 机械零位；ESP32 重启后需要重新归零。
5. 在系统仍为 IDLE 时选择基础模式和参考源，填写 LEFT/RIGHT 五元参数，点击
   “应用全部并读回”。页面会依次写入模式、左右参数并重新读取比较；不会发送
   100 Hz BLE 命令流。旧 `0.7.x` 固件没有 capability bit 3 时不会发送这些新命令。
6. 在阶段 9 助力卡片中只选择 `DISABLED` 或 `SHADOW`。当前只有透明策略已实现，
   PHASE_PROFILE/AAN 只是未来接口占位；页面不会提供 ACTIVE，且网页低频查询不能
   代替 ESP32 本地 100 Hz 控制任务。
7. 观察步态卡片的 STANDING/ACQUIRING/LOCKED/DEGRADED、左右相位、步频、置信度和
   质量位。当前相位原点是髋屈曲峰代理，不等于足跟着地；没有外部真值时不能据此
   声称论文级相位精度。
8. 确认左右关节均显示“最终有效”、配置读回一致且系统 ready 后，点击
   “START 并等待 RUNNING”。START ACK 只代表状态机接单，页面会继续轮询，只有到达
   `RUNNING` 且输出许可开启后才报告成功。
9. 运动期间保持自动轮询，以采集左右关节角度、速度和反馈年龄。RUNNING 中页面
   禁止修改模式和参数；切换必须先 STOP 并等待 IDLE。
10. 点击“STOP 并等待 IDLE”。STOP ACK 同样不是最终完成；页面会继续轮询，直到系统
   回到 `IDLE` 且运行请求和输出许可均已关闭。
11. 点击“导出关节 CSV”和“导出 BLE JSONL”，保存本轮关节样本、原始帧、解析结果和
   时间戳。正式阶段 9 验收应同时保存独立 UART 日志，用于核对 100 Hz 控制任务、
   TWAI 计数和双电机最终 REST/失能状态。
12. 确认系统已经回到 `IDLE` 后再断开 BLE。

> BLE 断开不等于 STOP。若在 `STARTING` 或 `RUNNING` 中掉线，设备会继续执行已经接收的
> 运行请求。应重新连接 `/exo`、先查询当前状态，再显式发送 STOP 并等待 IDLE；不能把
> 关闭网页、刷新页面或断开蓝牙当作停止操作。

## 导出内容

- 关节 CSV：连接 session ID、Web Bluetooth device ID、boot ID、固件、run ID、
  系统状态、人体侧别、接收时间、按 feedback age 修正的估计采样时间、request ID、
  关节 flags、方向、原始 mrad/mrad/s、显示用 deg/deg/s 和反馈年龄。
- BLE JSONL：每条 TX、Notify、系统事件或错误所属的 session/device、ISO 时间、
  单调时间、方向、说明、原始 HEX 字节和帧长度。

建议每轮 STOP 并确认 IDLE 后立即导出，避免浏览器刷新导致尚未下载的数据丢失。

## vinext 与开发命令

项目继续使用 vinext、Vite 和 Cloudflare 本地运行环境；BLE 验证本身不依赖 D1/R2。
`.openai/hosting.json`、`vite.config.ts`、`worker/` 和可选数据库示例仍属于原 vinext
工程结构。

- `npm run local`：在 `127.0.0.1:4173` 启动本地验证入口。
- `npm run dev`：启动 vinext 开发服务器。
- `npm run build`：验证生产构建。
- `npm test`：构建并运行页面渲染测试和产品 BLE v1 协议测试。
- `npm run lint`：运行 ESLint。
- `npm run db:generate`：仅在使用可选 Drizzle/D1 数据库时生成迁移。
