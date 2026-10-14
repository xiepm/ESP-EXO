# 2026-10-09 上位机五栏目改造

状态：FRONTEND BUILD / TEST / LOCAL RECORD BROWSER PASS；真人运动与助力 HIL NOT RUN。
阶段10仍不能标记整体完成。本轮只修改网页，不烧录、不使能、不归零、不START。

## 实现与职责

- 左侧固定「连接、设置、运动、实时数据、数据记录」，小屏仍使用左侧导航。
- BLE 连接继续使用产品协议 v1。服务发现期间断连的提示已区分为「蓝牙在读取产品服务时已断开」，
  不再把该异常直接描述成服务不存在；没有宣称定位其底层断连原因。
- 设置保留实际左右映射、方向、RAM 软件零位读回与现场确认，机械置零不在该流程中。
- 运动提供左右独立助力等级、SHADOW/ACTIVE 选择、保存并读回及独立启动/停止。
  START、身份、模式、零位与输出许可门禁沿用；编辑滑条与设备已保存值分开显示。
- 实时数据保留双髋人体角度/速度、反馈力矩估计、步态和遥测；默认五通道10 Hz，mask0x0079。
- 日期记录约1 Hz读取新的真实状态快照，记录器跨栏目保持挂载，不参与100 Hz控制或发送运动命令。
  本机IndexedDB保存原始点和摘要，按北京时间分段，支持日期筛选、髋角曲线、CSV/JSON导出与JSON导入。
  导入校验归属、序号、时间和数据来源，并重新计算统计，不相信文件自称有效率。
  已有ID拒绝覆盖；缺失测量保留null，步频只统计有效步态，曲线按真实时间间隔绘制。

代码：`web-bluetooth-console/app/exo/page.tsx`、`gait-history.tsx`、
`lib/exo-gait-history.ts`、`app/globals.css`、`app/exo/layout.tsx`。
保留原页面备份`ui-redesign/page.before.tsx`；未重建工程或引入新依赖。

## 当前板只读证据

改造前和结束后的UART只读核对均为Boot ID0x3503C3B7，START requests=0。
结束快照：IDLE、run requested=no、output_allowed=no；A/B REST且未请求使能，
有效反馈约100 Hz，双髋RAM零位有效，CAN错误计数为0。
LEFT→B/+1，RIGHT→A/−1；软件偏置−1.791°/−22.935°，当时人体角约0.022°。
网页改造期间已断开BLE，结束快照Connected=no，串口完成后关闭。
这些是该次读取时的证据，不保证后续状态始终不变。

最终读取100 Hz累计466951周期，deadline/skipped/output errors均0，算法错误0。
只读快照不是运动耐久、人体助力效果或遥测七通道20 Hz通过证据。

## 软件与浏览器验证

- `npm test`（含生产构建）：63/63 PASS。
- `npx tsc --noEmit`与`npm run lint`：退出码0。
- 真实网页逐栏检查：连接、标定设置、左右助力、实时数据缺失状态、数据记录。
  断连时START/配置禁用，未读状态不显示成真实设备0值；390 px窄屏导航/运动布局已检查。
- 用同一天既有真实只读BLE事务日志整理70个IDLE状态快照，作为一条「静止 / 调试观察」导入。
  记录日期2026-10-09、采样时长87.109秒、有效步态0、RUNNING样本0、平均步频缺失。
  不含模拟行走数据，不支持真人SHADOW/ACTIVE完成结论。
- 浏览器导入→刷新→查看记录成功；选择2026-10-10为空，2026-10-09显示记录。
- 通过真实网页按钮分别下载CSV与JSON。JSON元数据及70点与来源完全一致；CSV20列70点一致，
  null导出为空，度值与时间保留。核对脚本退出码0。

证据目录：`output/phase10/20261009T163003_product_calibration_0102/ui-redesign/`。
主要文件：`tests.txt`、`typescript.txt`、`lint.txt`、`record-after-reload-dom.txt`、
`records-date-empty-dom.txt`、`records-detail-dom.txt`、`record-export.csv`、`record-export.json`、
`record-roundtrip-result.json`、`motion.png`、`motion-mobile.png`、`records.png`、`manifest.json`。
UART原始文件在父目录`ui_redesign_precheck_uart.*`与`ui_redesign_final_uart.*`。

## 限制与后续

本轮未在新版界面发送真实START、助力配置或软件零位写入。
自动RUNNING记录/STOP结束、真人主动SHADOW、真人ACTIVE 0%/5%/10%仍需真实硬件验收；
断连重连、全七通道20 Hz持续负载、工程开关关闭后的完整运动回归仍不能由本轮软件测试替代。
日期记录当前没有实际步数/距离，也不能把髋屈曲峰相位等同足跟着地。
记录仅保存在当前浏览器，重要记录需要导出备份。
固件候选manifest保持烧录时身份，新的前端身份另存ui-redesign/manifest.json；工作区未提交或推送GitHub。
