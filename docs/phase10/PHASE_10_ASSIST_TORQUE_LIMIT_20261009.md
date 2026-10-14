# 2026-10-09 校准锁定提示与最大附加助力配置

状态：`0.10.3 SOFTWARE BUILD/REPLAY PASS; NOT FLASHED; TARGET BLE E2E NOT RUN`。
本记录不能替代真人或目标板验收。阶段10整体仍未完成，阶段11未完成。

## 当前现场结论

0.10.2此前ACTIVE 10%尝试产生CAN BUS_OFF，网页STOP落到ERROR/VERIFY_DISABLED。
本轮再次仅通过COM6/115200查询 `system_status`、`can_status`、`assist_status`：
仍为ERROR/BUS_OFF，主控运行请求/输出门均关闭，助力DISABLED/NOT_ARMED，
START requests=1/success=1、STOP requests=1/failure=1，未新增启停。
反馈已失效，不能认定电机已收到失能命令，不能用旧零位偏置或蓝牙连接状态认定安全。
动力电源是否切断仍等待操作者确认。串口打开前DTR/RTS均置False，用后已关闭。
本轮没有CAN恢复、START、使能、归零、机械置零或烧录。

网页修改热更新使BLE会话断开；断开本身不等于电机停机。本轮浏览器只验证了
未连接状态下的布局、导航和按钮锁定，没有伪造正常电机数据。

## 修改合同

- 校准仍要求IDLE、有效且新鲜的双髋反馈、本次左右对应/方向核对和穿戴中立位确认。
  ERROR时显示“故障锁定”、失败步骤和明确的反馈状态；未读取反馈时显示“尚未读取”，
  不把未读取或离线数据显示成0°。
- 运动栏目新增“最大附加助力力矩（N·m）”，与左右等级一起保存并读回。
  默认0.500 N·m，允许0～0.500 N·m、精度0.001 N·m，保留既有工程曲线硬上限。
  该上限是附加前馈力矩的绝对值，不是RS01额定/峰值力矩，也不包含基础阻尼或反馈力矩。
- profile1的每侧建议为：归一化相位曲线 × 配置最大值 × 本侧等级/100。
  最大0.200 N·m、左10%/右50%对应左峰值0.020 N·m、右峰值0.100 N·m。
  0上限或0%给出全零增量；非法值拒绝且纯计算输出清零。
- 最大值与策略/等级一同存于RAM，START锁存整份配置，运行时禁止改写；重启恢复默认。
  仍只有一个100 Hz控制任务、一个最终运动出口；SHADOW仍不施加附加助力。
  现有新步态事件、传感/步态序号、epoch、同周期时间戳、1秒渐入和斜率限制全部保留。
- 网页区分草稿峰值、设备已保存值和本轮锁存上限。新能力仅在INFO声明bit9后开放；
  0.10.2保持连接兼容，显示固定曲线值并锁定新上限写入，不发送未知操作码。
- 自动状态查询每秒低频读回助力门禁；遥测运行时每2秒串行读回系统/助力状态。
  STOP失败也刷新助力执行态。相同配置代次的后台读回不覆盖用户正在编辑的草稿。
  新轮询仍属于显示层，不参与主控实时控制；持续遥测负载真机回归尚未执行。

## BLE v1向后兼容扩展

帧头与20-byte最大帧长不变，INFO版本为0.10.3，产品capabilities为0x037F。
工程激励开关关闭；如隔离工程构建显式开启，则额外bit7，mask为0x03FF。

| 请求 | payload | 响应 |
| --- | --- | --- |
| GET_ASSIST_TORQUE_LIMIT `0x0A` | 空 | `0x8A`，8-byte |
| SET_ASSIST_CONFIG_WITH_LIMIT `0x19` | 原8-byte助力配置 + 最大值u16LE mN·m，共10-byte | 普通4-byte ACK `0x99` |

GET的8-byte payload为：schema=1、flags（bit0选择有效/bit1活动有效）、
selected maximum u16LE、active maximum u16LE、hard maximum u16LE。
没有活动锁存时active字段为0且bit1清零，网页解析为null，与“活动0 N·m”不同。
SET协议和业务层双重检查0～500 mN·m，并在IDLE维护互斥内一次修改整份选择配置。
旧 `0x16` 和UART旧 `assist_config` 保留已保存上限，不默默恢复为默认值。

共享向量（主机C与网页Node测试直接校验）：

```text
GET:             01 0A 31 00
SET 0.200 Nm:    01 19 32 0A 01 02 02 02 01 0A 32 00 C8 00
GET selected:    01 8A 31 08 01 01 C8 00 00 00 F4 01
```

## 软件证据与限制

证据目录：`output/phase10/20261009_assist_torque_limit/`。

| 验证 | 结果 |
| --- | --- |
| `tools/phase10_replay.ps1` | PASS 7/7；含1000相位点的双向限幅、0上限、NaN/越界清零 |
| `tools/phase10_assist_integration_replay.ps1` | PASS 4/4；直接运行固件传感/步态/助力/控制器，替代最终CAN发送；检查0.200 N·m配置真正约束最终命令、锁存、拒绝越界且保留选择代次 |
| `tools/phase10_assist_limit_replay.ps1` | PASS 4/4；直接链接固件纯C协议，验证原子配置/上限读回与旧请求 |
| `tools/phase10_ble_acceptance.py --self-test` | PASS 9/9；仅离线自测，没有BLE写入 |
| `npm test` | PASS 66/66（含构建） |
| `npx tsc --noEmit` / `npm run lint` | PASS |
| ESP-IDF5.5，现有工程 `idf.py build` | PASS，ESP32-S3、TX4/RX1、16 MB Flash、工程激励关闭；构建信息及SHA见manifest |
| 浏览器 | 仅未连接界面/导航验证，输入显示未生效，按钮保持锁定；截图/DOM已保存 |
| 新配置目标板BLE读写、校准恢复、真实限幅 | NOT RUN，未烧录0.10.3 |

运行期CAN/反馈失败尚未自动触发状态机清理的问题仍待修复，本次没有声称解决电气故障。
需要先确认断电并检查CAN接线/端接/接地/供电；不能仅靠烧录解除当前风险。
恢复后需独立验证新固件读回、零位校准、真人SHADOW及ACTIVE分级门禁。
Git工作区保留原有修改，本次未提交/推送；GitHub备份未更新。

后续同日CAN排查在该未烧录候选增加运行保护，详见
[CAN复核与保护证据](PHASE_10_CAN_DIAGNOSIS_20261009.md)。本文件manifest/binary保留为
较早候选快照；最新CAN保护构建独立冻结于`20261009_can_diagnosis/`，按SHA对应源码。
