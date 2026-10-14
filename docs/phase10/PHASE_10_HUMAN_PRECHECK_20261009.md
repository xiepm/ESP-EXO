# 2026-10-09 穿戴后只读预检

操作者报告“现在人体已经穿上去了，接下来怎么做”。本次约15:47–15:48（Asia/Shanghai）
仅执行UART诊断，没有归零、使能、START、机械置零或烧录。串口打开前DTR/RTS均关闭，
完成后已关闭串口。

## 当前证据

原始记录：`output/phase10/20261009T100926_web_motion/human_worn_precheck_uart.bin`
及同名`.txt`。它们是穿戴后的新快照，不是上午台架运动证据。

- 系统IDLE；run_requested=no、output_allowed=no；START次数0。
- Boot ID `0x97E1F35C`，不同于上午台架的`0xB471CB40`，表明当前为新的启动会话。
- A=0x10、B=0x20均反馈INVALID/OFFLINE、反馈计数0、mode UNKNOWN。
- 双侧RAM软件零位INVALID；START门禁为LEFT_ZERO_INVALID。
- CAN控制器RUNNING，RX GPIO1空闲高；TX error counter=164、bus error count=913，
  RX error/TX failed/RX missed均0。这不能证明当前电机通信正常。
- 基础配置为DAMPING+CONSTANT，左右q/dq/tau/Kp=0、Kd=0.1；助力TRANSPARENT/DISABLED。
- BLE未连接、Notify未订阅、遥测停止。
- 快照中的角度/力矩/故障字段0均没有有效反馈支持，不能据此宣称人体中立位或电机无故障。

电机电源关闭、CAN接线或收发器异常均属于待排除原因，当前尚未定位根因。
已询问操作者两台RS01电源是否打开；在恢复新鲜有效反馈前不进行归零或START。

## 后续操作顺序（尚未执行）

1. 维持停止，穿戴者有稳固支撑，现场有人能立即停止或切断电机电源；先核对电机供电和CAN。
2. 两侧持续有效反馈恢复后，在IDLE确认本次装配的LEFT/RIGHT映射及屈曲正方向。
3. 本次穿戴中立位确认后，通过网页捕获双侧RAM软件零位；不执行RS01机械置零。
4. 读回DAMPING+CONSTANT和零位置增益/零前馈、Kd=0.1，配置
   ACTIVE_PHASE+PHASE_PROFILE+SHADOW，采用已较稳定的五通道10 Hz遥测。
5. 真人主动运动SHADOW先验证反馈、步态、建议及实际附加助力严格为0。
   不使用ALTERNATING_SINE_TEST；SHADOW本身不会禁用底层工程位置轨迹。
6. SHADOW通过后，另行安排ACTIVE 0%、5%、10%分级验收；每一级独立STOP并保留数据。

源码`main/exo_ble.c`明确：BLE断开不会自动调用整机STOP，不能把断开蓝牙当作停止方式。
上述步骤是后续计划，不能标记真人SHADOW或ACTIVE已通过；阶段10总体状态不变。
