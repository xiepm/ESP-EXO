# 阶段 8：统一五元关节命令与基础模式设计

## 1. 目标和边界

阶段 8 把阶段 7“正弦控制器直接拼 RS01 参数”的专用路径，收敛为所有后续论文
算法共用的关节命令管线：

```text
BLE/UART（只在 IDLE 写 selected 配置）
                  |
                  v
START 锁存 active 配置和左右起点
                  |
唯一 100 Hz 任务 -> 参考源/基础模式 -> LEFT/RIGHT 五元命令
                  -> 关节坐标映射 -> A/B 排列 -> RS01 Motion 帧
```

本阶段不增加整机状态，不创建新的周期任务、队列、定时器或动态对象，也不实现
传感统一快照、步态估计、论文相位曲线和连续遥测。BLE 只负责低频静态配置，手机
不能代替 ESP32 的 100 Hz 本地控制时钟。

## 2. 五元关节命令

每侧命令为：

```c
typedef struct {
    float position_deg;
    float velocity_deg_s;
    float torque_ff_nm;
    float kp;
    float kd;
} exo_joint_command_t;
```

前三项使用人体髋关节坐标，屈曲为正；位置、速度和力矩单位分别为 `deg`、
`deg/s`、`N·m`。`kp/kd` 是 RS01 Motion 协议增益。RS01 手册给出控制关系
`t_ref = Kd(v_set-v_actual) + Kp(p_set-p_actual) + t_ff`，但没有给出足以证明实际
物理标定单位的资料，因此产品代码不把它们命名为已标定刚度或阻尼。

该 C 结构不是 BLE/CAN 线格式，禁止直接 `memcpy` 序列化。

## 3. 坐标映射和发送原子边界

`hip_joint` 是唯一关节到电机的换算边界：

```text
q_motor_rad  = deg_to_rad(zero_motor_deg + direction * q_joint_deg)
dq_motor_rad = direction * deg_to_rad(dq_joint_deg_s)
tau_motor    = direction * tau_joint
kp_motor     = kp
kd_motor     = kd
```

当前真机配置是 `LEFT -> B/+1`、`RIGHT -> A/-1`，但控制器不硬编码该关系。完整
双侧命令先经过 finite、协议量程、左右映射和重复执行器检查；任一侧失败时一帧
CAN 都不发送。全部成功后固定 A 后 B 零等待发送；A 失败仍尝试 B，并返回首个错误。
RS01 Motion 编码入口拒绝越界，不再静默把错误命令裁到边界。

## 4. 基础模式

所有产品模式底层都使用 RS01 `MOTION`，不在 100 Hz 路径调用 PP、CSP、独立速度
或电流模式。

| 模式 | 生成规则 |
|---|---|
| HOLD | `q=q_start,dq=0,tau=0`，使用配置 KP/KD |
| POSITION | `q_start` 五次渐入至配置 q，`dq=tau=0`，使用 KP/KD |
| VELOCITY | `q=q_start,kp=0,tau=0`，目标速度五次渐入，使用 KD |
| TORQUE | `q=q_start,dq=0,kp=kd=0`，前馈力矩五次渐入 |
| DAMPING | `q=q_start,dq=tau=kp=0`，KD 五次渐入 |
| IMPEDANCE | q、dq、tau、KP、KD 全部有效；动态项按启动 ramp 渐入 |

参考源与模式独立：`CONSTANT` 表示静态参数；`SINE` 当前只允许搭配 `IMPEDANCE`。
默认选择为 `IMPEDANCE + SINE`，且默认五元是 `q_bias=0,dq=0,tau=0,kp=0.2,
kd=0.10`，因此继续得到阶段 7 的 `q*=q_start+sine_offset`。SINE 是参考源，不是
第七种模式。

START 除检查第一帧外，还预映射 CONSTANT 最终端点或 SINE 的正负满幅包络，避免
配置在 ramp 中途才第一次越过电机绝对位置量程。

## 5. 生命周期和并发

- IDLE：BLE/UART 通过整机 maintenance mutex 修改 selected 模式或单侧五元；
- STARTING：状态机完成原有失能、Motion 模式、使能、零输出和 fresh feedback 后，
  控制器一次性锁存 selected 为 active，并锁存左右当前关节角；
- RUNNING：唯一 100 Hz 任务只读 active；所有配置写入返回 BUSY；
- STOPPING：输出门先关闭，控制器 disarm 等已进入的一个 tick 退出，再执行原有
  release、disable 和 REST 反馈确认；
- IDLE：active 回到 NONE，selected 继续保存在 RAM，重启恢复编译期默认值。

不实现 RUNNING 热切换、配置 revision、NVS 或 BLE 连接租约。模式切换流程固定为
`STOP -> IDLE -> SET -> GET读回 -> START`。

## 6. BLE v1兼容扩展

保留原 v1 帧头、旧 opcode、响应和 20-byte 上限，新增 capability bit 3：

- `GET_CONTROL_CONFIG (0x04)`；
- `GET_JOINT_COMMAND (0x05)`；
- `SET_CONTROL_CONFIG (0x14)`；
- `SET_JOINT_COMMAND (0x15)`。

单侧五元使用 15-byte 定点载荷：

```text
side:u8, position_mrad:i32, velocity_mrad_s:i32,
torque_mNm:i16, kp_x100:u16, kd_x1000:u16
```

完整帧为 19 bytes。GET_CONTROL_CONFIG 正常响应使用 5-byte payload，和 4-byte
通用 ACK 明确区分。网页自动维护 request_id；用户无需输入 sequence 或 revision。

## 7. 文件职责

- `exo_controller.*`：模式、参考、selected/active 锁存、arm/tick/disarm 和快照；
- `exo_joint_command.*`：双侧预检、A/B 重排和唯一实时发送出口；
- `hip_joint.*`：完整 q/dq/tau 方向与软件零位换算；
- `rs01_driver.*`：具名电机五元结构、量程校验和 Motion 帧发送；
- `exo_system.c`：保持六态和输出门，只把控制器调用改到通用接口；
- `exo_ble_protocol.*`：纯线格式；`exo_ble.c`：IDLE 业务适配；
- `web-bluetooth-console/`：模式/五元配置、读回和既有真机验收界面；
- `app_main.c`：只增加一次静态控制器初始化，不放控制算法。
