# 阶段 4：唯一 100 Hz 控制任务

## 目标

阶段 4 把应用的周期控制调度集中到一个永久运行的 FreeRTOS 任务。LEFT 和
RIGHT 在同一个周期内依次读取，所有周期性控制输出只能由该任务上下文触发。

本阶段解决的是调度归属，不增加新的控制算法。阶段 3 已验收的左右映射、方向、
RAM 软件零位、RS01 主动反馈和旧正弦阻抗参数保持不变。

## 阶段边界

本阶段包括：

- 一个静态分配、上电只创建一次的 100 Hz 控制任务；
- LEFT/RIGHT 髋关节快照的逐周期读取；
- 周期、执行时间、超期、跳周期、输出错误和栈余量诊断；
- 把旧阻抗模块的周期 `esp_timer` 改为由唯一控制任务调用的单步函数；
- UART/BLE 共用的按需 `control_status` 文本诊断；
- 将示例主文件改为产品入口文件，并让 `app_main()` 只负责初始化。

本阶段不包括：

- 产品 START/STOP、系统状态机或输出许可；
- 产品 BLE 协议；
- 新参考生成器、新控制律或五元关节命令；
- 统一传感快照、步态估计、相位曲线或连续遥测；
- 把旧阻抗算法正式迁移到 LEFT/RIGHT 关节坐标。

## 唯一控制任务

`exo_control_loop.c` 保存一个静态单例。公共接口只有：

```c
esp_err_t exo_control_loop_init(void);
bool exo_control_loop_get_snapshot(exo_control_loop_snapshot_t *snapshot);
```

没有控制任务的 `start/stop` 接口。任务从启动后永久运行，阶段 5 以后产品状态的
START/STOP 只能改变请求和输出许可，不得创建、删除或重启周期任务。

任务资源固定为：

- 一个 `StaticTask_t`；
- 一块 4096-byte 静态栈；
- Core 1；
- 优先级 9；
- 频率读取 `g_exo_hardware_config.control_frequency_hz`。

初始化拒绝不能被 FreeRTOS tick 整除的频率。当前 `CONFIG_FREERTOS_HZ=100`，
控制频率为 100 Hz，因此周期是一个 tick，即 10 ms。

## 每周期顺序

每个周期执行：

```text
xTaskDelayUntil 等待绝对周期点
  -> 记录微秒级开始时间
  -> 读取 LEFT hip_joint_snapshot_t 一次
  -> 读取 RIGHT hip_joint_snapshot_t 一次
  -> 调用旧阻抗兼容单步函数
  -> 记录执行结果和任务栈余量
  -> 原子更新只读诊断快照
```

`esp_timer_get_time()` 只测量时间，不负责调度。若一个周期已经超期，任务记录
deadline miss 和跳周期数，然后以当前 tick 重新对齐；不会连续快速补跑已经错过
的周期。

控制周期调用链不打印日志、不发送 BLE Notify、不动态分配内存，也不执行模式切换、
参数写入、软件归零或阻塞式 CAN 收发。

LEFT/RIGHT 两次读取属于同一个控制周期，但分别保留自身反馈时间戳和计数；阶段 4
不声称它们是严格同步采样。统一传感快照属于阶段 9。

## 诊断快照

`exo_control_loop_snapshot_t` 保存：

- 初始化、任务存活和任务身份；
- 配置频率和目标周期；
- 周期容差、周期样本数和超出容差的样本数；
- 周期计数与最近周期时间戳；
- 最近、最小、最大和平均周期；
- 最近和最大执行时间；
- deadline miss、跳周期和输出错误累计数；
- 最近输出错误；
- LEFT/RIGHT 的反馈计数、反馈时间戳和 `joint_valid`；
- 任务栈最低余量。

初始化完成后，运行期只有控制任务更新诊断状态。任务先在自己的局部状态中形成
一个完整周期，再通过短临界区一次发布完整副本；UART/BLE 只复制已发布快照，
不会读到跨周期混合字段。栈最低余量首次采样后约每秒更新一次，避免每周期扫描
整段未使用栈。执行时间覆盖关节读取、控制输出、栈扫描和局部诊断更新；最后固定
长度的快照复制不计入执行时间。

## 旧阻抗兼容路径

阶段 3 的旧阻抗模块原来创建 `esp_timer`，并在 ESP Timer 系统任务中每 10 ms
计算正弦和发送 A/B 控制帧。阶段 4 删除该调度源，增加：

```c
esp_err_t impedance_control_process_tick(int64_t now_us);
```

该函数没有任务、定时器、延时或日志，只在旧阻抗状态为 RUNNING 时计算并尝试发送
A/B 两帧；非运行状态无 I/O 且返回 `ESP_OK`。唯一控制任务是它的唯一周期调用者。

静态实时路径审计发现阶段 3 的普通发送接口会用 `portMAX_DELAY` 等待 TX mutex，
并允许每帧 `twai_transmit()` 等待 10 ms；一个周期两帧无法满足 10 ms 上界。因此
阶段 4 增加 `rs01_try_send_motion()`：运动参数校验、限幅和编码与普通接口完全相同，
但 TX mutex 和 TWAI 入队均为零等待，失败只返回给控制诊断且不打印日志。A/B 两帧
始终分别尝试，但不是事务原子；阶段 5 再由产品状态机禁止运行时维护命令插入。
普通调试和维护接口的等待及日志行为不变。

`impedance_control_start()` 仍由当前调试控制台执行准备序列，并在中心位置和开始
时间全部准备后才发布 RUNNING。`impedance_control_stop()` 先阻止新 tick，等待已经
进入的 tick 结束，再发送零增益帧并失能两台电机。

正弦频率、幅值、五次渐入、KP/KD、A 反相和 B 同相均保持阶段 3 行为。旧模块中的
A/B 是阶段 7 前的兼容实现；新控制循环及其公共 API 不使用 A/B。

## 命名边界

- TWAI、RS01 驱动、协议解析和维修命令继续使用执行器 A/B；
- 髋关节层和新的控制循环使用 LEFT/RIGHT；
- 产品 BLE 从阶段 6 起只表达 LEFT/RIGHT；
- 阶段 7 把旧正弦和阻抗控制正式迁移到 LEFT/RIGHT；
- 当前只有髋执行器，因此使用“左髋/右髋”，不提前建立整腿对象。

## 启动编排

阶段 4 将 `twai_network_example_master_main.c` 重命名为 `app_main.c`。启动顺序为：

```text
RS01 driver
-> hip joint objects
-> TWAI driver
-> TWAI RX task
-> RS01 proactive reporting
-> permanent control loop
-> BLE/UART consoles
-> app_main return
```

控制任务先于命令入口建立。`app_main()` 不包含周期循环或控制算法；初始化完成后
返回，由 ESP-IDF 删除 main task。

## 文件范围

```text
main/
|- app_main.c
|- exo_control_loop.c
|- include/exo_control_loop.h
|- impedance_control.c
|- impedance_control.h
|- ble_console.c
|- rs01_driver.c
|- include/rs01_driver.h
`- CMakeLists.txt

docs/phase4/
|- PHASE_4_DESIGN.md
`- PHASE_4_ACCEPTANCE.md

README.md
CONSOLE_COMMANDS.md
```

不修改 `hip_joint`、RS01 协议解析、普通发送语义、LEFT/RIGHT 映射、软件零位和
`sdkconfig`。RS01 只增加上述零等待、无日志的周期发送入口；理由和边界由静态
最坏路径审计记录，不改变帧编码或反馈解析。

## 完成标准

1. 只有一个应用级周期控制任务和一个周期时间基准。
2. 旧阻抗模块不再创建或启动周期定时器。
3. LEFT/RIGHT 在同一个任务周期内读取，新控制模块不出现 A/B。
4. 开机创建控制任务不会自动使能电机或产生运动输出。
5. 任务只创建一次，BLE 重连及旧阻抗启停不改变任务身份或导致频率翻倍。
6. 空闲和旧阻抗运行时均达到 100 Hz 时序验收且没有 deadline miss。
7. UART/BLE、TWAI、主动反馈、软件零位和 3x10 s 旧阻抗回归通过。
8. ESP-IDF v5.5 干净构建、尺寸和烧录通过，验收证据记录固件哈希。
