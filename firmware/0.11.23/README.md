# 0.11.23：行走记录同步保存USB控制周期诊断

状态：固件与网页修改、离线验证和构建完成。未烧录、未打开真实串口/BLE、未START、未使能。AFO策略仍为12，沿用0.11.22修订3的计算与保护。

## 下一次操作

下面步骤适用于烧录0.11.23并使用更新后的本地网页之后。

1. 保持USB烧录线连接，关闭占用同一COM口的IDF Monitor、串口助手和旧的人工USB采集功能。无需填写或硬编码COM号。
2. 先在网页连接蓝牙，读取设备信息。进入原来的“行走数据记录”，点击 **连接USB诊断**，在浏览器选择ESP32原生USB接口。
3. 等待“USB诊断已连接，同一设备启动编号已核对”。浏览器每次连接核对USB与BLE的bootId，且USB固件必须为0.11.23；错误设备、重启或不匹配版本不会混入有效高频记录。
4. 点击原来的 **开始记录**，再按原有模式、等级/上限、保存、START流程进行测试。建议在START前开始记录，以保留静止基线、授权、初始捕获和渐增全过程。记录按钮本身不START、不使能。
5. 测试时保持网页前台，观察USB接收数增长以及“缺口/解析异常”计数。BLE断开但USB仍连接时继续保留USB；两路均断开会结束本段。换设备/启动编号不匹配会结束本段，避免混淆。
6. 操作者按原流程停止助力后，点击 **停止并保存记录**。导出 **完整JSONL**；该文件同时包含BLE、USB、配置和异常。需要表格时额外下载 **USB诊断CSV**。原CSV仍是BLE采样数据，不用它代替高频USB数据。

记录保存在浏览器IndexedDB，每秒自动保存。旧版记录迁移后保留；关闭页面前先停止并导出。每段BLE上限72000个时刻、USB上限120000条诊断，达到任一上限显式结束记录，不覆盖开头。USB每控制周期两条腿记录，另有状态/计数快照，因此120000条并不等于120000个完整控制周期。

Web Serial需要电脑端支持该API的浏览器和安全上下文（本地localhost或HTTPS）。实现依据 [Chrome官方Web Serial文档](https://developer.chrome.com/docs/capabilities/serial/)，采用浏览器选择端口、显式释放读写锁后关闭。打开后DTR/RTS设为false，不发送复位脉冲；首次真实连接时仍需核对启动编号，因此建议在START前连接。

## 新增记录内容

| 数据 | 用途 |
|---|---|
| 固件版本、ELF SHA256、BLE/USB共同bootId | 判断是否正确固件、是否发生重启、是否选错设备；ELF哈希与manifest的elf_sha256比较，不能与bin哈希混用 |
| 实际AFO配置及generation，196字节原始编码和CRC | 保留真实窗口、速度、功率、变化率、Gaussian参数与渐增配置；不靠源码默认值猜测 |
| 控制周期id、sequence、采样us、坐标epoch、START permit | 对齐同一控制周期；识别掉帧、重启、重新授权和坐标变更 |
| 左右源时间、源计数、年龄、源间隔、左右时差 | 区分重复帧、过期、异步反馈和源回退；源时间是固件反馈时间，不宣称电机硬件同步采样时间 |
| 关节角度、原始角速度、电机估算力矩、温度、有效性标志 | 重建控制实际看见的输入；电机估算力矩不是人体交互力矩 |
| 上一条被算法保存的源时间、角度和速度 | 直接复核角度跳变校验，首次故障时也保留，不再依赖10 Hz插值重建 |
| 角度差分滤波速度、相位、周期、lambda、有效间隔数、事件年龄 | 定位捕获、事件超时、停步和渐增中断 |
| profileNm、desiredNm、limitedNm、suggestionNm、finalNm | 分别是方向门之前、方向门之后、限幅后、等级缩放后的策略建议、最终控制器FF命令 |
| 每侧command_valid与queuedNm | queuedNm只有该侧CAN命令成功入队时才有值，否则为null；入队成功仍不是实测关节力矩 |
| 故障、锁定原因、rearm、输出门位、拒绝码、系统返回值 | 区分算法零输出与下游拒绝，确认硬锁止原因 |
| 每个授权tick累计分类与最长连续零命令tick数 | 避免10 Hz漏采误计；累计tick数不是步态成功周期数，应按同一permit做差分 |
| 首次硬故障完整快照、系统历史故障、CAN故障时TEC/REC | 首次硬故障不随环形覆盖或STOP丢失；新START代次或重启清除。系统状态快照保留系统门故障 |
| 上一完整控制周期执行时间、当前控制dt | 下一次量化USB采集开启后的控制时序；执行时间明确滞后一周期 |
| USB连接/断开、缓冲覆盖、缺侧/缺行、解析错误与原始行 | 缺失保持缺失，不补零、不插值成“完整正常步态” |

`systemState`是计算入口的系统状态；输出门位和systemResult来自本周期实际输出调用。`usb_system`和`usb_counters`是异步读回，不应冒充某一AFOROW时刻的原子系统快照。

## 控制任务与USB传输

- 永久100 Hz任务只写固定128个双腿快照的环形缓冲，不格式化、不写NVS、不等待USB、不开新电机任务。
- 低优先级现有console任务响应新命令 `afo_trace <cursor>`。只读，最多返回24个双腿周期，以固定上界游标结束；USB慢或断开不会阻塞生产者。
- 每条控制周期具有单调id；读取不消耗环形记录。旧游标落后缓冲时，网页记录明确缺口。首次硬故障另存，不依赖环形容量。
- 网页只发送上述只读命令，不提供任意串口命令入口。正常约100 ms轮询一次，有积压时连续读取有界批次。有效数据率需真实USB链路测试，不能仅因目标100 Hz就声称无丢帧。
- 保持GPIO42待机供电、CAN TX4/RX5 1 Mbps、方向和零位存储、START授权及全部既有限幅/硬故障保护。

## 导出格式

完整JSONL的session schema升为2，继续保留旧`sample`类型；新增：

- `usb_identity`：版本、bootId、ELF哈希、环形边界。
- `usb_config`：真实配置generation、原始configHex与解码参数；同一generation只保存一次。
- `usb_trace`：控制周期腿快照。以bootId + id + side去重。两侧同id才是一组完整周期。
- `usb_first_fault`：保留的首次硬故障，不要把它重复计入usb_trace周期统计。
- `usb_counters`、`usb_system`：异步只读快照。
- `usb_gap`、`usb_error`：丢失范围、解析/传输异常；不代表零助力。

USB设备时间单位us，旧BLE时间单位ms，hostTime为浏览器接收时间。以设备时间和bootId/permit分组分析，不直接用USB批量接收的hostTime当100 Hz采样时间。

旧`simulate_afo_recording.py`仍读取BLE `sample`做低频插值回放；它会忽略新增USB记录。后续定位真实失助，应直接分析JSONL中的`usb_trace`与`usb_first_fault`，不能把旧脚本的插值结果称为USB原始反馈回放。

### schema1 USB只读协议

每批为AFOMETA、AFOCFG、AFOSYS、AFOCOUNTS、可选AFOFIRST、最多48行AFOROW，最后AFOEND。AFOROW/AFOFIRST字段顺序以 `web-bluetooth-console/lib/exo-usb-diagnostics.ts` 的TRACE_FIELDS与 `main/exo_afo_trace_wire.c` 为共同规范；C生成golden fixture验证浏览器解析，不采用内存结构体直接拷贝上线。

- gateFlags bit0..7：output_gate_open、base_generated、assist_suggested、assist_accepted、assist_applied、send_attempted、send_succeeded、send_partially_succeeded。
- flags bit0..4：算法输入valid、locked、rearm、该侧CAN入队成功、assist_result有效。
- sensorFlags bit0..7：snapshot_available、mapping_valid、direction_valid、zero_valid、feedback_valid、online、motor_fault、joint_valid。
- category：1非零入队命令、2硬故障、3工作窗口、4捕获/相位、5STOP、6下游零输出、7波形/方向零输出。非零分类阈值1e-6 N·m，不等于分析采用的±1 N·m。
- rejectReason：0无、1未请求、2SHADOW、3结果无效、4执行状态不符、5授权代次不符、6传感器序号不符、7步态序号不符、8时刻不符、9输出类型不符、10合成无效、255未知。
- faults：1输入、2时间、4窗口、8相位、16STOP、32硬锁止、64配置；lockReasons沿用现有定义。
- AFOCFG携带现有schema1的196字节配置编码（含CRC），这里只读，不执行导入或写配置。

## 验证与限制

证据位于 `output/can/20261118/v23_work`。

- 11组AFO主机回归通过，系统健康故障门8/8通过。
- 新USB固件回归通过：环形覆盖、非破坏性游标读取、首次故障及上一源样本保留、STOP保留、新授权清除、重启清除、编码边界。
- 网页构建、206项测试、TypeScript通过；覆盖分包读取、错误bootId、损坏/缺失帧、配置CRC、存储失败重试和容量上限。
- 独立浏览器真实IndexedDB测试通过：v1→v2迁移保留旧记录，10005条诊断刷新后仍完整，JSONL与USB CSV导出正确。测试没有访问用户浏览器或硬件。
- 六组原记录输出链回放共163128条腿快照，0.11.23与0.11.22r3的力矩、faults、rearm和locked逐点相同。
- ESP-IDF v5.5构建和镜像校验通过。静态栈帧：控制task 1728字节、trace_finish 448字节、console命令496字节、formatter464字节；这是单函数编译报告，不是完整调用栈或目标机高水位实测。128周期环形缓冲39936字节，两个首次故障槽624字节；链接报告DIRAM静态剩余125373字节，不等于运行时可用堆。
- 尚未验证真实USB吞吐、USB/BLE同时运行时的目标机时序、实际掉线行为及人体助力稳定性。下次日志会保存dt、执行时间、缺口和故障，便于把这些问题逐项验证。

下次测试评价应分别报告：记录覆盖率、初始捕获之后整周期无命令数量、双向均超过1 N·m的周期数量、保护触发和真实穿戴感受。不能用非零tick总数替代完整步态周期成功率。
