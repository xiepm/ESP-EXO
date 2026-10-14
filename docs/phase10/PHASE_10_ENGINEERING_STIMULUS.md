# 阶段 10.1：人体助力链与主动工程激励的边界

适用固件版本：`0.10.1`。`0.10.0` 仍是阶段 10 相位曲线、命令合成和遥测的
基线版本；本文件只定义其后的隔离工程验证能力。

## 1. 两条完全不同的因果链

产品要实现、最终也必须由真人验证的链路是：

```text
人体主动迈步
  -> RS01编码器反馈
  -> 100 Hz传感快照
  -> HIP_EVENT_TBE步态估计
  -> ACTIVE_PHASE相位曲线建议
  -> ACTIVE命令合成
  -> 外骨骼助力人体
```

这条产品链的软件对象、锁存合同、无效步态清零、SHADOW/ACTIVE隔离、唯一命令
合成和遥测已经实现。当前尚缺的是“穿戴者主动迈步 + ACTIVE 低幅输出”的真人真机
验收证据。因此准确结论是：**人体驱动助力链代码已经实现，人体 ACTIVE HIL 未完成**。

为避免每次验证步态估计器都必须先让人穿戴运动，`0.10.1` 增加一条测试链：

```text
ALTERNATING_SINE_TEST固定反相轨迹
  -> 唯一100 Hz控制任务和唯一CAN命令出口
  -> 电机/机械关节运动
  -> RS01编码器真实反馈
  -> 传感快照和HIP_EVENT_TBE步态估计
  -> ACTIVE_PHASE + SHADOW建议
  -> BLE遥测
```

它只能验证“命令下发后的真实反馈 -> 步态估计 -> SHADOW建议 -> 遥测”链路，不能
替代人体主动迈步，也不能证明人体意图、交互舒适性或 ACTIVE 助力正确。

## 2. `ALTERNATING_SINE_TEST` 的固定合同

| 项目 | 固定值/行为 |
| --- | --- |
| 控制组合 | `IMPEDANCE + ALTERNATING_SINE_TEST` |
| 左侧目标 | `center_left + ramp(t) * 10 deg * sin(2π * 0.5 Hz * t)` |
| 右侧目标 | `center_right - ramp(t) * 10 deg * sin(2π * 0.5 Hz * t)` |
| 左右相差 | `0.5 cycle`，即180°反相 |
| 渐入 | 3 s 五次平滑渐入，之后保持单位增益 |
| 角度单位 | 人体髋关节坐标下的 `deg`，屈曲为正 |
| 前馈力矩 | 测试参考自身不生成附加前馈力矩 |
| 工程测试增益 | `kp=5.0, kd=0.1`（本机真机验证值）；停止后恢复产品`kp=0, kd=0.1` |
| 参数来源 | 编译期只读工程常量，不由手机任意改写 |
| 产品属性 | 工程测试轨迹，不是文献轨迹，不是助力策略 |

五次渐入使用：

```text
x = clamp(t / 3 s, 0, 1)
ramp(x) = 10x^3 - 15x^4 + 6x^5
```

轨迹不自行创建任务、不读取 BLE、不发送第二对 CAN 命令，仍由现有唯一 100 Hz
控制任务求值并经统一五元关节命令出口发送。

自动验收工具完成采样后不会在摆幅端点直接停止，而会在最多3秒内等待左右反馈
同时进入产品既有的中立位容差（默认±5°），再发送STOP并恢复产品默认配置。

## 3. 隔离和门禁

1. 只有工程构建显式启用 `CONFIG_EXO_ENGINEERING_GAIT_STIMULUS=y` 时，固件才允许
   选择该参考源；产品构建默认关闭。
2. 测试参考只能搭配 `IMPEDANCE`。
3. 测试参考只允许助力执行状态为 `DISABLED` 或 `SHADOW`。
4. `ALTERNATING_SINE_TEST + ACTIVE` 必须在 START 前返回“不支持/未就绪”，不能先
   使能电机后再退化。
5. 每次新的activation，即使估计器在START前已经LOCKED，PHASE_PROFILE也必须等待
   本轮LEFT和RIGHT各自出现一次新的髋屈曲峰。等待期间结果无效、建议全零，并上报
   `WAITING_NEW_GAIT_EVENTS (8)`；第二侧新峰到齐的周期才从0开始1 s助力渐入。
6. 固件初始化默认仍是产品 `DAMPING + CONSTANT`，左右均为
   `q/dq/tau/kp=0, kd=0.1`，不能因为工程能力存在而默认运动。
7. 工程构建在INFO中声明 capability bit 7（`0x0080`）；已启用构建的完整mask为
   `0x00FF`，关闭工程开关时仍为 `0x007F`。
8. 退出、超时、BLE异常或验收断言失败后都必须请求整机 STOP；BLE遥测 STOP 本身
   不等于整机 STOP。
9. 工具不得自动归零，更不得执行 RS01 机械置零。

## 4. 它能证明和不能证明什么

| 观察结果 | 可形成的证据 | 不能据此声称 |
| --- | --- | --- |
| 左右反馈约反相且有足够摆幅 | 映射、方向、位置参考、CAN和反馈闭环实际联通 | 真人会自然产生相同轨迹 |
| 估计器进入LOCKED并输出双侧相位 | 反馈驱动的HIP_EVENT_TBE可消费真实关节运动 | 已识别人体运动意图 |
| SHADOW建议周期性非零 | 相位曲线能够消费估计结果 | 助力已经作用于人体 |
| `APPLIED_TORQUE=0`且real-output位为0 | SHADOW没有把建议写入最终命令 | ACTIVE低幅输出正确或舒适 |
| STOP后IDLE/REST | 测试流程退出和失能路径有效 | 所有故障工况均已覆盖 |

因此，主动测试激励 PASS 后，人体 ACTIVE HIL 仍然保留为独立、不可跳过的验收项。

## 5. 分级执行顺序

1. 主机纯数学测试：检查10 deg、0.5 Hz、3 s渐入和左右严格反相。
2. 主机真实C实现测试：直接编译控制器的纯求值入口，检查默认产品模式、测试门禁、
   端点和周期；不得用另一份Python公式冒充固件实现。
3. 固件工程构建：分别验证开关关闭和打开的构建；关闭时不能接受/公布测试参考。
4. 无运动真机预检：只读INFO、IDLE、零位、反馈、故障和控制配置。
5. 空载或受控台架测试：只允许 `SHADOW`，固定10 deg/0.5 Hz/3 s，记录完整遥测。
6. 停止并核对IDLE、A/B REST、实际附加助力为0；随后恢复并读回产品默认
   `DAMPING + CONSTANT`、左右`q/dq/tau/kp=0, kd=0.1`及
   `TRANSPARENT + DISABLED`。
7. 另行执行真人主动运动 SHADOW，再逐级执行真人 ACTIVE 0%、5%、10%验收。

第5级不能替代第7级。任一级失败，不得通过修改文字把结果写成 PASS，也不得跳过
STOP直接进入下一级。

## 6. 主机与真机工具

纯主机门禁和轨迹测试：

```powershell
python .\tools\phase10_motion_acceptance.py --self-test
```

默认连接真机仍然只读，不会START：

```powershell
python .\tools\phase10_motion_acceptance.py --name ESP-EXO
```

只有明确授权的工程台架才能执行主动激励：

```powershell
python .\tools\phase10_motion_acceptance.py `
  --name ESP-EXO `
  --allow-motion `
  --case alternating-sine-test `
  --confirm-motion "ESP-EXO PHASE10 ALTERNATING SINE MOTION" `
  --duration 30
```

该命令只配置 `SHADOW`。工具代码中没有把测试参考写成 `ACTIVE` 的执行分支，并在
`finally` 中无条件请求整机 STOP。

## 7. 2026-08-12 真机结果

开启工程能力的`0.10.1`已在COM5真机通过：153个完整样本，左右反馈峰峰值
`14.6/15.4 deg`、相关系数`-0.989`、有效相位`62%`、非零SHADOW建议`43%`；
实际附加助力全程为0。工具在`0.52/1.89 deg`的中立交叉处STOP，并恢复产品默认。
`TEST + ACTIVE`也已在目标板验证为START前`RUN_CONTRACT_INVALID`，A/B保持REST。
产品`DAMPING + CONSTANT + ACTIVE 10%`静止真机测试中，步态无效、建议为0、
实际施加0次，STOP后A/B回REST。

这只关闭工程SHADOW轨道；真人主动运动SHADOW及ACTIVE 0%/5%/10%仍为`NOT RUN`。
