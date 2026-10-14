# 阶段 10 相位曲线来源与版本记录

本文档是相位曲线进入固件前的来源登记表。代码能生成某条周期波形，不等于已经复现论文；只有相位原点、单位、符号、幅值和提取方法都可追溯时，才允许把来源类型改为“文献曲线”。

## 必填字段

| 字段 | 含义 |
| --- | --- |
| profile ID / version | 固件和导出数据中的稳定身份 |
| display name | 网页显示名称 |
| source type | `ENGINEERING_TEST`、`LITERATURE_DIGITIZED` 或 `MEASURED` |
| source reference | 论文 DOI/标题/图表，或测量记录路径 |
| source phase origin | 原论文/数据的零相位事件 |
| firmware phase origin | 当前为每侧髋屈曲峰值 |
| phase transform | 原点偏移、方向和回绕公式 |
| value meaning | 归一化力矩、绝对力矩或其他物理量 |
| units and sign | N·m；人体髋屈曲为正 |
| sample count | Flash LUT 点数 |
| peak torque | 100% level 对应的峰值 N·m |
| active allowed | 是否经过独立审核后允许进入命令合成器 |
| extraction method | 原始表格、曲线数字化或测量方法 |
| checksum | 源数据或生成数组的校验值 |

## Profile 1：阶段 10 工程链路测试曲线

| 字段 | 当前记录 |
| --- | --- |
| profile ID / version | `1 / 1` |
| display name | `ENGINEERING_TEST_V1` |
| source type | `ENGINEERING_TEST` |
| source reference | 无论文来源；仅用于插值、SHADOW、命令合成和低幅台架链路验收 |
| source phase origin | 人工定义的周期测试波形 |
| firmware phase origin | 每侧髋屈曲峰值 |
| phase transform | 无额外偏移；左右各自消费本侧相位 |
| value meaning | 归一化前馈力矩波形 |
| units and sign | 缩放后为 N·m；屈曲为正 |
| sample count | 左/右各16点；当前两侧数值相同但数组和相位输入独立 |
| peak torque | 100% level 时 `0.5 N·m` |
| active allowed | `true`；仅用于明确的低幅工程验收，不能作为论文算法效果结论 |
| extraction method | `sin(2πphase)` 的16点人工工程数组，源码保留6位小数 |
| checksum | 单侧16个 IEEE-754 float32 小端字节的 SHA-256：`cfa854bbe927b293f45c0b17c9876819937e04e5ed36621e1f4c3bb065166e6f` |

## Profile 0：全零链路诊断曲线

| 字段 | 当前记录 |
| --- | --- |
| profile ID / version | `0 / 1` |
| display name | `ZERO_DIAGNOSTIC_V1` |
| source type | `ENGINEERING_TEST` |
| source reference | 无论文来源；用于先打通 ACTIVE 路由而不附加力矩 |
| source/firmware phase origin | 每侧髋屈曲峰值；因曲线恒零，原点不改变数值 |
| value meaning | 恒为0的前馈力矩增量 |
| units and sign | N·m；人体髋屈曲为正 |
| sample count / peak torque | 左/右各16点；`0 N·m` |
| active allowed | `true`；用于 ACTIVE 0%/零曲线链路验收 |
| checksum | 单侧64个全零字节的 SHA-256：`f5a5fd42d16a20302798ef6ed309979b43003d2320d9f0e8ea9831a92759fb4b` |

以上 checksum 的输入都是“单侧16个数组元素依次编码为 IEEE-754 binary32、
little-endian 后拼接”的64字节，不包含 C 变量名、空白、右侧副本或峰值元数据。

后续导入论文曲线时必须新增 profile ID，不得在不升级版本的情况下覆盖 Profile 1，也不得把 Profile 1 改名冒充论文结果。
