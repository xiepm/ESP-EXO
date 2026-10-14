# 阶段10接续核对：2026-10-09

核对时间为北京时间2026-10-09 09:51至09:56左右。本文记录本轮新证据，不替代或改写2026年8月验收记录。
后续操作者确认型号和接线后的配置更新见末节；前面GPIO2、2 MB及工程开关y等内容保留为首次核对时的快照。

阶段状态保持：`0.10.1 ENGINEERING MOTION HIL PASS; HUMAN ACTIVE HIL NOT RUN`。
工程HIL的PASS来自历史记录；阶段10整体未完成，阶段11未完成。

## 已证实

### 仓库与实际开发环境

- 使用现有仓库 `D:\ESP32\esp_can`，没有重新搭建工程。
- 分支为 `main`；HEAD为 `9225bb300dde0f722d0f0ad0a30443a30b90c8bc`，说明为
  `docs: record phase 8 and phase 9 acceptance`。
- `origin` 为 `https://github.com/xiepm/ESP-EXO.git`。本地显示领先已缓存的
  `origin/main` 六个提交；本轮未fetch，因此不据此判断GitHub此刻的分支状态。
- 原有36个已跟踪文件修改，以及阶段10源码、工具、文档、网页和output/research等未跟踪内容均保留。
  没有reset、clean、checkout覆盖、暂存、提交或推送。
- 已阅读README、CONSOLE_COMMANDS、阶段9/10验收记录和工程激励边界，历史人体阶段9PASS不等同于阶段10人体ACTIVE PASS。
- 本轮通过仓库激活脚本实际运行 `idf.py --version`：ESP-IDF v5.5。
  `tools/cmake/version.cmake` 为5.5.0；IDF源码Git描述为v5.5。
- IDF路径为 `C:\Espressif\v5.5\esp-idf`，Python虚拟环境为
  `C:\Espressif\tools\python\v5.5\venv`；GCC为14.2.0，工具链版本
  `esp-14.2.0_20241119`。这些是本机环境事实，不是当前板上固件的IDF版本证据。
- 仓库源码及构建目标为ESP32-S3、固件0.10.1。
- `main/exo_config.c` 显式配置LEFT→B/+1、RIGHT→A/−1。这是仓库配置，当前装配侧别尚未复核。
- 当前仓库 `sdkconfig` 为TX GPIO4、RX GPIO2，且
  `CONFIG_EXO_ENGINEERING_GAIT_STIMULUS=y`。Kconfig的默认值虽然为n，不能覆盖已保存的y。

### 当前串口与CAN参数往返

pyserial枚举和Windows PnP均识别COM6：FTDI VID:PID `0403:6001`，序列号
`A5069RR4A`；COM3/COM4为蓝牙虚拟串口。FTDI身份只标识串口适配器，不能证明后接ESP32的型号或板卡身份。

两次查询会话均在open前设置 `dtr=False`、`rts=False`，115200 baud。
会话结束均执行finally关闭串口，并将控制台目标选择恢复为原来的0x10。
未请求复位、下载模式、烧录、START/STOP、使能/失能、软件/机械归零、上报开关或电机参数写入。
本轮只运行了主机端esptool的镜像文件解析，没有使用esptool连接COM6。

当前help返回 `RobStride Motor Control Console`，选择命令为数字CAN ID，
`system_status` 返回Unknown command，help中没有芯片或固件版本查询命令。

| 查询 | CAN回复 | 本轮结论 |
| --- | --- | --- |
| `motor 0x10` → `read_param 0x7005` | `0x110010FD`, `0570000000000000` | 节点0x10的run_mode参数读回为0 |
| `motor 0x20` → `read_param 0x7005` | `0x110020FD`, `0570000000000000` | 节点0x20的run_mode参数读回为0 |
| 查询前后 `can_status` | RUNNING；RX GPIO1 level=1 | 旧固件诊断报告接收引脚为GPIO1 |
| 查询前后TWAI计数 | TX/RX error、TX failed、bus error、arbitration lost、RX missed均为0 | 本次参数查询窗口未观察到上述错误 |

`0x7005` 是控制模式参数，不是反馈中的REST/使能状态，也不是无故障证明。

根据本地RS01手册读取只读寄存器后，得到以下**单次电机原始参数**：

| 节点 | 0x7019 mechPos | 换算电机角度 | 0x701B mechVel | 0x701C VBUS | 0x7026 |
| --- | --- | --- | --- | --- | --- |
| 0x10 | −0.601369917 rad | −34.455958 deg | +0.007002606 rad/s | 36.5 V | 0 |
| 0x20 | +0.364938974 rad | +20.909463 deg | −0.005705115 rad/s | 36.099998 V | 0 |

这些参数回复绑定了各自节点/寄存器，原始CAN帧已保留；未施加软件零偏和左右方向，不能直接称为LEFT/RIGHT髋角。
0x7026读回0只证明该寄存器的当前值，不证明主动上报已开启或实测频率为100 Hz。

旧控制台status仍显示角度、速度、力矩、温度全零，且不提供反馈有效位、年龄和帧计数。
它与非零mechPos参数读数不能作为同一有效位置快照使用，更不能用于中立位或无故障判断。
本次读取窗口只观察到参数应答，没有取得可用于主动上报频率验收的持续反馈序列。

### 当前旧固件与仓库0.10.1的差异

| 项目 | 当前COM6实际响应 | 仓库0.10.1实现 |
| --- | --- | --- |
| 控制台/目标 | RobStride旧接口、数字CAN ID | 产品工程控制台，A/B槽位和兼容CAN ID |
| 整机状态接口 | 无system_status | 六态状态机、START/STOP执行器、输出门 |
| 反馈显示 | rad/rad/s；全零、无新鲜度字段 | deg/deg/s；A/B独立快照、计数、有效/在线/年龄 |
| 型号/量程 | MOTOR_TYPE=5；V ±50 rad/s、T ±5.5 N·m | 专用RS01驱动；V ±44 rad/s、T ±17 N·m |
| 接收GPIO | 诊断显示GPIO1 | sdkconfig及exo_config读取GPIO2 |
| 产品步态/助力/遥测接口 | 本轮未读到、help未公布 | 仓库已有对应模块与BLE二进制协议 |

本地RS01手册PDF第42页（印刷页41）规定Motion/反馈速度和力矩编码为±44 rad/s和±17 N·m，
第48/49页列出本轮读取的寄存器。旧固件量程与当前RS01协议合同不一致，不能复用旧运动参数进行验收。
这也不能反向证明当前实际电机型号，电机身份仍需按现场标识确认。

旧接口的命令表和输出文本与Git基线中的 `main/ble_console.c` 类似，但GPIO等信息不足以锁定准确编译版本。
不将当前板称为已经烧录0.10.1，也不把当前板上功能缺口判成仓库0.10.1源码故障。

### 本轮软件复测与产品候选构建

下表均为本轮实际执行，返回码均为0；不构成真机HIL证据。

| 验证 | 结果 | 日志 |
| --- | --- | --- |
| phase10_replay.ps1 | 6/6 PASS | phase10_replay.txt |
| phase10_assist_integration_replay.ps1 | 3/3 PASS，直接链接真实C模块 | phase10_assist_integration_replay.txt |
| phase10_ble_acceptance.py --self-test | 9/9 PASS，无BLE连接 | phase10_ble_self_test.txt |
| phase10_motion_acceptance.py --self-test | 3/3 PASS，无BLE连接/运动 | phase10_motion_gate_self_test.txt |
| 网页npm test（包含生产构建） | 53/53 PASS | web_test.txt |
| 网页npx tsc --noEmit | PASS | web_typecheck.txt |
| 网页npm run lint | PASS | web_lint.txt |
| 常规git diff --check | PASS，仅LF/CRLF提示 | git_diff_check.txt |
| 工程激励关闭的独立候选构建 | PASS；665104 B（0xa2610），app分区剩37% | product_build.txt |
| 候选镜像离线esptool image_info | 校验和、validation hash有效 | product_image_info.txt |

候选配置复制自原sdkconfig，只关闭工程激励；原sdkconfig仍保持y，原build目录保留。
生成的sdkconfig.h无工程激励宏，build/project_description.json指向候选配置、esp32s3、0.10.1。
本轮真实C集成回放的host fake仍启用工程能力，不能将其解读为关闭开关后所有运行时门禁的覆盖。
关闭构建的capability/参考源拒绝行为仍须目标板验证。

候选仍使用TX4/RX2，**只是编译验证产物，当前接线未确认，不可直接作为已适配COM6的烧录方案**。

应用镜像SHA-256：
`DF1A07D3C5EBA5BEE8080F4193E2E153AEABF5775311E043FAA7BDFFE44E92E3`。

候选sdkconfig SHA-256：
`14D5942C4D73D9A611AFB24C9DF8EF6B540A7691FBD6C2AA86B9B8C2108EDB69`。

仓库原sdkconfig SHA-256：
`5F447CD681B20E91549C0E8325284243EB01E08768F00B8481CE742CBDC8849E`。

本轮两次检查未发现4173监听；访问历史/exo地址超时。网页测试PASS不代表服务正在运行，
也不代表网页按钮、蓝牙断连/重连或运动E2E已经验收。

## 尚未证实

- COM6后接ESP32的芯片、板卡型号、芯片revision/MAC及与历史COM5板的关系。
- 当前运行固件的准确版本、构建ID、来源提交及Flash镜像；FTDI序列号和旧接口指纹均不足以确认。
- 收发器TXD/RXD实际接线；已知旧固件诊断RX=1，与仓库RX=2不同，TX尚未知。
- 当前实际RS01标识、左右机械安装/方向；仓库的LEFT/RIGHT映射不能代替本次装配复核。
- A/B持续主动反馈的有效性、频率和年龄、实时故障/使能/REST状态、RAM软件零位和人体中立位。
- 当前板上的唯一100 Hz链路、产品BLE、步态与助力；目前只证实参数往返。
- 真人主动运动SHADOW、真人ACTIVE 0%/5%/10%；全部保持NOT RUN。
- 网页按钮真机E2E、遥测断连/重连、全7通道20 Hz持续负载、关闭工程激励后的目标板验收；均未补齐。
- 阶段10完整数据归档与Git收口；本轮只整理新核对证据，没有自动提交历史大量改动。

| 待解释事项 | 支持证据 | 目前限制 | 下一项最小核对 |
| --- | --- | --- | --- |
| 当前是旧接口固件 | help/system_status/量程响应 | 不能确定准确版本 | 板卡身份、获准后启动描述或Flash备份 |
| 当前板可直接烧录仓库配置 | 无足够支持 | RX1与RX2不一致、芯片未核实 | 现场芯片标识、TXD/RXD接线 |
| status零值表示中立且无故障 | 无有效性证据，参数角度非零 | 软件零偏及反馈状态未知 | 产品固件下新鲜反馈，再独立确认中立位 |
| 上报周期参数0表示已有100 Hz反馈 | 仅寄存器0 | 开关/持续帧序列未知 | 上报配置后计数与时间戳；须另行授权该配置操作 |

## 下一步

1. 先取得现场板卡/芯片标识、收发器TXD/RXD GPIO以及电机标识，保留人与执行器槽位的显式映射。
   当前已向操作者询问这些事实；不猜TX，不把旧诊断GPIO1视为完整物理接线图。
2. 如正常应用接口仍无法给出芯片/版本，提出一次独立的“进入ROM/复位、识别芯片并只读备份Flash”操作，
   取得本轮授权后执行。只读Flash不写入镜像，但进入/退出ROM可能复位、终止旧应用，不能当作无扰动串口读操作。
   备份后读取镜像描述/版本（若存在）、Flash/分区信息和SHA-256，保存可恢复的旧固件。
3. 根据已证实芯片和接线完善候选配置：若确为ESP32-S3且RXD实际为GPIO1，使用经确认的TX和RX1重新编译；
   若接线为4/2则调查旧诊断差异；若芯片不是S3，先审查现有工程目标兼容性，不能强刷S3镜像。
   工程激励保持关闭；产品默认DAMPING+CONSTANT、q/dq/tau/kp=0、kd=0.1，assist DISABLED。
4. 提供匹配目标板的配置差异、镜像SHA-256、分区/备份和无运动启动行为，取得独立烧录授权后升级。
   产品app_main启动会写上报周期/开启反馈请求，但不会使能电机；此初始化CAN参数写入也包含在升级操作说明内。
5. 升级后先无运动验证：UART/BLE INFO=0.10.1、产品capability预期0x007F、无工程能力；
   system IDLE/输出门关闭；双侧持续反馈、故障与REST状态、100 Hz/WCET/栈、CAN错误基线；
   此时零位未建立属于预期门禁，不自动归零。
6. 服务启动和网页只读连接确认后，通过真实网页操作补做遥测流启停、导出、断开/重连及全7通道20 Hz负载。
   分别记录静止和运动工况的丢包/队列/通知重试与控制统计，不将默认4通道历史PASS扩大到7通道。
7. 获得本轮中立位确认和软件归零授权后捕获RAM零位；机械置零不属于该操作。
   再独立取得运动授权，真人主动运动SHADOW先行，随后ACTIVE 0%、5%、10%逐级执行，
   每级保存相位有效性、建议/实际助力、epoch/序号和STOP后IDLE/REST证据。
   SHADOW测试也涉及基础阻尼命令/使能，不能因为无附加助力就省略运动授权。
8. 验收记录与原始数据对应后整理阶段10Git提交，再推进阶段11恢复性、清理和整体回归。
   历史“电机单独重启不自动恢复主动上报”缺口仍应保留到阶段11，不能被现有PASS覆盖。

## 本轮证据位置

统一目录：`output/phase10/20261009T095102+0800_reconnect_readonly/`。

- `ports.json`、`serial_transcript.txt`、`serial_rx.bin`：端口及首轮UART/CAN原始证据。
- `serial_parameters.txt`、`serial_parameter_rx.bin`、`serial_parameter_records.json`：补充寄存器原始证据。
- `decoded_parameter_samples.json`：校验节点/寄存器后按小端解码的单次参数。
- `source_manifest.json`：142个仓库已跟踪/相关未跟踪源文件的长度和SHA-256；生成于本报告写入前。
- `key_source_hashes.json`、`git_status.txt`、`git_head.txt`、`git_remotes.txt`：当前工作树及配置身份。
- `idf_environment.txt`与各项测试日志：本轮软件证据。
- `sdkconfig.product_candidate`、`product_build/`：未烧录的独立候选配置及产物。
- `rs01_manual_relevant_pages.txt`、`rs01_manual_p42/48/49.png`：本地手册相关页文本和视觉核对副本。

构建目录、二进制和临时渲染不应无差别纳入最终源代码提交；Git收口时按证据清单选择文件，不能清理或覆盖未经确认的其他内容。

## 2026-10-09后续：型号及TX4/RX1确认、产品配置更新

操作者在本次对话中先确认主控板为 `ESP32-S3-N16R8`、CAN RX接GPIO1，随后明确确认
“CAN收发器TXD现在仍接ESP32的GPIO4”。这消除了前面板型/接线资料缺失，证据来源为操作者现场确认，
并非已经执行ROM芯片探测或测得Flash容量；当前旧固件的准确版本仍未取得。

乐鑫[ESP32-S3-WROOM-1/1U官方规格表](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.html)
列出N16R8为16 MB Quad-SPI Flash和8 MB Octal-SPI PSRAM。本次按N16R8型号更新Flash容量声明，
继续使用DIO/40 MHz，不额外启用PSRAM或扩展分区；现有静态控制结构继续使用内部RAM。

本次配置修改（保留此前阶段10工作树内容）：

| 文件 | 本次改动 |
| --- | --- |
| sdkconfig | CAN TX=4/RX=1；Flash 2 MB→16 MB；关闭工程激励；PSRAM仍关闭 |
| main/Kconfig.projbuild | ESP32-S3的RX默认值改为1，保留其他芯片的原默认分支；中文说明已保存sdkconfig优先 |
| README.md | 更新当前板型、接线、存储及工程开关，区分操作者确认与历史左右标定 |
| CONSOLE_COMMANDS.md | 更新TX4/RX1，并纠正GPIO实际由sdkconfig/Kconfig读取的说明 |

候选构建仍使用现有工程；独立证据目录为
`output/phase10/20261009T100049_rx1_n16r8_candidate/`，保留四个文件修改前副本。
与上轮142项源码清单比较，只有上述四个已有文件变化，其他138项保持原SHA-256。
本接续文档为另行更新的记录，不属于上轮生成于文档写入前的源码清单。

构建状态：PASS，ESP-IDF v5.5、目标esp32s3、返回码0。生成sdkconfig.h已验证TX4/RX1、16 MB Flash，
且无工程激励/PSRAM使能宏。应用为665104 B（0xa2610），1 MB应用分区剩余37%，
bootloader为21088 B；分区仍为nvs@0x9000、phy_init@0xf000、factory@0x10000/1 MB。
16 MB Flash声明不代表本次扩大应用分区或启用了OTA。

离线esptool分别检查应用和bootloader文件，均返回0；bootloader头声明ESP32-S3、16 MB、DIO、40 MHz，
应用描述为0.10.1 / ESP-IDF v5.5，校验和和validation hash有效。这些是候选文件证据，不是板上运行版本证据。
构建日志未检出compiler warning/error或CMake warning/error；git diff --check返回0。
本次为配置改动，没有重复执行上轮已通过的算法/网页主机测试，也没有据此新增目标板PASS。

本候选应用SHA-256：
`8A26C0B81C5FD03A261154FC871BDD13D94FF87712EBE074A825A001167721A8`。

证据包括build.txt、app_image_info.txt、bootloader_image_info.txt、partition_table.txt、
hardware_confirmation.json、scope_verification.json、verification_summary.json及candidate_artifacts.json。
此前GPIO2/2 MB的候选保留为历史编译核对产物，后续升级应使用本节TX4/RX1、16 MB候选，
并先完成板上芯片/Flash读取及旧镜像备份。

本次没有连接串口、进入ROM、读取板上Flash、烧录、归零、使能或运动；旧固件仍在板上。
前面的TX未知/RX配置不一致已由操作者确认和源码修正解决，但产品0.10.1目标板验证仍为NOT RUN。

后续顺序：取得本轮复位/Flash备份和烧录授权 → 确认芯片/容量并保存旧Flash镜像、解析固件描述 →
烧录TX4/RX1的产品候选 → 先验证IDLE、输出关闭、A/B新鲜主动反馈及产品BLE →
独立确认中立位和软件归零授权 → 真人SHADOW、ACTIVE分级运动授权与验收。
历史中立位、软件零位、使能或运动授权不沿用；阶段10整体完成状态不改变。
