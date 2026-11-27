# ENCOS V1.19EAP CAN 协议覆盖

所有页码均为《ENCOS 调试技术手册 V1.19EAP》的印刷页码。第 7–11 章印刷 p.23–44 对应 PDF 第 28–49 页。[调试手册 PDF](<vendor/ENCOS 调试技术手册 V1.19EAP.pdf>)

编码按高位在前、低位在后（p.25、39），有符号整数按二补码，float32 按 IEEE 754 位表示。表中 uN 和 iN 分别表示 N 位无符号和有符号数。“×”表示编码倍率，解码除以该倍率。命令表各字段按报文中的顺序列出。除管理报文外，指令和反馈均使用 Motor ID 0x001–0x7FE。

测试名对应 encos_core 的 CTest 套件。PASS 指纯 C++ 离线或 MockTransport 测试。真实电机的问答、写入和运动仍为 NOT_RUN。

## 帧格式及管理

| 节和页 | TX | RX | P0 / 测试 |
|---|---|---|---|
| §7.1 p.23 | 标准数据帧。经典 CAN 1 Mbps、采样点 76.5%。FD+BRS 仲裁域同前，数据域 5 Mbps、采样点 88.2%。按逐项报文定义取 2–8 字节。 | 严查 ID、经典/FD、BRS、长度、扩展帧、RTR、错误帧。 | PASS，command_vectors、invalid_frames、transport。当前 USB 串口封装和只读收发另由 serial_can 测试。 |
| §7.2 p.23 | 0x7FF，4/6 B。示例 `00 01 00 03`。可在末尾追加 i16 偏置（°×100）。 | 0x7FF，4 B，`ID_H ID_L 01 03` 成功，`ID_H ID_L 01 00` 失败。 | PASS，command_vectors、feedback、timeouts。 |
| §7.3 p.24 | 0x7FF，6 B。示例 `00 01 00 04 00 02`。 | 0x7FF，4 B，`NEW_H NEW_L 01 04` 成功，`OLD_H OLD_L 01 00` 失败。 | PASS，command_vectors、feedback、timeouts。 |
| §7.4 p.24 | 0x7FF，6 B，`7F 7F 00 05 7F 7F`。 | 0x7FF，6 B，`7F 7F 01 05 7F 7F` 成功，失败无应答，ID 重置为 1。 | PASS，command_vectors、feedback、timeouts。 |
| §8.1 p.25 | 0x7FF，4 B，`FF FF 00 82`。 | 0x7FF 成功 `FF FF 01 ID_H ID_L`（5 B），失败 `80 80 01 80`（4 B）。 | PASS，command_vectors、feedback、timeouts。 |

p.4–5 要求通信模式、零点和 ID 设置后间隔至少 500 ms 才能发送下一帧，事务管理器也对 ID 重置施加这个间隔。上电等待和实际接口启用属于现场阶段。管理写入和配置序列化默认禁止，须显式选择离线权限。真实写入、模式切换和抱闸动作须由操作员单项确认。

## 控制

| 节和页 | 按位布局及倍率 | 示例原始字节 | 反馈 / P0 |
|---|---|---|---|
| §9.1.1 p.26 | 8 B。模式 0 u3、KP u12、KD u9、位置 rad u16、速度 rad/s u12、前馈 Nm u12。线性映射到运行时已核验的区间。按 p.26 示例对非负量化值向下取整。 | `01 47 FF 7F FF 7F F7 FF`。按页中示例范围可得原码 KP 163、KD 511、位置 32767、速度及扭矩 2047。 | 固定类型 1。command_vectors PASS，未提供本型号范围时拒绝编码。 |
| §9.1.2 p.27 | 8 B。模式 1 u3、位置 ° float32、速度阈值 rpm×10 u15、相电流阈值 A×10 u12、ACK u2。 | `20 00 00 00 00 7D 01 91`。p.43 ±90° 示例首三字节为 `28 56 80` / `38 56 80`。 | ACK 0 无回复，1–3 对应反馈类型 1–3。command_vectors、invalid_frames PASS。 |
| §9.1.3 pp.27–28 | 7 B。模式 2 u3、零 u3、ACK u2、速度 rpm float32、相电流阈值 A×10 u16。 | `41 42 48 00 00 00 64`，另测 p.44 三条。 | ACK 同上。command_vectors、invalid_frames PASS。 |
| §9.1.4 pp.28–29 子模式 0 | 3 B。模式 3 u3、子模式 u3、ACK u2、相电流目标 i16（A×100）。 | `61 01 F4`，另测 p.44 的 3 A 和 ±5 A。 | ACK 同上。command_vectors PASS。 |
| 同上，子模式 1 | 力矩目标 i16（Nm×100）。 | `65 01 F4`，另测 p.44 的 3 Nm 和 ±5 Nm。 | ACK 同上。command_vectors、invalid_frames PASS。 |
| 同上，子模式 2 | 驱动忽略末尾字段，编码器只允许零。 | `69 00 00` | ACK 同上。command_vectors、invalid_frames PASS。 |
| 同上，子模式 3 | 能耗制动，阈值 i16（A×100）。 | `6D 01 F4` | ACK 同上。command_vectors、safety PASS。离线编码接受非负阈值。 |
| 同上，子模式 4 | 再生制动，阈值 i16（A×100）。 | `71 01 F4` | ACK 同上。command_vectors、safety PASS。离线编码接受非负阈值。 |
| 同上，子模式 5–7 | 子模式 5 字段 i16 取 0/1，6/7 保留。 | `75 00 01` | 5 忽略 ACK，事务等待类型 6。6/7 拒绝。command_vectors、invalid_frames、timeouts PASS。 |
| §9.1.5 p.29 | 2 B。模式 4 u3、保留 u5 写零、抱闸状态 u8（0 吸合，1 打开）。 | `80 01` | `B2 yy`，独立于类型 6。feedback、timeouts、command_vectors PASS。 |

位置和速度的公共单位为 rad 和 rad/s。定点编码采用最近整数，半步远离零，超出表示范围直接报错。命令整数范围、型号标称值、实际安全限值各自核验。

## 配置

TX 首字节通常为 C0，其后 u8 配置码。表中以完整 TX 示例给出差异。除特别注明，成功回显格式为 `FF FE code values`，值的长度和请求一致。§10.4 的 `(80|error) code status` 为独立 3 B 一般应答。

| 节和页 | 十六进制代码 | TX / 参数 | 特殊 RX / P0 |
|---|---:|---|---|
| §9.2.1 pp.29–30 | 01 | 4 B。模式 6、保留位 0、ACK=1，首字节 C1。加速度 u16，rad/s²×100，0–2000。20 rad/s² 对应 `C1 01 07 D0`。示例 `C0 01 07 D0` 的 ACK=0 行为待确认。 | 5 B，`FF FE 01 07 D0` 对应 20 rad/s²。parameters、feedback、invalid_frames、encos_usb_can_cli PASS。 |
| §9.2.2 p.30 | 02 | 3 B，`C1 02 01`。u8 0 经典 CAN，1 FD，2 CANopen 未启用。 | 表列 4 B、标题写 5 B，均保留原始帧，不匹配事务。TX 离线 PASS，特殊 RX BLOCKED，parameters、feedback、timeouts。 |
| §9.2.3 p.31 | 04 | 4 B，`C0 04 00 FA`，u16 Kt（Nm/A×100）。 | 5 B，PASS。 |
| §9.2.4 p.31 | 05 | 6 B，`C0 05 00 00 01 F4`，KP min/max u16×1。 | 7 B，PASS。 |
| §9.2.5 p.32 | 06 | 6 B，`C0 06 00 00 00 32`，KD min/max u16×1。 | 7 B，PASS。 |
| §9.2.6 p.32 | 07 | 6 B。位置 min/max i16（rad×100）。−12.5 rad 的正确负数是 `FB 1E`，手册帧 `C0 07 FA 1E 04 E2` 会给出 −15.06 rad。 | 7 B，PASS，按字段定义编码。 |
| §9.2.7 p.33 | 08 | 6 B，`C0 08 F8 F8 07 08`，速度 min/max i16（rad/s×100）。 | 7 B，PASS。按 TX 及查询 26 的比例解释 RX 最小值，待对照固件。 |
| §9.2.8 p.33 | 09 | 6 B，`C0 09 FC 7C 03 84`，力矩 min/max i16（Nm×10）。 | 7 B，PASS。 |
| §9.2.9 p.34 | 0A | 6 B，`C0 0A FC 7C 03 84`，相电流 min/max i16（A×10）。 | 7 B，PASS。 |
| §9.2.10 p.34 | 0B | 4 B，`C0 0B 01 F4`，u16 ms。零关闭保护，实机验收不执行禁用。 | 5 B，PASS。 |
| §9.2.11 p.35 | 0C | 6 B，`C0 0C 00 64 01 F4`，KP u16×10000、KI u16×10。 | 7 B，PASS。 |
| §9.2.12 p.35 | 0D | 6 B，`C0 0D 02 58 27 10`，KP 与 KI u16×100000。 | 7 B，PASS。 |
| §9.2.13 p.36 | 0E | 6 B，`C0 0E 02 58 00 04`，KP 与 KD u16×100000。 | 7 B，PASS。 |
| §9.2.14 p.36 | 0F | 3 B，`C0 0F 02`，u8 模式 0–2，保留本报文原始枚举。 | 4 B，`FF FE 0F 02`。PASS。 |
| §9.2.15 p.37 | 10 | 8 B。首字节 C0 或上组号 1–10，Byte1 为 10，后续四个 u12（Nm/A×100）。例 `C1 10 0F A1 04 10 E1 18`。 | 8 B，`EE header Kt1..4`。系数与编组 PASS，回复模式 BLOCKED，header 原样保留。 |
| §9.2.16 p.37 | 11 | 4 B，`C0 11 03 E8`，i16 偏置 °×100。表头 3 B 与布局及示例冲突。 | 5 B，`FF FE 11 03 E8`。PASS，按 4 B 串行化。 |

所有配置项的通用 ACK 由 feedback 套件逐项测试。parameters 套件比对可编码的配置，feedback 套件比对特殊回复，invalid_frames 和 timeouts 验证格式、非法分支、回显不一致及事务隔离。0x03、其它未定义代码和 CANopen 拒绝。

## 查询与类型 5

§9.3 p.38 的普通 TX 为 2 B，首字节按示例用 E1，Byte1 为下表的十进制查询码对应的一字节值。查询 36 首字节为 E0 或上组号（1–10）。普通 RX 首字节为 `A0|error`，Byte1 为查询码。以下各行均由 parameters 和 feedback 覆盖，UUID、Kt 还由 timeouts 覆盖。保留码、非法枚举和长短帧以 invalid_frames 验证。

| 查询码（十进制） | RX 总字节 | 值与解码 | 源页 |
|---:|---:|---|---:|
| 1 | 6 | float32 ° → rad | 41 |
| 2 | 6 | float32 rpm → rad/s | 41 |
| 3 | 6 | float32 相电流 A | 41 |
| 4 | 6 | float32 功率，手册未标输入或输出 | 41 |
| 5 | 4 | u16 原值，查询表未注明倍率 | 41 |
| 22 | 4 | u16 /100，Kt Nm/A | 41 |
| 23 | 6 | 两个 u16 /1，KP min/max | 41 |
| 24 | 6 | 两个 u16 /1，KD min/max | 41 |
| 25 | 6 | 两个 i16 /100，位置 min/max rad | 41 |
| 26 | 6 | 两个 i16 /100，速度 min/max rad/s | 41 |
| 27 | 6 | 两个 i16 /10，扭矩 min/max Nm | 41 |
| 28 | 6 | 两个 i16 /10，电流 min/max A | 41 |
| 29 | 每帧 7 | u8 序号 1–3 加 4 B，按序号拼高、中、低，共 12 B | 41 |
| 30 | 8 | 硬件版本 3 B、软件版本 3 B，原值 | 41 |
| 31 | 4 | u16 ms，0 表示禁用 | 41 |
| 32 | 6 | 两个 u16 分别 /10000、/10，电流 KP/KI | 41–42 |
| 33 | 6 | 两个 u16 /100000，速度 KP/KI | 42 |
| 34 | 6 | 两个 u16 /100000，位置 KP/KD | 42 |
| 35 | 3 | u8 原值 1 关闭、2 单表、3 双表，单独于配置 0F 的编码 | 42 |
| 36 | 8 | 以 EE 开头，按 §9.2.15 特殊回复解析 | 37、42 |
| 37 | 3 | u8 抱闸状态 0/1 | 42 |
| 38 | 4 | i16 /100° → rad | 37、42 |
| 39 | 7 | u8 状态 0/1、i16 原始角、i16 校准角。保留两字段原值，厂商确认单位为度后才按 /100° → rad 换算 | 4、42 |

## 其它反馈

| 分支 | 长度和布局 | P0 / 测试 |
|---|---|---|
| §10.1 p.39，类型 1 | 8 B，类型 u3、错误 u5、位置 u16、速度 u12、相电流 u12、电机温度 u8、MOS 温度 u8。温度等于 (raw−50)/2 °C。位置、速度与相电流仅在对应范围经核验后换算。 | PASS，feedback、invalid_frames、safety。 |
| §10.2 p.40，类型 2 | 8 B，类型 u3、错误 u5、位置 float32（°）、相电流 i16 /100 A、电机温度 u8。速度及 MOS 温度缺失。 | PASS，feedback、invalid_frames、safety。 |
| §10.3 p.40，类型 3 | 8 B，类型 u3、错误 u5、速度 float32（rpm）、相电流 i16 /100 A、电机温度 u8。位置及 MOS 温度缺失。 | PASS，feedback、invalid_frames、safety。 |
| §10.4 p.40，类型 4 | 3 B，类型 u3、错误 u5、配置码 u8、结果 u8（0 失败、1 成功）。 | PASS，feedback、invalid_frames、timeouts。 |
| §10.6 p.42，类型 6 | 2 B，类型 u3、错误 u5、状态 u8（0 吸合，1 打开）。 | PASS，feedback、invalid_frames、timeouts。 |
| §10 p.39，错误码 | 0 无错、1 过热、2 过流、3 过压、4 欠压、5 编码器、6 制动过压、7 驱动故障。其余编码保留原帧。 | PASS，feedback、invalid_frames、safety。 |

§11 pp.43–44 中 3 条位置、3 条速度、3 条电流及 3 条力矩向量由 command_vectors 直接比对。p.43 混控示例的二进制文本两次给出的第七字节不同，且缺少正确的物理量缩放，只用于疑点分析。

设备额定与供电条件、编组位宽、应答冲突和实验前资料要求见 [待确认事项](issues.md)。

## 当前 USB‑CAN 路径

| 功能 | 当前范围和证据 |
|---|---|
| §8.1 ID 发现 | 放行固定只读查询。通过分析仪验证了实际发送；伪终端对照验证封装、回显过滤和匹配回复。 |
| §9.3 查询 | 只读入口放行十进制 1–5 及 22–39。查询 36 使用组号。UUID 与特殊应答沿用核心事务实现。实际电机回复待验证。 |
| §10 反馈 | 原始帧保留。ROS 发布可解码的电机状态及匹配成功的位置、速度、相电流查询结果。未核验的范围、Kt 或缺失量不推算。 |
| §7–9 命名指令 | `DISCOVER`、`QUERY`、`POS`、`MIXED` 等由 Protocol 编码，设备路径后附加命令或在持续会话内输入。`encos_usb_can_cli` 从伪终端另一端读取经典 CAN 封装并对照手册向量。 |
| §7、§9.1、§9.2 管理写入、运动和参数配置 | 终端每次只发操作员输入的一条指令，实机运动和配置的现场核验见 AGENTS.md。`--allow-query` 只读事务入口和 ROS 节点维持只读白名单。 |
| §10 持续收帧 | 持续会话启动后立即读串口。伪终端测试在命令前、发送后、空闲期均收到可解码反馈。本机近期发送数据的完整匹配标记为疑似回显。 |
| USB 包封装 | 标准经典 CAN，帧头 AA、两字节大端 ID、包总长度、CAN 数据、低八位累加校验。分析仪已验证双向转发，FD/BRS 标志和硬件时间戳待确认。 |

持续会话、命令对照、台架 demo 和验证命令见 [教程](../TUTORIAL.txt) 第 1、2.2、2.5、2.6、5 节。
独立 CMake 10 项、Jazzy 13 项均 PASS，构建和测试输出位于 `build/usb-terminal/`。
安装版持续会话在伪终端以 10 Hz 接收 10 帧，期间串口没有发送，再写入一次
`POS 1 30` 后继续收到类型 2 反馈，见同目录的 `installed-pty.txt`。
§9.1.2 印刷 p.27 的位置帧
包含模式 u3、位置 float32、转速 u15、电流 u12、ACK u2。按 30°、20 rpm、
1 A、ACK 2 代入，独立测试从伪终端读取的 CAN 数据为 `28 3E 00 00 00 32 00 2A`，
封装为 `AA 00 01 0D 28 3E 00 00 00 32 00 2A 7A`。
持续会话的真实分析仪收发与真实电机运动均为 NOT_RUN。

本轮复核两份 PDF 原图，包括调试手册印刷 pp.23–44 与数据手册 p.30。
修改 `protocol_commands.cpp` 的加速度编码、`cli_commands.cpp` 的帮助说明、
`usb_can.cpp` 的反馈输出，以及 `test_parameters.cpp` 和 `test_usb_cli.py`。
核心编码和 CLI 包覆核验使用手册向量或按独立位域推导的向量。终端测试覆盖类型
1–6、管理回复、B2、FF FE、EE、UUID 分片和含歧义的回显。本轮编译、测试和
demo 输出在 `build/protocol-review/`，独立 CMake 10 项、Jazzy 13 项均 PASS。
安装版按教程运行的四组收发记录见 `build/protocol-review/demo-pty.txt`。
硬件端通信和运动仍为 NOT_RUN，CAN FD+BRS 的 USB 封装仍待确认。
