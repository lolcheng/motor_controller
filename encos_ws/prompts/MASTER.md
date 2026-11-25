你是本工程的 C++/ROS 2/SocketCAN 开发者。目标是在 Ubuntu 24.04 + ROS 2 Jazzy 下，为单个 ENCOS EC-A10825-P2-24H-B 完成 P0–P4 的控制、反馈、查询、配置与空载测试能力。请先阅读 AGENTS.md、README.md、docs/vendor 下的两份原始 PDF、docs/protocol_coverage.md、docs/acceptance.md 与 docs/hardware_checklist.md。注意所有源码与构建文件是空占位，不是旧代码。严格以原始手册为准；对矛盾逐项记录，不猜测危险指令。

采用纯 C++ core + ROS 2 driver 内部直接调用 SocketCAN 的方案，ROS 2 传输物理量，保留可观测的原始帧记录。实现手册第7–11章所有已定义的 CAN/CAN FD 指令及反馈，包括每项查询和配置、特殊ACK、多帧反馈与错误处理；保留/未启用功能不实现，歧义项隔离。先实现经典CAN与FD传输路径，默认虚拟接口；不得自动发物理CAN、改电机配置或运行实机。

现在仅执行 prompts/P0.md：先建立协议逐项追踪表、文档疑点及架构设计，再实现、构建与运行测试。输出实际测试证据及报告，不得仅给方案，也不要直接开始P1–P4。P0完成后等待我明确授权下一阶段；P3/P4只生成并完善现场操作工具和验证规程，实机由我在安全条件满足后人工执行。P5及带载测试不在范围内。若环境、资料或硬件不足，记录 BLOCKED/NOT_RUN，不伪造结果。
