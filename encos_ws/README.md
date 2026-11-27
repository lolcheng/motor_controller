# ENCOS Gen 2 单电机通信

目标设备为 EC-A10825-P2-24H-B，环境为 Ubuntu 24.04、C++17 和 ROS 2 Jazzy。
实际收发走 USB‑CAN 串口。独立 C++ 命令行可持续接收并解码 CAN 反馈，
也可手动发送位置、速度、参数查询等已定义的指令。
ROS 2 节点按需发布原始帧和电机状态。

## 快速开始

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base build/colcon-build-log build --base-paths src \
  --build-base build/usb --install-base install/usb \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source install/usb/local_setup.bash

ros2 run encos_tools encos_usb_can --help
```

不带子命令启动持续会话。

```bash
ros2 run encos_tools encos_usb_can /dev/ttyUSB0
```

启动后立即读串口，空闲时照常打印 `RX`。输入 `DISCOVER`、`QUERY 1 VERSION`
或 `POS 1 30` 并回车，每行发一帧。输入 `EXIT` 退出。只需单次发送时，
也可把命令直接接在设备路径后，如 `encos_usb_can /dev/ttyUSB0 POS 1 30`。
单次发送结束后串口关闭，需要持续收反馈时使用会话。

`POS 1 30` 对应 `001#283E00000032002A`，默认 20 rpm、1 A、应答类型 2。
连接电机时，操作员确认单条命令会立即送到总线。真机接线、上电和运动验收按
AGENTS.md 执行。手册未给出独立的 `ENABLE` CAN 帧。

使用独立 CMake 构建时可直接运行 CLI，无需 ROS 2。串口设置为 115200、8N1、
无流控。分析仪发送 ID 1、DLC 8 的全零测试帧时，终端会显示原始帧。
只有可解码或经查询匹配的 ENCOS 反馈才会提供电机状态。

需要 ROS 2 时，关闭持续会话，再用下面的节点占用串口：

```bash
ros2 run encos_driver encos_usb_can_node --ros-args -p device:=/dev/ttyUSB0
```

节点默认只监听，发布 `/encos/raw_rx` 和 `/encos/motor_state`。
给节点设置 `allow_queries:=true` 可开放 ID 和参数查询服务。CLI 也保留
带应答匹配和超时的单次只读查询用法。

```bash
ros2 run encos_tools encos_usb_can /dev/ttyUSB0 QUERY 1 VERSION --allow-query
```

## 包与验证

- **encos_core** 提供 ENCOS 编解码、串口包封装、事务匹配和离线安全单测。
- **encos_tools** 提供独立 USB‑CAN 命令行。
- **encos_msgs** 定义原始帧、反馈、查询及实验接口。
- **encos_driver** 提供 USB‑CAN 只读 ROS 节点和录包 launch。
- **encos_experiments** 提供轨迹库、bag 导出和通信时序分析。

Action 消息和轨迹库保留供后续控制使用。当前节点提供监听与只读服务；
录包和接收频率分析可以直接用于实际分析仪的数据。用
`bash scripts/run_offline_tests.sh` 运行不依赖 ROS 的 C++ 协议和串口伪终端测试。
Jazzy 回归也使用伪终端，不需要真机或分析仪。分析仪已验证实物模块的双向
标准帧转发，真实电机回复、实际运动和 CAN FD+BRS 仍待现场验收。

完整命令对照和分析仪收发 demo 见教程第 2.5、2.6 节。

[操作教程](TUTORIAL.txt) · [目录树](STRUCTURE.txt) · [协议覆盖](docs/protocol_coverage.md)
