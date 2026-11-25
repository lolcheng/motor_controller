# P2 ROS 2 Jazzy 虚拟电机操作指南

以下运动和配置示例仅用于 vcan 上的虚拟电机。读写接口在一个隔离的网络命名空间内运行。P2 的 SocketCAN 对照内核设备类型，只接受 `vcan`。默认启动保持未授权，实际 CAN 总线没有接入。

## 环境和构建

已安装 Ubuntu 24.04 和 ROS 2 Jazzy 时，确保有 C++17 编译器、CMake、colcon、iproute2、can-utils、rosbag2 和 SQLite3 存储插件。需要补装时由系统管理员执行：

```bash
sudo apt install build-essential cmake python3-colcon-common-extensions iproute2 can-utils \
  ros-jazzy-ros-base ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-default-plugins
```

从工作区根目录构建五个包：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select encos_core encos_msgs encos_tools \
  encos_experiments encos_driver --build-base build/p2 --install-base install/p2 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source install/p2/local_setup.bash
```

## 启动和观察

多个终端可以共享由管理员创建的专用虚拟接口。管理员可手动运行 `sudo modprobe vcan` 以及 `sudo scripts/setup_vcan.sh ensure vcan0`，结束后运行 `sudo scripts/setup_vcan.sh clean vcan0`。接口检查命令为 `scripts/setup_vcan.sh check vcan0`。

需要无 sudo 的隔离实验时，先在一个终端进入用户和网络命名空间，再在该 shell 内开后台作业或附加自己的终端。P2 自动测试用以下环境：

```bash
unshare --user --map-root-user --net bash
ip link set lo up
scripts/setup_vcan.sh ensure vcan0
export ROS_DOMAIN_ID=111 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

所有终端都应先加载同一个 ROS 2 工作区环境，并使用同一个 `ROS_DOMAIN_ID`。终端一启动驱动和虚拟电机：

```bash
ros2 launch encos_driver vcan.launch.py interface:=vcan0 fd:=true frequency:=400 raw:=true
```

需要独立终端运行虚拟电机时，将 launch 参数 `virtual_motor:=false`，并在另一个终端运行：

```bash
ros2 run encos_tools encos_virtual_motor vcan0 fd --bootstrap-beacon
```

这个虚拟选项在收到版本查询时先给一帧仿真初始反馈，再给版本应答。驱动只在运动前初始化时接纳该帧。首选经典 CAN 时将两端改成 `fd:=false`、`classic`。终端二可监控 CAN 和 ROS 反馈：

```bash
candump -t a -x vcan0
ros2 topic echo --once --no-daemon --qos-reliability best_effort /encos/motor_state encos_msgs/msg/MotorState
ros2 topic hz /encos/control_sample
ros2 topic echo --no-daemon --qos-reliability reliable --qos-durability transient_local \
  /encos/fault encos_msgs/msg/MotorFault
```

在不同终端运行长时间观察命令。CAN 抓包的 ID 查询字节应为 `FF FF 00 82`，成功响应应为 `FF FF 01 00 01`。FD+BRS 可从抓包扩展标志和 `/encos/raw_rx` 核对。

## 查询、授权和有限控制

查询和状态快照均为虚拟端点。查询码使用十进制：

```bash
ros2 service call /encos/discover_motor encos_msgs/srv/DiscoverMotor '{}'
ros2 service call /encos/get_profile encos_msgs/srv/GetProfile '{}'
ros2 service call /encos/read_parameter encos_msgs/srv/ReadParameter '{code: 29}'
ros2 service call /encos/read_parameter encos_msgs/srv/ReadParameter '{code: 36, group: 2}'
```

通过 Topic 连续发布的命令示例。每次运行前确认上一个任务已经停止，`get_profile` 返回未授权：

```bash
ros2 run encos_experiments virtual_control.py position 0.10 --owner 7 --seconds 2 --arm-virtual --stop-on-exit
ros2 run encos_experiments virtual_control.py velocity 0.10 --owner 7 --seconds 1 --arm-virtual --stop-on-exit
ros2 run encos_experiments virtual_control.py mixed 0.05 --owner 7 --seconds 1 --arm-virtual --stop-on-exit
```

也可手动授权后发一条带 ROS 生成时间的指令。此示例会在有效期耗尽时锁存，这是过期演示：

```bash
ros2 service call /encos/arm encos_msgs/srv/Arm '{authority: 2, owner: 7, confirm: true}'
ros2 topic pub --once --qos-reliability best_effort /encos/motor_command encos_msgs/msg/MotorCommand \
  '{header: auto, valid_for: {nanosec: 100000000}, motor_id: 1, owner: 7, mode: 1, position_rad: 0.05, speed_limit_rad_s: 0.5, current_limit_a: 1.0, feedback: 2}'
ros2 service call /encos/reset_fault encos_msgs/srv/ResetFault '{confirm: true}'
```

电流、力矩、变阻尼、能耗、再生及电磁抱闸 Topic 模式需在启动时显式选择 `additional_virtual_modes:=true`。`virtual_control.py` 的模式名依次为 `current`、`torque`、`damping`、`dissipative`、`regenerative`、`electromagnetic-brake`。下列短时例子只对虚拟设备使用：

```bash
ros2 run encos_experiments virtual_control.py current 0.03 --owner 8 --seconds 0.2 --arm-virtual --stop-on-exit
```

配置需 `allow_writes:=true`、单项 `confirm: true`，且处于无运动管理状态。以下设置虚拟超时后读回：

```bash
ros2 service call /encos/write_parameter encos_msgs/srv/WriteParameter \
  '{code: 11, value: {kind: 1, values: [500.0, 0.0, 0.0, 0.0]}, confirm: true}'
ros2 service call /encos/read_parameter encos_msgs/srv/ReadParameter '{code: 31}'
```

## Action 和取消

先授权 Action 占用，再从任一终端提交有限任务：

```bash
ros2 service call /encos/arm encos_msgs/srv/Arm '{authority: 3, owner: 7, confirm: true}'
ros2 action send_goal --feedback /encos/run_experiment encos_msgs/action/RunExperiment \
  '{owner: 7, mode: 1, shape: 2, duration_s: 6.0, offset: 0.0, amplitude: 0.04, start_hz: 1.0, speed_limit_rad_s: 0.6, current_limit_a: 1.0, feedback: 1}'
```

Action 的 `shape` 取 0 保持、1 阶跃、2 正弦、3 线性扫频。位置和速度分别用 `mode: 1`、`mode: 2`。看到 `Goal accepted` 和进度反馈后，可在另一个终端使用标准 Action 取消服务停止当前任务：

```bash
ros2 service call /encos/run_experiment/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
```

手动停止使用 `ros2 service call /encos/stop encos_msgs/srv/Stop '{}'`。它撤销软件控制权并停止发送运动帧，虚拟电机随后由自己的看门狗停止。等待速度读回确认后观察 `UNARMED` 或故障锁存状态。正常完成、取消和人工停止都需要重新授权才能再开始。真实硬件急停、固件失能、通信保护、制动及零电流的物理含义各自独立。

## rosbag2 和 CSV

在执行有采样的 Topic 或 Action 前启动记录。新建路径，完成后先 Ctrl+C 正常关闭记录进程，再导出：

```bash
ros2 launch encos_driver record.launch.py output:=logs/p2_demo_bag
ros2 bag info logs/p2_demo_bag
ros2 run encos_experiments export_samples.py logs/p2_demo_bag logs/p2_demo.csv
ros2 run encos_experiments analyze_timing.py logs/p2_demo.csv --hz 400
```

关闭顺序为请求停止、确认状态、停止 bag、Ctrl+C 关闭驱动及虚拟电机、清理 vcan。自动集成测试和长测命令见 [P2 报告](reports/P2.md)。

## ROS 接口与 QoS

| Topic | 类型 / QoS | 用途 |
|---|---|---|
| `/encos/motor_command` | MotorCommand，Best Effort，Keep Last 1 | 已授权的单发布者目标。源 ROS 时间戳、有效期、ID 和 owner 必填。 |
| `/encos/motor_state` | MotorState，Best Effort，Keep Last 8 | 反馈测量、错误、抱闸、有效性、时间和软件安全状态。 |
| `/encos/control_sample` | ControlSample，Reliable，Keep Last 2048 | 已尝试的发送、反馈、匹配和实测时刻。 |
| `/encos/communication_stats` | CommunicationStats，Reliable，Keep Last 4 | 滚动延迟分位、区间速率、累计异常及溢出。 |
| `/encos/fault` | MotorFault，Reliable，Transient Local 1 | 状态转换和故障锁存。 |
| `/joint_states` | sensor_msgs/JointState，Best Effort，Keep Last 8 | 有效位置和速度，缺失的数组留空，effort 留空。 |
| `/encos/raw_tx`、`/encos/raw_rx` | RawFrame，Best Effort，Keep Last 128 | 可选的只读调试发布，含 FD/BRS 及时间戳。 |

所有 Service 位于 `/encos/`。Arm 授权 1 只读、2 Topic、3 Action。Topic 绑定首次有效命令的 DDS 发布者 GID 和调用方提供的 owner。它们用于并发控制权约束。Stop 和 ResetFault 分别处理停止和确认复位。DiscoverMotor、ReadParameter 和 GetProfile 提供发现、类型化查询和状态快照。WriteParameter、SetZero、SetMotorId、ResetMotorId、SetBrake 为受保护配置操作。写入成功只在相应回显和可用读回都一致时返回。GetProfile 始终读快照。其它非停止管理请求在运动授权期间拒绝。Action 名为 `/encos/run_experiment`。

启动参数包括 `can_interface`、`fd_brs`、`motor_id`、`control_hz`、`publish_ms`、`safety_feedback_ms`、`motor_model`、`joint_name`、`publish_raw`、`allow_virtual_writes`、各 `allow_virtual_*` 及 `limits.*`。YAML 可叠加传给 `--params-file`。安全参数在启动时读取。型号额定值、从虚拟设备读回的量化范围和软件实验限值各有来源。

位置模式使用 `position_rad`、`speed_limit_rad_s` 和 `current_limit_a`。速度模式使用 `velocity_rad_s` 和 `current_limit_a`。混控使用位置、速度、KP、KD 和 `feedforward_nm`。电流、力矩分别使用 `current_a`、`torque_nm`。制动阈值使用 `current_limit_a`。其它模式未定义的字段不参与编码。所有数值字段必须有限。反馈选择为 0–3，混控固定 1。安全所需字段缺失时，驱动在配置的最长间隔内请求一次类型 1，并在样本标记实际 ACK。抱闸走专用应答。

## 数据与时间

Topic 指令保留上层的 ROS 生成时间和驱动的 ROS 接收时间。驱动从接收时各自的 ROS/单调钟估计单调生成时刻，独立保存接收时刻。Action 样本的生成时间直接来自 C++ 单调钟。有效期用单调时钟计，固定周期采样最近仍有效且有权的目标，复用目标时原始生成和接收时刻保持一致。

CSV 的 `scheduled_mono_ns` 与 `cycle_started_mono_ns` 分别表示计划和运行周期。 `tx_call_before_mono_ns`、`tx_call_after_mono_ns` 是用户态调用界限。 `rx_kernel_realtime_ns` 是内核接收软件时间，`rx_mono_ns` 是 P1 换算的单调时间，另记用户态读取和解析。 `ros_publish_mono_ns` 在 Executor 开始发布相应反馈前采集，`ros_publish_ns` 使用 ROS 时钟。记录实际周期和字段各自的有效性及来源。缺失项在消息中带 NaN 和无效标志，在 CSV 中留空。估算力矩来自相电流和已核验的 Kt。电机内部采样、线路时刻及内核 TX 时刻均未获得。
