# ENCOS Gen 2 单电机驱动

目标为 Ubuntu 24.04、ROS 2 Jazzy、C++17、Linux SocketCAN，型号 EC-A10825-P2-24H-B。P0 提供独立 C++ 核心，P1 提供仅限 vcan 的 SocketCAN、虚拟电机、调试工具和性能测量。P2 增加 ROS 2 控制、配置、Action 和数据记录。阶段结果见 [P0 报告](docs/reports/P0.md)、[P1 报告](docs/reports/P1.md) 和 [P2 报告](docs/reports/P2.md)。

## ROS 2 Jazzy 使用

ROS 2 的依赖、构建、多终端命令和录包导出，见 [P2 操作指南](docs/ros2_p2.md)。以下保留 P0/P1 独立工具的运行方式。

## 构建和测试

```bash
bash scripts/run_offline_tests.sh

source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/encos_core src/encos_tools \
  --packages-select encos_core encos_tools --build-base build/p1-colcon \
  --install-base install/p1-colcon --cmake-args -DBUILD_TESTING=ON
```

### 用独立网络命名空间运行 P1 测试

```bash
unshare --user --map-root-user --net bash -ec '
  scripts/setup_vcan.sh ensure vcan0
  trap "scripts/setup_vcan.sh clean vcan0" EXIT
  export ENCOS_TEST_VCAN=vcan0 ENCOS_TEST_ADMIN=1
  colcon test --base-paths src/encos_core src/encos_tools \
    --packages-select encos_core encos_tools --build-base build/p1-colcon \
    --install-base install/p1-colcon
  colcon test-result --test-result-base build/p1-colcon --verbose
  scripts/test_vcan_processes.sh install/p1-colcon/encos_tools/lib/encos_tools logs/p1/processes-local vcan0
'
```

`setup_vcan.sh` 支持 `check`、`ensure` 和 `clean`，默认动作是检查 `vcan0`。脚本检查 Linux 设备类型和 MTU 72，重复初始化或清理同一虚拟接口可直接执行。如果系统禁止无特权用户命名空间，或者 vcan 模块未加载且不能自动加载，请由管理员提供专用 vcan 接口。普通 CTest 在没有默认 vcan 时会将集成项标记跳过。设置 `ENCOS_TEST_VCAN` 后，指定接口不可用会直接报错。`ENCOS_TEST_ADMIN` 仅供有权限的私有测试命名空间注入接口关闭。

### 多终端演示

先由操作者在共享的网络命名空间准备只供本次测试的 `vcan0`。具有管理员授权时可人工运行：

```bash
sudo modprobe vcan
sudo scripts/setup_vcan.sh ensure vcan0
scripts/setup_vcan.sh check vcan0
```

三个终端均从工作区根目录运行。先启动终端一的虚拟电机和终端三的抓包，看到虚拟电机输出 `ready` 后，再在终端二发指令。第一个终端启动虚拟电机：

```bash
install/p1-colcon/encos_tools/lib/encos_tools/encos_virtual_motor vcan0 fd --idle-beacon
```

第二个终端发出 ID、UUID 查询及一次位置指令：

```bash
tools_dir=install/p1-colcon/encos_tools/lib/encos_tools
"$tools_dir/encos_cli" vcan0 fd discover
"$tools_dir/encos_cli" vcan0 fd query 29
"$tools_dir/encos_cli" vcan0 fd position 0.10 --arm-virtual
```

第三个终端用系统 `can-utils` 的 `candump` 观察：

```bash
candump -t a -x vcan0
```

也可使用自带观察器：

```bash
install/p1-colcon/encos_tools/lib/encos_tools/encos_bus_monitor vcan0 fd
```

经典 CAN 演示将前两个终端的 `fd` 改成 `classic`，观察器可以继续用 `fd` 同时识别两种格式。ID 查询应看到 `7FF#FFFF0082` 及 `7FF#FFFF010001`。FD+BRS 的标志在扩展输出中可见。参数查询码用十进制，查询 Kt 组的示例为 `query 36 2`。`--idle-beacon` 是提供初始反馈的模拟选项，默认虚拟设备只做问答。退出工具后，由创建接口的操作者运行 `sudo scripts/setup_vcan.sh clean vcan0`。

### 故障注入和性能

在虚拟电机命令后追加 `--fault NAME`，可重复指定。选项包括 `silent`、`drop:3`、`delay:45`（毫秒）、`wrong-id`、`length`、`type`、`duplicate`、`uuid-missing`、`uuid-reverse`、`motor:2`、`hot` 和 `reject-writes`。测试用的退出及自动清理见 `scripts/test_vcan_processes.sh`。

在独立 vcan 总线上运行基准，不另启动虚拟电机。脚本为每个频率启动一个无异常注入的独立进程：

```bash
unshare --user --map-root-user --net bash -ec '
  scripts/setup_vcan.sh ensure vcan0
  trap "scripts/setup_vcan.sh clean vcan0" EXIT
  scripts/run_vcan_benchmark.sh install/p1-colcon/encos_tools/lib/encos_tools logs/p1/performance-local vcan0
'
```

输出包含逐周期 CSV、各频率摘要和虚拟电机进程日志。高频用例要求专用、无重复反馈的虚拟端点，失败后停发。vcan 不模拟经典 CAN/FD 仲裁、线路速率或电机控制器时序。

## 工程边界

- `encos_core` 是纯 C++ 协议、传输、事务与安全基础库。
- `encos_tools` 运行虚拟电机、调试 CLI、监视和性能测量。P1 的 SocketCAN 只绑定经内核核验为 `vcan` 的接口。
- ROS 消息和节点已提供，实机空载实验遵循 [验收门槛](docs/acceptance.md)，需要单独批准。调试手册印刷 p.4 和产品数据手册 p.30 的供电数值冲突，现场上电前须以厂商对具体型号和固件的书面确认作为依据。

两份官方原稿、逐项协议和遗留疑点见 [厂商资料](docs/vendor/README.md)、[协议覆盖](docs/protocol_coverage.md) 和 [待确认事项](docs/open_issues.md)。
