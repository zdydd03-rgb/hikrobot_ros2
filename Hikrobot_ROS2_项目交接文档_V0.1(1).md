# Hikrobot ROS 2 项目交接文档 V0.1

**用途：** 用于在新的 ChatGPT 对话中无缝继续 Hikrobot 机械臂 ROS 2 驱动的真机测试与后续开发。  
**状态日期：** 2026-08-26  
**工作空间：** `/home/ai/hikrobot_ros2`  
**目标平台：** Ubuntu 22.04 + ROS 2 Humble + C++17

---

## 0. 新对话开场建议

在新对话中可以直接粘贴下面这段作为上下文：

> 我正在继续开发 `/home/ai/hikrobot_ros2`，这是一个基于海康机器人 MPSDK 的 ROS 2 Humble 驱动。当前 6 个 ROS 2 package 已在 Ubuntu 22.04 上 `colcon build` 全部通过，Driver 节点、Lifecycle、Topic、Service、MoveJ/MoveL/MoveC/MoveS Action 均已在无真机环境下成功注册。底层 SDK 实机测试此前已经验证了连接、HAR 状态订阅、ControlAccess、PROGRAMMER、AUTO、MotorOn/Off、Legacy Realtime Motion、MoveJ 以及 DO。当前尚未进行新的 ROS 2 Driver 真机 HIL 测试。请从 `Lifecycle configure` 真机连接测试开始继续，并严格按本交接文档的顺序推进。

---

# 1. 项目目标与开发原则

本项目的目标是把海康机械臂 MPSDK 封装为 ROS 2 Humble 驱动，后续可继续扩展到 MoveIt 2 / FollowJointTrajectory。

当前第一版设计原则：

- ROS 2：Humble。
- Ubuntu：22.04。
- C++：C++17。
- 控制器实机基线：V2.1.2 控制器。
- SDK：`MPSDK_PUBLIC_FULL_V2.4.0_Build20251106_SVN1686294`。
- 动态库：`libMPSDK_PUBLIC_COMPAT.so`。
- 运动接口：Legacy `MP_Motion_API.h`，不使用 Motion API2 作为当前运动基线。
- Driver 使用 `rclcpp_lifecycle::LifecycleNode`。
- Driver 激活时自动执行 ControlAccess → PROGRAMMER → AUTO → MotorOn → Realtime Motion。
- MoveJ / MoveL / MoveC / MoveS 第一版全部提供 ROS 2 Action。
- 运动速度第一版统一使用 SDK 已明确支持的 `speed_percent`（1–100）。
- 所有运动共享统一 `PathID / nPathID` 分配器。
- `0x82068043` 在 Add 阶段按“换新 PathID + 有限重试”恢复。
- 当前不直接实现 `ros2_control::SystemInterface`；先验证控制器侧 Legacy Motion queue 模型。

---

# 2. 工作空间目录

当前工作空间：

```text
/home/ai/hikrobot_ros2
├── docs
│   ├── ARCHITECTURE.md
│   ├── HIL_TEST_PLAN.md
│   └── ROS_API.md
├── IMPLEMENTATION_STATUS.md
├── README.md
├── scripts
│   ├── build_humble.sh
│   └── core_compile_check.sh
├── src
│   ├── hikrobot_bringup
│   ├── hikrobot_description
│   ├── hikrobot_driver
│   ├── hikrobot_mpsdk_core
│   ├── hikrobot_mpsdk_vendor
│   └── hikrobot_msgs
└── tests
    └── test_sequence_manager.cpp
```

六个 ROS 2 package：

1. `hikrobot_mpsdk_vendor`：MPSDK 的 CMake/运行时环境适配。
2. `hikrobot_mpsdk_core`：不依赖 ROS 的 C++ SDK 封装及运动执行核心。
3. `hikrobot_msgs`：Action / Msg / Srv。
4. `hikrobot_driver`：Lifecycle Driver Node。
5. `hikrobot_bringup`：YAML 与 launch。
6. `hikrobot_description`：描述包骨架；当前尚未加入真实 URDF/mesh。

---

# 3. 已知 SDK / 控制器事实基线

## 3.1 当前 SDK

```text
/home/ai/SDK/MPSDK_PUBLIC_FULL_V2.4.0_Build20251106_SVN1686294
```

关键目录：

```text
Include/Public
Libs/Linux/LX64/64
```

关键库：

```text
libMPSDK_PUBLIC_COMPAT.so
libMPSDK_ADAPTER.so
libMPSDK_LOG.so
libMPSDKV212.so
libMPSDKV220.so
```

此前已经执行：

```bash
ldd "$HIKROBOT_MPSDK_ROOT/Libs/Linux/LX64/64/libMPSDK_PUBLIC_COMPAT.so" | grep "not found"
```

结果无输出，因此当前 MPSDK 主要动态依赖没有 `not found`。

## 3.2 实机控制器

SDK 日志此前识别实机控制器版本为：

```text
V2.1.2_1444184_20241223
```

因此现阶段驱动运动链以 V2.1.2 + compatibility layer 的 Legacy Motion 实测行为为准。

---

# 4. ROS 2 开发前已经完成的底层 SDK 实机测试

这些测试发生在 ROS 2 Driver 之前，是当前 Driver 设计的事实基础。

| 能力 | 状态 | 说明 |
|---|---|---|
| Create / Destroy Handle | 已验证 | SDK 基础句柄正常 |
| Connect / Disconnect | 已验证 | 控制器连接正常 |
| Robot State Read | 已验证 | 主动状态读取正常 |
| HAR Subscribe | 已验证 | 异步状态回调正常 |
| ControlAccess | 已验证 | Request / Release 正常 |
| PROGRAMMER | 已验证 | 用户切换链路正常 |
| Alarm clear | 已验证 | 清告警流程已测试 |
| AUTO mode | 已验证 | 工作模式切换正常 |
| MotorOn / MotorOff | 已验证 | 电机上下电正常 |
| Enter / Exit Realtime Motion | 已验证 | Legacy realtime session 正常 |
| MoveJ | 已实机验证 | J6 +5° 并回原位的成功基线 |
| Digital Output | 已实机验证 | ON → Read → OFF → Read |
| MoveL | SDK/Demo 支持 | ROS 真机待测 |
| MoveC | SDK/Demo 支持 | ROS 真机待测 |
| MoveS | SDK/Demo 支持 | ROS 真机待测 |

MoveJ 成功基线核心链路：

```text
MP_EnterRealTimeMotionMode
→ MP_SetMotionRunMode
→ MP_ClearMotionTarget
→ MP_AddMovjTarget
→ MP_StartRealTimeMotion
→ HAR 实际关节反馈确认到位
```

退出：

```text
MP_StopRealTimeMotion
→ MP_ExitRealTimeMotionMode
→ MotorOff
→ 恢复原 WorkMode
→ ReleaseControlAccess
→ Unsubscribe
→ Disconnect
→ DestroyHandle
```

---

# 5. 0x82068043 / PathID 的关键结论

官方错误定义：

```text
0x82068043
HKR_E_IN_SYS_CALL_REMOTE_FIRET_CMD_REPEAT
```

实机日志表明，V2.1.2 下 `ST_MOTION_MOV_PARAM.nPathID` 会进入控制器内部作为类似 `nInstructionSeq` 的运动指令序号。

实际异常模式：

```text
第一次运行：PathID = 1 成功
程序没有完成后续不同 ID 的运动就异常/提前结束
控制器内部仍遗留最后 instruction sequence = 1

新进程再次使用 PathID = 1 作为首条运动
→ MP_AddMovjTarget 返回 0x82068043
```

重要结论：

- 这不是简单的“目标队列没有 Clear”。
- `MP_ClearMotionTarget()` 主要清除尚未执行的目标点，不保证清除控制器内部遗留 instruction sequence。
- 即使 Stop / ExitRealtime / Disconnect / DestroyHandle 成功，也不能据此假设控制器的该历史序号被清除。
- RETURN 当时能让后续重新从 ID=1 开始成功，并不是因为 RETURN 本身“清错”，而是 RETURN 使用了另一个 PathID，把控制器最后序号推进了。

因此 ROS 2 Driver 已按以下原则实现：

1. 所有 MoveJ / MoveL / MoveC / MoveS 共用一个 `MotionSequenceManager`。
2. PathID 不由 ROS 用户传入。
3. PathID 使用持久化 high-water mark。
4. 新 ID 先持久化，再发送给控制器（persist-before-send）。
5. 如果 `MP_AddMov*Target()` 返回 `0x82068043`，自动申请新 ID 并有限重试。
6. 仅在 Add 阶段允许这种自动重试。
7. `MP_StartRealTimeMotion()` 之后出现通信不确定时，不自动重发整个运动，避免重复运动。

该恢复逻辑目前**代码已完成、Core 已编译，但尚未通过新的 ROS Driver 真机 HIL 测试**。

---

# 6. 当前 .bashrc / 环境部署

用户已经把 ROS 与 SDK 环境写进 `~/.bashrc`。当前已明确包含：

```bash
# >>> fishros initialize >>>
source /opt/ros/humble/setup.bash
# <<< fishros initialize <<<

source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /home/ai/fr_ws/src/camport_ros2/install/setup.bash
source /home/ai/fr_ws/install/setup.bash
source /home/ai/fbg_ws/install/setup.bash
source /home/ai/teleop_ws/install/setup.bash
source ~/hikrobot_ros2/install/setup.bash

export MPSDK_ROOT=/home/ai/SDK/MPSDK_PUBLIC_FULL_V2.4.0_Build20251106_SVN1686294
export MPSDK_INC=$MPSDK_ROOT/Include/Public
export MPSDK_LIB=$MPSDK_ROOT/Libs/Linux/LX64/64
export LD_LIBRARY_PATH=$MPSDK_LIB:$LD_LIBRARY_PATH
```

必须补充（构建脚本与 Vendor CMake 的正式 SDK 根目录变量）：

```bash
export HIKROBOT_MPSDK_ROOT=$MPSDK_ROOT
```

原因：当前 `build_humble.sh` 与 `hikrobot_mpsdk_vendor` 正式使用 `HIKROBOT_MPSDK_ROOT`；
`MPSDK_ROOT` 仅用于兼容已有环境配置及派生 `MPSDK_INC` / `MPSDK_LIB`，不会被这两处读取。

PROGRAMMER 密码：用户已决定为测试方便可以长期写入本机环境。不要在交接文档中记录实际密码值，只记录变量名：

```bash
export HIKROBOT_PROGRAMMER_PASSWORD='...'
```

注意：当前 `.bashrc` 同时 overlay 多个 ROS 2 workspace。如后续出现 package 冲突、消息版本冲突或 CMake/运行时行为异常，可用一个只 source `/opt/ros/humble` 和 `~/hikrobot_ros2/install/setup.bash` 的干净终端进行排查。

---

# 7. 构建过程中已经处理过的问题

## 7.1 build_humble.sh 与 `set -u`

初版脚本在 source Humble setup 时开启 nounset，出现：

```text
/opt/ros/humble/setup.bash: 行 8: AMENT_TRACE_SETUP_FILES: 未绑定的变量
```

已修改构建脚本，使 ROS setup 在 `set +u` 状态下加载，再恢复严格模式。

## 7.2 vendor 没有向 core 正确传播 MPSDK include

曾出现：

```text
fatal error: MP_Public_API.h: 没有那个文件或目录
```

原因不是 SDK 路径错误，而是 `hikrobot_mpsdk_vendor` 初版 CMake target 导出不完整。

已修复为：

- vendor 建立并导出 `hikrobot_mpsdk_vendor::hikrobot_mpsdk` target；
- target 传播 MPSDK include path；
- target 传播 `libMPSDK_PUBLIC_COMPAT.so` 链接；
- core 正式链接该 namespaced target。

## 7.3 Threads::Threads 导出依赖

随后曾出现：

```text
Target "hikrobot_mpsdk_core" links to target "Threads::Threads"
but the target was not found
```

已修复：

- vendor `find_package(Threads REQUIRED)`；
- vendor `ament_export_dependencies(Threads)`；
- core 显式 `find_package(Threads REQUIRED)`；
- core 链接 `Threads::Threads`；
- core 导出 Threads dependency。

修复后完成全量构建。

---

# 8. 当前已经确认成功的构建结果

最终执行：

```bash
cd ~/hikrobot_ros2
./scripts/build_humble.sh
```

结果：

```text
Starting >>> hikrobot_mpsdk_vendor
Starting >>> hikrobot_msgs
Starting >>> hikrobot_description
Finished <<< hikrobot_description
Finished <<< hikrobot_mpsdk_vendor
Starting >>> hikrobot_mpsdk_core
Finished <<< hikrobot_mpsdk_core
Finished <<< hikrobot_msgs
Starting >>> hikrobot_driver
Finished <<< hikrobot_driver
Starting >>> hikrobot_bringup
Finished <<< hikrobot_bringup

Summary: 6 packages finished
```

因此当前构建层状态：**6/6 package PASS**。

---

# 9. 已完成的无真机 ROS 2 软件测试

## 9.1 Package discovery

已执行：

```bash
ros2 pkg list | grep hikrobot
```

成功发现：

```text
hikrobot_bringup
hikrobot_description
hikrobot_driver
hikrobot_mpsdk_core
hikrobot_mpsdk_vendor
hikrobot_msgs
```

## 9.2 Driver executable

已执行：

```bash
ros2 pkg executables hikrobot_driver
```

得到：

```text
hikrobot_driver hikrobot_driver_node
```

## 9.3 MoveJ interface

已执行：

```bash
ros2 interface show hikrobot_msgs/action/MoveJ
```

当前 MoveJ Action：

```text
Goal
- string[] joint_names
- float64[] positions
- uint8 speed_percent
- uint8 blend_radius
- builtin_interfaces/Duration timeout

Result
- result_code
- sdk_error_code
- path_id
- execution_time
- message

Feedback
- path_id
- progress_percent
- target_positions
- actual_positions
- position_error
```

ResultCode 已定义：

```text
SUCCESS
INVALID_GOAL
DRIVER_NOT_READY
ROBOT_NOT_READY
BUSY
SDK_ERROR
TIMEOUT
CANCELED
STATE_LOST
```

## 9.4 Driver node 启动

无真机状态下已成功启动：

```bash
ros2 run hikrobot_driver hikrobot_driver_node \
  --ros-args \
  --params-file ~/hikrobot_ros2/src/hikrobot_bringup/config/hikrobot_driver.yaml
```

节点存在：

```text
/hikrobot_driver
```

另外存在 TF listener implementation node。

## 9.5 当前 Topic

已实际看到：

```text
/diagnostics
/hikrobot_driver/status
/hikrobot_driver/tcp_pose
/hikrobot_driver/transition_event
/joint_states
/parameter_events
/rosout
/tf
/tf_static
```

注意实际命名空间以 `/hikrobot_driver/...` 为准，应据此维护 `docs/ROS_API.md`。

## 9.6 当前 Service

已实际看到：

```text
/hikrobot_driver/change_state
/hikrobot_driver/get_available_states
/hikrobot_driver/get_available_transitions
/hikrobot_driver/get_state
/hikrobot_driver/get_transition_graph
/hikrobot_driver/get_digital_input
/hikrobot_driver/get_digital_output
/hikrobot_driver/set_digital_output
/hikrobot_driver/stop_motion
```

另有标准参数服务。

## 9.7 当前 Action

已实际看到：

```text
/hikrobot_driver/move_j
/hikrobot_driver/move_l
/hikrobot_driver/move_c
/hikrobot_driver/move_s
```

因此四种运动 ROS Action Server 均已成功注册到 ROS graph。

---

# 10. 当前完成状态总表

## 10.1 开发与构建

| 项目 | 状态 |
|---|---|
| 架构设计 | DONE |
| ROS package 结构 | DONE |
| MPSDK Vendor | DONE |
| SDK Core | DONE |
| ROS Msg/Srv/Action | DONE |
| Lifecycle Driver | DONE |
| MoveJ 封装 | DONE |
| MoveL 封装 | DONE |
| MoveC 封装 | DONE |
| MoveS 封装 | DONE |
| IO 封装 | DONE |
| PathID 持久化代码 | DONE |
| 0x82068043 恢复代码 | DONE |
| rosdep | DONE |
| colcon build 6/6 | DONE |
| ROS graph 无真机测试 | DONE |

## 10.2 真机 HIL

| 项目 | 状态 |
|---|---|
| Driver + 真机 Configure | TODO |
| MPSDK Connect via ROS | TODO |
| HAR callback via ROS | TODO |
| Lifecycle Activate | TODO |
| ControlAccess via ROS | TODO |
| PROGRAMMER via ROS | TODO |
| AUTO via ROS | TODO |
| MotorOn via ROS | TODO |
| Realtime Motion via ROS | TODO |
| `/joint_states` 实际反馈 | TODO |
| `/hikrobot_driver/status` 实际反馈 | TODO |
| `/hikrobot_driver/tcp_pose` 实际反馈 | TODO |
| MoveJ Action HIL | TODO |
| PathID / 0x82068043 崩溃恢复 HIL | TODO |
| ROS DO/DI HIL | TODO |
| MoveL HIL | TODO |
| MoveC HIL | TODO |
| MoveS HIL | TODO |
| Deactivate / cleanup HIL | TODO |
| 自动 launch HIL | TODO |

---

# 11. 真机接入后的下一步测试流程

**不要重新从 CMake / rosdep / SDK 路径开始。当前这些已经通过。**

真机回来后从下面开始。

## STEP 1 — 网络基础

确认控制器 IP 与 YAML：

```text
192.168.2.64:9000
```

执行：

```bash
ping -c 3 192.168.2.64
```

确认本机机械臂网卡仍位于 `192.168.2.x` 网段。

## STEP 2 — 启动 Driver，不自动 Activate

终端 1：

```bash
source /opt/ros/humble/setup.bash
source ~/hikrobot_ros2/install/setup.bash

ros2 run hikrobot_driver hikrobot_driver_node \
  --ros-args \
  --params-file ~/hikrobot_ros2/src/hikrobot_bringup/config/hikrobot_driver.yaml
```

确认：

```bash
ros2 lifecycle get /hikrobot_driver
```

预期：

```text
unconfigured
```

## STEP 3 — Configure：第一条真机 ROS 测试

终端 2：

```bash
ros2 lifecycle set /hikrobot_driver configure
```

目标链路：

```text
MP_CreateHandle
→ MP_ConnectRobotWithIP
→ MP_HAR_SubscribeMessage
→ State Cache
```

成功后：

```bash
ros2 lifecycle get /hikrobot_driver
```

预期：

```text
inactive
```

如果 Configure 失败：**停止，不执行 Activate。** 保存终端 1 的完整日志用于排查。

## STEP 4 — Activate

在确认机械臂现场安全、急停/安全门正常后：

```bash
ros2 lifecycle set /hikrobot_driver activate
```

期望链路：

```text
RequestControlAccess
→ SwitchUser(PROGRAMMER)
→ EliminateAllAlarm
→ MotorOff
→ Set AUTO
→ Confirm AUTO
→ MotorOn
→ Confirm MotorOn
→ EnterRealTimeMotionMode
→ SetMotionRunMode
→ ACTIVE
```

## STEP 5 — 只检查状态，不运动

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /hikrobot_driver/status --once
ros2 topic echo /hikrobot_driver/tcp_pose --once
```

检查：

- `/joint_states.position` 应为 rad，不是 SDK degree。
- TCP ROS 位置应为 m，不是 SDK mm。
- TCP 姿态应由 SDK ZYX Euler 转为 ROS quaternion。
- 状态时间戳与 stale watchdog 正常。

## STEP 6 — MoveJ Action

MoveJ 是 ROS 运动链第一条 HIL 基准。目标应使用当前安全位置附近的小角度运动；此前 SDK 基线使用 J6 +5°。

验证完整链：

```text
ROS MoveJ Goal
→ MotionExecutor
→ MotionSequenceManager
→ ClearMotionTarget
→ AddMovjTarget
→ StartRealTimeMotion
→ HAR / Motion Feedback
→ 到位稳定判定
→ Action SUCCESS
```

测试前先读取当前关节位置，不要把历史关节角直接作为新的绝对目标。

## STEP 7 — PathID 崩溃恢复

MoveJ 正常后专门验证 `0x82068043`：

1. 正常执行一个 MoveJ 并记录 PathID。
2. 制造符合测试设计的异常退出/未完整后续流程。
3. 重启 ROS Driver。
4. 再下发运动。
5. 验证 Driver 不会因固定重复 ID 永久卡死。
6. 如果 Add 返回 `0x82068043`，验证自动 reserve 新 ID 后恢复。
7. 确认 Start 之后的错误不会触发整个运动自动重发。

PathID 状态文件默认预期在：

```text
~/.ros/hikrobot/path_id.state
```

测试期间不要随意删除该文件。

## STEP 8 — IO

验证 ROS Service 封装是否复现 06D 成功结果：

```text
DO ON
→ Readback
→ DO OFF
→ Readback
```

V2.1.2 IO 枚举兼容逻辑此前已确认需要旧兼容接口。

## STEP 9 — MoveL

MoveL 第一版参数：

- `target_pose`
- `speed_percent`
- `blend_radius`
- `timeout`

内部目标转换：

```text
ROS PoseStamped
→ base frame
→ m → mm
→ quaternion → ZYX Euler → degree
→ MP_AddMovlTarget
```

先使用低速度与短距离。

## STEP 10 — MoveC

验证：

```text
via_pose + target_pose
→ ST_MOTION_MOVC_PARAM
→ MP_AddMovcTarget
```

完成判定以最终 target pose 为准。

## STEP 11 — MoveS

验证：

```text
via_poses[] + target_pose
→ ST_MOTION_MOVS_PARAM
→ MP_AddMovsTarget
```

官方结构附加点数量上限应由 Driver 在 Goal 接收阶段预先校验。

## STEP 12 — 正常退出

```bash
ros2 lifecycle set /hikrobot_driver deactivate
ros2 lifecycle set /hikrobot_driver cleanup
```

预期：

```text
Deactivate:
StopMotion
→ ExitRealtime
→ MotorOff
→ 恢复 WorkMode
→ ReleaseControlAccess

Cleanup:
Unsubscribe
→ Disconnect
→ DestroyHandle
```

---

# 12. 当前暂时不要做的事情

在 HIL 基线完成前，不建议：

- 不要重新切换到 Motion API2。
- 不要推翻 Legacy MoveJ 成功链路。
- 不要第一时间接 MoveIt 2。
- 不要先写 ros2_control 高频 `write()`。
- 不要在 Start 成功后因为网络状态不确定而自动重新发送同一运动。
- 不要让 MoveJ / MoveL / MoveC / MoveS 分别拥有独立从 1 开始的 PathID。
- 不要删除 `path_id.state` 作为常规“清错方式”。
- 不要在第一轮真机测试直接使用自动 configure + activate launch；先手动 Lifecycle 验证。

---

# 13. 后续开发阶段（HIL 完成后）

HIL 验证完成后可进入：

1. 固化 `ros2 launch hikrobot_bringup hikrobot_driver.launch.py` 自动启动流程。
2. 更新 `ROS_API.md` 使其完全匹配实际 `/hikrobot_driver/...` 接口。
3. 增加真实机器人 URDF/Xacro/mesh。
4. 增加 FollowJointTrajectory adapter。
5. 接 MoveIt 2。
6. 根据实机证明再决定是否实现 `ros2_control`。
7. 增加更完整 diagnostics / alarm mapping / capability profile。
8. 建立自动化 HIL regression checklist。

---

# 14. 交接时需要优先保留的关键文件

```text
/home/ai/hikrobot_ros2/
├── src/hikrobot_mpsdk_vendor/CMakeLists.txt
├── src/hikrobot_mpsdk_core/CMakeLists.txt
├── src/hikrobot_mpsdk_core/src/robot_runtime.cpp
├── src/hikrobot_mpsdk_core/src/motion_sequence_manager.cpp
├── src/hikrobot_driver/src/hikrobot_driver_node.cpp
├── src/hikrobot_bringup/config/hikrobot_driver.yaml
├── src/hikrobot_msgs/action/*.action
├── scripts/build_humble.sh
├── docs/ARCHITECTURE.md
├── docs/HIL_TEST_PLAN.md
└── IMPLEMENTATION_STATUS.md
```

若后续出现问题，应优先依据：

1. 当前 ROS Driver 实际日志；
2. 已成功的旧 `05D MoveJ` / `06D DO` 实机测试行为；
3. 官方 V2.4.0 SDK 头文件与 Demo；
4. V2.1.2 compatibility layer 实际日志。

不要依据未验证的假设改动底层控制链。

---

# 15. 当前项目一句话状态

> **Hikrobot ROS 2 Driver V0.1 已完成 Ubuntu 22.04 + ROS 2 Humble 下的软件开发、依赖部署、6/6 package 编译和无真机 ROS graph 验证；Legacy SDK 的 MoveJ、状态、控制权、使能、Realtime Motion、DO 等底层实机基线已存在；下一阶段从 ROS Lifecycle `configure` 开始进行新 Driver 的真机 HIL 验证，然后依次完成 Activate、状态、MoveJ、PathID/0x82068043 恢复、IO、MoveL、MoveC、MoveS。**
