# Hikrobot ROS 2 Driver

面向六轴海康机械臂的 ROS 2 Humble Driver，使用 MPSDK 的 **Legacy 实时运动接口**。提供 Lifecycle、关节/笛卡尔运动 Action、数字 IO、只读工具管理、Flange/TCP 状态、PathID 持久化与健康监测。

安装、选择任意位置的 MPSDK、编译与动态库检查统一见 [部署指南](DEPLOYMENT.md)。本仓库当前只维护这两份说明文档；历史开发记录保留在 Git 历史中。

## 快速配置环境

已有Python 3和完整机械臂SDK时，在仓库根目录运行：

```bash
python3 tools/quick_deploy.py
source ~/.bashrc
```

工具自动搜索/校验SDK、隐藏输入并确认PROGRAMMER密码，将配置保存到用户bashrc，之后新开的交互式Bash自动加载。
密码按要求明文保存，bashrc及备份权限为0600，工具不会显示密码。首次仍需安装依赖和构建；
它不启动Driver、不连接硬件。参数、备份和手动部署步骤见[部署指南](DEPLOYMENT.md)。

## 当前版本范围

| 项目 | 状态与证据 |
|---|---|
| IO | **2026-10-24 用户确认真机测试成功**；尚未附逐通道命令、原始日志及测试二进制版本，不扩展为所有异常场景通过 |
| Legacy MoveL Queue | **11 点真机测试失败，本版仅按 1～10 点使用**；失败原因尚未确认 |
| 队列源码准入 | 仍为 1～32 点，**尚未在代码层拒绝超过 10 点**。调用方必须落实本版 10 点使用上限，不能把源码容量当作已验证能力 |
| Lifecycle、MoveJ、Cancel/Stop、PathID、Tool/TCP、Base TCP MoveL | 有旧版本的 HIL 通过记录；不等于后续所有代码变更都已回归 |
| MoveC、MoveS | 已实现；仓库尚无充分证据认定当前版本真机验收通过 |
| Health/恢复/永久 SDK 阻塞 | 有离线逻辑测试，异常场景仍需专门 HIL |

历史环境为 Ubuntu 22.04、ROS 2 Humble、MPSDK V2.4.0 Build20251106 SVN1686294、控制器 `V2.1.2_1444184_20241223`。机械臂 SDK 当前构建布局为 Linux x86_64。Driver 不使用 Motion API2，不包含完整机器人模型、MoveIt 集成、碰撞规划或图像时间同步节点。

旧 HIL 报告和审查记录可从[整合前的文档快照](https://github.com/zdydd03-rgb/hikrobot_ros2/tree/df5293f3d2e07a88780f4ae0d82c2db20294a4b7/docs)追溯。历史 TCP 跟踪误差不是手眼绝对精度，旧坐标和 PathID 不能作为新现场默认值。

## 工程结构与调用链

```text
ROS 2 Application / CLI
  -> hikrobot_driver / HikrobotDriverNode
     -> hikrobot_mpsdk_core / RobotRuntime
        -> 单一 MP_HANDLE + 共享 SDK mutex
           -> libMPSDK_PUBLIC_COMPAT.so / Legacy API
              -> Hikrobot Controller
```

| Package | 职责 |
|---|---|
| `hikrobot_msgs` | 5 个 Action、6 个 Service、RobotStatus/ToolInfo |
| `hikrobot_mpsdk_vendor` | 导出外部 SDK 头文件、链接目标和运行时库路径 hook |
| `hikrobot_mpsdk_core` | ROS 无关 Runtime、运动、IO Executor、PathID、CommandChannelHealth |
| `hikrobot_driver` | Lifecycle Node、ROS 接口、坐标/单位校验和状态发布 |
| `hikrobot_bringup` | Launch 与参数 YAML |
| `hikrobot_description` | 机器人模型占位；尚无真实 URDF、尺寸、关节极限或网格 |

当前GitHub **main仅发布机械臂包**，不再包含 `mv3d_rgbd_ros2`、`mv3d_rgbd_ros2_interface`。远端移除在独立副本中完成，本地原工作空间的相机源码未删除；原开发分支和Git历史仍保留旧版本。获取当前版本请按[部署指南第1节](DEPLOYMENT.md#1-获取源码与准备环境)克隆main到新目录。

运动在一个可 join 的 worker 中执行；三个 IO Service 共用一个 `IoCommandExecutor` worker。两类任务、Tool 与 Lifecycle 都共享同一个 SDK Handle 和 SDK command mutex。HAR/Motion SDK callback 仅更新内存缓存；connection callback 更新连接/健康状态。ROS 主程序使用 MultiThreadedExecutor。

### `src` 外目录的保留与清理

| 路径 | 处理与理由 |
|---|---|
| `tools/quick_deploy.py`、`tests/test_quick_deploy.py` | 保留：机械臂SDK/密码持久化配置及临时文件离线测试 |
| `tests/test_sequence_manager.cpp` | 保留：独立 PathID 持久化、重建与耗尽回归；尚未注册为 package CTest |
| `tools/tool_readonly_probe.cpp` | 保留并扁平化：独立 SDK 只读 Tool 诊断，可区分 SDK 与 ROS 问题；不随 Driver 自动运行 |
| 原 `docs/`、`IMPLEMENTATION_STATUS.md`、子目录 README | 内容整合到这两份文档，移除重复入口；原文可在 Git 历史查回 |
| 原 `scripts/` | 移除：构建命令已统一；旧 core_compile_check 漏编新 logic 源码，旧全工作空间脚本会顺带要求相机 SDK |
| 原固定三点视觉 MoveL 脚本 | 移除：包含历史请求专用坐标，不是通用 Driver 功能；历史版本仍可追溯 |
| `build/`、`install/`、`log/` | 本地生成物，已忽略、不提交；本次不删除，避免破坏已有构建环境和日志证据 |
| `Mv3dRgbdLog/`、`MvSdkLog/` | 本地 SDK 日志，加入忽略，不删除 |
| 本地 `Bd/`、`visual_heat_seam_moves*.py` | 按用户要求保持本地未跟踪，不修改、不提交、不删除 |
| `.git/` | 版本历史，保留；环境管理目录同样不清理 |

## 节点、Lifecycle 与接口权限

默认节点 `/hikrobot_driver`，包 `hikrobot_driver`，可执行文件 `hikrobot_driver_node`。下面的完整名称基于空 namespace 和默认节点名；改名后 `~/` 接口会随之变化，`/joint_states`、`/diagnostics` 是绝对名称。

| 状态/操作 | 实际行为 |
|---|---|
| 启动 | `unconfigured`；不自动连接、申请控制权或上电 |
| Configure | 创建 Handle、注册 connection callback、连接、订阅 HAR、等首帧（3 s）、保存原 work mode、初始化 PathID、加载 Active Tool；进入 INACTIVE |
| Activate | 申请控制权 → PROGRAMMER → 清报警 → 注册 motion callback → MotorOff 并确认 → AUTO 并验证 → MotorOn 并确认 → EnterRealTimeMotionMode → EQUILIBRIUM → 刷新 Tool；开启 lifecycle publishers |
| Deactivate | 拒新运动、取消/Stop、join worker、退出实时模式、MotorOff、恢复原 work mode、释放控制权；仍保留连接/HAR/健康监测 |
| Cleanup | 停止监测、暂停 IO 并等 idle，正常情况下退激活、unsubscribe、撤销 connection callback、disconnect/destroy Handle、清缓存 |
| Command Recovery | 不销毁可能仍在使用的 Handle，不自动重连/重放；下次 configure 被拒绝，要求进程替换 |

读 IO/Tool 要求 configured 且命令通道健康，不要求 ACTIVE；写 DO 和所有运动要求 ACTIVE。接口在节点构造时创建，**ROS graph 中可见不等于可用**。

```bash
ros2 lifecycle get /hikrobot_driver
ros2 lifecycle set /hikrobot_driver configure
ros2 service call /hikrobot_driver/get_active_tool hikrobot_msgs/srv/GetActiveTool '{}'
ros2 lifecycle set /hikrobot_driver activate
# 工作结束后
ros2 lifecycle set /hikrobot_driver deactivate
ros2 lifecycle set /hikrobot_driver cleanup
```

Configure 会连接硬件；Activate 会进入 AUTO 并上电。标准 Lifecycle 服务包括 `get_state`、`change_state`、`get_available_states`、`get_available_transitions`、`get_transition_graph`，均位于 `/hikrobot_driver/` 下，类型为对应的 `lifecycle_msgs/srv/*`。

## C++ Runtime 入口

直接集成core时，公共接口定义在
[`robot_runtime.hpp`](src/hikrobot_mpsdk_core/include/hikrobot_mpsdk_core/robot_runtime.hpp)，
请求、结果和状态类型在[`types.hpp`](src/hikrobot_mpsdk_core/include/hikrobot_mpsdk_core/types.hpp)。
这些C++方法不是额外ROS服务；直接调用时应自行遵循Node原有单位、frame和工具校验。

| 方法 | 用途 |
|---|---|
| `configure(RuntimeConfig, error)`、`activate(password, error)`、`deactivate()`、`cleanup()` | 生命周期；前两者返回bool，其余void |
| `configured()`、`connected()`、`active()`、`busy()`、`command_recovery_required()` | 查询运行标志；部分查询会poll命令健康状态 |
| `monitor_health(communication_timeout)` | 返回RuntimeHealthSnapshot，不调用SDK |
| `robot_state()`、`motion_state()` | 读取缓存快照 |
| `get_active_tool()`、`get_tool_info_list()` | 返回ToolResult / ToolListResult |
| `execute_move_j`、`execute_move_l`、`execute_move_c`、`execute_move_s` | 参数均为request、feedback、cancel_requested，返回MotionResult |
| `execute_follow_cartesian_path(request, feedback, cancel_requested)` | 返回CartesianPathResult |
| `stop_motion()` | 返回int64 SDK/本地错误码 |
| `get_digital_input(name)`、`get_digital_output(name)`、`set_digital_output(name, value)` | 返回IoResult |
| `list_digital_outputs(sdk_error)` | 返回vector<string>，SDK错误经引用参数输出，无ROS对应接口 |
| `last_sdk_error()`、`last_error_message()` | 最后记录的错误，不代替每次调用的Result |

## 全部机械臂 Action

### 公共参数与坐标合同

- `speed_percent`：`uint8`，1～100，原样传 SDK `nSpeedPercent`，不是 m/s 或 rad/s。
- `blend_radius`：`uint8`，0～10，原样传 `nBlendRadius`；不自行解释为毫米。SDK 的零值可采用预设交融，不能据此保证精确停点。
- `timeout`：`builtin_interfaces/Duration`，零值使用 `default_timeout_sec`，默认 30 s。Follow 校验规范非负 Duration；其它四个 Action 对非法 Duration 的处理不完全相同，调用方统一传规范值。
- 一次只允许一个运动 Goal。准入阶段拒收没有自定义 Result；接受后失败才返回 Result 并 ABORT。
- MoveJ 的 ROS 关节单位为 rad，SDK 为 deg。空 `joint_names` 按参数配置顺序；非空时按名字映射，恰好 6 个 `positions`。
- 所有笛卡尔 `PoseStamped` 表示 **Base 下控制器 Active Tool 的 TCP**，不是法兰或相机点。`frame_id` 必须精确等于 `base_frame`，默认 `base_link`；不做 TF 转换。
- 输入位置 m、姿态 quaternion；校验有限值与非零 quaternion，归一化后转换为 SDK mm/ZYX Euler deg。
- 每次请求先刷新工具，要求 flange-mounted（`mount_position=0`），固定本次 Tool 快照，传 Base 坐标 `1:0`、Active Tool ID、workpiece `-1`。参数 `tool_id` 不选择笛卡尔工具。
- 当前 TCP 到首点及相邻点都做平移/旋转步长校验；不是全路径碰撞或关节可达性证明。

### Goal 与 Legacy API

以下列出的字段是每个 Goal 的完整非注释定义；公共字段在每个 Action 中实际存在。

| Endpoint | 类型 | Goal 字段 | Runtime → Legacy |
|---|---|---|---|
| `/hikrobot_driver/move_j` | `hikrobot_msgs/action/MoveJ` | `string[] joint_names`、`float64[] positions`、`uint8 speed_percent`、`uint8 blend_radius`、`Duration timeout` | `execute_move_j` → `MP_AddMovjTarget`；joint CS 0:0 |
| `/hikrobot_driver/move_l` | `hikrobot_msgs/action/MoveL` | `PoseStamped target_pose`、`uint8 speed_percent`、`uint8 blend_radius`、`Duration timeout` | `execute_move_l` → `MP_AddMovlTarget` |
| `/hikrobot_driver/move_c` | `hikrobot_msgs/action/MoveC` | `PoseStamped via_pose`、`PoseStamped target_pose`、`uint8 speed_percent`、`uint8 blend_radius`、`Duration timeout` | `execute_move_c` → `MP_AddMovcTarget` |
| `/hikrobot_driver/move_s` | `hikrobot_msgs/action/MoveS` | `PoseStamped[] via_poses`、`PoseStamped target_pose`、`uint8 speed_percent`、`uint8 blend_radius`、`Duration timeout` | `execute_move_s` → `MP_AddMovsTarget`；ROS 最多49个 via 加终点 |
| `/hikrobot_driver/follow_cartesian_path` | `hikrobot_msgs/action/FollowCartesianPath` | `PoseStamped[] poses`、`uint8 speed_percent`、`uint8 blend_radius`、`Duration timeout` | `execute_follow_cartesian_path` → LegacyMoveLQueueExecutor → 多次 `MP_AddMovlTarget`；本版只用1～10点 |

`PoseStamped` 完整类型为 `geometry_msgs/PoseStamped`。每类运动先 `MP_ClearMotionTarget`，Add 后调用一次 `MP_StartRealTimeMotion`。MoveS 是一次 spline Add，不等于多点 MoveL Queue。

### Result、ROS 状态与 Feedback

MoveJ/L/C/S 的 Result 相同：

```text
uint8 result_code
int64 sdk_error_code
int32 path_id
builtin_interfaces/Duration execution_time
string message
```

五个 Action 共用 `result_code`：

| 值 | 常量 | 含义 |
|---|---|---|
| 0 | SUCCESS | 达到该执行器的成功判据 |
| 1 | INVALID_GOAL | 目标/参数不符合合同 |
| 2 | DRIVER_NOT_READY | Driver/实时模式/通道/恢复状态不就绪 |
| 3 | ROBOT_NOT_READY | 机器人状态不满足执行条件 |
| 4 | BUSY | 已有运动占用 |
| 5 | SDK_ERROR | SDK 或本地执行错误 |
| 6 | TIMEOUT | 运动超时，已进入停止处理 |
| 7 | CANCELED | 内部观察到取消/Stop |
| 8 | STATE_LOST | HAR 不可用或过期 |

ROS `SUCCEEDED` 对应成功；标准 CancelGoal 对应 `CANCELED`；Service StopMotion 等外部停止可使自定义 result_code 为 CANCELED，而 ROS 最终状态为 `ABORTED`。`sdk_error_code=0` 本身不能证明成功，必须一起看 ROS 状态、result_code、failure_stage/message。MoveL/C/S 的 Tool 准备失败可能返回 INVALID_GOAL 并携带 SDK 错误；Follow 对此区分 SDK_ERROR。

| Action | Feedback 完整普通字段 |
|---|---|
| MoveJ | `int32 path_id`、`float64 progress_percent`、`float64[] target_positions`、`actual_positions`、`position_error`；角度均为 rad |
| MoveL/C/S | `int32 path_id`、`float64 progress_percent`、`PoseStamped target_pose`、`actual_pose`、`float64 position_error_m`、`orientation_error_rad`；actual 为 TCP |

普通四运动在目标容限内、HAR stopped，按约50 ms轮询累计 `stable_samples`。它们不具备 Follow 的 HAR 序列去重或严格终态队列证据；Cancel/超时会请求 Stop，但不等待 Follow 那种独立 stopped callback 确认。SDK 阻塞可超过 Action timeout，Tool refresh 也不包含在普通运动核心计时内。

### FollowCartesianPath 专项

**11点HIL失败，当前应用输入须限制为1～10点。** 源码还保留32点容量和worker补队列，但这不是当前可用能力承诺。历史控制器十点窗口是实测观察，不是 Legacy SDK 头的官方容量定义。

全部 N 点先持久化 2N 个 ID，再 Clear/Add，首窗口最多10点，Start一次。PathID/target ID 分别为 `base+2*i`、`base+2*i+1`。Start前精确碰撞可整批重分配并重试；Start后绝不自动重放，补队列 Add 失败也不重分配。Feedback `queued_targets` 是累计已Add数量，不是实时队列长度。

完成要求 post-Start 匹配的 Motion callback、已有 running 证据、最终PathID stopped且left_paths=0，引用最终target；或采用受约束的pre/next均0兼容分支（必须晚于先前匹配running序列）。还需最终TCP容限、HAR stopped、不同的新HAR序列累计stable_samples、全部点已Add。仅物理到点不能判成功。

Follow Result：

```text
uint8 result_code
int64 sdk_error_code
int32[] path_ids
uint8 failure_stage
bool failure_target_index_valid
uint32 failure_target_index
builtin_interfaces/Duration execution_time
string message
```

`failure_stage`：NONE=0、VALIDATION=1、ID_RESERVATION=2、CLEAR=3、ADD=4、START=5、EXECUTION=6、STOP=7；target index 从0开始，只有valid=true时有意义。`path_ids` 只描述最终完整队列尝试，失败重试消耗的ID不回退。

Follow 的 Feedback 字段如下（由当前 `.action` 定义直接摘录）：

```text
uint8 stage
uint32 total_targets
uint32 queued_targets
bool current_segment_valid
uint32 current_segment_index
int32 current_path_id
bool actual_tcp_valid
geometry_msgs/PoseStamped actual_tcp
float64 final_position_error_m
float64 final_orientation_error_rad
uint64 motion_callback_sequence
bool motion_callback_post_start
bool motion_callback_valid
int32 motion_callback_path_id
int32 motion_callback_previous_target_id
int32 motion_callback_next_target_id
bool motion_callback_stopped
int32 motion_callback_left_path_count
int64 motion_callback_age_ms
bool motion_callback_path_matches
bool observed_running_queue_evidence
bool observed_terminal_queue_evidence
bool final_in_tolerance
int32 har_motion_status
uint64 har_sample_sequence
bool new_har_sample
uint32 stable_sample_count
uint32 required_stable_samples
string completion_blocker
string diagnostic_summary
```

`stage`：VALIDATING=0、RESERVING_IDS=1、CLEARING_QUEUE=2、QUEUEING=3、READY_TO_START=4、EXECUTING=5、STOPPING=6。Stop确认窗口独立于Action timeout，默认3000 ms，自发起Stop前的请求时间起算；要有请求前快照之后的新matching stopped callback。Stop失败/无法确认时stage=STOP并锁运动恢复；sdk_error_code仍可能为0，必须读message。

### 运动调用模板

目标由现场复核的 YAML 文件提供，不在文档中给出假设坐标。`GOAL_DIR` 由操作者设置，文件不是仓库自带样例；超时/速度/blend也要复核。Pose 映射必须包含 `header.frame_id`、`pose.position.{x,y,z}` 和 `pose.orientation.{x,y,z,w}`。

```yaml
# MoveJ 文件字段模板；大写标记必须换成经过确认的数值
joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
positions: [J1_RAD, J2_RAD, J3_RAD, J4_RAD, J5_RAD, J6_RAD]
speed_percent: APPROVED_PERCENT
blend_radius: APPROVED_BLEND
timeout: {sec: APPROVED_SECONDS, nanosec: 0}
```

笛卡尔文件的公共字段同上；MoveL填写`target_pose`完整PoseStamped；MoveC填写`via_pose`和`target_pose`；MoveS填写`via_poses`列表和`target_pose`；Follow只填写`poses`列表（1～10点），不能用`via_poses`替代。

```bash
: "${GOAL_DIR:?请设置已复核的Goal YAML目录}"
ros2 action send_goal /hikrobot_driver/move_j hikrobot_msgs/action/MoveJ "$(cat "$GOAL_DIR/move_j.yaml")" --feedback
ros2 action send_goal /hikrobot_driver/move_l hikrobot_msgs/action/MoveL "$(cat "$GOAL_DIR/move_l.yaml")" --feedback
ros2 action send_goal /hikrobot_driver/move_c hikrobot_msgs/action/MoveC "$(cat "$GOAL_DIR/move_c.yaml")" --feedback
ros2 action send_goal /hikrobot_driver/move_s hikrobot_msgs/action/MoveS "$(cat "$GOAL_DIR/move_s.yaml")" --feedback
ros2 action send_goal /hikrobot_driver/follow_cartesian_path hikrobot_msgs/action/FollowCartesianPath "$(cat "$GOAL_DIR/follow_cartesian_path.yaml")" --feedback
```

每个Action还有标准的`/_action/send_goal`、`get_result`、`cancel_goal`服务与feedback/status话题。取消从另一终端执行；零UUID/零stamp表示取消该Action server所有目标：

```bash
ros2 service call /hikrobot_driver/follow_cartesian_path/_action/cancel_goal action_msgs/srv/CancelGoal '{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}'
```

其它Action替换路径中的`follow_cartesian_path`即可。关闭CLI或Ctrl-C不等于已取消机器人；Cancel响应后仍需观察最终Result及物理停机。

## 全部机械臂 Service

所有Endpoint前缀都是`/hikrobot_driver/`，类型前缀为`hikrobot_msgs/srv/`。共同Response字段为`bool success`、`int64 sdk_error_code`、`string message`。

| Endpoint / 类型 | Request完整字段 | Response额外字段 | 权限与Legacy API |
|---|---|---|---|
| `get_digital_input` / `GetDigitalInput` | `string name` | `bool value` | configured且命令通道健康；`MP_GetConfigIOLevelStatus`，DI |
| `get_digital_output` / `GetDigitalOutput` | `string name` | `bool value` | 同上；`MP_GetConfigIOLevelStatus`，DO |
| `set_digital_output` / `SetDigitalOutput` | `string name`、`bool value` | 无 | ACTIVE且core active；`MP_SetConfigIOLevelStatus`，DO |
| `stop_motion` / `StopMotion` | 空 | 无 | 置取消标志并`MP_StopRealTimeMotion`；无有效实时模式时可能直接返回0 |
| `get_active_tool` / `GetActiveTool` | 空 | `hikrobot_msgs/ToolInfo tool` | configured且健康；`MP_GetActivateTool` + `MP_HAR_GetToolInfo`，成功更新缓存 |
| `get_tool_info_list` / `GetToolInfoList` | 空 | `hikrobot_msgs/ToolInfo[] tools` | configured且健康；`MP_HAR_GetToolInfoList` |

不存在TCP/Flange getter Service、工具设置/切换Service、ROS IO列举Service。ToolManager是只读工具查询与缓存职责，没有独立同名类。`list_digital_outputs()`仅是core C++方法，经`MP_GetConfigIOAttributeList_Deprecated`，不属于上述ROS服务。

IO的name是控制器配置名称。Set不隐式读回、不重试；任何历史特殊名称也走相同路径，没有ProtectedArcOutputState、arc mutex、高电平Get抑制或-10004。超长名称会按SDK buffer截断，调用方不要依赖截断。

```bash
ros2 service call /hikrobot_driver/get_active_tool hikrobot_msgs/srv/GetActiveTool '{}'
ros2 service call /hikrobot_driver/get_tool_info_list hikrobot_msgs/srv/GetToolInfoList '{}'
: "${DI_NAME:?请设置实际DI配置名}"
: "${DO_NAME:?请设置实际安全DO配置名}"
# 以下YAML引号模板要求名称不含单引号
ros2 service call /hikrobot_driver/get_digital_input hikrobot_msgs/srv/GetDigitalInput "{name: '$DI_NAME'}"
ros2 service call /hikrobot_driver/get_digital_output hikrobot_msgs/srv/GetDigitalOutput "{name: '$DO_NAME'}"
ros2 service call /hikrobot_driver/set_digital_output hikrobot_msgs/srv/SetDigitalOutput "{name: '$DO_NAME', value: true}"
ros2 service call /hikrobot_driver/set_digital_output hikrobot_msgs/srv/SetDigitalOutput "{name: '$DO_NAME', value: false}"
ros2 service call /hikrobot_driver/stop_motion hikrobot_msgs/srv/StopMotion '{}'
```

StopMotion的success只表示SDK请求结果，不是安全级急停，也不是物理静止证明。

## Topic 与数据结构

| Topic | 类型 | 条件/内容 |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | SensorDataQoS；6关节位置rad；valid即可发布，未额外排除stale，须同时检查status |
| `/hikrobot_driver/flange_pose` | `geometry_msgs/msg/PoseStamped` | HAR `stBasePose`，Base下法兰；valid、非stale、至少6关节 |
| `/hikrobot_driver/tcp_pose` | `geometry_msgs/msg/PoseStamped` | `T_base_flange × T_flange_tool`；还需有效flange-mounted工具缓存 |
| `/hikrobot_driver/status` | `hikrobot_msgs/msg/RobotStatus` | 连接/active、HAR有效性、机器人状态、错误 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 连接、Recovery、HAR、急停/servo错误或正常 |
| `/hikrobot_driver/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 标准Lifecycle状态迁移 |
| `/rosout`、`/parameter_events` | `rcl_interfaces/msg/Log`、`ParameterEvent` | 标准日志与参数事件 |

前五者默认20 Hz，仅在ACTIVE且ros_active_时发布；INACTIVE仍收HAR和监测，但这些Topic无新数据。没有原始HAR全帧Topic。Flange与TCP的frame_id都是base_frame，消息时间是ROS发布时间，未映射控制器采样时间；不发布TF。工具缓存只在Configure/Activate、显式get_active_tool、Cartesian请求准备时刷新，外部工具切换后应显式刷新，且仍存在refresh到Add之间的控制器改Tool竞争窗口。

```bash
ros2 topic echo /hikrobot_driver/status --once
ros2 topic echo /joint_states --qos-reliability best_effort --once
ros2 topic echo /hikrobot_driver/flange_pose --once
ros2 topic echo /hikrobot_driver/tcp_pose --once
ros2 topic echo /diagnostics --once
```

RobotStatus完整定义：

```text
std_msgs/Header header
bool connected
bool active
bool state_valid
bool state_stale
int32 servo_status
int32 motion_status
int32 emergency_status
int32 controller_status
int32 safety_gate_status
int32 robot_motion_mode
int32 joint_count
float64 motion_execution_percent
int32 current_path_id
int64 last_sdk_error
string last_error_message
```

ToolInfo完整定义（位置转换为m，Euler转换quaternion；质量直接复制SDK值）：

```text
int32 id
string name
string description
int32 mount_position
geometry_msgs/Pose flange_to_tool
float64 mass
geometry_msgs/Pose mass_center
bool convex_hull_enabled
```

Tool ID校验范围0～9，工具列表count为0～10。`flange_to_tool`表示法兰到工具；`mount_position=0`表示法兰安装。不能把历史Tool 2的坐标、质量或标定当作所有现场值。

## 全部主要参数

当前节点没有动态参数应用回调。多数参数在Configure读取，状态发布timer在构造时创建；`ros2 param set`成功不代表SDK/执行器已更新，应按读取位置重配或重启。

| 参数 | Node默认 / 随包YAML | 含义 |
|---|---|---|
| `robot_ip`、`robot_port` | `192.168.2.64`、9000 | 示例控制器地址，部署时修改 |
| `har_publish_ms` | 50 | SDK请求HAR周期ms |
| `joint_names` | joint1～joint6 | 6轴名称及默认顺序 |
| `base_frame`、`tool_frame` | base_link、tool0 | 后者当前仅存储，不产生TF或选择工具 |
| `programmer_password_env` | HIKROBOT_PROGRAMMER_PASSWORD | 读取密码的环境变量名 |
| `sequence_state_file` | 空 | 解析为`$HOME/.ros/hikrobot/path_id.state`；HOME缺失回退`/tmp/hikrobot_path_id.state` |
| `sequence_collision_retries` | 3 | pre-Start精确碰撞的总尝试次数，至少1 |
| `default_timeout_sec` | 30.0 | Goal timeout为0时回退 |
| `joint_tolerance_rad` | 0.0034906585 | 约0.2° |
| `position_tolerance_m` | 0.001 | 1mm |
| `orientation_tolerance_rad` | 0.0174532925 | 约1° |
| `stable_samples` | 5 | 稳定判定数量 |
| `state_stale_timeout_ms` | 1000 | 运动options阈值；对外robot_state()/active()仍有固定1000ms判断 |
| `stop_confirmation_timeout_ms` | 3000 | Follow停止确认窗口 |
| `sdk_command_lock_timeout_ms` | 1000 | 等锁/IO准入预算 |
| `sdk_command_watchdog_ms` | 3000 | SDK调用监测/IO运行等待预算 |
| `health_monitor.enabled` | true | 被动健康timer |
| `health_monitor.check_period_ms` | 1000 | 健康检查周期 |
| `health_monitor.communication_timeout_ms` | 3000 | HAR通信超时阈值 |
| `max_cartesian_translation_step_m` | **0.05 / 0.600** | Node默认50mm，随包YAML实际600mm；其50mm注释过时，配置本次未改 |
| `max_cartesian_rotation_step_rad` | 0.1745329252 | 约10° |
| `tool_id`、`workpiece_id` | 0、-1 | 兼容参数；Cartesian强制使用Active Tool/Base合同 |
| `state_publish_rate_hz` | 20.0 | 状态timer频率，构造时最低按1Hz处理 |

Configure要求六个joint_names，相关保护/停止/health参数为正，watchdog大于lock timeout；启用health时通信超时至少为3个检查周期和10个HAR周期。

## IO Executor、Health 与恢复边界

三个IO服务都进入单worker。Task的QUEUED/RUNNING/COMPLETED/CANCELLED转换由task mutex保护，并非使用atomic枚举。拿共享SDK锁并复查健康后，QUEUED→RUNNING成功才进入Vendor；排队超时取消后不能迟发IO。

| 本地错误码 | 含义 |
|---|---|
| `-10001` | 等锁/排队未能入SDK，任务未执行且被取消；通道健康后才考虑人工重试 |
| `-10002` | 已入SDK后超时，物理结果未知；SDK调用不能取消，后续命令被禁用 |
| `-10003` | 未配置、通道不可用、Cleanup或永久Recovery等拒绝；已poison时需替换进程 |

默认IO等待预算为入SDK前1000ms，加运行后3000ms及调度开销。它不覆盖DDS发现/排队/整个ROS端到端硬时限。永久阻塞worker会持有Handle/SDK锁，服务可先返回，但析构join仍可能永远等待。Tool、运动、Stop和Lifecycle的同步Vendor调用没有同样的IO代理返回边界。

Health timer在ACTIVE/INACTIVE读取connection callback、HAR年龄、CommandChannelHealth；不调用SDK、不创建专用线程或第二Handle、不等SDK mutex。日志仅在状态转换输出。HAR默认50ms，通信阈值3000ms；急停由HAR中`E_MP_EMERGENCY_ACTIVE=1`识别，不等同于断连。

断连、已配置调用watchdog、HAR超时或transport错误`0x20000..0x2000B`可锁Command Recovery；重连回调或迟到成功不会解锁。仅motion recovery且通道健康时可Lifecycle清理重配；command poisoned则要求进程替换。健康故障不会自动迁移ROS Lifecycle，也不自动Stop/MotorOff。Cleanup SUCCESS不等于硬件一定已释放。

## PathID与尚存的软件限制

PathID由Driver分配，客户端不得提供。MotionSequenceManager先写临时文件、fsync、rename，持久化成功后才Add；失败尝试消耗的块不回退，接近2,000,000,000时拒绝回绕。精确`0x82068043`首条行号冲突只在Start前有界换ID；Start后不重试运动。

真实state文件禁止删除、清零、回退；保持相同运行用户、HOME和配置路径，并只启动一个分配进程。实现没有跨进程flock；原子rename不等于跨进程互斥。文件缺失/损坏/不可读会按时间种子初始化，可能重遇历史ID，不能宣称所有异常都fail closed；目录fsync错误也未完全上报。

此前源码审查还发现未充分集成验证的风险：普通运动极早Cancel/Start时序、Cleanup与服务并发、poisoned Handle的callback lifetime、worker异常边界、MoveS core直接调用的50-via边界与ROS49门禁不一致、补队列迟到callback。它们不是本次已复现或已修复的结论；10点文档范围也不解决所有并发问题。

## 离线验证与HIL记录方式

机械臂当前注册7个CTest目标、97个GTest用例；2026-10-24本轮独立重建后全部通过（纯离线）：

| 目标 | 用例数 | 覆盖 |
|---|---:|---|
| test_tool_transform | 16 | 单位、变换、工具范围、TCP误差、步长保护 |
| test_command_channel_health | 8 | transport、watchdog、断连/HAR超时、永久恢复锁 |
| test_legacy_io_dispatch | 5 | DI/DO一次调用、普通名称、无重试 |
| test_io_command_executor | 7 | 单worker、准入取消、运行超时、pause与正常join |
| test_legacy_movel_queue_executor | 41 | FakeBackend队列/ID/取消/停止确认/终态证据；20/32点仅离线覆盖 |
| test_cartesian_contract | 14 | frame、有限值、quaternion、Base/Tool、Duration |
| test_health_monitor | 6 | 纯状态转换与日志事件 |

没有完整Fake MPSDK动态库、RobotRuntime端到端或ROS Service有界返回集成测试。永久阻塞测试用受控假任务解除阻塞，不能证明析构可返回。独立根目录PathID测试不在97内。离线构建/测试命令见部署指南；软件PASS不能替代真机PASS。

HIL应记录commit和库版本、配置、Goal/UUID、Tool、PathID、原始Motion/HAR序列、完整Result与物理最终状态。后续先定位11点失败，当前使用仍限10点；IO异常、MoveC/MoveS和恢复时序分别验收。本次文档整合及部署检查不运行Driver、不连接控制器、不执行运动/IO、不访问真实PathID文件。
