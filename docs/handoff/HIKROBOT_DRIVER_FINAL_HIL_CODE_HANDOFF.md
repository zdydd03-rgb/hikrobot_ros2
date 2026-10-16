# Hikrobot ROS2 Driver 最终 HIL 代码交接

审查日期：2026-10-16，Asia/Shanghai。工作空间：`/home/ai/hikrobot_ros2`，ROS 2 Humble，Linux x86_64，C++17。

本文依据 **HEAD + 当前全部工作区修改 + 未跟踪源码**。仅新增本 Markdown；没有改源码/配置、运行 Driver、连接控制器、执行运动/IO、读取或修改真实 PathID 状态、切分支、reset、commit 或 push。为遵守“只新增交接 Markdown”，本次没有重新 build/test（会写构建结果），离线结论来自已有日志、CTest 注册和 XML，并核对构建依赖及产物。

证据等级：**源码已实现**仅表示静态审查；**离线测试通过**仅表示本文列明的已有软件测试；**历史 HIL**仅限仓库报告对应旧版本；**当前版本 HIL 尚未验证**适用于本轮所有验收项。需要用户从另一个对话提供 **Motion HIL 历史交接文档**，补齐后期 MoveS/MoveC/队列等实测的版本、命令、完整反馈与结论；不能将口头开发完成或注释中的 HIL-proven 当作当前验收通过。

## 1. 当前真实 Git 与环境

- 分支：`experiment/pathid-exhaustion-fail-safe`。
- HEAD：`117adf6f24969608c34a7e3465835a65700ed5d8`。
- 提交标题：`fix: refill legacy MoveL queue window`；提交时间 2026-09-14 17:07:56 +0800。
- 检查了 `git branch --show-current`、`git status --short`、`git log -1 --format=fuller`、`git diff --stat`、`git diff --check`。初始 `diff --check` 无输出，返回 0。没有暂存修改。
- 已跟踪差异：17 个文件，647 行新增、301 行删除；**不包含未跟踪新源码**。两轮整改尚未 commit/push，不能只检出 HEAD 测试。

新增本文前完整状态（M 修改、D 删除、?? 未跟踪）：

```text
 M IMPLEMENTATION_STATUS.md
 M docs/ARCHITECTURE.md
 M docs/ROS_API.md
 M docs/handoff/CURRENT_HANDOFF.md
 M src/hikrobot_bringup/config/hikrobot_driver.yaml
 M src/hikrobot_driver/CMakeLists.txt
 M src/hikrobot_driver/include/hikrobot_driver/hikrobot_driver_node.hpp
 M src/hikrobot_driver/src/hikrobot_driver_node.cpp
 M src/hikrobot_mpsdk_core/CMakeLists.txt
 M src/hikrobot_mpsdk_core/include/hikrobot_mpsdk_core/command_channel_health.hpp
 D src/hikrobot_mpsdk_core/include/hikrobot_mpsdk_core/protected_arc_output.hpp
 M src/hikrobot_mpsdk_core/include/hikrobot_mpsdk_core/robot_runtime.hpp
 M src/hikrobot_mpsdk_core/src/command_channel_health.cpp
 D src/hikrobot_mpsdk_core/src/protected_arc_output.cpp
 M src/hikrobot_mpsdk_core/src/robot_runtime.cpp
 M src/hikrobot_mpsdk_core/tests/test_command_channel_health.cpp
 D src/hikrobot_mpsdk_core/tests/test_protected_arc_output.cpp
?? Mv3dRgbdLog/Mv3dRgbd_00.log
?? MvSdkLog/CamCtrl_00.log
?? MvSdkLog/MVGigEVisionSDK_00.log
?? src/hikrobot_driver/include/hikrobot_driver/health_monitor.hpp
?? src/hikrobot_driver/src/health_monitor.cpp
?? src/hikrobot_driver/tests/test_health_monitor.cpp
?? src/hikrobot_mpsdk_core/include/hikrobot_mpsdk_core/io_command_executor.hpp
?? src/hikrobot_mpsdk_core/include/hikrobot_mpsdk_core/legacy_io_dispatch.hpp
?? src/hikrobot_mpsdk_core/src/io_command_executor.cpp
?? src/hikrobot_mpsdk_core/src/legacy_io_dispatch.cpp
?? src/hikrobot_mpsdk_core/tests/test_io_command_executor.cpp
?? src/hikrobot_mpsdk_core/tests/test_legacy_io_dispatch.cpp
```

其中 `protected_arc_output.hpp/.cpp` 和对应测试是工作区删除；Health Monitor、Executor、dispatch 的头/实现/测试是新建未跟踪文件，交接拷贝必须包括它们。两个日志目录是相机运行遗留，不是 Driver 新功能。

环境证据：`/opt/ros/humble`、`build/`、`install/`、`log/` 均存在；vendor CMake cache 为 RelWithDebInfo，SDK root：

```text
/home/ai/SDK/MPSDK_PUBLIC_FULL_V2.4.0_Build20251106_SVN1686294
```

源码与最近离线构建的一致性核查：当前有效 link.txt 中 core shared 只含 `robot_runtime.cpp.o`，再链接 `hikrobot_mpsdk_core_logic.a` 的六个当前 logic 对象；Driver 链接当前 contract/health。有效链接对象的源码及仓库内依赖头没有较对象更新的情况。旧目录遗留 `hikrobot_mpsdk_core.dir/src/tool_transform.cpp.o`，早于源码，但已不在当前 shared 的链接命令里，实际用 logic target 中较新的对象。Driver 安装可执行文件为 build 符号链接，逐字节一致；core 安装库与 build 库 Build ID 同为 `e691a67296d3f247ceff01251351beb33327c807`，`.text` SHA256 同为 `1e89dfead4888edd9b8286a9e85f698c2bb41aee89af7f5440f9e4cac8ba997d`；整体文件因安装时 RUNPATH 处理不同不能要求全文件 hash 相等。这支持当前产物与最近离线构建一致，但旧日志没有逐文件源码 hash，不能据此给出可复现构建的密码学保证。附录提供当前源码 hash，后续 HIL 应另存同一快照。

## 2. 全工作空间 Package 与架构

共 8 个 ROS2 Package；没有把相机包漏掉，也没有把它们误算为机器人控制链。

| Package | 目录/核心类与源码 | 构建依赖、运行关系 |
|---|---|---|
| hikrobot_msgs | `action/` 5 个、`srv/` 6 个、`msg/` RobotStatus/ToolInfo，CMake 注册 rosidl | ament_cmake、rosidl_default_generators，builtin_interfaces、geometry_msgs、std_msgs；运行 rosidl_default_runtime |
| hikrobot_mpsdk_vendor | CMake shim、`env-hooks/hikrobot_mpsdk_library_path.sh.in`；无控制节点 | ament_cmake、Threads；导出 SDK interface target，依赖外部 SDK，不分发 SDK |
| hikrobot_mpsdk_core | `include/hikrobot_mpsdk_core/`、`src/`、`tests/`；RobotRuntime::Impl、MotionSequenceManager、LegacyMoveLQueueExecutor、IoCommandExecutor、CommandChannelHealth、工具变换函数 | ament_cmake、vendor、Threads；纯 C++，无 ROS runtime 依赖；shared runtime + 内部 static logic |
| hikrobot_driver | `include/hikrobot_driver/`、`src/`、`tests/`；HikrobotDriverNode、HealthTransitionTracker、cartesian_contract；`main.cpp` MultiThreadedExecutor | rclcpp、rclcpp_lifecycle、rclcpp_action、builtin_interfaces、sensor_msgs、geometry_msgs、diagnostic_msgs、tf2、tf2_geometry_msgs、msgs、core |
| hikrobot_bringup | `launch/hikrobot_driver.launch.py`、`config/hikrobot_driver.yaml` | exec driver、launch、launch_ros、lifecycle_msgs、ament_index_python；只启动一个 LifecycleNode |
| hikrobot_description | README、CMake；URDF 占位包 | ament_cmake；没有真实尺寸/关节极限/模型，不提供可依赖的碰撞或运动学验证 |
| mv3d_rgbd_ros2_interface | `srv/Image.srv`、`srv/PointCloud.srv` | rosidl、sensor_msgs，服务定义使用 std_msgs/Bool；不是 hikrobot_msgs 服务 |
| mv3d_rgbd_ros2 | `common/HikCamera`、`src/hik_camera.cpp`，ImagePublish/ImageSubscriber/ImageServer/ImageClient/PointCloudServer/PointCloudClient，`launch/*.xml`、`config/ShowImage.rviz` | rclcpp、sensor_msgs、camera_info_manager、pcl_conversions、interface；CMake 另找 cv_bridge、OpenCV、PCL；外部 `/opt/Mv3dRgbdSDK`；独立相机链 |

相机主要 Topic：`/camera/depth`、`/camera/rgb`、可配置 aligned-depth、`/camera/leftIr`、`/camera/rightIr`、`/depthCamInfo`、`/rgbCamInfo`；服务 `SaveImage`、`SavePointCloud`。其 package.xml 对 cv_bridge/OpenCV/PCL 的声明与 CMake 不完全对齐，不能假设全工作区安装依赖只看机器人包就足够。

```text
ROS2 Application / ros2 CLI
  -> hikrobot_driver / HikrobotDriverNode (LifecycleNode)
      -> RobotRuntime (hikrobot_mpsdk_core)
          -> 单一 MP_HANDLE + SDK timed_mutex
          -> MPSDK Legacy (PUBLIC_COMPAT + 同目录依赖)
              -> Hikrobot Controller

motion worker -> execute_move_* / LegacyMoveLQueueExecutor -> 同一 SDK call()
IO services -> IoCommandExecutor 单 worker -> legacy_io_dispatch -> 同一 SDK call()
SDK HAR/motion callbacks -> 短时缓存 mutex -> 状态发布/运动完成判断
SDK connection callback -> 原子连接状态 + CommandChannelHealth
health wall timer -> 缓存/CommandChannelHealth -> 转换日志、Recovery lock
```

RobotRuntime 拥有 Handle、生命周期 mutex、SDK timed_mutex、状态缓存、PathID manager、运动 busy/恢复标志与 IO Executor。ROS Node 负责生命周期、单位/坐标验证、Action reservation、worker、发布器和 Tool cache；没有第二个机器人控制 Handle。state/health 各 MutuallyExclusive callback group，IO/Tool 服务共用 Reentrant group，Stop 另有 Reentrant group；运动 Action server 使用默认组，实际运动在一个可 join 的 std::thread。ROS 多线程执行器让 health/stop 有机会调度，但不承诺实时调度。

## 3. 当前真实 ROS2 接口

节点绝对名 `/hikrobot_driver`，可执行文件 `hikrobot_driver_node`。以下 `~/` 都展开到 `/hikrobot_driver/`。接口在构造函数创建，存在于 ROS graph 不表示已 Configure/Activate。

### 3.1 全部 Driver Action

| Endpoint | 准确类型 | Goal | Runtime -> Legacy Add |
|---|---|---|---|
| `/hikrobot_driver/move_j` | hikrobot_msgs/action/MoveJ | string[] joint_names；float64[] positions；uint8 speed_percent/blend_radius；Duration timeout | execute_move_j -> MP_AddMovjTarget |
| `/hikrobot_driver/move_l` | hikrobot_msgs/action/MoveL | PoseStamped target_pose；同上速度/blend/timeout | execute_move_l -> MP_AddMovlTarget |
| `/hikrobot_driver/move_c` | hikrobot_msgs/action/MoveC | PoseStamped via_pose、target_pose；同上 | execute_move_c -> MP_AddMovcTarget |
| `/hikrobot_driver/move_s` | hikrobot_msgs/action/MoveS | PoseStamped[] via_poses、PoseStamped target_pose；同上 | execute_move_s -> MP_AddMovsTarget |
| `/hikrobot_driver/follow_cartesian_path` | hikrobot_msgs/action/FollowCartesianPath | PoseStamped[] poses；同上 | execute_follow_cartesian_path -> LegacyMoveLQueueExecutor -> 多次 MP_AddMovlTarget |

前四个共同 Result：`uint8 result_code, int64 sdk_error_code, int32 path_id, Duration execution_time, string message`。五者共用结果枚举：SUCCESS=0、INVALID_GOAL=1、DRIVER_NOT_READY=2、ROBOT_NOT_READY=3、BUSY=4、SDK_ERROR=5、TIMEOUT=6、CANCELED=7、STATE_LOST=8。ROS Goal 在准入阶段 REJECT 时不会产生自定义 INVALID_GOAL Result；必须区分拒收与接受后 abort。

MoveJ Feedback：path_id、progress_percent、target_positions[]、actual_positions[]、position_error[]（rad）。MoveL/C/S Feedback：path_id、progress_percent、target_pose、actual_pose（TCP）、position_error_m、orientation_error_rad。

FollowCartesianPath Result：共同 result_code/sdk_error_code/execution_time/message，改用 `int32[] path_ids`，另有 failure_stage、failure_target_index_valid、failure_target_index（从 0 开始）。failure_stage NONE=0、VALIDATION=1、ID_RESERVATION=2、CLEAR=3、ADD=4、START=5、EXECUTION=6、STOP=7。

Follow Feedback：stage（VALIDATING=0 至 STOPPING=6）、total_targets、queued_targets（累计 Add 成功数，不是控制器当前队列长度）、current_segment_valid/index、current_path_id、actual_tcp_valid/actual_tcp、final_position_error_m、final_orientation_error_rad，以及完整 motion_callback_*、observed_running_queue_evidence、observed_terminal_queue_evidence、final_in_tolerance、har_motion_status、har_sample_sequence、new_har_sample、stable_sample_count/required_stable_samples、completion_blocker、diagnostic_summary。附录逐字列出全部字段和枚举，采集时保留原文。

使用条件：Node 的 ros_active_ 与 core active() 均为真，速度 1..100、blend 0..10，goal_reserved_ 独占；Runtime 另有 motion_busy。MoveJ 恰 6 个 positions，空 joint_names 按配置顺序，非空按名字匹配。MoveS ROS via 上限 49；Follow poses 1..32，规范 Duration。Cartesian 还需有效新鲜 HAR、flange-mounted Active Tool、准确 Base frame、合法位姿及逐段保护校验。Follow core 明确要求 servo=1、motion=0、emergency=0；普通 execute_common 没有同样完整的启动前机器人状态门禁，不能视为完全等价。

### 3.2 全部 Driver 自定义 Service

| Endpoint / 类型（hikrobot_msgs/srv/） | Request | Response | 条件 / Runtime / SDK |
|---|---|---|---|
| get_digital_input / GetDigitalInput | string name | bool success/value，int64 sdk_error_code，string message | configured + command connected；get_digital_input -> Executor -> MP_GetConfigIOLevelStatus (DI) |
| get_digital_output / GetDigitalOutput | string name | 同 GetDI | configured + command connected；get_digital_output -> Executor -> MP_GetConfigIOLevelStatus (DO) |
| set_digital_output / SetDigitalOutput | string name，bool value | bool success，int64 sdk_error_code，string message | ROS 额外要求 ACTIVE + core active；set_digital_output -> Executor -> MP_SetConfigIOLevelStatus (DO) |
| stop_motion / StopMotion | 空 | bool success，int64 sdk_error_code，string message | request_motion_cancel + stop_motion -> MP_StopRealTimeMotion；没有 realtime/Handle 时 core 返回 0；成功只表示请求结果 |
| get_active_tool / GetActiveTool | 空 | bool success，int64 sdk_error_code，ToolInfo tool，string message | configured + command connected；get_active_tool -> MP_GetActivateTool + MP_HAR_GetToolInfo，成功更新 Node cache |
| get_tool_info_list / GetToolInfoList | 空 | bool success，int64 sdk_error_code，ToolInfo[] tools，string message | configured + command connected；get_tool_info_list -> MP_HAR_GetToolInfoList |

不存在独立 TCP/Flange getter Service；通过 Topic 读取。ToolManager 是上述只读接口/转换/缓存的职责集合，没有单独 ToolManager 类，也没有设置/激活/创建工具 Service。`list_digital_outputs()` 是 core C++ 方法，使用 `MP_GetConfigIOAttributeList_Deprecated`，没有对应 ROS Service，也不属于三个 IO Service 的 Executor 路径。

ToolInfo：id/name/description/mount_position、flange_to_tool Pose、mass、mass_center Pose、convex_hull_enabled；位置由 SDK mm -> ROS m，Euler deg -> quaternion，质量直接复制 SDK 值。工具 ID 为 0..9、工具列表 count 为 0..10，定长字符串处理见 `tool_transform.cpp`。

框架接口另有 lifecycle_msgs/srv 的 change_state、get_state、get_available_states、get_available_transitions、get_transition_graph；参数的 describe/get/list/set/set_atomically 等标准 ROS 服务；每个 Action 自动生成 send_goal/get_result/cancel_goal 与 feedback/status。不要把这些当成额外自定义业务 API。

### 3.3 主要 Topic

| Topic | 类型 | 数据和限制 |
|---|---|---|
| /joint_states | sensor_msgs/msg/JointState | SensorDataQoS；6 关节 rad；valid 即可发布，未额外拒绝 stale |
| /hikrobot_driver/flange_pose | geometry_msgs/msg/PoseStamped | Base 下 HAR stBasePose，m/quaternion；valid、非 stale、至少 6 关节 |
| /hikrobot_driver/tcp_pose | geometry_msgs/msg/PoseStamped | Base 下 T_base_flange × T_flange_tool；还需工具缓存有效、mount=0 |
| /hikrobot_driver/status | hikrobot_msgs/msg/RobotStatus | connected/active、state_valid/stale、servo/motion/emergency/controller/safety_gate/robot_motion_mode、joint_count、motion_execution_percent、current_path_id、last_sdk_error/message |
| /diagnostics | diagnostic_msgs/msg/DiagnosticArray | Recovery/连接/状态/安全错误或正常；不是完整 runtime health snapshot |
| /hikrobot_driver/transition_event | lifecycle_msgs/msg/TransitionEvent | ROS Lifecycle 框架状态迁移 |
| /rosout、/parameter_events | rcl_interfaces/msg/Log、rcl_interfaces/msg/ParameterEvent | 标准框架接口 |

前五个是 Lifecycle publishers，仅在 ros_active_ 发布（默认 20 Hz），INACTIVE 无状态 Topic 新消息，但仍收 HAR、可只读 IO/Tool、监测健康。没有直接公开“原始 HAR 全帧” Topic，没有 TF broadcaster；tool_frame 参数目前仅存储，TCP Pose 的 frame_id 也是 base_frame。

## 4. Lifecycle 的实际步骤

源码定位：`hikrobot_driver_node.cpp:284` 起；`robot_runtime.cpp:813` 起。

**Configure**：读取/校验参数（6 joint_names、正数保护阈值；watchdog > lock；health timeout 至少 3 个 check 周期且至少 10 个 HAR 周期）-> 解析 PathID 路径 -> Runtime lifecycle_mutex -> 拒绝已有 command Recovery -> cleanup 旧资源 -> MP_CreateHandle -> 注册 connection callback（MP_SetUserCallback，CONNECT_STATUS，pUserData=Impl）-> MP_ConnectRobotWithIP -> MP_HAR_SubscribeMessage（HAR ROBOT_STATUS，uPublishTime=har_publish_ms，pUserData=cache）-> cache CV 等第一帧，最多 3 s -> MP_GetRobotWorkMode 保存原模式 -> 创建 MotionSequenceManager -> 确认未断连 -> configured、mark_configured_connected、开启 health_monitoring、resume IO executor -> Node 读取 Active Tool，失败则 cleanup 并返回 FAILURE。Configure 会真实建立通信，不应当作本次审查命令执行。

**Activate**：Node 从指定环境变量取非空密码 -> Runtime 检查 configured、Handle、HAR 可用且 1 s 内新鲜、emergency=0、motion stopped -> MP_RequestControlAccess -> 验证密码 buffer（非空且 <64）-> MP_SwitchUser(PROGRAMMER)，清本地临时密码 buffer -> MP_EliminateAllAlarm，等 300 ms -> MP_SetMotionUserCallback -> MP_SetMotorOff 并等 HAR OFF（最多 5 s）-> MP_SetRobotWorkMode(AUTO)，等 300 ms -> MP_GetRobotWorkMode 验证 -> MP_SetMotorOn 并等 HAR ON（最多 5 s）-> MP_EnterRealTimeMotionMode -> MP_SetMotionRunMode(EQUILIBRIUM) -> core active=true -> Node 再刷新 Tool -> activate publishers、ros_active=true。失败通常走 deactivate_internal 回滚；请求控制权失败直接返回。密码验证发生于申请控制权之后。

**Deactivate**：Node ros_active=false -> cancel token -> core Stop -> join motion worker -> Runtime lifecycle_mutex -> active=false -> 正常路径 Stop、100 ms、ExitRealTimeMotionMode、100 ms -> 条件允许时 MotorOff 并等 5 s -> 恢复原 work mode -> ReleaseControlAccess -> 清认证/current_path_id -> Node deactivate publishers。保持 HAR/连接/configured，Health Monitor 继续工作。没有恢复到激活前用户身份的 SDK 调用。

**Cleanup**：Node cancel、join worker -> Runtime lifecycle_mutex -> 关闭 health_monitoring -> IO executor pause_and_wait_idle(watchdog)，排队/已取出未入 SDK 的任务被取消 -> 正常时 deactivate_internal -> HAR unsubscribe、100 ms -> reset connection callback -> disconnect -> destroy Handle、100 ms -> configured=false、clean disconnected、reset sequence 与缓存 -> Node 清 Tool cache/goal reservation。

**Recovery cleanup**：若 command Recovery 已锁定，或等 IO idle 超时使当前调用进入 Recovery，停止后续 SDK 释放，保留 Handle，configured/connected/active=false；下次 configure 显式拒绝，要求替换进程。只发生 motion recovery（例如队列 Stop 未确认）且 command channel 未 poisoned 时，可按 deactivate/cleanup/configure/activate 恢复。HAR/connection 恢复不会自动解除 command Recovery，也不会自动重启运动。

**阻塞边界**：所有已进入 Vendor API 的调用均可能阻塞；try_lock_for 只限制等待 SDK mutex，不限制 fn()。MP_CreateHandle 还直接调用。第一帧和 servo 等待是本地有时限等待，不能约束其前后的 SDK 调用。Configure 在 configured=false 阶段不会通过 call() begin_call 监测；Activate、Tool、Stop、运动 Add/Start、普通生命周期 SDK 调用也没有 IO executor 的有界等待代理。Node 的 join_motion_worker、Runtime 的 lifecycle_mutex、IO worker 析构 join 均可能无限等待。Lifecycle 回调打印成功/返回 SUCCESS 不等于所有硬件释放已确认，尤其 Recovery 分支。

## 5. 运动执行链、坐标与完成语义

`Action -> handle_*_goal -> accept_* worker -> [Cartesian prepare] -> RobotRuntime::execute_* -> execute_common / LegacyMoveLQueueExecutor -> MP_ClearMotionTarget -> MP_AddMov*Target -> MP_StartRealTimeMotion -> Controller`。

- MoveJ：Goal positions 为 rad，按 joint_names 映射后乘 180/pi 为 SDK deg；Legacy joint target，坐标选择强制 joint 0:0。预留 2 ID，path=id_base，target=id_base+1。
- MoveL：Base 下 **Active Tool TCP 目标**；2 ID，MP_AddMovlTarget。
- MoveC：via + target 都是同一 Base 下 Active Tool TCP；3 ID，终点 target=id_base+1，via=id_base+2，MP_AddMovcTarget。
- MoveS：via_poses + target 都是同一合同；via_count+2 ID，MP_AddMovsTarget，一次 spline Add。ROS 上限 49 via。SDK MP_SUBPOS_SPLINE_MNUM=50 是数组容量，结构注释说附加点最大 49；core 直接调用仅检查 `>50`（与 ROS 门禁不完全一致），不要从 C++ core 传 50 个 via。
- MoveL/C/S/Follow：frame_id 必须精确等于 base_frame，空/其它 frame 拒绝，不做 TF；有限值检查、非零 quaternion 安全归一化，getRPY(ZYX) -> deg，m -> mm。Tool 必须 mount_position=0；每次 Cartesian worker 在 Add 前刷新 Active Tool，覆盖 options 为 tool.id、Base 1:0、workpiece=-1；配置 tool_id 不选择 Cartesian Tool。HAR stBasePose 按 flange 合同使用；TCP=flange×tool，用 TCP 误差判完成，不能把 flange_pose 直接当目标 TCP。逐段保护含当前 TCP->第一个点与所有相邻点；不构成完整曲线/碰撞/关节极限验证。
- 速度为 SDK nSpeedPercent 1..100；不是 m/s 或 rad/s。blend_radius 原样传 nBlendRadius 0..10；SDK 头称交融半径但未给此字段物理单位，不自行解释为 mm。零值在头文件说明中可取工程预设交融；不能保证 blend=0 就一定无交融。其它线速度/加速度字段为初始化默认 0。
- Duration 零使用 default_timeout_sec=30；Follow 严格检查 sec>=0、nanosec<1e9；普通四 Action 没有相同 Duration 合法性检查，ns<=0 回退默认，正值截到 ms，<1 ms 可能变 0 后在 core 拒绝。普通四 Action 的 timeout 从 execute_common 起算，不覆盖先前 Tool refresh/ROS 排队，阻塞 SDK 仍可越过期限。

普通四运动共用 execute_common (`robot_runtime.cpp:615`)：校验 options、active/realtime/command/recovery；CAS motion_busy；持久化 ID 后 Clear、Add。Add 碰 `0x82068043` 可重新预留并重试，最多 max(1, sequence_collision_retries) **总尝试数**；Start 只尝试一次，失败不重放。循环每 50 ms 看 HAR，反馈约 100 ms；当前误差容限内且 HAR motion stopped，stable_samples 次判成功。这里按轮询累计，**没有 Follow 的 HAR sequence 去重，也不要求新 matching motion callback/running 证据**；已有目标/重复 HAR 的完成语义应回归。Cancel、HAR stale、emergency!=0、timeout 都请求 Stop 并返回对应 code；普通运动不等待新 stopped callback 确认。progress_percent 优先 motion callback，否则 HAR，不应当用百分比独立判 PASS。

ROS Result SUCCESS -> succeed；CANCELED 且 goal_handle is_canceling -> canceled，否则 abort（例如 StopMotion 引起的内部取消）；其它 abort。sdk_error_code=0 不能单独证明成功，需 result_code + ROS 状态 + message；部分本地失败没有 Vendor 错误码。MoveL/C/S 在 prepare_cartesian_goal 的 Tool SDK 读取失败时仍填 INVALID_GOAL，同时携带 sdk_error_code；Follow 对同类问题区分 SDK_ERROR/INVALID_GOAL。不要只按 result_code 猜根因。

### 5.1 FollowCartesianPath：当前仍是 Legacy MoveL Queue

实现：`legacy_movel_queue_executor.cpp:15,324,368,400,500,540,615,652,686,752`；Runtime backend 在 `robot_runtime.cpp:419` 起。

1. 校验 1..32、有界参数/有限值/工具合同、runtime/recovery/busy；检查初始 HAR 新鲜、六轴、servo=1、motion=0、emergency=0。
2. 全部 N 点一次预留并持久化 2N ID（最多 64），path_i=base+2i，target_i=base+2i+1。先持久化，再任何 Clear/Add。
3. Clear，首次只 Add min(N,10) 个，Start 一次；Start 前通过 cancel token + stop_generation + start_stop_mutex 抑制已经取消/Stop 的启动。Start 失败请求 Stop、标恢复，不自动重启。
4. **10 点窗口仍存在**：kController212QueueWindow=10。源码注释/旧交接称来源是控制器 2.1.2 Legacy HIL 行为，并以 API2 作为旁证；不是 Legacy MP_Motion_API.h 官方容量承诺，更不代表 Driver 用 API2。修复前 20 点仅运行前 10 点的记录属于失败证据，不能当修复后 PASS。
5. 补队列只在执行 worker，callback 仅更新缓存；每个递增 motion sequence 作一次决策。未 stopped 且还有点：available=10-min(10,max(0,left_path_count))，Add min(available,remaining)。每个目标正常仅 Add 一次，不 Clear/二次 Start/分拆 Action。重复/倒序 sequence 不重复补。注意补队列分支以 callback valid/new/not-stopped 为条件，没有像完成判断一样显式要求 callback 的 PathID 匹配；异常/迟到回调情形需 HIL 关注。
6. 启动前碰撞可 Clear 后整批重分配，失败块仍消耗；补队列 Add 失败（含碰撞）不能重新分配/重放，Stop 并报 index/stage。新 stopped callback 若仍有未提交点、left_paths>0 或不在最后 PathID，立即 queue_underflow_or_premature_stop。
7. 完成需 post-start matching motion callback，已观察 running queue evidence，最后 PathID stopped 且 left_paths=0，引用最终 target ID；兼容 pre/next 都为 0 的 Legacy sentinel，但要求先有匹配 running，终态 sequence 晚于最新 running。再满足最终 TCP 容限、HAR stopped、**新的 post-start HAR sequence**累计 stable_samples，且全部点已提交。仅物理 TCP 到点不能判 Action PASS。
8. Cancel/Stop generation、状态丢失、急停、deadline 到达都走 Stop 分类。独立 Stop confirmation 窗口默认 3000 ms，自 **发起 Stop 之前** requested_at 起算，不复用已到期 Action deadline。要求 motion.sequence > 请求前快照、received_at >= requested_at、stopped、PathID 属当前集合；Stop 调用期间来的回调也可有效。Vendor Stop 阻塞会消耗窗口且不能被本地计时中断。
9. Stop 确认成功保留原 code/stage；SDK Stop 失败或未确认则 failure_stage=STOP 并 motion recovery lock。确认超时可能 sdk_error_code=0，必须读 stage/message。timeout Result 返回时间可晚于 Action timeout，加上停止确认或 SDK 阻塞；不是硬实时上界。

## 6. IO API 调用链与简化状态

`ROS get_di/get_do/set_do callback -> RobotRuntime::{get_digital_input,get_digital_output,set_digital_output} -> io_executor.execute -> dispatch_digital_get/set -> Impl::call(operation, mark_running) -> sdk_mutex -> MP_GetConfigIOLevelStatus / MP_SetConfigIOLevelStatus`。

Get：ST_IO_LEVEL_ATTR.szIOName，eType=E_MP_COMMON_IO_TYPE，unionDetailType.eCommonType=E_MP_IO_DI/DO，ST_IO_LEVEL_VALUE.nValue!=0 为 true。Set：ST_IO_LEVEL_INFO.stIOLevelAttr.szIOName，eCommonType=E_MP_IO_DO，nValue=1/0。名字来自控制器实际配置，不能猜通道数字/名称；strncpy 按 SDK buffer 截断，未做超长/空名的完整提前校验。dispatch 一次调用、无重试、Set 不隐式 Get 验证。

三个 Service 的真正 SDK 路径全部进 Executor；未配置/恢复/Set 非 ACTIVE 时可在入队前失败，这不是旧 SDK 旁路。`list_digital_outputs()` 仍直接经 call() 调 deprecated list API，但没有暴露为这三个 ROS Service。

已从当前 `src/hikrobot*` 运行代码及 CMake 注册移除 ProtectedArcOutputState、protected_arc_output 文件、arc mutex、高电平 Get 抑制及 -10004。普通输出名（包括历史特殊名称）一视同仁；测试名 ProtectedNameIsAnOrdinaryDigitalOutputRead 是反回归测试，不是遗留保护状态。build 中旧 `test_protected_arc_output.gtest.xml`/对象/可执行文件或旧文档不代表仍在有效运行链。

## 7. Health Monitor

Node wall timer 默认 1000 ms（独立 callback group），HAR 请求周期 50 ms，通信超时 3000 ms，HAR 状态默认 stale 1000 ms。对外 robot_state()/active() 使用源码固定的1000 ms；state_stale_timeout_ms 参数主要传入运动 options，改变该参数不会同步改变所有状态发布门限。通信超时与动作 stale 是两个概念。定时器实际触发还受 ROS 调度影响；3 s 阈值不保证恰在第 3000 ms 日志/锁定。

`monitor_health()` -> core monitor_health -> command poll + HAR cache age，随后 HealthTransitionTracker 只对转换记录日志，避免每轮刷屏。急停来源是 HAR eEmergencyStatus，**恰等 E_MP_EMERGENCY_ACTIVE=1**；未知非零值不被 health 当作急停，但运动逻辑仍将非零视为不允许运动。日志包括 monitor started、communication healthy、disconnect/reconnect、HAR timeout/restored、emergency activated/released、command timeout、Recovery required。

Configured 的 INACTIVE 与 ACTIVE 都监测；Cleanup 后 monitoring_active=false，tracker 重置。INACTIVE 无 lifecycle status/diagnostics 发布，应从 rosout 看监测；不要因 Topic 静默认定没收 HAR。ACTIVE 健康丢失并不自动改变 ROS Lifecycle 状态，ros_active_ 可仍为真，runtime active/connected=false，Topic 可继续报错；没有自动 reconnect/activate。

**监测器不发起 SDK 调用，不创建专属线程/新 Handle，不等待 sdk_mutex，也不执行 Stop/MotorOff。** 它由既有 MultiThreadedExecutor 调度，读 CommandChannelHealth/robot cache/日志 tracker/error 等短 mutex；callbacks 不调用 Vendor SDK，静态未发现 health 持锁反向等待 sdk_mutex 的环路。但这不是端到端无死锁/实时保证：ROS executor 线程耗尽、SDK 回调时序、其它 join/生命周期锁及 Vendor 内部锁不在纯 tracker 测试覆盖内。急停日志也不是可取代硬件急停的执行通道。

## 8. IoCommandExecutor 与有界返回的准确范围

`io_command_executor.hpp/.cpp`：每个 Runtime 创建 **一个** worker 线程，mutex/CV FIFO，shared_ptr<Task> 保存 operation/result，禁止每次请求再建线程或 detach。

任务状态 kQueued -> kRunning -> kCompleted，或 kQueued -> kCancelled。**不是 std::atomic<IoTaskState>**，而是在 task mutex 下原子化检查/转换。worker 从 queue 取出后仍是 queued，只有拿到共享 SDK mutex、复查 health 后调用 mark_running，queued->running 成功才真正进入 SDK；超时/cleanup 已取消任务则 mark_running=false，防止服务已报失败却迟发 IO。

- 入队开始到真正入 SDK 前，queue_timeout 默认 1000 ms；含排队及等待 SDK mutex。未开始就取消，-10001，不执行该任务。
- 从 mark_running 的 running_at 起算 execution_timeout 默认 3000 ms。CV 到时禁接收、取消所有尚未 running 的任务，显式通知 command health，原请求返回 -10002 `SDK command timeout; outcome unknown`；正在 SDK 的任务仍运行。后续请求 -10003 fail fast，不重试 Set，不臆定输出状态。
- 正常调度条件下，runtime 等待约 <= queue_timeout + execution_timeout，再加本地短锁/调度；不是整个 ROS Service 从发起到收到响应的硬时限（发现、executor 排队、DDS 与 OS 延迟不受此值约束）。尚无 ROS Service 集成测试证明端到端时限。
- timeout 后 worker 返回也不清 Recovery；task 是堆对象，不捕获 service 栈变量；SDK 入参在 worker operation 栈内保持至 Vendor 返回。cleanup pause 后等待 worker 真正 idle 才可正常销毁 Handle。
- **永久阻塞**：worker 保留 SDK mutex/Handle，服务调用者可以先返回，但 Runtime/Executor 析构必须 join，不能保证进程优雅退出。Impl 把 io_executor 声明为最后成员，使其最先析构 join，防止 worker 访问已销毁健康/锁/Handle 数据；不使用 detach。外部停止进程与确认机器人/输出安全状态需 HIL 操作流程，不由本审查执行。
- worker operation 没有统一 catch；若内部 C++ operation 抛异常可能终止进程，当前测试只验证正常操作及受控返回错误，不是任意异常容错证明。

## 9. CommandChannelHealth 与错误码

源码 `command_channel_health.hpp:11`、`.cpp`。独立短 mutex 保存 connected、recovery_required、call_in_progress、开始时间/operation、fault。call() 在等待共享 SDK mutex 前后都检查 Recovery；begin/end 追踪已配置后的同步 SDK 调用；poll 达 watchdog 锁定，回调断连/HAR 超时/0x20000..0x2000B transport 返回也锁定。普通业务错误不自动作为网络错误。连接 bool 原子反映 callback 原始状态，health.connected 还叠加 recovery；两者不能混用。

| 本地错误码 | 含义与处理 |
|---|---|
| -10001 kLocalSdkCommandBusy | 等 SDK command lock 或 IO queued 阶段超时；IO 任务未进 Vendor、已取消，可在当前命令结束且健康允许后人工重试 |
| -10002 kLocalSdkCommandTimeout | 已进入 Vendor 的命令超过 watchdog，结果未知；无法中断 SDK；本次 IO 返回此码，后续禁用 |
| -10003 kLocalSdkRecoveryRequired | 命令通道不可用/未配置/已锁定，或 Executor shutdown/cleanup 拒绝；poisoned 情形需进程重启，不能仅 Lifecycle 循环 |

这些是 Driver 自定义负数，不是 Vendor SDK 官方错误。Vendor 原码按 int64 返回；`0x82068043`=2181464131，对应本地 SDK 头 `HKR_E_IN_SYS_CALL_REMOTE_FIRET_CMD_REPEAT`（首条运动行号与遗留行号重复）。SDK 断连后 callback connected=true 也不解除 Recovery；清状态函数 mark_clean_disconnected 保留已锁定 fault，configure 在其前即拒绝。禁用 health timer 不等于禁用所有 command health：call/accessor poll 与 IO 显式 timeout 仍可锁定，但被阻塞的其它同步调用不会被本地强制返回。

## 10. PathID 分配、持久化与恢复

`motion_sequence_manager.cpp:21,28,41,97`。Node 参数 sequence_state_file 空时解析 `$HOME/.ros/hikrobot/path_id.state`，在本环境通常即 `/home/ai/.ros/hikrobot/path_id.state`；HOME 缺失回退 `/tmp/hikrobot_path_id.state`。core RuntimeConfig 自身默认是 /tmp，不能混淆 ROS Node 的实际配置。本次没有访问真实文件，故未确认磁盘内容/当前高水位/权限；路径依据当前 Node/YAML 确认仍然有效。

load 有效范围 0 <= value < 2,000,000,000；缺失、不可读、解析失败或范围错误时**不是 fail closed**，使用 100000+(epoch_seconds % 1e9) 种子。reserve_block 要 1..64，实例内 mutex，先以 int64 算 high+count；next_high>=2e9 抛 overflow，拒绝回绕。写 `.tmp`（0600）、write/fsync/close、rename，再尝试父目录 fsync；成功后才更新内存 high_water。持久化失败不 Add；保留已消耗失败块，不回退。

支持正常的顺序进程重启加载高水位，但**没有跨进程 flock/唯一实例协调**，不能多进程共用该文件同时分配，固定 .tmp 名也会冲突；原子 rename 不等于跨进程互斥。文件损坏时 reseed 可能回退/碰撞，必须由人工核实并保留原始证据，禁止删除、清零、回退状态。目录 fsync 失败被忽略，不能承诺任意断电下绝对持久性。SDK 运动队列 Clear 不清 PathID 磁盘状态；Controller 历史冲突用有界 Add 重试，新 ID 块仍持久化，Start 后永不自动 replay。

## 11. Launch、YAML 与参数

`hikrobot_driver.launch.py:9` 的默认 YAML 是 `expanduser('~/hikrobot_ros2/src/hikrobot_bringup/config/hikrobot_driver.yaml')`，不是 ament share 路径；跨 HOME/拷贝工作空间时必须传 config_file 绝对路径。launch 只创建 LifecycleNode，没有 EmitEvent/自动 configure/activate。节点名 hikrobot_driver，空 namespace。

vendor CMake 使用 `$HIKROBOT_MPSDK_ROOT/Include/Public` 和 `Libs/Linux/LX64/64/libMPSDK_PUBLIC_COMPAT.so`，检查 Public/Motion 头与库，链接 Threads/dl，rpath-link 解决同目录二级依赖；安装的 env hook 为 LD_LIBRARY_PATH 前置配置时 SDK 目录。因此运行前 source install/setup.bash，搬迁 SDK 后需重新匹配构建环境，不要只复制一个 .so。密码来自 HIKROBOT_PROGRAMMER_PASSWORD（可用 programmer_password_env 改名），不在 YAML。

完整当前 YAML 见附录。关键默认值与差异：

| 参数 | Node 默认 / 当前 YAML | 用途 |
|---|---|---|
| robot_ip/robot_port | 192.168.2.64 / 9000 | Controller 目标，启动后 Configure 才连接 |
| har_publish_ms | 50 | SDK 请求 HAR 周期 |
| sdk_command_lock_timeout_ms / sdk_command_watchdog_ms | 1000 / 3000 | IO 两阶段时限、共享 command health |
| health_monitor.enabled/check_period_ms/communication_timeout_ms | true / 1000 / 3000 | 被动监测 |
| joint_names | joint1..joint6 | 6 轴 ROS 顺序 |
| base_frame / tool_frame | base_link / tool0 | 后者当前没有 TF/选工具效果 |
| sequence_state_file / sequence_collision_retries | 空 / 3 | 默认 HOME 状态文件；总尝试次数 |
| default_timeout_sec | 30.0 | timeout 零值回退 |
| joint_tolerance_rad / position_tolerance_m / orientation_tolerance_rad | 0.0034906585 / 0.001 / 0.0174532925 | 约 0.2°、1 mm、1° |
| stable_samples / state_stale_timeout_ms | 5 / 1000 | 运动完成、Follow 状态年龄 |
| stop_confirmation_timeout_ms | 3000 | Follow 停止确认 |
| max_cartesian_translation_step_m | **Node 0.05 / YAML 0.600** | 当前启动配置允许相邻点 600 mm；注释 50 mm 不正确 |
| max_cartesian_rotation_step_rad | 0.1745329252 | 10°逐段保护 |
| tool_id / workpiece_id | 0 / -1 | 兼容参数；Cartesian 被 Active Tool/Base 合同覆盖 |
| state_publish_rate_hz | 20.0 | 构造阶段创建 timer，最低 max(1,hz) |

多数参数在 configure 读取到成员；state publisher timer 在构造阶段确定。没有动态 on_set_parameters 回调重新应用 SDK/运行状态；ros2 param set 成功不能保证运行时立即生效，需按实际读取位置确认。

## 12. 可供下一次 HIL 使用的 CLI

**以下仅供后续具备现场授权的测试执行；本次没有执行。** 激活会上电并进 AUTO；SetDO/Action 将改变硬件。先取得 Motion HIL 历史交接、确认安全目标和实际 IO 名。本文不提供假设坐标，也不自动从当前状态生成目标。

### 12.1 环境与启动

每个终端先执行前两条 source；启动终端准备密码（不打印密码）。若需重新构建，在离线构建阶段执行下面 build，不能用老 `core_compile_check.sh` 替代。

```bash
cd /home/ai/hikrobot_ros2
source /opt/ros/humble/setup.bash
export HIKROBOT_MPSDK_ROOT=/home/ai/SDK/MPSDK_PUBLIC_FULL_V2.4.0_Build20251106_SVN1686294
# 离线重建命令，仅下一阶段按需执行；本次未运行
colcon build --symlink-install --packages-up-to hikrobot_bringup --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source /home/ai/hikrobot_ros2/install/setup.bash
read -r -s -p 'PROGRAMMER password: ' HIKROBOT_PROGRAMMER_PASSWORD
export HIKROBOT_PROGRAMMER_PASSWORD
ros2 launch hikrobot_bringup hikrobot_driver.launch.py config_file:=/home/ai/hikrobot_ros2/src/hikrobot_bringup/config/hikrobot_driver.yaml
```

另一终端：

```bash
source /opt/ros/humble/setup.bash
source /home/ai/hikrobot_ros2/install/setup.bash
ros2 pkg prefix hikrobot_driver
ros2 node info /hikrobot_driver
ros2 param dump /hikrobot_driver
ros2 lifecycle get /hikrobot_driver
ros2 lifecycle set /hikrobot_driver configure
# 此时 INACTIVE：健康日志/HAR缓存工作，但下列状态 Topic 要 Activate 后才有新消息
ros2 service call /hikrobot_driver/get_active_tool hikrobot_msgs/srv/GetActiveTool '{}'
ros2 service call /hikrobot_driver/get_tool_info_list hikrobot_msgs/srv/GetToolInfoList '{}'
ros2 lifecycle set /hikrobot_driver activate
ros2 lifecycle get /hikrobot_driver
ros2 topic echo /hikrobot_driver/status --once
ros2 topic echo /joint_states --qos-reliability best_effort --once
ros2 topic echo /hikrobot_driver/flange_pose --once
ros2 topic echo /hikrobot_driver/tcp_pose --once
ros2 topic echo /diagnostics --once
ros2 topic hz /hikrobot_driver/status
```

没有原始 HAR Topic；status/joint_states/flange_pose 是 HAR 派生，tcp_pose 再合成 Tool。INACTIVE 监测读取 `/rosout` 或 launch 终端，不能期待 status --once 立即返回。

### 12.2 DI/DO（实际配置名由现场提供）

先把 `HIL_DI_NAME`、`HIL_DO_NAME` 设置为历史交接和现场确认的一路安全信号；名称模板不等于已批准该输出。以下简单 YAML 引号写法要求名称不含单引号。

```bash
: "${HIL_DI_NAME:?请先设置已确认的DI配置名}"
: "${HIL_DO_NAME:?请先设置已确认的安全DO配置名}"
ros2 service call /hikrobot_driver/get_digital_input hikrobot_msgs/srv/GetDigitalInput "{name: '${HIL_DI_NAME}'}"
ros2 service call /hikrobot_driver/get_digital_output hikrobot_msgs/srv/GetDigitalOutput "{name: '${HIL_DO_NAME}'}"
ros2 service call /hikrobot_driver/set_digital_output hikrobot_msgs/srv/SetDigitalOutput "{name: '${HIL_DO_NAME}', value: true}"
ros2 service call /hikrobot_driver/get_digital_output hikrobot_msgs/srv/GetDigitalOutput "{name: '${HIL_DO_NAME}'}"
ros2 service call /hikrobot_driver/set_digital_output hikrobot_msgs/srv/SetDigitalOutput "{name: '${HIL_DO_NAME}', value: false}"
ros2 service call /hikrobot_driver/get_digital_output hikrobot_msgs/srv/GetDigitalOutput "{name: '${HIL_DO_NAME}'}"
```

### 12.3 运动 Goal 模板和历史安全参数引用

`HIL_GOAL_DIR` 是**待用户提供/现场复核**的安全 Goal YAML 目录，不是仓库已存在目录。文件内容必须符合下面字段模板；速度/timeout/blend、关节角、via/target 取经批准历史 HIL 样本，不能直接搬用其它姿态/工具下的坐标。旧安全样本可追溯 `docs/hil/V0.2-03_BASE_CARTESIAN_AND_VISUAL_MOVEL_HIL_REPORT.md` 的 Base Z +10 mm 与返回测试，但那是旧版本特定起点/Tool；`tools/hil/visual_three_point_movel.py` 也不是 MoveS/Follow 目标已验证的证明。没有可由源码确认适用于本次现场的完整安全目标集。

模板中的大写标记是待替换值，**不是可执行 Goal 数值**；PoseStamped 的 header 可省 stamp（默认 0），必须填准确 frame_id。

```yaml
# move_j.yaml
joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
positions: [J1_RAD, J2_RAD, J3_RAD, J4_RAD, J5_RAD, J6_RAD]
speed_percent: APPROVED_PERCENT
blend_radius: APPROVED_BLEND
timeout: {sec: APPROVED_TIMEOUT_SEC, nanosec: 0}
```

所有 Action 都使用如下 Duration（与 speed_percent 同级）：

```yaml
timeout: {sec: APPROVED_TIMEOUT_SEC, nanosec: 0}
```

Cartesian pose 的展开模板：

```yaml
# POSE 表示这个完整映射；不是接口字段名
header: {frame_id: base_link}
pose:
  position: {x: APPROVED_X_M, y: APPROVED_Y_M, z: APPROVED_Z_M}
  orientation: {x: APPROVED_QX, y: APPROVED_QY, z: APPROVED_QZ, w: APPROVED_QW}
```

```yaml
# move_l.yaml：TARGET_POSE_MAPPING 替换为上述完整 PoseStamped 映射
target_pose: TARGET_POSE_MAPPING
speed_percent: APPROVED_PERCENT
blend_radius: APPROVED_BLEND
timeout: {sec: APPROVED_TIMEOUT_SEC, nanosec: 0}
```

```yaml
# move_c.yaml
via_pose: VIA_POSE_MAPPING
target_pose: TARGET_POSE_MAPPING
speed_percent: APPROVED_PERCENT
blend_radius: APPROVED_BLEND
timeout: {sec: APPROVED_TIMEOUT_SEC, nanosec: 0}
```

```yaml
# move_s.yaml
via_poses: [VIA1_POSE_MAPPING, VIA2_POSE_MAPPING]
target_pose: TARGET_POSE_MAPPING
speed_percent: APPROVED_PERCENT
blend_radius: APPROVED_BLEND
timeout: {sec: APPROVED_TIMEOUT_SEC, nanosec: 0}
```

```yaml
# follow_cartesian_path.yaml；1..32 个完整 PoseStamped，不含 via_poses/target_pose
poses: [TARGET1_POSE_MAPPING, TARGET2_POSE_MAPPING]
speed_percent: APPROVED_PERCENT
blend_radius: APPROVED_BLEND
timeout: {sec: APPROVED_TIMEOUT_SEC, nanosec: 0}
```

安全文件准备并复核后，一次只运行一个 Action，记录 Goal UUID、反馈、Result 与 HAR 状态：

```bash
: "${HIL_GOAL_DIR:?请先指定经复核的安全Goal YAML目录}"
ros2 action send_goal /hikrobot_driver/move_j hikrobot_msgs/action/MoveJ "$(cat "$HIL_GOAL_DIR/move_j.yaml")" --feedback
ros2 action send_goal /hikrobot_driver/move_l hikrobot_msgs/action/MoveL "$(cat "$HIL_GOAL_DIR/move_l.yaml")" --feedback
ros2 action send_goal /hikrobot_driver/move_c hikrobot_msgs/action/MoveC "$(cat "$HIL_GOAL_DIR/move_c.yaml")" --feedback
ros2 action send_goal /hikrobot_driver/move_s hikrobot_msgs/action/MoveS "$(cat "$HIL_GOAL_DIR/move_s.yaml")" --feedback
ros2 action send_goal /hikrobot_driver/follow_cartesian_path hikrobot_msgs/action/FollowCartesianPath "$(cat "$HIL_GOAL_DIR/follow_cartesian_path.yaml")" --feedback
```

### 12.4 Stop、Action Cancel、退出

```bash
ros2 service call /hikrobot_driver/stop_motion hikrobot_msgs/srv/StopMotion '{}'
# 标准 Action CancelGoal；零 UUID + 零 stamp = 取消该 Action server 的所有 goal
ros2 service call /hikrobot_driver/follow_cartesian_path/_action/cancel_goal action_msgs/srv/CancelGoal '{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}'
# MoveJ/L/C/S：仅把以上服务路径中的 follow_cartesian_path 换成 move_j/move_l/move_c/move_s
ros2 lifecycle set /hikrobot_driver deactivate
ros2 lifecycle get /hikrobot_driver
ros2 lifecycle set /hikrobot_driver cleanup
ros2 lifecycle get /hikrobot_driver
```

Cancel 从另一个终端执行，不把结束 ros2 CLI/Ctrl-C 当作保证已取消机器人。CancelGoal 返回接受取消仍要等待 Action result 与新停机状态；StopMotion success 只证明调用结果。Recovery 时 Lifecycle 迁移成功不代表电机/模式/Handle 已被硬件释放，按 fault/message 及控制器现场状态判定。

## 13. 当前离线测试证据与覆盖缺口

现有日志 `log/test_2026-10-16_13-42-51/{hikrobot_mpsdk_core,hikrobot_driver}/stdout.log`：core 5/5、driver 2/2 CTest 通过；当前 7 份有效 GTest XML 均 timestamp `2026-10-16T13:44:22`、failures/errors=0，总 **97** 用例。CTest stdout 时长约 0.63 s + 0.12 s；日志目录名是运行开始，XML timestamp 是最后文件记录，勿强行等同。

| CTest 名称 | GTest 数 | 覆盖 |
|---|---:|---|
| test_tool_transform | 16 | mm/m、quaternion、flange×tool、工具范围、逐段平移/旋转保护与 TCP 误差 |
| test_command_channel_health | 8 | transport 范围、断连重连保持锁、watchdog、HAR 超时、显式 Service deadline poison |
| test_legacy_io_dispatch | 5 | GetDI/GetDO、历史特殊名称普通读取、Set高/低一次调用、不重试错误 |
| test_io_command_executor | 7 | 单 worker 串行、queued 超时、等待共享 SDK mutex 取消、running 超时及后续拒绝、pause 已取出未开始任务、正常析构 join |
| test_legacy_movel_queue_executor | 41 | FakeBackend：1/10/20/32窗口补队列、重复/倒序回调、失败/碰撞/取消/Stop、持久化与耗尽、终态证据、HAR去重、零target兼容、独立停止确认窗口 |
| test_cartesian_contract | 14 | frame/数值/quaternion/单位/工具/Base 合同、Duration校验 |
| test_health_monitor | 6 | 纯 HealthTransitionTracker 转换：启动、断连/HAR/急停/超时/禁监测状态 |

源码和 CTest 注册一致：core CMakeLists.txt 5 个、driver 2 个。`test_protected_arc_output.gtest.xml` 是已删除目标遗留，不能计入有效测试（粗扫 build 可能给出更大数量）。其它相机包的 lint/旧工作区结果不计入 Driver 97 项。

`tests/test_sequence_manager.cpp` 是独立 assert main，覆盖同文件重建 manager 后继续高水位、边界耗尽、持久值不回退；**未注册进当前 package CTest，也不在 97 内**，不能仅凭源码存在宣称这次运行通过。

有 FakeBackend/注入函数测试，**没有替代真实 MPSDK 动态库的完整 Fake SDK，也没有实例化 RobotRuntime 的端到端集成测试或 ROS Service 有界返回集成测试**。IO executor 用受控假调用阻塞后主动解除来完成测试，不证明永久阻塞时析构可返回；Health tests 不驱动真实 HAR/connection callback 或 SDK mutex/生命周期竞争。普通 MoveJ/L/C/S 的 Runtime Add/Start 完成链也未被该测试集完整执行。

下一阶段可重新验证（会写 build/log，只作为命令交接）：

```bash
colcon test --packages-select hikrobot_mpsdk_core hikrobot_driver --event-handlers console_direct+
ctest --test-dir build/hikrobot_mpsdk_core -N
ctest --test-dir build/hikrobot_driver -N
# 查看有效7份 XML；若使用 colcon test-result --verbose，必须识别旧 protected_arc_output 等遗留结果
colcon test-result --verbose
```

## 14. 已知限制、潜在问题与复现条件（只记录，不修复）

| 定位 | 风险 / 复现条件 | HIL 关注 |
|---|---|---|
| bringup/config/hikrobot_driver.yaml:33；driver/src/hikrobot_driver_node.cpp:113 | YAML 0.600 m 与注释50mm/Node默认0.05不符；按默认 launch 加载即出现 | 测前以 param dump 核实现场允许保护值，本文未改配置 |
| core/src/io_command_executor.cpp:36,54；robot_runtime.cpp:541,808 | 永久 Vendor IO 阻塞后服务可超时但析构 join 卡住；Recovery 保留 Handle | 不以 graceful exit 当已保证；输出可能已动作 |
| core/src/robot_runtime.cpp:298,832；driver/src/hikrobot_driver_node.cpp:416,463 | 非 IO 同步 SDK 调用、lifecycle mutex/join 可无限等待；Configure未纳入 begin_call监测 | Stop/Lifecycle/Tool不承诺IO同样的有界返回 |
| core/src/robot_runtime.cpp:541,569,578,584,589 | cleanup 正常分支仅在 deactivate 后再次整体查 Recovery；后续 unsubscribe/reset/disconnect 返回值多被忽略，若中途 poison，后续 call 可被拒绝但 flags/handle 仍清理 | 故障注入时核查真释放；不要只看 Lifecycle SUCCESS |
| core/src/robot_runtime.cpp:550,808；SDK callbacks:224,243,263 | Recovery保留 Handle/回调；若没有阻塞worker使析构等待而对象先销毁，SDK之后回调可能触达已释放 pUserData（静态风险，未实证） | 进程替换与 callback lifetime 未被 integration tests 证明 |
| core/src/robot_runtime.cpp:209,1087,1387 | 生命周期普通字段/handle 与服务并发读写缺少统一生命周期互斥；Executor pause 保护IO执行，但入队前检查/直接 Tool或stop/list方法并未全被同一门禁覆盖 | cleanup 与服务/运动并发需受控测试，不能笼统声称无race |
| core/src/motion_sequence_manager.cpp:28,41,97 | 文件坏/不可读会 reseed；无跨进程锁、固定tmp；父目录fsync错误忽略 | 单实例，保留真实文件，异常不能回退重置 |
| core/src/robot_runtime.cpp:770 | 普通运动stable按轮询累计，可重复HAR，不要求post-start运动证据 | 同点、慢HAR/临界stale、Start后Stop时序回归 |
| core/src/robot_runtime.cpp:714；driver/src/hikrobot_driver_node.cpp:572,766 | 普通运动无Follow的Start/Stop generation握手；accepted worker 会重置共享cancel标志，极早Cancel/Stop有时序窗口（静态风险） | 接受后立即Cancel/Stop，捕获是否仍Start；不得只测执行中取消 |
| core/src/legacy_movel_queue_executor.cpp:686 | 补队列不显式要求matching PathID，假/迟到not-stopped回调可能驱动Add | 记录原始callback/queued_targets，关注迟到回调 |
| core/src/robot_runtime.cpp:1332；driver/src/hikrobot_driver_node.cpp:537 | core MoveS via容量允许到50，ROS仅49；SDK注释49 | ROS接口遵从49，直接core另有边界风险 |
| driver/src/hikrobot_driver_node.cpp:644,1245 | Tool刷新到Add存在控制器改Tool窗口；发布用缓存、joint_states不拒stale、Pose时间戳为发布时间 | 锁定现场Tool配置；一起看status stale，禁止只看消息有更新 |
| core/src/io_command_executor.cpp:200；driver/src/hikrobot_driver_node.cpp:768 | worker operation没有统一异常边界；普通运动worker也无Follow那样完整try/catch | 内存/函数异常容错尚未证明 |
| scripts/core_compile_check.sh:10 | 旧脚本只编译sequence/runtime，漏新logic源且会rm其专用构建目录，不能证明当前完整core可链接 | 本次未执行；使用colcon/CMake有效目标 |

上表“静态风险”不等于已复现缺陷；应保留触发条件、现场日志和版本再下结论。共享 SDK mutex、command Recovery、IO Worker 生命周期、health HAR 失联判定、Node callback group 改动会影响所有运动和 Lifecycle；**即使 MoveJ/L/S/Follow/PathID 的具体 Add 源码没改，也存在当前版本回归风险**。

## 15. 真机前必须确认与历史证据边界

1. 保存当前分支/HEAD/全部未提交文件和本文 hash；核对运行 prefix、core 动态库解析路径、SDK版本/Controller固件/机器人型号。不要只复制已跟踪文件或 checkout HEAD。
2. 提供另一个对话的 Motion HIL 历史交接，核对 MoveS/MoveC/Follow 的真实 PASS 与版本；未提供前保持待验证。仓库 `docs/hil/V0.1_to_V0.2-02_HIL_REPORT.md` 有旧 MoveJ/生命周期/PathID/Tool/TCP 历史 PASS，`V0.2-03_BASE_CARTESIAN_AND_VISUAL_MOVEL_HIL_REPORT.md` 有旧 Base TCP MoveL往返/视觉分段记录，并明确 MoveC/MoveS Pending、IO deferred；不能提升为本轮通过。
3. `docs/handoff/CURRENT_HANDOFF.md` 的2026-09-14队列记录说明旧20点执行到第9段失败，修复后20/32点连续性仍待HIL；其旧Git/测试计数与本篇冲突时，以当前源码/本篇快照为准。
4. 现场确认安全输出的物理负载、实际DI/DO名称、工具ID/安装位/标定、Base frame、起始姿态、全部目标及返回路径；不自动复用旧坐标。先解决600mm/50mm配置理解差异。
5. 真实PathID文件必须保留，确认唯一Driver进程、运行用户/HOME与文件路径/权限，无清零/回退/多实例；本文未读取高水位，不可假定剩余容量。
6. 准备独立硬件停机手段，命令通道Recovery时软件Stop可能拒绝；明确当前运行结果未知的DO不能自动重发；从控制器确认实际电机/模式/IO状态。
7. 记录 `/rosout`、status、joint_states、flange/tcp、diagnostics、Action全Feedback/Result/UUID与时间；未激活时以日志观察health，别误认无Topic=无HAR。

## 16. 最终 HIL 建议回归范围与验收记录

每项记录“工作区hash/库/SDK/固件、输入、关键反馈、最终硬件状态、ROS状态+自定义result、结论”。当前统一标记 **本版本未验**，按现场由低风险到动作逐步执行：

| 范围 | 建议覆盖与PASS依据 |
|---|---|
| Lifecycle正常 | 初启unconfigured、Configure首帧/Tool、INACTIVE只读、Activate精确SDK链、Deactivate保留监测、Cleanup释放，再重配；核对硬件状态 |
| Health | ACTIVE与INACTIVE稳定、HAR断流/恢复、connection断连/重连、急停置位/释放、watchdog；日志不刷屏、Recovery不自动解锁、ROS/core状态区分 |
| IO正常 | DI、DO读，高/低Set各一次及显式Get，历史特殊名称高电平可Get；无-10004，无自动Set重试 |
| IO异常 | 等锁超时未执行、入SDK超时unknown、后续failfast、排队取消与已取出未开始取消、cleanup/析构边界；本地受控假环境先做永久阻塞，不能随意让现场输出失控 |
| MoveJ/L/C/S | 经复核小目标与返回、单位/Tool/frame/速度/blend、非法输入拒绝、busy，正常Result、Cancel/Stop、超时/stale/急停；普通运动无严格Stop确认需额外核实物理停止 |
| FollowQueue | 1/10/20/32目标，窗口补充、单Start、全部PathID、running/final证据、zero-target sentinel、HAR独立样本；全程连续性不能只看终点误差 |
| Follow异常 | 接受即Cancel、Add间Cancel、补队列Cancel、Stop抑制Start、underflow、refill错误/碰撞不重播、Stop确认失败Recovery、独立stop窗口 |
| PathID | 同进程多命令、正常进程重启继续高水位、碰撞有界重试且不回退；耗尽/损坏/持久失败优先在隔离临时文件软件测试，禁止动真文件 |
| 并发/回归 | IO与运动/Tool争共享mutex、health持续可调度、Lifecycle与服务受控竞争、故障后禁止新动作；不能因运动文件未改跳过 |

## 17. 当前 Driver HIL 测试入口速查表

| 入口 | 最短定位 / 用法 |
|---|---|
| 启动 | `ros2 launch hikrobot_bringup hikrobot_driver.launch.py config_file:=/home/ai/hikrobot_ros2/src/hikrobot_bringup/config/hikrobot_driver.yaml` |
| Lifecycle | `ros2 lifecycle get /hikrobot_driver`；`ros2 lifecycle set /hikrobot_driver configure` / activate / deactivate / cleanup |
| 机器人/HAR派生 | `/hikrobot_driver/status`、`/joint_states`（仅ACTIVE发布） |
| Flange/TCP | `/hikrobot_driver/flange_pose`、`/hikrobot_driver/tcp_pose`；都是Base frame |
| Tool | `/hikrobot_driver/get_active_tool`、`get_tool_info_list`，请求 `{}` |
| IO | `get_digital_input`、`get_digital_output` 请求 `{name: 实际配置名}`；`set_digital_output` 另加 `value` |
| 单次运动 | move_j / move_l / move_c / move_s，类型见第3节，安全Goal文件引用见第12节 |
| 连续队列 | follow_cartesian_path，1..32 TCP poses，关注完整诊断Feedback |
| Stop/Cancel | stop_motion `{}`；`<Action>/_action/cancel_goal`，准确请求见12.4 |
| 故障定位 | `/rosout`、`/diagnostics`、status last_sdk_error/message、Result failure_stage |
| 离线入口 | core 5 + driver 2 CTest，97有效GTest；RobotRuntime/ROS集成测试缺失 |
| 首要测前项 | 600mm配置差异、完整未提交源码、Motion HIL历史交接、真实Tool/IO/目标、PathID唯一实例 |

## 附录 A. 当前接口文件原文

以下从工作区直接摘录；`---` 分隔 Goal/Result/Feedback 或 Request/Response，所有常量与字段以此为准。

### `src/hikrobot_msgs/action/FollowCartesianPath.action`

```text
# Ordered Base-frame Active-Tool TCP poses. Version 1 supports 1..32 Legacy MoveL targets.
geometry_msgs/PoseStamped[] poses
uint8 speed_percent
uint8 blend_radius
builtin_interfaces/Duration timeout
---
uint8 SUCCESS=0
uint8 INVALID_GOAL=1
uint8 DRIVER_NOT_READY=2
uint8 ROBOT_NOT_READY=3
uint8 BUSY=4
uint8 SDK_ERROR=5
uint8 TIMEOUT=6
uint8 CANCELED=7
uint8 STATE_LOST=8
uint8 result_code
int64 sdk_error_code
int32[] path_ids
uint8 STAGE_NONE=0
uint8 STAGE_VALIDATION=1
uint8 STAGE_ID_RESERVATION=2
uint8 STAGE_CLEAR=3
uint8 STAGE_ADD=4
uint8 STAGE_START=5
uint8 STAGE_EXECUTION=6
uint8 STAGE_STOP=7
uint8 failure_stage
bool failure_target_index_valid
uint32 failure_target_index
builtin_interfaces/Duration execution_time
string message
---
uint8 STAGE_VALIDATING=0
uint8 STAGE_RESERVING_IDS=1
uint8 STAGE_CLEARING_QUEUE=2
uint8 STAGE_QUEUEING=3
uint8 STAGE_READY_TO_START=4
uint8 STAGE_EXECUTING=5
uint8 STAGE_STOPPING=6
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

### `src/hikrobot_msgs/action/MoveC.action`

```text
geometry_msgs/PoseStamped via_pose
geometry_msgs/PoseStamped target_pose
uint8 speed_percent
uint8 blend_radius
builtin_interfaces/Duration timeout
---
uint8 SUCCESS=0
uint8 INVALID_GOAL=1
uint8 DRIVER_NOT_READY=2
uint8 ROBOT_NOT_READY=3
uint8 BUSY=4
uint8 SDK_ERROR=5
uint8 TIMEOUT=6
uint8 CANCELED=7
uint8 STATE_LOST=8
uint8 result_code
int64 sdk_error_code
int32 path_id
builtin_interfaces/Duration execution_time
string message
---
int32 path_id
float64 progress_percent
geometry_msgs/PoseStamped target_pose
geometry_msgs/PoseStamped actual_pose
float64 position_error_m
float64 orientation_error_rad
```

### `src/hikrobot_msgs/action/MoveJ.action`

```text
# Goal
string[] joint_names
float64[] positions
uint8 speed_percent
uint8 blend_radius
builtin_interfaces/Duration timeout
---
# Result
uint8 SUCCESS=0
uint8 INVALID_GOAL=1
uint8 DRIVER_NOT_READY=2
uint8 ROBOT_NOT_READY=3
uint8 BUSY=4
uint8 SDK_ERROR=5
uint8 TIMEOUT=6
uint8 CANCELED=7
uint8 STATE_LOST=8
uint8 result_code
int64 sdk_error_code
int32 path_id
builtin_interfaces/Duration execution_time
string message
---
# Feedback
int32 path_id
float64 progress_percent
float64[] target_positions
float64[] actual_positions
float64[] position_error
```

### `src/hikrobot_msgs/action/MoveL.action`

```text
geometry_msgs/PoseStamped target_pose
uint8 speed_percent
uint8 blend_radius
builtin_interfaces/Duration timeout
---
uint8 SUCCESS=0
uint8 INVALID_GOAL=1
uint8 DRIVER_NOT_READY=2
uint8 ROBOT_NOT_READY=3
uint8 BUSY=4
uint8 SDK_ERROR=5
uint8 TIMEOUT=6
uint8 CANCELED=7
uint8 STATE_LOST=8
uint8 result_code
int64 sdk_error_code
int32 path_id
builtin_interfaces/Duration execution_time
string message
---
int32 path_id
float64 progress_percent
geometry_msgs/PoseStamped target_pose
geometry_msgs/PoseStamped actual_pose
float64 position_error_m
float64 orientation_error_rad
```

### `src/hikrobot_msgs/action/MoveS.action`

```text
geometry_msgs/PoseStamped[] via_poses
geometry_msgs/PoseStamped target_pose
uint8 speed_percent
uint8 blend_radius
builtin_interfaces/Duration timeout
---
uint8 SUCCESS=0
uint8 INVALID_GOAL=1
uint8 DRIVER_NOT_READY=2
uint8 ROBOT_NOT_READY=3
uint8 BUSY=4
uint8 SDK_ERROR=5
uint8 TIMEOUT=6
uint8 CANCELED=7
uint8 STATE_LOST=8
uint8 result_code
int64 sdk_error_code
int32 path_id
builtin_interfaces/Duration execution_time
string message
---
int32 path_id
float64 progress_percent
geometry_msgs/PoseStamped target_pose
geometry_msgs/PoseStamped actual_pose
float64 position_error_m
float64 orientation_error_rad
```

### `src/hikrobot_msgs/srv/GetActiveTool.srv`

```text
---
bool success
int64 sdk_error_code
hikrobot_msgs/ToolInfo tool
string message
```

### `src/hikrobot_msgs/srv/GetDigitalInput.srv`

```text
string name
---
bool success
bool value
int64 sdk_error_code
string message
```

### `src/hikrobot_msgs/srv/GetDigitalOutput.srv`

```text
string name
---
bool success
bool value
int64 sdk_error_code
string message
```

### `src/hikrobot_msgs/srv/GetToolInfoList.srv`

```text
---
bool success
int64 sdk_error_code
hikrobot_msgs/ToolInfo[] tools
string message
```

### `src/hikrobot_msgs/srv/SetDigitalOutput.srv`

```text
string name
bool value
---
bool success
int64 sdk_error_code
string message
```

### `src/hikrobot_msgs/srv/StopMotion.srv`

```text
---
bool success
int64 sdk_error_code
string message
```

### `src/hikrobot_msgs/msg/RobotStatus.msg`

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

### `src/hikrobot_msgs/msg/ToolInfo.msg`

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

## 附录 B. 当前 YAML 原文

```yaml
hikrobot_driver:
  ros__parameters:
    robot_ip: "192.168.2.64"
    robot_port: 9000
    har_publish_ms: 50
    # IO Services wait this long to enter the SDK, then this long after SDK entry.
    # The single IO worker returns the Service on timeout, but cannot cancel a
    # vendor call already blocked inside the SDK.
    sdk_command_lock_timeout_ms: 1000
    sdk_command_watchdog_ms: 3000
    # Purely passive runtime health monitoring: connection callback, HAR cache,
    # and command-channel watchdog. It never calls the vendor SDK.
    health_monitor:
      enabled: true
      check_period_ms: 1000
      communication_timeout_ms: 3000

    joint_names: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    base_frame: "base_link"
    tool_frame: "tool0"

    # Password itself is NOT stored here.
    programmer_password_env: "HIKROBOT_PROGRAMMER_PASSWORD"

    # Empty => $HOME/.ros/hikrobot/path_id.state
    sequence_state_file: ""
    sequence_collision_retries: 3

    default_timeout_sec: 30.0
    joint_tolerance_rad: 0.0034906585   # 0.2 deg
    position_tolerance_m: 0.001
    orientation_tolerance_rad: 0.0174532925  # 1 deg
    max_cartesian_translation_step_m: 0.600  # 50 mm initial-HIL guard
    max_cartesian_rotation_step_rad: 0.1745329252  # 10 deg initial-HIL guard
    stable_samples: 5
    state_stale_timeout_ms: 1000
    # Independent bounded window used after a path stop request; it does not extend Action timeout.
    stop_confirmation_timeout_ms: 3000

    # Retained compatibility defaults. tool_id does not select the Tool for Cartesian Actions;
    # MoveL/MoveC/MoveS use the controller Active Tool with Base coordinate 1:0.
    tool_id: 0
    workpiece_id: -1
    state_publish_rate_hz: 20.0
```

## 附录 C. 当前有效测试逐项名称

名称来自当前有效目标的测试XML；完整源码位于两包 `tests/`。

### test_tool_transform

- `hikrobot_mpsdk_core.ToolTransform.ConvertsMillimetresAndNormalizesQuaternion`
- `hikrobot_mpsdk_core.ToolTransform.IdentityToolReturnsFlange`
- `hikrobot_mpsdk_core.ToolTransform.IdentityFlangeReturnsTool`
- `hikrobot_mpsdk_core.ToolTransform.RotatesToolTranslationBeforeAdding`
- `hikrobot_mpsdk_core.ToolTransform.ToolTwoRegressionComposition`
- `hikrobot_mpsdk_core.ToolTransform.RejectsInvalidCacheAndExternalMount`
- `hikrobot_mpsdk_core.ToolTransform.ValidatesToolBoundsAndBoundedStrings`
- `hikrobot_mpsdk_core.CartesianStep.AcceptsTenMillimetreTranslation`
- `hikrobot_mpsdk_core.CartesianStep.RejectsTranslationOverFiftyMillimetres`
- `hikrobot_mpsdk_core.CartesianStep.RejectsRotationOverTenDegrees`
- `hikrobot_mpsdk_core.CartesianStep.QuaternionSignDoesNotChangeAngularDistance`
- `hikrobot_mpsdk_core.CartesianStep.ChecksEveryMoveCAndMoveSSegment`
- `hikrobot_mpsdk_core.CartesianStep.RejectsNonPositiveLimits`
- `hikrobot_mpsdk_core.CartesianStep.RejectsNonFiniteOrZeroQuaternionTransforms`
- `hikrobot_mpsdk_core.CartesianFeedback.UsesTcpInsteadOfFlangeForActualAndError`
- `hikrobot_mpsdk_core.CartesianFeedback.ToolTwoOffsetChangesCompletionError`

### test_command_channel_health

- `hikrobot_mpsdk_core.CommandChannelHealthTest.ClassifiesTheOfficialSdkNetworkErrorRange`
- `hikrobot_mpsdk_core.CommandChannelHealthTest.ReturnedTransportErrorRequiresProcessRecovery`
- `hikrobot_mpsdk_core.CommandChannelHealthTest.WatchdogPoisonsUnknownOutcomeWithoutEndingCall`
- `hikrobot_mpsdk_core.CommandChannelHealthTest.DisconnectCallbackStaysPoisonedAfterReconnectCallback`
- `hikrobot_mpsdk_core.CommandChannelHealthTest.CleanDisconnectDoesNotCreateRecoveryRequirement`
- `hikrobot_mpsdk_core.CommandChannelHealthTest.FeedbackTimeoutRequiresRecovery`
- `hikrobot_mpsdk_core.CommandChannelHealthTest.FeedbackTimeoutDoesNotReplaceEarlierCommandTimeout`
- `hikrobot_mpsdk_core.CommandChannelHealthTest.ServiceDeadlineCanPoisonAnInFlightCallDirectly`

### test_legacy_io_dispatch

- `hikrobot_mpsdk_core.LegacyIoDispatchTest.GetDigitalInputCallsSdkOnce`
- `hikrobot_mpsdk_core.LegacyIoDispatchTest.ProtectedNameIsAnOrdinaryDigitalOutputRead`
- `hikrobot_mpsdk_core.LegacyIoDispatchTest.SetHighCallsOnlySetOnce`
- `hikrobot_mpsdk_core.LegacyIoDispatchTest.SetLowCallsOnlySetOnce`
- `hikrobot_mpsdk_core.LegacyIoDispatchTest.SdkFailureIsReturnedWithoutRetry`

### test_io_command_executor

- `hikrobot_mpsdk_core.IoCommandExecutorTest.NormalGetAndSetStyleTasksReturnValues`
- `hikrobot_mpsdk_core.IoCommandExecutorTest.MultipleRequestsUseExactlyOneSerialWorker`
- `hikrobot_mpsdk_core.IoCommandExecutorTest.QueuedTimeoutCancelsTaskBeforeItCanExecute`
- `hikrobot_mpsdk_core.IoCommandExecutorTest.SharedSdkMutexWaitCanBeCancelledBeforeSdkEntry`
- `hikrobot_mpsdk_core.IoCommandExecutorTest.RunningSdkTimeoutPoisonsAndLaterRequestsFailFast`
- `hikrobot_mpsdk_core.IoCommandExecutorTest.PauseCancelsDequeuedButNotStartedTaskSafely`
- `hikrobot_mpsdk_core.IoCommandExecutorTest.NormalDestructionJoinsWorker`

### test_legacy_movel_queue_executor

- `hikrobot_mpsdk_core.QueueExecutorTest.RejectsInvalidRequestsWithoutSdkOrIdConsumption`
- `hikrobot_mpsdk_core.QueueExecutorTest.RejectsInvalidToolAndBusyOrRecoveryStates`
- `hikrobot_mpsdk_core.QueueExecutorTest.PersistsUniqueCompleteIdBlockBeforeAddsAndStartsOnce`
- `hikrobot_mpsdk_core.QueueExecutorTest.OneTargetUsesOneInitialAddAndNoRefill`
- `hikrobot_mpsdk_core.QueueExecutorTest.TenTargetsFillInitialWindowAndStartOnce`
- `hikrobot_mpsdk_core.QueueExecutorTest.TwentyTargetsRefillOneControllerWindowWithOneStart`
- `hikrobot_mpsdk_core.QueueExecutorTest.ThirtyTwoTargetsStayWithinWindowAndStartOnce`
- `hikrobot_mpsdk_core.QueueExecutorTest.RepeatedMotionCallbackSequenceCannotRepeatRefill`
- `hikrobot_mpsdk_core.QueueExecutorTest.OutOfOrderMotionCallbackCannotRepeatRefill`
- `hikrobot_mpsdk_core.QueueExecutorTest.RefillAddFailureStopsWithoutRetryingOrSecondStart`
- `hikrobot_mpsdk_core.QueueExecutorTest.RefillCollisionDoesNotAllocateNewIdsOrRetryPath`
- `hikrobot_mpsdk_core.QueueExecutorTest.CancelDuringRefillStopsWithoutSubmittingLaterTargets`
- `hikrobot_mpsdk_core.QueueExecutorTest.PrematureStoppedCallbackReportsQueueUnderflowImmediately`
- `hikrobot_mpsdk_core.QueueExecutorTest.ExhaustionAndPersistenceFailureOccurBeforeAnySdkCall`
- `hikrobot_mpsdk_core.QueueExecutorTest.AppliesOneToolSnapshotAndBaseOptionsToEveryAdd`
- `hikrobot_mpsdk_core.QueueExecutorTest.PartialAddFailureClearsQueueAndNeverStarts`
- `hikrobot_mpsdk_core.QueueExecutorTest.CleanupFailureBlocksSubsequentMotion`
- `hikrobot_mpsdk_core.QueueExecutorTest.CollisionRetriesWholeQueueWithFreshPersistedIds`
- `hikrobot_mpsdk_core.QueueExecutorTest.StartFailureIsStoppedNotRetriedAndRequiresRecovery`
- `hikrobot_mpsdk_core.QueueExecutorTest.CancellationBeforeWorkerConsumesNoIds`
- `hikrobot_mpsdk_core.QueueExecutorTest.CancellationBetweenAddsClearsAndNeverStarts`
- `hikrobot_mpsdk_core.QueueExecutorTest.StopAfterLastAddSuppressesStart`
- `hikrobot_mpsdk_core.QueueExecutorTest.ExecutingCancelRequestsAndConfirmsStop`
- `hikrobot_mpsdk_core.QueueExecutorTest.StaleHarStopsExecution`
- `hikrobot_mpsdk_core.QueueExecutorTest.WrongPathCallbackCannotProveCompletion`
- `hikrobot_mpsdk_core.QueueExecutorTest.RepeatedHarSampleCannotSupplyStableSamples`
- `hikrobot_mpsdk_core.QueueExecutorTest.MissingTerminalQueueEvidenceCannotSucceedAtFinalPose`
- `hikrobot_mpsdk_core.QueueExecutorTest.Controller212ZeroTargetSentinelCompletesAfterRunningEvidence`
- `hikrobot_mpsdk_core.QueueExecutorTest.ZeroTargetSentinelWithoutRunningEvidenceCannotComplete`
- `hikrobot_mpsdk_core.QueueExecutorTest.ZeroTargetSentinelMustBeNewerThanRunningCallback`
- `hikrobot_mpsdk_core.QueueExecutorTest.WrongNonzeroFinalTargetCannotComplete`
- `hikrobot_mpsdk_core.QueueExecutorTest.ExpiredMotionDeadlineUsesIndependentStopConfirmationWindow`
- `hikrobot_mpsdk_core.QueueExecutorTest.StopConfirmationTimeoutSetsRecoveryLockWithoutUnboundedWait`
- `hikrobot_mpsdk_core.QueueExecutorTest.StopSdkFailurePreservesErrorAndSetsRecoveryLock`
- `hikrobot_mpsdk_core.QueueExecutorTest.CachedStoppedCallbackBeforeStopCannotConfirmNewRequest`
- `hikrobot_mpsdk_core.QueueExecutorTest.FreshStoppedCallbackWithWrongPathCannotConfirmStop`
- `hikrobot_mpsdk_core.QueueExecutorTest.FreshMatchingStoppedCallbackConfirmsStop`
- `hikrobot_mpsdk_core.QueueExecutorTest.MatchingStoppedCallbackDuringBlockingStopCallConfirmsStop`
- `hikrobot_mpsdk_core.QueueExecutorTest.MotionCallbackSequenceAndSnapshotDiagnosticsAreConsistent`
- `hikrobot_mpsdk_core.QueueExecutorTest.HarSequenceNotTimestampDrivesStableSampleCounting`
- `hikrobot_mpsdk_core.QueueExecutorTest.CompletionDiagnosticsIdentifyEachBlockingCondition`

### test_cartesian_contract

- `hikrobot_driver.CartesianContract.AcceptsBaseFrameAndConvertsMetresToMillimetres`
- `hikrobot_driver.CartesianContract.RejectsEmptyFrame`
- `hikrobot_driver.CartesianContract.RejectsNonBaseFrame`
- `hikrobot_driver.CartesianContract.RejectsNonFinitePosition`
- `hikrobot_driver.CartesianContract.RejectsPositionOutsideSdkNumericRange`
- `hikrobot_driver.CartesianContract.RejectsNonFiniteQuaternion`
- `hikrobot_driver.CartesianContract.RejectsZeroQuaternion`
- `hikrobot_driver.CartesianContract.NormalizesQuaternionAndConvertsToZyxEulerDegrees`
- `hikrobot_driver.CartesianContract.SafelyNormalizesVeryLargeFiniteQuaternion`
- `hikrobot_driver.CartesianContract.ValidatesNormalizedOrZeroDuration`
- `hikrobot_driver.CartesianContract.UsesControllerActiveToolAndBaseCoordinate`
- `hikrobot_driver.CartesianContract.RejectsInvalidToolCache`
- `hikrobot_driver.CartesianContract.RejectsExternalMountAndOutOfRangeId`
- `hikrobot_driver.CartesianContract.MoveJOptionsAreOutsideCartesianContract`

### test_health_monitor

- `hikrobot_driver.HealthTransitionTrackerTest.ReportsHealthyStartupOnlyOnce`
- `hikrobot_driver.HealthTransitionTrackerTest.DistinguishesDisconnectAndReconnect`
- `hikrobot_driver.HealthTransitionTrackerTest.ReportsHarTimeoutTransitionWithoutLogSpam`
- `hikrobot_driver.HealthTransitionTrackerTest.UsesExplicitEmergencyState`
- `hikrobot_driver.HealthTransitionTrackerTest.DistinguishesCommandTimeoutFromGenericRecovery`
- `hikrobot_driver.HealthTransitionTrackerTest.UnmonitoredLifecycleStateResetsTransitions`

## 附录 D. 当前源码/配置/接口 SHA256 清单

只包含8个package里的源码/头/接口/CMake/package.xml/launch/YAML及根测试、构建脚本；不读SDK日志、密码或真实PathID文件。用于下个对话与本次工作区对齐；本文自身不列入以避免循环hash。

```text
ef0fd75dcf7313022720bc022b9067c6b0c62a086170670d11e86ba36d736b1f  scripts/build_humble.sh
ea4a94f85f850686c450e4831ac4035ba229379a79c16ffd02c4942063c4cdad  scripts/core_compile_check.sh
f286c21bf3de12b10d32b87af71f8cb23c589c6bb0c49b48cc212690593ad76e  src/hikrobot_bringup/CMakeLists.txt
ae4e075bca3966e3e59677fec894512636f4008f6bc10fcb468509aaea12f14f  src/hikrobot_bringup/config/hikrobot_driver.yaml
1a006c440b702852def6a1e903172b1084ec260f844e06499040e2bdf6d905cc  src/hikrobot_bringup/launch/hikrobot_driver.launch.py
a92dc9a1419880b13567ef11d49e9a345140b4aae9f0f800a5dcfd532e984fcf  src/hikrobot_bringup/package.xml
d88b00fd73e5a1dd4ce10f706c7034871820a853f4aee63ba5ac75020851d560  src/hikrobot_description/CMakeLists.txt
5e57f2ceb62c0adfec62e88c6ed680aab5254285ac6f9ac60d2fa30f98b47d6b  src/hikrobot_description/package.xml
b6dc3c970542f635e36907d840f9c071274ba4fbcc857ea54550d8dca67b7241  src/hikrobot_driver/CMakeLists.txt
97e7e1dc40d79ae13e85a91c72ba165031892c33034b82a970197c5fda6de373  src/hikrobot_driver/include/hikrobot_driver/cartesian_contract.hpp
9204b48663bcb2f9072b5023b9471b747cabfde7de6d5b8284ab7b7d090fec35  src/hikrobot_driver/include/hikrobot_driver/health_monitor.hpp
76ba47c0f6ead4d95f82b4d532409bde944be8c82c6f4b33597d0ea3e1aa088f  src/hikrobot_driver/include/hikrobot_driver/hikrobot_driver_node.hpp
98b67e5bca801e536944f7ab1a168bb48129622ab14b05de364daed6a7e66771  src/hikrobot_driver/package.xml
df1270bdc7ac27b57a6e2c2ca3ada4144438e9e8c2021323ea893f4bf508f70d  src/hikrobot_driver/src/cartesian_contract.cpp
402f59f44ddab1f662adbae0780328ac14bfec0c0b87bbcd20c56649a58b2749  src/hikrobot_driver/src/health_monitor.cpp
357f287b1a950792e9440403877857a12853f980d3e9060391df5468099253af  src/hikrobot_driver/src/hikrobot_driver_node.cpp
8d74f50f2cfab3a76ddfc8165aa0c979fb656c521c492ffb97a44add9a86a3c3  src/hikrobot_driver/src/main.cpp
417728d65eea930ef55a275dd6d1325d3ac786ebdf5b52541e7acf43372d45fa  src/hikrobot_driver/tests/test_cartesian_contract.cpp
fbe4e63010318501fff4c6038a3915753c24eabfa5238319515743381a6d769b  src/hikrobot_driver/tests/test_health_monitor.cpp
66c21c0112643c9dc0a5c63f6bceffaa7f9f8574165ac0de6f155ffaa5b1e8d6  src/hikrobot_mpsdk_core/CMakeLists.txt
16a546d753bd9da2faa1c56fb41822d141b5c6408cc9709655319c12653989ff  src/hikrobot_mpsdk_core/include/hikrobot_mpsdk_core/command_channel_health.hpp
2d6b55f644ac7e55f2ae42be1adb05790d21ef254fabaacfbf2a1ab81bbbd553  src/hikrobot_mpsdk_core/include/hikrobot_mpsdk_core/io_command_executor.hpp
6091a018e4d871727ac84552cc2c3b2c2e596ce3fbde9bfdc96ec554ab330c40  src/hikrobot_mpsdk_core/include/hikrobot_mpsdk_core/legacy_io_dispatch.hpp
ae6378432a27c495cdc29e06bbc027396149ed9027633f052c615bc72deaac44  src/hikrobot_mpsdk_core/include/hikrobot_mpsdk_core/legacy_movel_queue_executor.hpp
a02bfce6fd310b5f9ba0772d696ec5dec3302b42dd338350094d3de086fbdcc0  src/hikrobot_mpsdk_core/include/hikrobot_mpsdk_core/motion_sequence_manager.hpp
5cb4f5f5a39ccab86e13ff5aac67290723c19f0f64e374423490ac142820182a  src/hikrobot_mpsdk_core/include/hikrobot_mpsdk_core/robot_runtime.hpp
c4cdaa0144e4d73f57bc1f4d32c805f64b12fd5a0f545428f0926ece65ceb4c2  src/hikrobot_mpsdk_core/include/hikrobot_mpsdk_core/types.hpp
47f5b5a4c46c7f4bee1145dda8bdb5450a8bf6fb4bd38b5ea00ac1cd5fb561a9  src/hikrobot_mpsdk_core/package.xml
8bc8aab7e85693de18189f6c40ae3ab7522afac3d2e3212ce812ec688405fa8a  src/hikrobot_mpsdk_core/src/command_channel_health.cpp
9c318c38c2409867aa259a6d2397b7babaae90154c7a69ca4855d3173f0e0c0b  src/hikrobot_mpsdk_core/src/io_command_executor.cpp
73110dd14c95264dbc6a596c1cde578cf8160677867aaa723cc76d48035fba22  src/hikrobot_mpsdk_core/src/legacy_io_dispatch.cpp
93d898fad931643f91ebd824f8c0b6b668a3bdc99b0c27f949133fa8727979ab  src/hikrobot_mpsdk_core/src/legacy_movel_queue_executor.cpp
7450d6107ca678cf61a3ef289bf61f85056701c05398c31965bb50049c69ade5  src/hikrobot_mpsdk_core/src/motion_sequence_manager.cpp
1f275d702e096343f3943f74ec39a4d005ba91839bfb1368066b7e03c4e96589  src/hikrobot_mpsdk_core/src/robot_runtime.cpp
3c2424505b6778e790cc32c3ea2faaf278b26476572963e0341cb91d406cdcea  src/hikrobot_mpsdk_core/src/tool_transform.cpp
b0d4b080427e020f885de98484b50800a505929bd825fb2752643973287e7ea0  src/hikrobot_mpsdk_core/tests/test_command_channel_health.cpp
550b880baab83d78af96e1386a7f8566b6d9bc37190921ce00152a0d58911dee  src/hikrobot_mpsdk_core/tests/test_io_command_executor.cpp
50a3cae4e4f2f489c00ba7d3c1035296eb99775275cce791f525318d53feab74  src/hikrobot_mpsdk_core/tests/test_legacy_io_dispatch.cpp
a9ec07e12136366b50efc5d7686434925d2c079739253cf4300f8a1bdeb5391c  src/hikrobot_mpsdk_core/tests/test_legacy_movel_queue_executor.cpp
38508ad0820d549793ca13b3bb4c449c4ceac54208415cda2258140cf340a568  src/hikrobot_mpsdk_core/tests/test_tool_transform.cpp
cc9f8a9464f12f493d9c808fc7ba1dfe55e5310958fdd72ea726adc01fbe0c5c  src/hikrobot_mpsdk_vendor/CMakeLists.txt
2a2b1134c495390499190d5c049d39bda7036b9a76d110d6a76a0b609fca890f  src/hikrobot_mpsdk_vendor/package.xml
c65d45fbcff3751aead5efdcdd10ff21f3ac00bff84889f21e0fa0d420eb9666  src/hikrobot_msgs/CMakeLists.txt
54ae7fe6fbb1a8b5a4c9c97a91deee1977e60e8ae6a41b9914be0e9df4f94142  src/hikrobot_msgs/action/FollowCartesianPath.action
298d8fdbaed850607bb8b1928c00bd833bd1b55082ffb720ea37e5ebef8340b4  src/hikrobot_msgs/action/MoveC.action
82b55c28b31c7e362b75351c0661146a36244b72e1108861e4df9649fc29096b  src/hikrobot_msgs/action/MoveJ.action
afd6590ed705038c70d93af7d940cc2c75d1f269de79663aa0d16b0564e4c4b4  src/hikrobot_msgs/action/MoveL.action
595d98272de5ba63604a54299137a539c7a37b5cd617a958b51950ab65cfdc3c  src/hikrobot_msgs/action/MoveS.action
27e6545411a3f0b07750ef66e92a909cc99e6a38f9c174cc7d35b25fdbaea726  src/hikrobot_msgs/msg/RobotStatus.msg
d5999f1f8e5696c4ed708f1e866acbd11965e20fd2eee89f2571ddcc586f060c  src/hikrobot_msgs/msg/ToolInfo.msg
7f430c0d427ef5d84565a98798a8b018d843dbfa42927a82c47a963cf25588ef  src/hikrobot_msgs/package.xml
735ea20ed2932782537bb521c6461e1d7851a91217bcba478f97b46f18a3b344  src/hikrobot_msgs/srv/GetActiveTool.srv
8cbf5ec40311fa15fdb8687af3473dd54096f6a9409024abff206d7d8f5d7e3f  src/hikrobot_msgs/srv/GetDigitalInput.srv
8cbf5ec40311fa15fdb8687af3473dd54096f6a9409024abff206d7d8f5d7e3f  src/hikrobot_msgs/srv/GetDigitalOutput.srv
93b2628761ba400f8dcd38318349d9d273e9b6c64ef0ab0d7ec9450c56548bb3  src/hikrobot_msgs/srv/GetToolInfoList.srv
043b1e43271f555b239e6ce4d6f5716b87adfb872e16fd3cc05adca6a6a474dc  src/hikrobot_msgs/srv/SetDigitalOutput.srv
c7049b3568154227e5dff82ac95aa5342d995ad56058528c22c09d055b88d874  src/hikrobot_msgs/srv/StopMotion.srv
a5c0b304ac524ec9e60769761b2a0ed9ffe2fe8420ff2ffa7eae2277ea668e80  src/mv3d_rgbd_ros2/CMakeLists.txt
0665228fa9f2e85f0c07d7640604aec2746b806a80fd4fa8e7c860505420791b  src/mv3d_rgbd_ros2/common/ConvertImageType.hpp
a46867217a547b3dd36d23e37244ab85943df9eedf9ced271cff19f80056b60d  src/mv3d_rgbd_ros2/common/Utils.hpp
9a7d9b3d3521fda83d870690ee647ae92947cb0a21fca853fd1945cf2a2d8664  src/mv3d_rgbd_ros2/common/common.hpp
7cd9f987e2df89dfa23f4c366f8110e5c20b41416362e918adfbddcfe3b1d975  src/mv3d_rgbd_ros2/common/hik_camera.h
fbdaddb8dfe92c7f2d81be2dc1d0a2eee2e49b719744234f349263290878e070  src/mv3d_rgbd_ros2/launch/capture_and_save.xml
3d9a7ccba48f83d9d8cbb558fea849c92b5afbf16b5a3b644a12a494b4516cf0  src/mv3d_rgbd_ros2/launch/connect_by_serial_number.xml
84adbb1387629b9f6f158c9f9a1f63527d52d4315852dc4b67d71d5c7874b94e  src/mv3d_rgbd_ros2/launch/multiple_camera.xml
a193f17358fdf52d747480ba9aedcfbf55cb83b6f9e842235c421e77a13e9c88  src/mv3d_rgbd_ros2/launch/obtain_point_cloud.xml
2a80916fa5d6a6562d3b236d25a6e0ebd2b29895bffc9f51c1c17f65d5b9d592  src/mv3d_rgbd_ros2/package.xml
7022899dc0ef5fe00f722b341162eec5eee20780dcef5d6f701248ba1df5d9a1  src/mv3d_rgbd_ros2/src/create_camera_image_client.cpp
dce83fc63277529bcf1018b93044a881171f2818e7aa1f6163410e23673ef2a6  src/mv3d_rgbd_ros2/src/create_camera_image_pub.cpp
b057cba47aa8ded17b8e70d17b77911a959a6ff061d8e8e050a5209b92542a93  src/mv3d_rgbd_ros2/src/create_camera_image_server.cpp
b29e0a8d11c5cb891497a13671b9661a342af061aba9b7cb1353ec53eda3d2ce  src/mv3d_rgbd_ros2/src/create_camera_image_sub.cpp
6ea461bdda634b9018a7da34ad9811c46a22cedcc9eb7a6df7a42e36d5a01c20  src/mv3d_rgbd_ros2/src/create_camera_point_cloud_client.cpp
1e4a087e6dc05f38dbd95fd64682d272a6026ab393f556da80f1a08061b789bb  src/mv3d_rgbd_ros2/src/create_camera_point_cloud_server.cpp
569874166c301801e3ca4a5775e12e3a8388f38e2e258287f5f5485926897ff0  src/mv3d_rgbd_ros2/src/hik_camera.cpp
13a77965b87eefbdaa08df74a6f8545f969db8842fe9be52b2197ad45c595406  src/mv3d_rgbd_ros2_interface/CMakeLists.txt
6596d5512ad8139bff81cd48128117500ebf6558f5d822b2e3d85ba01ba44cb8  src/mv3d_rgbd_ros2_interface/package.xml
9967b306066c3596e8c70bdd1e45c184c4b9b561fe55448d1b7161b58c737994  src/mv3d_rgbd_ros2_interface/srv/Image.srv
c89dadb5db0ac7242f022d8c7759bff2a382b63293afe73edd1f01f7d9a1395e  src/mv3d_rgbd_ros2_interface/srv/PointCloud.srv
eaea13897864907cadda780f4cc95e64bf1d07438b625229d0716a0761b4a454  tests/test_sequence_manager.cpp
```
