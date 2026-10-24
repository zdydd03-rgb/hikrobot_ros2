# 机械臂 Driver 部署指南

本文只部署机械臂相关 ROS 2 包，不构建或修改 `mv3d_rgbd_ros2`、`mv3d_rgbd_ros2_interface`。项目能力、全部ROS API、Lifecycle和已知限制见 [README](README.md)。

目标环境是Ubuntu 22.04、ROS 2 Humble、Linux x86_64。机械臂当前vendor shim使用SDK内的`Libs/Linux/LX64/64`布局，不因此承诺ARM兼容。MPSDK由设备供应商单独提供，本仓库不分发。**SDK可以放在任意可读目录**；下面通过用户输入和CMake参数绑定，不要求`/home/ai/SDK`或`/opt`下某个位置。

## 1. 获取源码与准备环境

在已安装ROS 2 Humble的机器上：

```bash
source /opt/ros/humble/setup.bash
# 当前发布内容位于开发分支，默认main可能不是这版代码
git clone --branch experiment/pathid-exhaustion-fail-safe \
  https://github.com/zdydd03-rgb/hikrobot_ros2.git
cd hikrobot_ros2
export HIKROBOT_WS="$PWD"
```

已有克隆则直接进入其根目录并设置`HIKROBOT_WS="$PWD"`，不需要重复clone或切换分支。

安装编译工具与机械臂ROS依赖（由操作者执行）：

```bash
sudo apt update
sudo apt install build-essential cmake git python3-colcon-common-extensions python3-rosdep \
  ros-humble-rclcpp ros-humble-rclcpp-action ros-humble-rclcpp-lifecycle \
  ros-humble-builtin-interfaces ros-humble-sensor-msgs ros-humble-geometry-msgs \
  ros-humble-diagnostic-msgs ros-humble-tf2 ros-humble-tf2-geometry-msgs \
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
  ros-humble-launch-ros ros-humble-ament-cmake-gtest
```

`rosdep`已初始化并更新的系统，可进一步核对仅机械臂依赖：

```bash
rosdep install --from-paths \
  src/hikrobot_mpsdk_vendor src/hikrobot_msgs src/hikrobot_mpsdk_core \
  src/hikrobot_driver src/hikrobot_bringup src/hikrobot_description \
  --ignore-src --rosdistro humble -r -y
```

若提示rosdep未初始化，按该机器ROS安装流程执行一次`sudo rosdep init`和`rosdep update`；已初始化的不重复init。rosdep不会安装私有MPSDK，也不能修复错误SDK版本/架构。

## 2. 选择实际SDK，不猜安装位置

向供应商取得与控制器匹配的Linux x86_64完整MPSDK并解压。历史使用版本为`V2.4.0 Build20251106 SVN1686294`。选择的是SDK根目录，即同时包含`Include`和`Libs`的那层，不是zip文件、`Include/Public`或单个`.so`。

```bash
read -r -p 'MPSDK根目录的实际路径: ' HIKROBOT_MPSDK_ROOT
export HIKROBOT_MPSDK_ROOT="$(realpath -e -- "$HIKROBOT_MPSDK_ROOT")"
export HIKROBOT_MPSDK_LIBDIR="$HIKROBOT_MPSDK_ROOT/Libs/Linux/LX64/64"
```

当前源码要求布局：

```text
<你选择的SDK根目录>/
  Include/Public/
    MP_Public_API.h
    MP_Public_Struct.h
    MP_Public_Error.h
    MP_Public_Defines.h
    MP_HAR_API.h
    MP_HAR_Struct.h
    MP_Motion_API.h
    MP_Motion_Struct.h
    MP_IO_API.h
    MP_IO_Struct.h
  Libs/Linux/LX64/64/
    libMPSDK_PUBLIC_COMPAT.so
    <SDK同目录其它依赖库>
```

只读检查实际输入与架构：

```bash
: "${HIKROBOT_MPSDK_ROOT:?SDK根目录未设置}"
: "${HIKROBOT_MPSDK_LIBDIR:?SDK库目录未设置}"
for header in MP_Public_API.h MP_Public_Struct.h MP_Public_Error.h MP_Public_Defines.h \
  MP_HAR_API.h MP_HAR_Struct.h MP_Motion_API.h MP_Motion_Struct.h MP_IO_API.h MP_IO_Struct.h; do
  test -r "$HIKROBOT_MPSDK_ROOT/Include/Public/$header" || { echo "缺少头文件: $header"; exit 1; }
done
test -r "$HIKROBOT_MPSDK_LIBDIR/libMPSDK_PUBLIC_COMPAT.so" || { echo '缺少主SDK库'; exit 1; }
uname -m
file "$HIKROBOT_MPSDK_LIBDIR/libMPSDK_PUBLIC_COMPAT.so"
export LD_LIBRARY_PATH="$HIKROBOT_MPSDK_LIBDIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ldd "$HIKROBOT_MPSDK_LIBDIR/libMPSDK_PUBLIC_COMPAT.so"
```

期望主库为ELF 64-bit x86-64，`ldd`没有`not found`。历史SDK二级依赖包括`libMPSDK_ADAPTER.so`、`libMPSDK_LOG.so`、`libMPSDKV212.so`、`libMPSDKV220.so`；具体以手中SDK的`ldd`输出为准。**保留完整SDK库目录及其symlink**，只复制PUBLIC_COMPAT一个文件通常不足。

供应商若提供不同目录结构/架构，当前shim不会自动猜测：先核对SDK版本和布局，不要把不匹配的库重命名成期望文件。

## 3. 构建机械臂包

在新的构建终端，只source系统ROS，不预先source其它旧工作空间。回到本仓库，并保留上节的SDK变量：

```bash
cd "$HIKROBOT_WS"
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --packages-up-to hikrobot_bringup \
  --executor sequential --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  "-DHIKROBOT_MPSDK_ROOT=$HIKROBOT_MPSDK_ROOT"
```

预期构建5包：`hikrobot_mpsdk_vendor`、`hikrobot_msgs`、`hikrobot_mpsdk_core`、`hikrobot_driver`、`hikrobot_bringup`。`hikrobot_description`是独立模型占位，不在bringup依赖链；需要注册该空包时可额外构建它。不要把无包选择的全工作空间`colcon build`误当成机械臂专用命令，它会选择两个相机包并要求另外的相机SDK。

SDK变量作用与优先级：

| 配置 | 作用 |
|---|---|
| `HIKROBOT_MPSDK_ROOT`环境变量 | vendor首次配置的CMake cache默认值 |
| `-DHIKROBOT_MPSDK_ROOT=...` | 显式更新CMake cache；迁移SDK后要重新传，避免旧cache继续指向旧目录 |
| `LD_LIBRARY_PATH` | 当前进程查找SDK及其间接依赖的运行时路径；不是编译头文件路径 |
| vendor导出的interface target | 将include、PUBLIC_COMPAT绝对路径、Threads/dl和rpath-link传递给core |
| 安装的vendor环境hook | source本次install时，将**构建时选定**的SDK库目录加入LD_LIBRARY_PATH |

CMake日志应明确打印本次SDK root/include/library路径。`rpath-link`只解决链接阶段的间接库定位，不保证安装后的运行时可加载，下一节必须检查。

### SDK位置改变或机器迁移

旧install中的导出target和hook含原SDK绝对路径，原来生成的二进制也依赖ABI。单改当前shell环境变量不能更新这些文件。推荐采用新的构建目录，不先删除原构建：

```bash
colcon --log-base log_robot_redeploy build --packages-up-to hikrobot_bringup \
  --build-base build_robot_redeploy --install-base install_robot_redeploy \
  --executor sequential --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  "-DHIKROBOT_MPSDK_ROOT=$HIKROBOT_MPSDK_ROOT"
# 本次选择了新的install，后续都source这个环境
source "$HIKROBOT_WS/install_robot_redeploy/setup.bash"
```

不要同时叠加旧install和新install。下节默认展示标准`install/`；采用新目录时替换该路径，包prefix读取仍有效。不要复制另一个机器的`build/`/`install/`来代替编译，也不要将真实PathID文件作为清cache对象。

## 4. 核实运行时真正链接到所选SDK

标准构建完成后：

```bash
source "$HIKROBOT_WS/install/setup.bash"
export HIKROBOT_CORE_PREFIX="$(ros2 pkg prefix hikrobot_mpsdk_core)"
export HIKROBOT_DRIVER_PREFIX="$(ros2 pkg prefix hikrobot_driver)"
export HIKROBOT_BRINGUP_PREFIX="$(ros2 pkg prefix hikrobot_bringup)"
export HIKROBOT_CORE_LIB="$HIKROBOT_CORE_PREFIX/lib/libhikrobot_mpsdk_core.so"
export HIKROBOT_DRIVER_BIN="$HIKROBOT_DRIVER_PREFIX/lib/hikrobot_driver/hikrobot_driver_node"
readelf -d "$HIKROBOT_CORE_LIB"
ldd "$HIKROBOT_CORE_LIB"
ldd "$HIKROBOT_DRIVER_BIN"
ldd -r "$HIKROBOT_CORE_LIB"
```

核对：

1. 三个prefix来自刚构建的install，而不是其它工作空间。
2. core解析到本次选定的`libMPSDK_PUBLIC_COMPAT.so`和预期二级SDK库。
3. Driver解析到本次core，所有输出没有`not found`，core的`ldd -r`没有未解析符号。
4. `readelf`的NEEDED/RUNPATH可辅助诊断，但运行时最终解析以当前环境的`ldd`为准。

这些命令不会启动Driver或创建SDK Handle。不要通过执行机械臂可执行文件来代替纯动态库检查。在真正使用硬件前，先完成下面的离线测试和配置核对。

## 5. 离线验证

```bash
colcon test --packages-select hikrobot_mpsdk_core hikrobot_driver \
  --event-handlers console_direct+
ctest --test-dir build/hikrobot_mpsdk_core -N
ctest --test-dir build/hikrobot_driver -N
colcon test-result --verbose
```

现有源码注册core5个、Driver2个CTest目标，共97个GTest。旧build可能残留已删除的`test_protected_arc_output` XML；清点当前注册目标，不能把所有历史XML合计当作本次结果。采用新build目录时相应给`colcon test --build-base ... --install-base ...`和ctest替换路径。

根目录独立PathID测试需另编译（不带`-DNDEBUG`，以保持assert有效），只使用自己的临时state，不接触真实`~/.ros/hikrobot/path_id.state`；不要并行运行多个同名测试：

```bash
export HIKROBOT_SEQUENCE_CHECK_DIR="$(mktemp -d /tmp/hikrobot-sequence-check.XXXXXX)"
g++ -std=c++17 -Wall -Wextra -Wpedantic -pthread \
  -Isrc/hikrobot_mpsdk_core/include \
  tests/test_sequence_manager.cpp src/hikrobot_mpsdk_core/src/motion_sequence_manager.cpp \
  -o "$HIKROBOT_SEQUENCE_CHECK_DIR/test_sequence_manager"
"$HIKROBOT_SEQUENCE_CHECK_DIR/test_sequence_manager"
```

CTest/FakeBackend通过是软件验证，不证明11点队列可用。本版仍仅按1～10点使用，详见README。

## 6. 参数与启动准备

启动前可将安装的YAML复制到操作者管理的位置。Launch默认从`hikrobot_bringup`的**安装share目录**解析YAML，不依赖工作空间名或HOME下固定源码路径；`config_file`始终可显式覆盖。

```bash
export HIKROBOT_DEFAULT_CONFIG="$HIKROBOT_BRINGUP_PREFIX/share/hikrobot_bringup/config/hikrobot_driver.yaml"
cat "$HIKROBOT_DEFAULT_CONFIG"
# 将你审核后的参数文件路径填入；也可先复制默认文件再编辑
read -r -p '已审核的Driver YAML绝对路径: ' HIKROBOT_DRIVER_CONFIG
export HIKROBOT_DRIVER_CONFIG="$(realpath -e -- "$HIKROBOT_DRIVER_CONFIG")"
```

YAML顶层节点key默认是`hikrobot_driver`，参数放在`ros__parameters`下。至少确认：

- 控制器实际IP/port；随包的`192.168.2.64:9000`只是历史配置，不是自动发现。
- 六轴joint_names、Base frame、控制器Active Tool及安装位置。
- 平移/旋转步长与容差：随包YAML实际平移值0.600m，Node默认0.05m，原50mm注释不准确；依据现场审核后的配置值执行。
- `sequence_state_file`、运行用户/HOME及持久权限。默认`$HOME/.ros/hikrobot/path_id.state`，不能删除、清零、回退；只能一个Driver进程分配ID。
- watchdog大于lock timeout；启用health时通信阈值至少3个check周期、10个HAR周期。
- 当前仅支持按文档<=10点使用，代码尚未强制收紧32点准入。

密码用环境变量，不写入仓库/YAML：

```bash
read -r -s -p 'PROGRAMMER password: ' HIKROBOT_PROGRAMMER_PASSWORD
export HIKROBOT_PROGRAMMER_PASSWORD
ros2 launch hikrobot_bringup hikrobot_driver.launch.py \
  "config_file:=$HIKROBOT_DRIVER_CONFIG"
```

Launch只创建unconfigured节点，不自动连接。上述是未来部署命令，整理文档时没有执行。若由systemd/容器启动，必须在该进程环境中source本次install、提供SDK运行时路径、密码环境和稳定的HOME/state路径；SSH终端里的export不会自动传到系统服务。SDK应一直可读，不能在构建后移除。

## 7. 现场手动Lifecycle

另开终端source系统ROS和同一次install，释放示教器控制权后才Activate：

```bash
source /opt/ros/humble/setup.bash
source "$HIKROBOT_WS/install/setup.bash"
ros2 node info /hikrobot_driver
ros2 param dump /hikrobot_driver
ros2 lifecycle get /hikrobot_driver
ros2 lifecycle set /hikrobot_driver configure
ros2 service call /hikrobot_driver/get_active_tool hikrobot_msgs/srv/GetActiveTool '{}'
ros2 lifecycle set /hikrobot_driver activate
ros2 topic echo /hikrobot_driver/status --once
```

Configure真实连接并订阅HAR；INACTIVE时Topic未发布，用日志和只读服务确认。Activate包含控制权/PROGRAMMER/AUTO/上电操作，不能把它当作无副作用探测。IO与运动完整命令、Stop/Cancel和字段见README；本指南不生成任意运动坐标。

结束时：

```bash
ros2 service call /hikrobot_driver/stop_motion hikrobot_msgs/srv/StopMotion '{}'
ros2 lifecycle set /hikrobot_driver deactivate
ros2 lifecycle set /hikrobot_driver cleanup
ros2 lifecycle get /hikrobot_driver
```

通过实际状态确认停止与释放。Command Recovery可能跳过SDK释放；永久阻塞可能使退出join无法完成。软件Stop不代替独立硬件急停，命令通道poisoned时不能靠自动重试恢复输出状态。

## 8. 独立SDK只读诊断工具（可选）

`tools/tool_readonly_probe.cpp`脱离ROS查询Active Tool、工具列表和工具详情。编译不会连接；运行会连接SDK，但不申请控制权、不改变用户/模式/电机、不开运动、不写IO。仅在Driver已退出、没有其它SDK连接owner时人工运行。

```bash
export HIKROBOT_PROBE_DIR="$(mktemp -d /tmp/hikrobot-tool-probe.XXXXXX)"
g++ -std=c++17 -Wall -Wextra -Wpedantic tools/tool_readonly_probe.cpp \
  -I"$HIKROBOT_MPSDK_ROOT/Include/Public" \
  -L"$HIKROBOT_MPSDK_LIBDIR" -Wl,-rpath,"$HIKROBOT_MPSDK_LIBDIR" \
  -lMPSDK_PUBLIC_COMPAT -pthread -ldl -o "$HIKROBOT_PROBE_DIR/tool_readonly_probe"
# 仅供现场授权后执行；变量必须是实际控制器地址/端口
: "${ROBOT_IP:?请设置实际控制器IP}"
: "${ROBOT_PORT:?请设置实际控制器端口}"
"$HIKROBOT_PROBE_DIR/tool_readonly_probe" "$ROBOT_IP" "$ROBOT_PORT"
```

## 9. 常见故障与验收标准

| 现象 | 排查 |
|---|---|
| HIKROBOT_MPSDK_ROOT未设置/找不到头 | 根目录层级是否正确；显式传`-D`；SDK必须为完整包 |
| 仍指向旧SDK | 检查vendor CMakeCache、环境hook和prefix；在新目录用新`-D`重建，不混source旧overlay |
| cannot find PUBLIC_COMPAT / wrong ELF class | 检查文件及symlink、x86_64/64位架构和SDK版本 |
| 二级SDK `.so` not found | 完整库目录保留；source本次install；为当前进程配置LD_LIBRARY_PATH |
| undefined symbol | 头/库版本混用或ABI不匹配；检查core `ldd -r`和实际解析路径，不只看文件名 |
| 相机SDK相关错误 | 是否误用了无包选择的全工作空间构建；本指南仅选择机械臂5包，相机包不在本次范围 |
| 找不到参数文件 | Launch从安装share取默认；自定义文件传绝对config_file，并使用正确节点key |
| Configure没有首帧 | 实际IP/端口、网络、控制器状态、HAR；首帧等待3s不约束Vendor连接调用本身 |
| 控制权申请失败 | 检查示教器控制权及控制器状态；历史曾有`0x82178005`，不臆造通用错误含义 |
| -10001/-10002/-10003 | 见README：未执行/结果未知/通道拒绝要分别处理，不自动重发DO或运动 |
| Lifecycle成功但资源不确定 | 读具体日志/Recovery及硬件状态，不能只看迁移返回码 |

部署成功应同时满足：所选机械臂包编译成功、core/Driver正确解析所选SDK且无缺库/未解析符号、有效离线测试通过、安装参数可解析。随后才做单独的现场连接与HIL；离线链接成功不保证控制器版本兼容、现场网络和机械动作安全。

## 10. 本次离线核验记录（2026-10-24）

本轮使用临时构建/安装目录，将实际MPSDK通过**非默认且含空格的路径别名**传入
`-DHIKROBOT_MPSDK_ROOT`，检查了新的cache、导出target、环境hook与实际库解析。

- 仅机械臂bringup依赖链的5个包重新构建成功；占位description包另外通过配置/安装检查。
- 新构建运行7个有效CTest目标、97个GTest，零失败/错误。
- 安装后的core和Driver均解析到所选SDK；core `ldd -r`无缺库或未解析符号。
- 只构造安装后的Launch描述，确认默认YAML来自安装share，未执行LaunchService或启动节点。
- 两个相机包与本轮开始的HEAD逐文件比较完全一致；不属于本次构建验收范围。
- 文档相对链接、Bash代码块语法、Launch Python语法及`git diff --check`通过。

沙箱内colcon曾停在CMake已完成的配置阶段，非沙箱重跑构建/测试正常；这是本轮工具执行
环境的现象，不据此修改Driver。SDK根路径别名验证不等于所有SDK版本、架构或新控制器都通过。
本次没有运行Driver/Tool probe、连接硬件、执行运动/IO或访问真实PathID状态文件。
