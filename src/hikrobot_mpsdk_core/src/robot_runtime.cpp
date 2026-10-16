#include "hikrobot_mpsdk_core/robot_runtime.hpp"
#include "hikrobot_mpsdk_core/command_channel_health.hpp"
#include "hikrobot_mpsdk_core/io_command_executor.hpp"
#include "hikrobot_mpsdk_core/legacy_io_dispatch.hpp"
#include "hikrobot_mpsdk_core/legacy_movel_queue_executor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "MP_Public_API.h"
#include "MP_Public_Struct.h"
#include "MP_Public_Error.h"
#include "MP_Public_Defines.h"
#include "MP_HAR_API.h"
#include "MP_HAR_Struct.h"
#include "MP_Motion_API.h"
#include "MP_Motion_Struct.h"
#include "MP_IO_API.h"
#include "MP_IO_Struct.h"

namespace hikrobot_mpsdk_core
{
namespace
{

constexpr int64_t kErrFirstCmdRepeat =
  static_cast<int64_t>(HKR_E_IN_SYS_CALL_REMOTE_FIRET_CMD_REPEAT);
constexpr double kPi = 3.14159265358979323846;

std::string hex_error(int64_t code)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::uppercase << code;
  return oss.str();
}

std::string sdk_call_failure_message(const std::string & operation, int64_t code)
{
  if (code == kLocalSdkCommandBusy) {
    return operation + " could not acquire the SDK command lock; retry after the current command returns";
  }
  if (code == kLocalSdkRecoveryRequired) {
    return operation + " rejected because SDK command-channel recovery is required; restart the Driver process";
  }
  return operation + " failed: " + hex_error(code);
}

void normalize_io_failure(IoResult & result, const std::string & operation)
{
  if (result.sdk_error == kLocalSdkCommandBusy) {
    result.message =
      "IO command timed out before entering the vendor SDK; task was cancelled/not executed";
  } else if (result.sdk_error == kLocalSdkCommandTimeout) {
    result.message = "SDK command timeout; outcome unknown";
  } else if (result.sdk_error == kLocalSdkRecoveryRequired) {
    result.message = "SDK command-channel recovery is required; restart the Driver process";
  } else {
    result.message = sdk_call_failure_message(operation, result.sdk_error);
  }
}

bool cartesian_target_reached(
  const RobotState & robot, const ToolInfo & tool, const EulerPose & target,
  const MotionOptions & options, MotionFeedback & feedback)
{
  const auto actual = cartesian_tcp_error(robot, tool, target);
  if (!actual) {
    return false;
  }
  feedback.actual_tcp_valid = true;
  feedback.actual_tcp = actual->actual_tcp;
  feedback.position_error_mm = actual->position_error_m * 1000.0;
  feedback.orientation_error_deg = actual->orientation_error_rad * 180.0 / kPi;
  return feedback.position_error_mm <= options.position_tolerance_mm &&
         feedback.orientation_error_deg <= options.orientation_tolerance_deg;
}

bool validate_options(const MotionOptions & options, std::string & error)
{
  if (options.speed_percent < 1 || options.speed_percent > 100) {
    error = "speed_percent must be in [1, 100]";
    return false;
  }
  if (options.blend_radius < 0 || options.blend_radius > 10) {
    error = "blend_radius must be in [0, 10]";
    return false;
  }
  if (options.stable_samples <= 0) {
    error = "stable_samples must be positive";
    return false;
  }
  if (options.timeout.count() <= 0) {
    error = "timeout must be positive";
    return false;
  }
  return true;
}

void fill_ctrl_param(ST_MOTION_CTRL_PARAM & out, const MotionOptions & options)
{
  out.nSpeedPercent = options.speed_percent;
  out.nBlendRadius = options.blend_radius;
  out.stCsSelect.nCsType = options.coordinate_type;
  out.stCsSelect.nCsID = options.coordinate_id;
  out.nToolId = options.tool_id;
  out.nWorkPieceId = options.workpiece_id;
  // Leave dLinearSpeed / acc / dec / jerk at 0 for maximum V2.1.2 compatibility.
}

void fill_euler_target(ST_MOTION_TARGET_INFO & out, int32_t target_id, const EulerPose & pose)
{
  out.nTargetId = target_id;
  out.eTargetType = E_MP_MOTION_TARGET_TYPE::E_MP_TARGET_POSE_EULER;
  out.szTargetPos[0] = pose.x_mm;
  out.szTargetPos[1] = pose.y_mm;
  out.szTargetPos[2] = pose.z_mm;
  out.szTargetPos[3] = pose.roll_deg;
  out.szTargetPos[4] = pose.pitch_deg;
  out.szTargetPos[5] = pose.yaw_deg;
}

void fill_additional_pose(
  ST_ADDITIONAL_MOTION_TARGET_POSE & out, int32_t target_id, const EulerPose & pose)
{
  out.nTargetId = target_id;
  out.szAdditionPose[0] = pose.x_mm;
  out.szAdditionPose[1] = pose.y_mm;
  out.szAdditionPose[2] = pose.z_mm;
  out.szAdditionPose[3] = pose.roll_deg;
  out.szAdditionPose[4] = pose.pitch_deg;
  out.szAdditionPose[5] = pose.yaw_deg;
}

ToolInfo tool_from_sdk(const ST_HAR_TOOL & source)
{
  ToolInfo out;
  out.id = source.nID;
  out.name = bounded_sdk_string(source.szName, sizeof(source.szName));
  out.description = bounded_sdk_string(source.szDescription, sizeof(source.szDescription));
  out.mount_position = static_cast<int>(source.eMountPosition);
  out.flange_to_tool = {
    source.stBody.stPoseE.dX, source.stBody.stPoseE.dY, source.stBody.stPoseE.dZ,
    source.stBody.stPoseE.dRoll, source.stBody.stPoseE.dPitch, source.stBody.stPoseE.dYaw};
  out.mass = source.stBody.dMass;
  out.mass_center = {
    source.stBody.stMassCenter.dX, source.stBody.stMassCenter.dY, source.stBody.stMassCenter.dZ,
    source.stBody.stMassCenter.dRoll, source.stBody.stMassCenter.dPitch,
    source.stBody.stMassCenter.dYaw};
  out.convex_hull_enabled = source.eCvxEnable == E_MP_STATUS_ENABLE;
  return out;
}

}  // namespace

struct RobotRuntime::Impl : LegacyMoveLQueueBackend
{
  struct RobotCache
  {
    mutable std::mutex mutex;
    std::condition_variable cv;
    ST_HAR_ROBOT_STATUS status{};
    bool received{false};
    uint64_t sequence{0};
    std::chrono::steady_clock::time_point received_at{};
  };

  struct MotionCache
  {
    mutable std::mutex mutex;
    ST_MOTION_MOV_STATE state{};
    bool received{false};
    uint64_t sequence{0};
    std::chrono::steady_clock::time_point received_at{};
  };

  RuntimeConfig config{};
  MP_HANDLE handle{0};
  mutable std::atomic_bool connected{false};
  std::atomic_bool health_monitoring{false};
  bool subscribed{false};
  std::atomic_bool configured{false};
  mutable std::atomic_bool active{false};
  bool control_acquired{false};
  bool authenticated{false};
  bool motion_callback_registered{false};
  bool motor_may_be_on{false};
  bool realtime_entered{false};
  bool original_mode_known{false};
  E_MP_ROBOT_WORK_MODE original_mode{E_MP_WORKING_MODE_MANUAL_LS};

  ST_HAR_SUBSCRIBE_MESSAGE_DATA subscription{};
  ST_MOTION_CALLBACK_DATA motion_callback{};
  ST_CALLBACK_DATA connection_callback{};
  bool connection_callback_registered{false};
  RobotCache robot_cache{};
  MotionCache motion_cache{};

  std::unique_ptr<MotionSequenceManager> sequence;

  mutable std::timed_mutex sdk_mutex;
  mutable CommandChannelHealth command_health;
  mutable std::mutex lifecycle_mutex;
  std::atomic_bool motion_busy{false};
  mutable std::atomic_bool motion_recovery_required{false};
  std::atomic<int32_t> current_path_id{0};
  std::atomic<uint64_t> stop_generation_counter{0};
  mutable std::mutex start_stop_mutex;
  mutable std::atomic<int64_t> last_sdk_error{0};
  mutable std::mutex error_mutex;
  mutable std::string last_error_message;
  // Declared last so its destructor joins the worker before any SDK/health
  // state referenced by an in-flight task is destroyed.
  mutable IoCommandExecutor io_executor;

  static void robot_status_callback(ST_COMMON_CALLBACK_DATA * data)
  {
    if (!data || !data->pData || !data->pUser) {
      return;
    }
    auto * cache = reinterpret_cast<RobotCache *>(data->pUser);
    auto * state = reinterpret_cast<ST_HAR_ROBOT_STATUS *>(data->pData);
    {
      std::lock_guard<std::mutex> lock(cache->mutex);
      cache->status = *state;
      cache->received = true;
      ++cache->sequence;
      cache->received_at = std::chrono::steady_clock::now();
    }
    cache->cv.notify_all();
  }

  static void motion_state_callback(ST_COMMON_CALLBACK_DATA * data)
  {
    if (!data || !data->pData || !data->pUser) {
      return;
    }
    auto * cache = reinterpret_cast<MotionCache *>(data->pUser);
    auto * state = reinterpret_cast<ST_MOTION_MOV_STATE *>(data->pData);
    std::lock_guard<std::mutex> lock(cache->mutex);
    cache->state = *state;
    cache->received = true;
    ++cache->sequence;
    cache->received_at = std::chrono::steady_clock::now();
  }

  void set_error(int64_t sdk_error, const std::string & message) const
  {
    last_sdk_error.store(sdk_error, std::memory_order_release);
    std::lock_guard<std::mutex> lock(error_mutex);
    last_error_message = message;
  }

  static void connection_status_callback(ST_COMMON_CALLBACK_DATA * data)
  {
    if (!data || !data->pData || !data->pUser) {
      return;
    }
    auto * self = reinterpret_cast<Impl *>(data->pUser);
    const bool is_connected = *reinterpret_cast<bool *>(data->pData);
    self->connected.store(is_connected, std::memory_order_release);
    if (self->configured.load(std::memory_order_acquire)) {
      self->command_health.on_connection_status(is_connected);
      if (!is_connected) {
        self->active.store(false, std::memory_order_release);
        self->motion_recovery_required.store(true, std::memory_order_release);
        self->set_error(
          kLocalSdkRecoveryRequired,
          "SDK connection callback reported disconnected; restart the Driver process before reuse");
      }
    }
  }

  CommandChannelSnapshot refresh_command_health() const
  {
    const auto snapshot = command_health.poll(
      std::chrono::steady_clock::now(),
      std::chrono::milliseconds(config.sdk_command_watchdog_ms));
    if (snapshot.recovery_required) {
      active.store(false, std::memory_order_release);
      motion_recovery_required.store(true, std::memory_order_release);
      set_error(snapshot.fault_code, snapshot.fault_message);
    }
    return snapshot;
  }

  int64_t call(
    const std::function<int64_t()> & fn,
    const std::string & operation = "SDK command",
    const IoCommandExecutor::StartCallback & mark_io_running = {})
  {
    if (configured.load(std::memory_order_acquire)) {
      const auto health = refresh_command_health();
      if (health.recovery_required) {
        return kLocalSdkRecoveryRequired;
      }
    }

    std::unique_lock<std::timed_mutex> lock(sdk_mutex, std::defer_lock);
    if (!lock.try_lock_for(std::chrono::milliseconds(config.sdk_command_lock_timeout_ms))) {
      const auto health = refresh_command_health();
      if (health.recovery_required) {
        return kLocalSdkRecoveryRequired;
      }
      set_error(
        kLocalSdkCommandBusy,
        operation + " could not acquire the SDK command lock; retry after the current command returns");
      return kLocalSdkCommandBusy;
    }

    // Health may have become poisoned while this caller waited for the SDK
    // mutex. Recheck after acquiring it so an already-queued caller cannot run
    // after a watchdog timeout or disconnect was reported.
    if (configured.load(std::memory_order_acquire)) {
      const auto health = refresh_command_health();
      if (health.recovery_required) {
        return kLocalSdkRecoveryRequired;
      }
    }

    if (mark_io_running && !mark_io_running()) {
      return kLocalSdkCommandBusy;
    }

    const bool monitor_health = configured.load(std::memory_order_acquire);
    if (monitor_health) {
      command_health.begin_call(operation, std::chrono::steady_clock::now());
    }
    const int64_t ret = fn();
    CommandChannelSnapshot completed_health;
    if (monitor_health) {
      command_health.end_call(ret, operation, std::chrono::steady_clock::now());
      completed_health = refresh_command_health();
    }
    if (!monitor_health || !completed_health.recovery_required) {
      last_sdk_error.store(ret, std::memory_order_release);
    }
    return ret;
  }

  RobotState robot_state_snapshot(std::chrono::milliseconds stale_timeout) const
  {
    RobotState out;
    std::lock_guard<std::mutex> lock(robot_cache.mutex);
    if (!robot_cache.received) {
      return out;
    }
    const auto & s = robot_cache.status;
    out.sequence = robot_cache.sequence;
    out.valid = true;
    out.received_at = robot_cache.received_at;
    out.stale = (std::chrono::steady_clock::now() - robot_cache.received_at) > stale_timeout;
    out.servo_status = static_cast<int>(s.eServoStatus);
    out.motion_status = static_cast<int>(s.eMovStatus);
    out.emergency_status = static_cast<int>(s.eEmergencyStatus);
    out.controller_status = static_cast<int>(s.eControllerStatus);
    out.safety_gate_status = static_cast<int>(s.eSafetyGateStatus);
    out.robot_motion_mode = static_cast<int>(s.eRobotMotionMode);
    out.joint_count = std::max(0, std::min(6, s.nJntNum));
    for (int i = 0; i < out.joint_count; ++i) {
      out.joints_deg[static_cast<std::size_t>(i)] = s.dJnt[i];
    }
    out.base_pose.x_mm = s.stBasePose.dX;
    out.base_pose.y_mm = s.stBasePose.dY;
    out.base_pose.z_mm = s.stBasePose.dZ;
    out.base_pose.roll_deg = s.stBasePose.dRoll;
    out.base_pose.pitch_deg = s.stBasePose.dPitch;
    out.base_pose.yaw_deg = s.stBasePose.dYaw;
    out.motion_execution_percent = s.dCurMotionExecutionPercent;
    return out;
  }

  MotionState motion_state_snapshot() const
  {
    MotionState out;
    std::lock_guard<std::mutex> lock(motion_cache.mutex);
    if (!motion_cache.received) {
      return out;
    }
    out.sequence = motion_cache.sequence;
    out.valid = true;
    out.received_at = motion_cache.received_at;
    out.path_id = motion_cache.state.nPathID;
    out.previous_target_id = motion_cache.state.nPreTargetID;
    out.next_target_id = motion_cache.state.nNextTargetID;
    out.stopped = motion_cache.state.eStopFlag == E_MP_MOTION_STOP_FLAG::E_MP_MOTION_STOPED;
    out.left_path_count = motion_cache.state.nLeftPathNum;
    out.percent = motion_cache.state.dPercent;
    return out;
  }

  bool wait_first_state(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(robot_cache.mutex);
    return robot_cache.cv.wait_for(lock, timeout, [this]() {return robot_cache.received;});
  }

  bool wait_servo(int expected, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::unique_lock<std::mutex> lock(robot_cache.mutex);
    while (std::chrono::steady_clock::now() < deadline) {
      if (robot_cache.received && static_cast<int>(robot_cache.status.eServoStatus) == expected) {
        return true;
      }
      robot_cache.cv.wait_for(lock, std::chrono::milliseconds(100));
    }
    return false;
  }

  int64_t stop_motion_internal()
  {
    stop_generation_counter.fetch_add(1, std::memory_order_acq_rel);
    std::lock_guard<std::mutex> start_stop_lock(start_stop_mutex);
    if (!handle || !realtime_entered) {
      return E_MP_SUCCESS;
    }
    return call([this]() {return MP_StopRealTimeMotion(handle);});
  }

  int64_t clear_motion_targets() override
  {
    return call([this]() {return MP_ClearMotionTarget(handle);});
  }

  int64_t add_move_l(
    int32_t path_id, int32_t target_id, const EulerPose & target,
    const MotionOptions & options) override
  {
    ST_MOTION_MOV_PARAM param{};
    param.nPathID = path_id;
    fill_euler_target(param.stMotionTarget, target_id, target);
    fill_ctrl_param(param.stMotionParam, options);
    return call([this, &param]() {return MP_AddMovlTarget(handle, &param);});
  }

  GuardedStartResult start_motion_if_allowed(
    uint64_t expected_stop_generation,
    const std::atomic_bool * cancel_requested) override
  {
    std::lock_guard<std::mutex> start_stop_lock(start_stop_mutex);
    if ((cancel_requested && cancel_requested->load(std::memory_order_acquire)) ||
      stop_generation_counter.load(std::memory_order_acquire) != expected_stop_generation)
    {
      return {};
    }
    GuardedStartResult out;
    out.attempted = true;
    out.sdk_error = call([this]() {return MP_StartRealTimeMotion(handle);});
    return out;
  }

  StopRequestResult request_stop() override
  {
    StopRequestResult out;
    // Snapshot the callback generation immediately before requesting Stop. A
    // cached callback from before this point cannot confirm this request, while
    // a callback delivered during the blocking SDK call remains valid evidence.
    {
      std::lock_guard<std::mutex> lock(motion_cache.mutex);
      out.motion_sequence_before_request = motion_cache.sequence;
    }
    out.requested_at = std::chrono::steady_clock::now();
    out.sdk_error = stop_motion_internal();
    return out;
  }

  uint64_t stop_generation() const override
  {
    return stop_generation_counter.load(std::memory_order_acquire);
  }

  RobotState robot_state(std::chrono::milliseconds stale_timeout) const override
  {
    return robot_state_snapshot(stale_timeout);
  }

  MotionState motion_state() const override
  {
    return motion_state_snapshot();
  }

  std::chrono::steady_clock::time_point now() const override
  {
    return std::chrono::steady_clock::now();
  }

  void sleep_for(std::chrono::milliseconds duration) override
  {
    std::this_thread::sleep_for(duration);
  }

  void deactivate_internal()
  {
    active = false;
    if (command_health.snapshot().recovery_required) {
      current_path_id.store(0, std::memory_order_release);
      return;
    }
    if (realtime_entered && handle != 0) {
      (void)stop_motion_internal();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      (void)call([this]() {return MP_ExitRealTimeMotionMode(handle);});
      realtime_entered = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (motor_may_be_on && control_acquired && authenticated && handle != 0) {
      (void)call([this]() {return MP_SetMotorOff(handle);});
      (void)wait_servo(E_MP_MOTOR_STATUS_DISABLE, std::chrono::milliseconds(5000));
      motor_may_be_on = false;
    }

    if (control_acquired && authenticated && original_mode_known && handle != 0) {
      ST_ROBOT_WORK_MODE mode{};
      mode.eWorkMode = original_mode;
      (void)call([this, &mode]() {return MP_SetRobotWorkMode(handle, &mode);});
    }

    if (control_acquired && handle != 0) {
      (void)call([this]() {return MP_ReleaseControlAccess(handle);});
      control_acquired = false;
    }
    authenticated = false;
    current_path_id.store(0, std::memory_order_release);
  }

  void cleanup_internal()
  {
    health_monitoring.store(false, std::memory_order_release);
    const auto io_wait = std::chrono::milliseconds(config.sdk_command_watchdog_ms);
    if (!io_executor.pause_and_wait_idle(io_wait)) {
      command_health.mark_call_timeout("IO command during cleanup", io_wait);
      (void)refresh_command_health();
    }

    if (command_health.snapshot().recovery_required) {
      // A timed-out SDK call may still be executing with this handle. Destroying or
      // reusing it would introduce a use-after-free/race inside the vendor SDK.
      // Keep the process permanently poisoned and require process replacement.
      configured.store(false, std::memory_order_release);
      connected.store(false, std::memory_order_release);
      active.store(false, std::memory_order_release);
      current_path_id.store(0, std::memory_order_release);
      return;
    }

    deactivate_internal();

    if (command_health.snapshot().recovery_required) {
      // Do not continue unsubscribe/disconnect/destroy after a lifecycle SDK
      // operation makes the handle outcome uncertain.
      configured.store(false, std::memory_order_release);
      connected.store(false, std::memory_order_release);
      active.store(false, std::memory_order_release);
      current_path_id.store(0, std::memory_order_release);
      return;
    }

    if (subscribed && handle != 0) {
      (void)call([this]() {return MP_HAR_UnsubscribeMessage(handle, &subscription.stMessage);});
      subscribed = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (connection_callback_registered && handle != 0) {
      connection_callback.fCallback = nullptr;
      connection_callback.pUserData = nullptr;
      (void)call(
        [this]() {return MP_SetUserCallback(handle, &connection_callback);},
        "MP_SetUserCallback(reset connection status)");
      connection_callback_registered = false;
    }

    if (connected && handle != 0) {
      (void)call([this]() {return MP_DisconnectRobot(handle);});
      connected = false;
    }

    if (handle != 0) {
      (void)call([this]() {return MP_DestroyHandle(handle);});
      handle = 0;
      motion_callback_registered = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    configured = false;
    command_health.mark_clean_disconnected();
    if (!command_health.snapshot().recovery_required) {
      motion_recovery_required.store(false, std::memory_order_release);
    }
    sequence.reset();
    {
      std::lock_guard<std::mutex> lock(robot_cache.mutex);
      robot_cache.received = false;
      robot_cache.sequence = 0;
      robot_cache.status = {};
    }
    {
      std::lock_guard<std::mutex> lock(motion_cache.mutex);
      motion_cache.received = false;
      motion_cache.sequence = 0;
      motion_cache.state = {};
    }
  }

  template<typename AddFn, typename CompletionFn>
  MotionResult execute_common(
    const MotionOptions & options,
    std::size_t id_block_size,
    const AddFn & add_fn,
    const CompletionFn & completion_fn,
    const FeedbackCallback & feedback,
    const std::atomic_bool * cancel_requested)
  {
    MotionResult result;
    std::string validation_error;
    if (!validate_options(options, validation_error)) {
      result.code = MotionResultCode::kInvalidGoal;
      result.message = validation_error;
      return result;
    }

    const auto health = refresh_command_health();
    if (!active || !realtime_entered || !health.connected || health.recovery_required) {
      result.code = MotionResultCode::kDriverNotReady;
      result.message = health.recovery_required ? health.fault_message :
        "driver is not active/realtime-ready or command-connected";
      return result;
    }
    if (motion_recovery_required.load(std::memory_order_acquire)) {
      result.code = MotionResultCode::kDriverNotReady;
      result.message =
        "motion recovery required; deactivate, cleanup, configure, and activate manually";
      return result;
    }

    bool expected = false;
    if (!motion_busy.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
      result.code = MotionResultCode::kBusy;
      result.message = "another motion is already active";
      return result;
    }
    struct BusyGuard
    {
      std::atomic_bool & busy;
      ~BusyGuard() {busy.store(false, std::memory_order_release);}
    } busy_guard{motion_busy};

    const auto started_at = std::chrono::steady_clock::now();
    int32_t path_id = 0;
    int64_t ret = E_MP_SUCCESS;
    bool added = false;

    for (int attempt = 0; attempt < std::max(1, config.sequence_collision_retries); ++attempt) {
      int32_t id_base = 0;
      try {
        id_base = sequence->reserve_block(id_block_size);
      } catch (const std::exception & e) {
        result.code = MotionResultCode::kSdkError;
        result.message = std::string("failed to persist PathID: ") + e.what();
        set_error(-1, result.message);
        return result;
      }
      path_id = id_base;
      current_path_id.store(path_id, std::memory_order_release);

      ret = call([this]() {return MP_ClearMotionTarget(handle);});
      if (ret != E_MP_SUCCESS) {
        result.code = MotionResultCode::kSdkError;
        result.sdk_error = ret;
        result.path_id = path_id;
        result.message = "MP_ClearMotionTarget failed: " + hex_error(ret);
        set_error(ret, result.message);
        return result;
      }

      ret = add_fn(id_base);
      if (ret == E_MP_SUCCESS) {
        added = true;
        break;
      }
      if (ret != kErrFirstCmdRepeat) {
        result.code = MotionResultCode::kSdkError;
        result.sdk_error = ret;
        result.path_id = path_id;
        result.message = "motion Add failed: " + hex_error(ret);
        set_error(ret, result.message);
        return result;
      }

      set_error(ret, "PathID collision detected; reserving a new persisted sequence ID");
    }

    if (!added) {
      result.code = MotionResultCode::kSdkError;
      result.sdk_error = kErrFirstCmdRepeat;
      result.path_id = path_id;
      result.message = "PathID collision recovery exhausted after bounded Add retries";
      set_error(kErrFirstCmdRepeat, result.message);
      return result;
    }

    // After Start is attempted we do not re-add/restart automatically. The controller may
    // already have accepted execution even if communication becomes uncertain.
    ret = call([this]() {return MP_StartRealTimeMotion(handle);});
    if (ret != E_MP_SUCCESS) {
      result.code = MotionResultCode::kSdkError;
      result.sdk_error = ret;
      result.path_id = path_id;
      result.message = "MP_StartRealTimeMotion failed: " + hex_error(ret) +
        "; motion was NOT automatically retried";
      set_error(ret, result.message);
      return result;
    }

    int stable_samples = 0;
    const auto deadline = started_at + options.timeout;
    auto next_feedback = started_at;

    while (std::chrono::steady_clock::now() < deadline) {
      if (cancel_requested && cancel_requested->load(std::memory_order_acquire)) {
        const int64_t stop_ret = stop_motion_internal();
        result.code = MotionResultCode::kCanceled;
        result.sdk_error = stop_ret;
        result.path_id = path_id;
        result.message = stop_ret == E_MP_SUCCESS ? "motion canceled" :
          "cancel requested; MP_StopRealTimeMotion returned " + hex_error(stop_ret);
        result.execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - started_at);
        return result;
      }

      const RobotState robot = robot_state_snapshot(options.state_stale_timeout);
      const MotionState motion = motion_state_snapshot();
      if (!robot.valid || robot.stale) {
        (void)stop_motion_internal();
        result.code = MotionResultCode::kStateLost;
        result.path_id = path_id;
        result.message = "HAR robot state became stale; motion was stopped and not retried";
        result.execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - started_at);
        return result;
      }
      if (robot.emergency_status != 0) {
        (void)stop_motion_internal();
        result.code = MotionResultCode::kRobotNotReady;
        result.path_id = path_id;
        result.message = "emergency status became non-zero during motion";
        result.execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - started_at);
        return result;
      }

      MotionFeedback fb;
      fb.path_id = path_id;
      fb.robot = robot;
      fb.motion = motion;
      fb.progress_percent =
        (motion.valid && motion.percent >= 0.0) ? motion.percent : robot.motion_execution_percent;
      const bool in_tolerance = completion_fn(robot, fb);
      if (in_tolerance && robot.motion_status == E_MP_MOV_STATUS_ALREADY_STOP) {
        ++stable_samples;
      } else {
        stable_samples = 0;
      }

      const auto now = std::chrono::steady_clock::now();
      if (feedback && now >= next_feedback) {
        feedback(fb);
        next_feedback = now + std::chrono::milliseconds(100);
      }

      if (stable_samples >= options.stable_samples) {
        result.code = MotionResultCode::kSuccess;
        result.sdk_error = E_MP_SUCCESS;
        result.path_id = path_id;
        result.message = "motion reached target stably";
        result.execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - started_at);
        set_error(E_MP_SUCCESS, "");
        return result;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    const int64_t stop_ret = stop_motion_internal();
    result.code = MotionResultCode::kTimeout;
    result.sdk_error = stop_ret == E_MP_SUCCESS ? 0 : stop_ret;
    result.path_id = path_id;
    result.message = "motion timed out; stop command issued";
    result.execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started_at);
    return result;
  }
};

RobotRuntime::RobotRuntime() : impl_(std::make_unique<Impl>()) {}

RobotRuntime::~RobotRuntime()
{
  cleanup();
}

bool RobotRuntime::configure(const RuntimeConfig & config, std::string & error)
{
  std::lock_guard<std::mutex> lifecycle_lock(impl_->lifecycle_mutex);
  if (impl_->command_health.snapshot().recovery_required) {
    error = "SDK command channel recovery is required; restart the Driver process before configure";
    impl_->set_error(kLocalSdkRecoveryRequired, error);
    return false;
  }
  impl_->cleanup_internal();
  impl_->config = config;
  if (config.sdk_command_lock_timeout_ms <= 0 || config.sdk_command_watchdog_ms <= 0 ||
    config.sdk_command_watchdog_ms <= config.sdk_command_lock_timeout_ms)
  {
    error =
      "sdk_command_lock_timeout_ms must be positive and less than sdk_command_watchdog_ms";
    impl_->set_error(-1, error);
    return false;
  }

  impl_->handle = MP_CreateHandle();
  if (impl_->handle == 0) {
    error = "MP_CreateHandle failed";
    impl_->set_error(-1, error);
    return false;
  }

  impl_->connection_callback = {};
  impl_->connection_callback.eType = E_MP_CALLBACK_FUN_CONNECT_STATUS;
  impl_->connection_callback.fCallback = Impl::connection_status_callback;
  impl_->connection_callback.pUserData = impl_.get();
  int64_t ret = impl_->call(
    [this]() {return MP_SetUserCallback(impl_->handle, &impl_->connection_callback);},
    "MP_SetUserCallback(connection status)");
  if (ret != E_MP_SUCCESS) {
    error = "MP_SetUserCallback(connection status) failed: " + hex_error(ret);
    impl_->set_error(ret, error);
    impl_->cleanup_internal();
    return false;
  }
  impl_->connection_callback_registered = true;

  ST_ROBOT_CONNECT_INFO conn{};
  conn.stCtrlServerInfo.nPort = config.robot_port;
  std::strncpy(
    conn.stCtrlServerInfo.szIpAddr, config.robot_ip.c_str(),
    sizeof(conn.stCtrlServerInfo.szIpAddr) - 1);

  ret = impl_->call([this, &conn]() {
    return MP_ConnectRobotWithIP(impl_->handle, &conn);
  }, "MP_ConnectRobotWithIP");
  if (ret != E_MP_SUCCESS) {
    error = "MP_ConnectRobotWithIP failed: " + hex_error(ret);
    impl_->set_error(ret, error);
    impl_->cleanup_internal();
    return false;
  }
  impl_->connected = true;

  impl_->subscription = {};
  impl_->subscription.stMessage.eMessageType = E_MP_HAR_ROBOT_STATUS_MSG_TYPE;
  impl_->subscription.stMessage.uPublishTime = config.har_publish_ms;
  impl_->subscription.fCallback = Impl::robot_status_callback;
  impl_->subscription.pUserData = &impl_->robot_cache;

  ret = impl_->call([this]() {
    return MP_HAR_SubscribeMessage(impl_->handle, &impl_->subscription);
  });
  if (ret != E_MP_SUCCESS) {
    error = "MP_HAR_SubscribeMessage failed: " + hex_error(ret);
    impl_->set_error(ret, error);
    impl_->cleanup_internal();
    return false;
  }
  impl_->subscribed = true;

  if (!impl_->wait_first_state(std::chrono::milliseconds(3000))) {
    error = "HAR subscription succeeded but no robot state arrived within 3 s";
    impl_->set_error(-1, error);
    impl_->cleanup_internal();
    return false;
  }

  ST_ROBOT_WORK_MODE mode{};
  ret = impl_->call([this, &mode]() {return MP_GetRobotWorkMode(impl_->handle, &mode);});
  if (ret != E_MP_SUCCESS) {
    error = "MP_GetRobotWorkMode failed: " + hex_error(ret);
    impl_->set_error(ret, error);
    impl_->cleanup_internal();
    return false;
  }
  impl_->original_mode = mode.eWorkMode;
  impl_->original_mode_known = true;

  try {
    impl_->sequence = std::make_unique<MotionSequenceManager>(config.sequence_state_file);
  } catch (const std::exception & e) {
    error = std::string("failed to initialize PathID state: ") + e.what();
    impl_->set_error(-1, error);
    impl_->cleanup_internal();
    return false;
  }

  if (!impl_->connected.load(std::memory_order_acquire)) {
    error = "controller disconnected while configure was completing";
    impl_->set_error(kLocalSdkRecoveryRequired, error);
    impl_->cleanup_internal();
    return false;
  }
  impl_->configured = true;
  impl_->command_health.mark_configured_connected();
  impl_->health_monitoring.store(true, std::memory_order_release);
  impl_->io_executor.resume();
  impl_->set_error(E_MP_SUCCESS, "");
  return true;
}

bool RobotRuntime::activate(const std::string & programmer_password, std::string & error)
{
  std::lock_guard<std::mutex> lifecycle_lock(impl_->lifecycle_mutex);
  if (!impl_->configured || impl_->handle == 0) {
    error = "runtime is not configured";
    return false;
  }
  if (impl_->active) {
    return true;
  }

  const RobotState initial = impl_->robot_state_snapshot(std::chrono::milliseconds(1000));
  if (!initial.valid || initial.stale) {
    error = "robot state is unavailable/stale before activation";
    return false;
  }
  if (initial.emergency_status != 0) {
    error = "cannot activate while emergency status is non-zero";
    return false;
  }
  if (initial.motion_status != E_MP_MOV_STATUS_ALREADY_STOP) {
    error = "cannot activate while robot is already moving";
    return false;
  }

  int64_t ret = impl_->call([this]() {return MP_RequestControlAccess(impl_->handle);});
  if (ret != E_MP_SUCCESS) {
    error = "MP_RequestControlAccess failed: " + hex_error(ret);
    impl_->set_error(ret, error);
    return false;
  }
  impl_->control_acquired = true;

  if (programmer_password.empty() || programmer_password.size() >= MP_BUFFER_LEN_64) {
    error = "programmer password is empty or too long for the SDK buffer";
    impl_->deactivate_internal();
    return false;
  }

  ST_USER_INFO user{};
  user.eUserType = E_MP_USER_PROGRAMMER;
  std::strncpy(user.szUserPwd, programmer_password.c_str(), sizeof(user.szUserPwd) - 1);
  ret = impl_->call([this, &user]() {return MP_SwitchUser(impl_->handle, &user);});
  std::memset(user.szUserPwd, 0, sizeof(user.szUserPwd));
  if (ret != E_MP_SUCCESS) {
    error = "MP_SwitchUser(PROGRAMMER) failed: " + hex_error(ret);
    impl_->set_error(ret, error);
    impl_->deactivate_internal();
    return false;
  }
  impl_->authenticated = true;

  ret = impl_->call([this]() {return MP_EliminateAllAlarm(impl_->handle);});
  if (ret != E_MP_SUCCESS) {
    error = "MP_EliminateAllAlarm failed: " + hex_error(ret);
    impl_->set_error(ret, error);
    impl_->deactivate_internal();
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  impl_->motion_callback = {};
  impl_->motion_callback.eType = E_MP_CALLBACK_FUN_MOTION_STATE;
  impl_->motion_callback.fCallback = Impl::motion_state_callback;
  impl_->motion_callback.pUserData = &impl_->motion_cache;
  ret = impl_->call([this]() {
    return MP_SetMotionUserCallback(impl_->handle, &impl_->motion_callback);
  });
  if (ret != E_MP_SUCCESS) {
    error = "MP_SetMotionUserCallback failed: " + hex_error(ret);
    impl_->set_error(ret, error);
    impl_->deactivate_internal();
    return false;
  }
  impl_->motion_callback_registered = true;

  // The robot may already be enabled when activation starts. Treat it as "may be on"
  // until the OFF state is positively confirmed so rollback will retry MotorOff if needed.
  impl_->motor_may_be_on = initial.servo_status != E_MP_MOTOR_STATUS_DISABLE;
  ret = impl_->call([this]() {return MP_SetMotorOff(impl_->handle);});
  if (ret != E_MP_SUCCESS ||
    !impl_->wait_servo(E_MP_MOTOR_STATUS_DISABLE, std::chrono::milliseconds(5000)))
  {
    error = ret != E_MP_SUCCESS ? "MP_SetMotorOff failed: " + hex_error(ret) :
      "servo did not confirm OFF within 5 s";
    impl_->set_error(ret, error);
    impl_->deactivate_internal();
    return false;
  }
  impl_->motor_may_be_on = false;

  ST_ROBOT_WORK_MODE auto_mode{};
  auto_mode.eWorkMode = E_MP_WORKING_MODE_AUTO;
  ret = impl_->call([this, &auto_mode]() {
    return MP_SetRobotWorkMode(impl_->handle, &auto_mode);
  });
  if (ret != E_MP_SUCCESS) {
    error = "MP_SetRobotWorkMode(AUTO) failed: " + hex_error(ret);
    impl_->set_error(ret, error);
    impl_->deactivate_internal();
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ST_ROBOT_WORK_MODE verify{};
  ret = impl_->call([this, &verify]() {return MP_GetRobotWorkMode(impl_->handle, &verify);});
  if (ret != E_MP_SUCCESS || verify.eWorkMode != E_MP_WORKING_MODE_AUTO) {
    error = ret != E_MP_SUCCESS ? "MP_GetRobotWorkMode verify failed: " + hex_error(ret) :
      "controller did not confirm AUTO mode";
    impl_->set_error(ret, error);
    impl_->deactivate_internal();
    return false;
  }

  ret = impl_->call([this]() {return MP_SetMotorOn(impl_->handle);});
  impl_->motor_may_be_on = (ret == E_MP_SUCCESS);
  if (ret != E_MP_SUCCESS ||
    !impl_->wait_servo(E_MP_MOTOR_STATUS_ENABLE, std::chrono::milliseconds(5000)))
  {
    error = ret != E_MP_SUCCESS ? "MP_SetMotorOn failed: " + hex_error(ret) :
      "servo did not confirm ON within 5 s";
    impl_->set_error(ret, error);
    impl_->deactivate_internal();
    return false;
  }

  ret = impl_->call([this]() {return MP_EnterRealTimeMotionMode(impl_->handle);});
  if (ret != E_MP_SUCCESS) {
    error = "MP_EnterRealTimeMotionMode failed: " + hex_error(ret);
    impl_->set_error(ret, error);
    impl_->deactivate_internal();
    return false;
  }
  impl_->realtime_entered = true;

  ST_MOTION_MODE motion_mode{};
  motion_mode.eMode = E_MP_MOTION_EQUILIBRIUM_MODE;
  ret = impl_->call([this, &motion_mode]() {
    return MP_SetMotionRunMode(impl_->handle, &motion_mode);
  });
  if (ret != E_MP_SUCCESS) {
    error = "MP_SetMotionRunMode failed: " + hex_error(ret);
    impl_->set_error(ret, error);
    impl_->deactivate_internal();
    return false;
  }

  impl_->active = true;
  impl_->set_error(E_MP_SUCCESS, "");
  return true;
}

void RobotRuntime::deactivate()
{
  std::lock_guard<std::mutex> lifecycle_lock(impl_->lifecycle_mutex);
  impl_->deactivate_internal();
}

void RobotRuntime::cleanup()
{
  if (!impl_) {
    return;
  }
  std::lock_guard<std::mutex> lifecycle_lock(impl_->lifecycle_mutex);
  impl_->cleanup_internal();
}

bool RobotRuntime::configured() const
{
  return impl_->configured.load(std::memory_order_acquire);
}

bool RobotRuntime::connected() const
{
  const auto health = impl_->refresh_command_health();
  return configured() && impl_->connected.load(std::memory_order_acquire) && health.connected;
}

bool RobotRuntime::active() const
{
  const auto state = impl_->robot_state_snapshot(std::chrono::milliseconds(1000));
  const bool emergency_active = state.valid && state.emergency_status == E_MP_EMERGENCY_ACTIVE;
  return impl_->active.load(std::memory_order_acquire) && connected() && !emergency_active;
}

bool RobotRuntime::busy() const {return impl_->motion_busy.load(std::memory_order_acquire);}

bool RobotRuntime::command_recovery_required() const
{
  return impl_->refresh_command_health().recovery_required;
}

RuntimeHealthSnapshot RobotRuntime::monitor_health(
  std::chrono::milliseconds communication_timeout)
{
  RuntimeHealthSnapshot out;
  out.monitoring_active = impl_->health_monitoring.load(std::memory_order_acquire) &&
    impl_->configured.load(std::memory_order_acquire);
  out.sdk_connected = impl_->connected.load(std::memory_order_acquire);

  auto command = impl_->refresh_command_health();
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(impl_->robot_cache.mutex);
    out.har_received = impl_->robot_cache.received;
    out.har_sequence = impl_->robot_cache.sequence;
    if (out.har_received) {
      out.har_age = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - impl_->robot_cache.received_at);
      out.emergency_status = static_cast<int>(impl_->robot_cache.status.eEmergencyStatus);
    }
  }

  // The official enum defines ACTIVE as exactly 1. Unknown enum values are not
  // silently promoted to an emergency-stop assertion.
  out.emergency_active = out.har_received &&
    out.emergency_status == static_cast<int>(E_MP_EMERGENCY_ACTIVE);
  out.har_timed_out = out.monitoring_active && out.sdk_connected && out.har_received &&
    communication_timeout.count() > 0 && out.har_age >= communication_timeout;
  if (out.har_timed_out && !command.recovery_required) {
    impl_->command_health.mark_feedback_timeout(out.har_age);
    command = impl_->refresh_command_health();
  }

  out.command_connected = command.connected;
  out.recovery_required = command.recovery_required;
  out.sdk_call_in_progress = command.call_in_progress;
  out.sdk_call_elapsed = command.call_elapsed;
  out.fault_code = command.fault_code;
  out.fault_message = command.fault_message;
  out.active_operation = command.active_operation;
  return out;
}

RobotState RobotRuntime::robot_state() const
{
  return impl_->robot_state_snapshot(std::chrono::milliseconds(1000));
}

MotionState RobotRuntime::motion_state() const
{
  return impl_->motion_state_snapshot();
}

ToolResult RobotRuntime::get_active_tool()
{
  ToolResult out;
  const auto health = impl_->refresh_command_health();
  if (!impl_->configured || !health.connected || health.recovery_required ||
    impl_->handle == MP_INVALID_HANDLE)
  {
    out.sdk_error = health.fault_code == 0 ? kLocalSdkRecoveryRequired : health.fault_code;
    out.message = health.fault_message.empty() ?
      "runtime is not configured/connected" : health.fault_message;
    return out;
  }

  ST_TOOL_SIMPLE_INFO active{};
  ST_HAR_TOOL detail{};
  int64_t ret = E_MP_SUCCESS;
  bool invalid_tool_id = false;
  ret = impl_->call([this, &active, &detail, &invalid_tool_id]() {
    int64_t current = MP_GetActivateTool(impl_->handle, &active);
    if (current == E_MP_SUCCESS && !valid_tool_id(active.nToolID)) {
      invalid_tool_id = true;
      return static_cast<int64_t>(-1);
    }
    if (current == E_MP_SUCCESS) {
      ST_TOOL_INDEX index{};
      index.nToolID = active.nToolID;
      current = MP_HAR_GetToolInfo(impl_->handle, &index, &detail);
    }
    return current;
  }, "read active Tool snapshot");
  if (invalid_tool_id) {
    out.sdk_error = -1;
    out.message = "MP_GetActivateTool returned out-of-range Tool ID";
    return out;
  }
  if (ret != E_MP_SUCCESS) {
    out.sdk_error = ret;
    out.message = "Tool read failed: " + hex_error(ret);
    return out;
  }
  out.success = true;
  out.tool = tool_from_sdk(detail);
  out.message = "ok";
  return out;
}

ToolListResult RobotRuntime::get_tool_info_list()
{
  ToolListResult out;
  const auto health = impl_->refresh_command_health();
  if (!impl_->configured || !health.connected || health.recovery_required ||
    impl_->handle == MP_INVALID_HANDLE)
  {
    out.sdk_error = health.fault_code == 0 ? kLocalSdkRecoveryRequired : health.fault_code;
    out.message = health.fault_message.empty() ?
      "runtime is not configured/connected" : health.fault_message;
    return out;
  }

  ST_HAR_TOOL_LIST list{};
  const int64_t ret = impl_->call(
    [this, &list]() {return MP_HAR_GetToolInfoList(impl_->handle, &list);},
    "MP_HAR_GetToolInfoList");
  if (ret != E_MP_SUCCESS) {
    out.sdk_error = ret;
    out.message = "MP_HAR_GetToolInfoList failed: " + hex_error(ret);
    return out;
  }
  if (!valid_tool_count(list.nNum)) {
    out.sdk_error = -1;
    out.message = "MP_HAR_GetToolInfoList returned out-of-range tool count";
    return out;
  }
  out.tools.reserve(static_cast<std::size_t>(list.nNum));
  for (int32_t i = 0; i < list.nNum; ++i) {
    out.tools.push_back(tool_from_sdk(list.szTool[i]));
  }
  out.success = true;
  out.message = "ok";
  return out;
}

MotionResult RobotRuntime::execute_move_j(
  const MoveJRequest & request, const FeedbackCallback & feedback,
  const std::atomic_bool * cancel_requested)
{
  return impl_->execute_common(
    request.options, 2,
    [this, &request](int32_t id_base) {
      ST_MOTION_MOV_PARAM param{};
      param.nPathID = id_base;
      param.stMotionTarget.nTargetId = id_base + 1;
      param.stMotionTarget.eTargetType = E_MP_TARGET_POSE_JNT;
      for (std::size_t i = 0; i < request.joints_deg.size(); ++i) {
        param.stMotionTarget.szTargetPos[i] = request.joints_deg[i];
      }
      fill_ctrl_param(param.stMotionParam, request.options);
      // Match the HIL-proven 05D MoveJ baseline: joint target uses joint CS type 0.
      param.stMotionParam.stCsSelect.nCsType = 0;
      param.stMotionParam.stCsSelect.nCsID = 0;
      return impl_->call([this, &param]() {return MP_AddMovjTarget(impl_->handle, &param);});
    },
    [&request](const RobotState & robot, MotionFeedback & fb) {
      if (robot.joint_count < 6) {
        return false;
      }
      bool ok = true;
      for (std::size_t i = 0; i < 6; ++i) {
        fb.joint_error_deg[i] = robot.joints_deg[i] - request.joints_deg[i];
        ok = ok && std::abs(fb.joint_error_deg[i]) <= request.options.joint_tolerance_deg;
      }
      return ok;
    },
    feedback, cancel_requested);
}

MotionResult RobotRuntime::execute_move_l(
  const MoveLRequest & request, const FeedbackCallback & feedback,
  const std::atomic_bool * cancel_requested)
{
  return impl_->execute_common(
    request.options, 2,
    [this, &request](int32_t id_base) {
      ST_MOTION_MOV_PARAM param{};
      param.nPathID = id_base;
      fill_euler_target(param.stMotionTarget, id_base + 1, request.target);
      fill_ctrl_param(param.stMotionParam, request.options);
      return impl_->call([this, &param]() {return MP_AddMovlTarget(impl_->handle, &param);});
    },
    [&request](const RobotState & robot, MotionFeedback & fb) {
      return cartesian_target_reached(robot, request.tool, request.target, request.options, fb);
    },
    feedback, cancel_requested);
}

MotionResult RobotRuntime::execute_move_c(
  const MoveCRequest & request, const FeedbackCallback & feedback,
  const std::atomic_bool * cancel_requested)
{
  return impl_->execute_common(
    request.options, 3,
    [this, &request](int32_t id_base) {
      ST_MOTION_MOVC_PARAM param{};
      param.nPathID = id_base;
      fill_euler_target(param.stMotionTarget, id_base + 1, request.target);
      fill_additional_pose(param.stAdditionalPosInfo, id_base + 2, request.via);
      fill_ctrl_param(param.stMotionParam, request.options);
      return impl_->call([this, &param]() {return MP_AddMovcTarget(impl_->handle, &param);});
    },
    [&request](const RobotState & robot, MotionFeedback & fb) {
      return cartesian_target_reached(robot, request.tool, request.target, request.options, fb);
    },
    feedback, cancel_requested);
}

MotionResult RobotRuntime::execute_move_s(
  const MoveSRequest & request, const FeedbackCallback & feedback,
  const std::atomic_bool * cancel_requested)
{
  if (request.via.size() > MP_SUBPOS_SPLINE_MNUM) {
    MotionResult out;
    out.code = MotionResultCode::kInvalidGoal;
    out.message = "MoveS via pose count exceeds SDK limit of 49";
    return out;
  }

  return impl_->execute_common(
    request.options, request.via.size() + 2,
    [this, &request](int32_t id_base) {
      ST_MOTION_MOVS_PARAM param{};
      param.nPathID = id_base;
      fill_euler_target(param.stMotionTarget, id_base + 1, request.target);
      param.nAdditionalPoseNum = static_cast<int32_t>(request.via.size());
      for (std::size_t i = 0; i < request.via.size(); ++i) {
        fill_additional_pose(
          param.szAdditionalPosInfo[i], id_base + 2 + static_cast<int32_t>(i), request.via[i]);
      }
      fill_ctrl_param(param.stMotionParam, request.options);
      return impl_->call([this, &param]() {return MP_AddMovsTarget(impl_->handle, &param);});
    },
    [&request](const RobotState & robot, MotionFeedback & fb) {
      return cartesian_target_reached(robot, request.tool, request.target, request.options, fb);
    },
    feedback, cancel_requested);
}

CartesianPathResult RobotRuntime::execute_follow_cartesian_path(
  const FollowCartesianPathRequest & request,
  const CartesianPathFeedbackCallback & feedback,
  const std::atomic_bool * cancel_requested)
{
  if (!impl_->sequence) {
    CartesianPathResult out;
    out.code = MotionResultCode::kDriverNotReady;
    out.failure_stage = PathFailureStage::kValidation;
    out.message = "PathID manager is not configured";
    return out;
  }
  LegacyMoveLQueueExecutor executor(
    *impl_, *impl_->sequence, impl_->motion_busy, impl_->motion_recovery_required,
    impl_->current_path_id, impl_->config.sequence_collision_retries);
  auto result = executor.execute(
    request, active() && impl_->realtime_entered, feedback, cancel_requested);
  impl_->set_error(
    result.sdk_error,
    result.code == MotionResultCode::kSuccess ? std::string{} : result.message);
  return result;
}

int64_t RobotRuntime::stop_motion()
{
  return impl_->stop_motion_internal();
}

IoResult RobotRuntime::set_digital_output(const std::string & name, bool value)
{
  IoResult out;
  const auto health = impl_->refresh_command_health();
  if (health.recovery_required) {
    out.sdk_error = kLocalSdkRecoveryRequired;
    out.message = "SDK command-channel recovery is required; restart the Driver process";
    return out;
  }
  if (!impl_->configured || !health.connected || impl_->handle == 0) {
    out.sdk_error = kLocalSdkRecoveryRequired;
    out.message = "runtime command channel is not configured/connected";
    return out;
  }

  auto * impl = impl_.get();
  const std::string operation = "MP_SetConfigIOLevelStatus(DO)";
  out = impl->io_executor.execute(
    [impl, name, value, operation](const IoCommandExecutor::StartCallback & mark_running) {
      return dispatch_digital_set(
        name, value,
        [impl, operation, mark_running](const std::string & io_name, bool io_value) {
          ST_IO_LEVEL_INFO info{};
          std::strncpy(
            info.stIOLevelAttr.szIOName, io_name.c_str(),
            sizeof(info.stIOLevelAttr.szIOName) - 1);
          info.stIOLevelAttr.eCommonType = E_MP_IO_DO;
          info.stIOLevelValue.nValue = io_value ? 1 : 0;
          return impl->call([impl, &info]() {
            return MP_SetConfigIOLevelStatus(impl->handle, &info);
          }, operation, mark_running);
        });
    },
    std::chrono::milliseconds(impl->config.sdk_command_lock_timeout_ms),
    std::chrono::milliseconds(impl->config.sdk_command_watchdog_ms),
    [impl, operation]() {
      const auto timeout = std::chrono::milliseconds(impl->config.sdk_command_watchdog_ms);
      impl->command_health.mark_call_timeout(operation, timeout);
      (void)impl->refresh_command_health();
    });
  if (!out.success) {
    normalize_io_failure(out, "MP_SetConfigIOLevelStatus");
    if (out.sdk_error == kLocalSdkCommandTimeout) {
      impl_->set_error(out.sdk_error, out.message);
      return out;
    }
    const auto after = impl_->refresh_command_health();
    if (after.recovery_required && !after.fault_message.empty()) {
      out.sdk_error = kLocalSdkRecoveryRequired;
      out.message = "SDK command-channel recovery is required; restart the Driver process";
    }
    impl_->set_error(out.sdk_error, out.message);
  }
  return out;
}

IoResult RobotRuntime::get_digital_output(const std::string & name) const
{
  IoResult out;
  const auto health = impl_->refresh_command_health();
  if (health.recovery_required) {
    out.sdk_error = kLocalSdkRecoveryRequired;
    out.message = "SDK command-channel recovery is required; restart the Driver process";
    return out;
  }
  if (!impl_->configured || !health.connected || impl_->handle == 0) {
    out.sdk_error = kLocalSdkRecoveryRequired;
    out.message = "runtime command channel is not configured/connected";
    return out;
  }

  auto * impl = impl_.get();
  const std::string operation = "MP_GetConfigIOLevelStatus(DO)";
  out = impl->io_executor.execute(
    [impl, name, operation](const IoCommandExecutor::StartCallback & mark_running) {
      return dispatch_digital_get(
        name, DigitalIoType::kOutput,
        [impl, operation, mark_running](
          const std::string & io_name, DigitalIoType, bool & io_value)
        {
          ST_IO_LEVEL_ATTR attr{};
          std::strncpy(attr.szIOName, io_name.c_str(), sizeof(attr.szIOName) - 1);
          attr.stIOSignalType.eType = E_MP_COMMON_IO_TYPE;
          attr.stIOSignalType.unionDetailType.eCommonType = E_MP_IO_DO;
          ST_IO_LEVEL_VALUE value{};
          const int64_t ret = impl->call([impl, &attr, &value]() {
            return MP_GetConfigIOLevelStatus(impl->handle, &attr, &value);
          }, operation, mark_running);
          io_value = value.nValue != 0;
          return ret;
        });
    },
    std::chrono::milliseconds(impl->config.sdk_command_lock_timeout_ms),
    std::chrono::milliseconds(impl->config.sdk_command_watchdog_ms),
    [impl, operation]() {
      const auto timeout = std::chrono::milliseconds(impl->config.sdk_command_watchdog_ms);
      impl->command_health.mark_call_timeout(operation, timeout);
      (void)impl->refresh_command_health();
    });
  if (!out.success) {
    normalize_io_failure(out, "MP_GetConfigIOLevelStatus(DO)");
    if (out.sdk_error == kLocalSdkCommandTimeout) {
      impl_->set_error(out.sdk_error, out.message);
      return out;
    }
    const auto after = impl_->refresh_command_health();
    if (after.recovery_required && !after.fault_message.empty()) {
      out.sdk_error = kLocalSdkRecoveryRequired;
      out.message = "SDK command-channel recovery is required; restart the Driver process";
    }
    impl_->set_error(out.sdk_error, out.message);
  }
  return out;
}

IoResult RobotRuntime::get_digital_input(const std::string & name) const
{
  IoResult out;
  const auto health = impl_->refresh_command_health();
  if (health.recovery_required) {
    out.sdk_error = kLocalSdkRecoveryRequired;
    out.message = "SDK command-channel recovery is required; restart the Driver process";
    return out;
  }
  if (!impl_->configured || !health.connected || impl_->handle == 0) {
    out.sdk_error = kLocalSdkRecoveryRequired;
    out.message = "runtime command channel is not configured/connected";
    return out;
  }

  auto * impl = impl_.get();
  const std::string operation = "MP_GetConfigIOLevelStatus(DI)";
  out = impl->io_executor.execute(
    [impl, name, operation](const IoCommandExecutor::StartCallback & mark_running) {
      return dispatch_digital_get(
        name, DigitalIoType::kInput,
        [impl, operation, mark_running](
          const std::string & io_name, DigitalIoType, bool & io_value)
        {
          ST_IO_LEVEL_ATTR attr{};
          std::strncpy(attr.szIOName, io_name.c_str(), sizeof(attr.szIOName) - 1);
          attr.stIOSignalType.eType = E_MP_COMMON_IO_TYPE;
          attr.stIOSignalType.unionDetailType.eCommonType = E_MP_IO_DI;
          ST_IO_LEVEL_VALUE value{};
          const int64_t ret = impl->call([impl, &attr, &value]() {
            return MP_GetConfigIOLevelStatus(impl->handle, &attr, &value);
          }, operation, mark_running);
          io_value = value.nValue != 0;
          return ret;
        });
    },
    std::chrono::milliseconds(impl->config.sdk_command_lock_timeout_ms),
    std::chrono::milliseconds(impl->config.sdk_command_watchdog_ms),
    [impl, operation]() {
      const auto timeout = std::chrono::milliseconds(impl->config.sdk_command_watchdog_ms);
      impl->command_health.mark_call_timeout(operation, timeout);
      (void)impl->refresh_command_health();
    });
  if (!out.success) {
    normalize_io_failure(out, "MP_GetConfigIOLevelStatus(DI)");
    if (out.sdk_error == kLocalSdkCommandTimeout) {
      impl_->set_error(out.sdk_error, out.message);
      return out;
    }
    const auto after = impl_->refresh_command_health();
    if (after.recovery_required && !after.fault_message.empty()) {
      out.sdk_error = kLocalSdkRecoveryRequired;
      out.message = "SDK command-channel recovery is required; restart the Driver process";
    }
    impl_->set_error(out.sdk_error, out.message);
  }
  return out;
}

std::vector<std::string> RobotRuntime::list_digital_outputs(int64_t & sdk_error) const
{
  std::vector<std::string> out;
  sdk_error = -1;
  if (!impl_->configured || impl_->handle == 0) {
    return out;
  }
  ST_IO_ATTRIBUTE_LIST_DEPRECATED list{};
  sdk_error = impl_->call([this, &list]() {
    return MP_GetConfigIOAttributeList_Deprecated(impl_->handle, &list);
  }, "MP_GetConfigIOAttributeList_Deprecated");
  if (sdk_error != E_MP_SUCCESS) {
    return out;
  }
  for (int i = 0; i < list.nNum; ++i) {
    if (list.stIOAttrList[i].eType == MP_IO_DO_DEPRECATED) {
      out.emplace_back(list.stIOAttrList[i].szIOName);
    }
  }
  return out;
}

int64_t RobotRuntime::last_sdk_error() const
{
  return impl_->last_sdk_error.load(std::memory_order_acquire);
}

std::string RobotRuntime::last_error_message() const
{
  std::lock_guard<std::mutex> lock(impl_->error_mutex);
  return impl_->last_error_message;
}

}  // namespace hikrobot_mpsdk_core
