#include "hikrobot_mpsdk_core/legacy_movel_queue_executor.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace hikrobot_mpsdk_core
{
namespace
{

constexpr int64_t kSdkSuccess = 0;
constexpr int64_t kErrFirstCmdRepeat = static_cast<int64_t>(0x82068043ULL);
constexpr int kServoEnabled = 1;
constexpr int kMotionAlreadyStopped = 0;

std::string hex_error(int64_t code)
{
  std::ostringstream out;
  out << "0x" << std::hex << std::uppercase <<
    static_cast<uint32_t>(static_cast<uint64_t>(code));
  return out.str();
}

bool finite_pose(const EulerPose & pose)
{
  return std::isfinite(pose.x_mm) && std::isfinite(pose.y_mm) &&
         std::isfinite(pose.z_mm) && std::isfinite(pose.roll_deg) &&
         std::isfinite(pose.pitch_deg) && std::isfinite(pose.yaw_deg);
}

bool validate_request(const FollowCartesianPathRequest & request, std::string & error)
{
  if (request.targets.empty() || request.targets.size() > kMaxFollowCartesianPathTargets) {
    error = "FollowCartesianPath requires 1..32 targets";
    return false;
  }
  const auto & options = request.options;
  if (options.speed_percent < 1 || options.speed_percent > 100) {
    error = "speed_percent must be in [1, 100]";
    return false;
  }
  if (options.blend_radius < 0 || options.blend_radius > 10) {
    error = "blend_radius must be in [0, 10]";
    return false;
  }
  if (options.timeout.count() <= 0 || options.stable_samples <= 0 ||
    options.state_stale_timeout.count() <= 0 ||
    options.stop_confirmation_timeout.count() <= 0)
  {
    error = "timeout, stable_samples, state_stale_timeout, and "
      "stop_confirmation_timeout must be positive";
    return false;
  }
  if (options.coordinate_type != 1 || options.coordinate_id != 0 ||
    options.workpiece_id != -1 || options.tool_id != request.tool.id)
  {
    error = "FollowCartesianPath requires Base 1:0, no workpiece, and the Tool snapshot ID";
    return false;
  }
  if (!valid_tool_id(request.tool.id) || request.tool.mount_position != 0) {
    error = "FollowCartesianPath requires a valid flange-mounted Active Tool snapshot";
    return false;
  }
  for (std::size_t i = 0; i < request.targets.size(); ++i) {
    if (!finite_pose(request.targets[i])) {
      error = "FollowCartesianPath target " + std::to_string(i) + " is not finite";
      return false;
    }
  }
  return true;
}

bool canceled(const std::atomic_bool * flag)
{
  return flag && flag->load(std::memory_order_acquire);
}

std::optional<std::size_t> segment_for_path(
  const std::vector<int32_t> & path_ids, int32_t path_id)
{
  const auto found = std::find(path_ids.begin(), path_ids.end(), path_id);
  if (found == path_ids.end()) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(std::distance(path_ids.begin(), found));
}

int64_t age_ms(
  std::chrono::steady_clock::time_point now,
  std::chrono::steady_clock::time_point received_at)
{
  if (received_at == std::chrono::steady_clock::time_point{}) {
    return -1;
  }
  return std::max<int64_t>(0, std::chrono::duration_cast<std::chrono::milliseconds>(
      now - received_at).count());
}

void update_motion_diagnostics(
  CartesianPathFeedback & fb, const MotionState & motion,
  uint64_t motion_sequence_at_start, const std::vector<int32_t> & path_ids,
  std::chrono::steady_clock::time_point now)
{
  fb.motion_callback_sequence = motion.sequence;
  fb.motion_callback_post_start = motion.sequence > motion_sequence_at_start;
  fb.motion_callback_valid = motion.valid;
  fb.motion_callback_path_id = motion.path_id;
  fb.motion_callback_previous_target_id = motion.previous_target_id;
  fb.motion_callback_next_target_id = motion.next_target_id;
  fb.motion_callback_stopped = motion.stopped;
  fb.motion_callback_left_path_count = motion.left_path_count;
  fb.motion_callback_age_ms = age_ms(now, motion.received_at);
  const auto segment = motion.valid && fb.motion_callback_post_start ?
    segment_for_path(path_ids, motion.path_id) : std::nullopt;
  fb.motion_callback_path_matches = segment.has_value();
  fb.current_segment_valid = segment.has_value();
  if (segment) {
    fb.current_segment_index = *segment;
    fb.current_path_id = motion.path_id;
  }
}

std::string completion_blocker(
  const CartesianPathFeedback & fb, bool tcp_error_valid, bool final_segment,
  bool final_target_referenced)
{
  if (!fb.motion_callback_valid) {
    return "no_motion_callback";
  }
  if (!fb.motion_callback_post_start) {
    return "motion_callback_not_new_after_start";
  }
  if (!fb.motion_callback_path_matches) {
    return "motion_callback_path_id_mismatch";
  }
  if (!fb.observed_running_queue_evidence) {
    return "missing_running_queue_evidence";
  }
  if (!fb.observed_terminal_queue_evidence) {
    if (!final_segment) {
      return "final_segment_not_observed";
    }
    if (!fb.motion_callback_stopped) {
      return "motion_callback_not_stopped";
    }
    if (fb.motion_callback_left_path_count != 0) {
      return "left_path_count_not_zero";
    }
    if (!final_target_referenced) {
      return "final_target_id_not_referenced";
    }
    return "missing_terminal_queue_evidence";
  }
  if (!tcp_error_valid || !fb.actual_tcp_valid) {
    return "tcp_error_unavailable";
  }
  if (!fb.final_in_tolerance) {
    return "final_tcp_out_of_tolerance";
  }
  if (fb.har_motion_status != kMotionAlreadyStopped) {
    return "har_motion_status_not_stopped";
  }
  if (!fb.new_har_sample) {
    return "no_new_har_sample";
  }
  if (fb.stable_sample_count < fb.required_stable_samples) {
    return "accumulating_stable_samples";
  }
  return "complete";
}

std::string diagnostic_summary(const CartesianPathFeedback & fb)
{
  std::ostringstream out;
  out << std::boolalpha
      << "completion{blocker=" << fb.completion_blocker
      << ", motion_seq=" << fb.motion_callback_sequence
      << ", post_start=" << fb.motion_callback_post_start
      << ", valid=" << fb.motion_callback_valid
      << ", path_id=" << fb.motion_callback_path_id
      << ", pre_target=" << fb.motion_callback_previous_target_id
      << ", next_target=" << fb.motion_callback_next_target_id
      << ", stopped=" << fb.motion_callback_stopped
      << ", left_paths=" << fb.motion_callback_left_path_count
      << ", age_ms=" << fb.motion_callback_age_ms
      << ", path_match=" << fb.motion_callback_path_matches
      << ", segment=";
  if (fb.current_segment_valid) {
    out << fb.current_segment_index;
  } else {
    out << "none";
  }
  out << ", running_evidence=" << fb.observed_running_queue_evidence
      << ", terminal_evidence=" << fb.observed_terminal_queue_evidence
      << ", final_in_tolerance=" << fb.final_in_tolerance
      << ", har_motion_status=" << fb.har_motion_status
      << ", har_seq=" << fb.har_sample_sequence
      << ", new_har=" << fb.new_har_sample
      << ", stable=" << fb.stable_sample_count << '/' << fb.required_stable_samples
      << ", position_error_mm=" << fb.final_position_error_mm
      << ", orientation_error_deg=" << fb.final_orientation_error_deg << '}';
  return out.str();
}

std::string diagnostic_change_key(const CartesianPathFeedback & fb)
{
  std::ostringstream out;
  out << static_cast<int>(fb.stage) << ':' << fb.motion_callback_sequence << ':'
      << fb.motion_callback_valid << ':'
      << fb.motion_callback_path_id << ':' << fb.motion_callback_previous_target_id << ':'
      << fb.motion_callback_next_target_id << ':' << fb.motion_callback_stopped << ':'
      << fb.motion_callback_left_path_count << ':' << fb.motion_callback_path_matches << ':'
      << fb.observed_running_queue_evidence << ':' << fb.observed_terminal_queue_evidence << ':'
      << fb.final_in_tolerance << ':' << fb.har_motion_status << ':' << fb.new_har_sample << ':'
      << fb.stable_sample_count << ':' << fb.completion_blocker;
  return out.str();
}

}  // namespace

LegacyMoveLQueueExecutor::LegacyMoveLQueueExecutor(
  LegacyMoveLQueueBackend & backend, MotionSequenceManager & sequence,
  std::atomic_bool & motion_busy, std::atomic_bool & recovery_required,
  std::atomic<int32_t> & current_path_id, int collision_retries)
: backend_(backend), sequence_(sequence), motion_busy_(motion_busy),
  recovery_required_(recovery_required), current_path_id_(current_path_id),
  collision_retries_(collision_retries)
{
}

CartesianPathResult LegacyMoveLQueueExecutor::execute(
  const FollowCartesianPathRequest & request, bool runtime_ready,
  const CartesianPathFeedbackCallback & feedback,
  const std::atomic_bool * cancel_requested)
{
  CartesianPathResult result;
  const auto started_at = backend_.now();
  const auto finish = [this, started_at, &result]() {
      result.execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        backend_.now() - started_at);
    };
  const auto publish = [&feedback](const CartesianPathFeedback & value) {
      if (feedback) {
        feedback(value);
      }
    };

  CartesianPathFeedback fb;
  fb.total_targets = request.targets.size();
  publish(fb);

  std::string validation_error;
  if (!validate_request(request, validation_error)) {
    result.code = MotionResultCode::kInvalidGoal;
    result.failure_stage = PathFailureStage::kValidation;
    result.message = validation_error;
    finish();
    return result;
  }
  if (!runtime_ready) {
    result.code = MotionResultCode::kDriverNotReady;
    result.failure_stage = PathFailureStage::kValidation;
    result.message = "driver is not active/realtime-ready";
    finish();
    return result;
  }
  if (recovery_required_.load(std::memory_order_acquire)) {
    result.code = MotionResultCode::kDriverNotReady;
    result.failure_stage = PathFailureStage::kValidation;
    result.message =
      "motion recovery required; deactivate, cleanup, configure, and activate manually";
    finish();
    return result;
  }
  if (canceled(cancel_requested)) {
    result.code = MotionResultCode::kCanceled;
    result.failure_stage = PathFailureStage::kValidation;
    result.message = "path canceled before execution started";
    finish();
    return result;
  }

  bool expected = false;
  if (!motion_busy_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
    result.code = MotionResultCode::kBusy;
    result.failure_stage = PathFailureStage::kValidation;
    result.message = "another motion is already active";
    finish();
    return result;
  }
  struct BusyGuard
  {
    std::atomic_bool & busy;
    ~BusyGuard() {busy.store(false, std::memory_order_release);}
  } busy_guard{motion_busy_};

  const auto initial = backend_.robot_state(request.options.state_stale_timeout);
  if (!initial.valid || initial.stale || initial.joint_count < 6) {
    result.code = MotionResultCode::kStateLost;
    result.failure_stage = PathFailureStage::kValidation;
    result.message = "valid non-stale HAR state is required before queueing";
    finish();
    return result;
  }
  if (initial.emergency_status != 0 ||
    initial.servo_status != kServoEnabled ||
    initial.motion_status != kMotionAlreadyStopped)
  {
    result.code = MotionResultCode::kRobotNotReady;
    result.failure_stage = PathFailureStage::kValidation;
    result.message = "robot is not servo-enabled, stopped, and free of emergency status";
    finish();
    return result;
  }

  const auto deadline = started_at + request.options.timeout;
  const uint64_t accepted_stop_generation = backend_.stop_generation();
  const std::size_t block_size = request.targets.size() * 2;
  const int max_attempts = std::max(1, collision_retries_);
  std::vector<int32_t> active_path_ids;
  std::vector<int32_t> active_target_ids;
  bool queued = false;

  const auto clear_after_failure = [this, &result](const std::string & context) {
      const int64_t clear_ret = backend_.clear_motion_targets();
      if (clear_ret != kSdkSuccess) {
        recovery_required_.store(true, std::memory_order_release);
        result.message += "; " + context + " cleanup MP_ClearMotionTarget failed: " +
          hex_error(clear_ret) + "; lifecycle cleanup is required before another motion";
        return false;
      }
      return true;
    };

  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    if (canceled(cancel_requested) || backend_.stop_generation() != accepted_stop_generation) {
      result.code = MotionResultCode::kCanceled;
      result.failure_stage = PathFailureStage::kValidation;
      result.message = "path canceled before ID reservation";
      finish();
      return result;
    }
    if (backend_.now() >= deadline) {
      result.code = MotionResultCode::kTimeout;
      result.failure_stage = PathFailureStage::kIdReservation;
      result.message = "path timed out before ID reservation";
      finish();
      return result;
    }

    fb.stage = PathExecutionStage::kReservingIds;
    fb.queued_targets = 0;
    fb.current_segment_valid = false;
    publish(fb);

    int32_t id_base = 0;
    try {
      id_base = sequence_.reserve_block(block_size);
    } catch (const std::exception & error) {
      result.code = MotionResultCode::kSdkError;
      result.sdk_error = -1;
      result.failure_stage = PathFailureStage::kIdReservation;
      result.message = std::string("failed to persist complete path ID block: ") + error.what();
      finish();
      return result;
    }

    active_path_ids.clear();
    active_target_ids.clear();
    for (std::size_t i = 0; i < request.targets.size(); ++i) {
      active_path_ids.push_back(id_base + static_cast<int32_t>(2 * i));
      active_target_ids.push_back(id_base + static_cast<int32_t>(2 * i + 1));
    }
    current_path_id_.store(active_path_ids.front(), std::memory_order_release);

    fb.stage = PathExecutionStage::kClearingQueue;
    fb.current_path_id = active_path_ids.front();
    publish(fb);
    const int64_t clear_ret = backend_.clear_motion_targets();
    if (clear_ret != kSdkSuccess) {
      result.code = MotionResultCode::kSdkError;
      result.sdk_error = clear_ret;
      result.failure_stage = PathFailureStage::kClear;
      result.message = "MP_ClearMotionTarget failed: " + hex_error(clear_ret);
      finish();
      return result;
    }

    bool retry_collision = false;
    for (std::size_t i = 0; i < request.targets.size(); ++i) {
      if (canceled(cancel_requested) || backend_.stop_generation() != accepted_stop_generation) {
        result.code = MotionResultCode::kCanceled;
        result.failure_stage = PathFailureStage::kAdd;
        result.failure_target_index_valid = true;
        result.failure_target_index = i;
        result.message = "path canceled while queueing before target " + std::to_string(i);
        (void)clear_after_failure("cancellation");
        finish();
        return result;
      }
      if (backend_.now() >= deadline) {
        result.code = MotionResultCode::kTimeout;
        result.failure_stage = PathFailureStage::kAdd;
        result.failure_target_index_valid = true;
        result.failure_target_index = i;
        result.message = "path timed out while queueing before target " + std::to_string(i);
        (void)clear_after_failure("timeout");
        finish();
        return result;
      }

      fb.stage = PathExecutionStage::kQueueing;
      fb.current_segment_valid = true;
      fb.current_segment_index = i;
      fb.current_path_id = active_path_ids[i];
      publish(fb);

      const int64_t add_ret = backend_.add_move_l(
        active_path_ids[i], active_target_ids[i], request.targets[i], request.options);
      if (add_ret != kSdkSuccess) {
        result.code = MotionResultCode::kSdkError;
        result.sdk_error = add_ret;
        result.failure_stage = PathFailureStage::kAdd;
        result.failure_target_index_valid = true;
        result.failure_target_index = i;
        result.message = "MP_AddMovlTarget failed at target " + std::to_string(i) + ": " +
          hex_error(add_ret);
        const bool cleanup_ok = clear_after_failure("partial queue");
        if (add_ret == kErrFirstCmdRepeat && cleanup_ok && attempt + 1 < max_attempts) {
          result.code = MotionResultCode::kSdkError;
          result.sdk_error = 0;
          result.failure_stage = PathFailureStage::kNone;
          result.failure_target_index_valid = false;
          result.failure_target_index = 0;
          result.message.clear();
          retry_collision = true;
          break;
        }
        if (add_ret == kErrFirstCmdRepeat && cleanup_ok) {
          result.message += "; bounded whole-queue PathID collision retries exhausted";
        }
        finish();
        return result;
      }
      fb.queued_targets = i + 1;
      publish(fb);
    }

    if (retry_collision) {
      continue;
    }
    // Failed collision attempts remain consumed in the persisted allocator high-water mark,
    // but the Action result describes only the complete queue from the final attempt.
    result.path_ids = active_path_ids;
    queued = true;
    break;
  }

  if (!queued) {
    result.code = MotionResultCode::kSdkError;
    result.sdk_error = kErrFirstCmdRepeat;
    result.failure_stage = PathFailureStage::kAdd;
    result.message = "bounded whole-queue PathID collision retries exhausted";
    finish();
    return result;
  }

  fb.stage = PathExecutionStage::kReadyToStart;
  fb.queued_targets = request.targets.size();
  fb.current_segment_valid = false;
  publish(fb);

  if (canceled(cancel_requested) || backend_.stop_generation() != accepted_stop_generation ||
    backend_.now() >= deadline)
  {
    const bool was_timeout = backend_.now() >= deadline;
    result.code = was_timeout ? MotionResultCode::kTimeout : MotionResultCode::kCanceled;
    result.failure_stage = PathFailureStage::kStart;
    result.message = was_timeout ? "path timed out after queueing and before Start" :
      "path canceled after queueing and before Start";
    (void)clear_after_failure(was_timeout ? "pre-Start timeout" : "pre-Start cancellation");
    finish();
    return result;
  }

  const GuardedStartResult start =
    backend_.start_motion_if_allowed(accepted_stop_generation, cancel_requested);
  if (!start.attempted) {
    result.code = MotionResultCode::kCanceled;
    result.failure_stage = PathFailureStage::kStart;
    result.message = "Start suppressed because a cancel/stop was already observed";
    (void)clear_after_failure("suppressed Start");
    finish();
    return result;
  }
  if (start.sdk_error != kSdkSuccess) {
    result.code = MotionResultCode::kSdkError;
    result.sdk_error = start.sdk_error;
    result.failure_stage = PathFailureStage::kStart;
    result.message = "MP_StartRealTimeMotion failed: " + hex_error(start.sdk_error) +
      "; Start was attempted and the path was not automatically retried";
    const auto stop = backend_.request_stop();
    result.message += stop.sdk_error == kSdkSuccess ?
      "; stop requested, physical stop not confirmed" :
      "; MP_StopRealTimeMotion also failed: " + hex_error(stop.sdk_error);
    recovery_required_.store(true, std::memory_order_release);
    result.message += "; lifecycle cleanup is required before another motion";
    finish();
    return result;
  }

  fb.stage = PathExecutionStage::kExecuting;
  bool observed_running_queue_evidence = false;
  uint64_t latest_running_motion_sequence = 0;
  bool observed_terminal_queue_evidence = false;
  int stable_samples = 0;
  const uint64_t motion_sequence_at_start = backend_.motion_state().sequence;
  const uint64_t robot_sequence_at_start =
    backend_.robot_state(request.options.state_stale_timeout).sequence;
  uint64_t last_counted_robot_sequence = robot_sequence_at_start;
  std::string last_diagnostic_key;
  fb.required_stable_samples = static_cast<uint32_t>(request.options.stable_samples);
  const auto stop_confirmation_timeout = request.options.stop_confirmation_timeout;

  const auto stop_and_classify =
    [this, &result, &fb, &publish, &active_path_ids, motion_sequence_at_start,
      stop_confirmation_timeout](
      MotionResultCode code, PathFailureStage original_stage, const std::string & reason) {
      fb.stage = PathExecutionStage::kStopping;
      fb.diagnostic_summary = diagnostic_summary(fb);
      publish(fb);
      std::string stop_diagnostic_key = diagnostic_change_key(fb);
      const auto stop = backend_.request_stop();
      bool stop_confirmed = false;
      const auto stop_deadline = stop.requested_at + stop_confirmation_timeout;
      while (stop.sdk_error == kSdkSuccess && backend_.now() < stop_deadline) {
        const auto motion = backend_.motion_state();
        update_motion_diagnostics(
          fb, motion, motion_sequence_at_start, active_path_ids, backend_.now());
        fb.diagnostic_summary = diagnostic_summary(fb);
        const auto diagnostic_key = diagnostic_change_key(fb);
        if (diagnostic_key != stop_diagnostic_key) {
          publish(fb);
          stop_diagnostic_key = diagnostic_key;
        }
        if (motion.valid && motion.sequence > stop.motion_sequence_before_request &&
          motion.received_at >= stop.requested_at && motion.stopped &&
          segment_for_path(active_path_ids, motion.path_id))
        {
          stop_confirmed = true;
          break;
        }
        backend_.sleep_for(std::chrono::milliseconds(50));
      }
      result.code = code;
      result.failure_stage = stop_confirmed ? original_stage : PathFailureStage::kStop;
      result.sdk_error = stop.sdk_error == kSdkSuccess ? 0 : stop.sdk_error;
      result.message = reason;
      if (stop.sdk_error != kSdkSuccess) {
        result.message += "; MP_StopRealTimeMotion failed: " + hex_error(stop.sdk_error);
      } else if (stop_confirmed) {
        result.message += "; stop confirmed by a new matching motion callback after sequence " +
          std::to_string(stop.motion_sequence_before_request);
      } else {
        result.message += "; stop confirmation timed out after " +
          std::to_string(stop_confirmation_timeout.count()) +
          " ms without a new matching stopped callback after sequence " +
          std::to_string(stop.motion_sequence_before_request);
      }
      result.message += "; last " + diagnostic_summary(fb);
      if (!stop_confirmed) {
        recovery_required_.store(true, std::memory_order_release);
        result.message += "; lifecycle cleanup is required before another motion";
      }
    };

  while (backend_.now() < deadline) {
    if (canceled(cancel_requested) || backend_.stop_generation() != accepted_stop_generation) {
      stop_and_classify(
        MotionResultCode::kCanceled, PathFailureStage::kExecution, "path cancellation requested");
      finish();
      return result;
    }

    const RobotState robot = backend_.robot_state(request.options.state_stale_timeout);
    const MotionState motion = backend_.motion_state();
    if (!robot.valid || robot.stale) {
      stop_and_classify(
        MotionResultCode::kStateLost, PathFailureStage::kExecution,
        "HAR robot state became unavailable/stale");
      finish();
      return result;
    }
    if (robot.emergency_status != 0) {
      stop_and_classify(
        MotionResultCode::kRobotNotReady, PathFailureStage::kExecution,
        "emergency status became non-zero during path execution");
      finish();
      return result;
    }

    update_motion_diagnostics(
      fb, motion, motion_sequence_at_start, active_path_ids, backend_.now());
    const auto segment = fb.motion_callback_path_matches ?
      segment_for_path(active_path_ids, motion.path_id) : std::nullopt;
    if (segment) {
      fb.current_segment_valid = true;
      fb.current_segment_index = *segment;
      fb.current_path_id = motion.path_id;
      if (!motion.stopped || motion.left_path_count > 0) {
        observed_running_queue_evidence = true;
        latest_running_motion_sequence = std::max(
          latest_running_motion_sequence, motion.sequence);
      }
      const int32_t final_target_id = active_target_ids.back();
      const bool final_target_referenced =
        motion.previous_target_id == final_target_id ||
        motion.next_target_id == final_target_id;
      // Controller 2.1.2 Legacy HIL reports the completed final path as
      // pre_target=0,next_target=0 rather than echoing the supplied TargetID.
      // Accept that sentinel only after a distinct matching running callback;
      // final PathID, stopped and zero remaining paths are still mandatory.
      const bool legacy_zero_target_sentinel =
        motion.previous_target_id == 0 && motion.next_target_id == 0 &&
        observed_running_queue_evidence &&
        motion.sequence > latest_running_motion_sequence;
      if (*segment + 1 == active_path_ids.size() && motion.stopped &&
        motion.left_path_count == 0 &&
        (final_target_referenced || legacy_zero_target_sentinel))
      {
        observed_terminal_queue_evidence = true;
      }
    } else {
      fb.current_segment_valid = false;
    }

    const auto error = cartesian_tcp_error(robot, request.tool, request.targets.back());
    if (error) {
      fb.actual_tcp_valid = true;
      fb.actual_tcp = error->actual_tcp;
      fb.final_position_error_mm = error->position_error_m * 1000.0;
      fb.final_orientation_error_deg =
        error->orientation_error_rad * 180.0 / 3.14159265358979323846;
    } else {
      fb.actual_tcp_valid = false;
    }

    const bool new_post_start_robot_sample = robot.sequence > robot_sequence_at_start &&
      robot.sequence != last_counted_robot_sequence;
    const bool final_in_tolerance = error &&
      fb.final_position_error_mm <= request.options.position_tolerance_mm &&
      fb.final_orientation_error_deg <= request.options.orientation_tolerance_deg;
    if (new_post_start_robot_sample) {
      last_counted_robot_sequence = robot.sequence;
      if (observed_running_queue_evidence && observed_terminal_queue_evidence &&
        final_in_tolerance && robot.motion_status == kMotionAlreadyStopped)
      {
        ++stable_samples;
      } else {
        stable_samples = 0;
      }
    }

    fb.observed_running_queue_evidence = observed_running_queue_evidence;
    fb.observed_terminal_queue_evidence = observed_terminal_queue_evidence;
    fb.final_in_tolerance = final_in_tolerance;
    fb.har_motion_status = robot.motion_status;
    fb.har_sample_sequence = robot.sequence;
    fb.new_har_sample = new_post_start_robot_sample;
    fb.stable_sample_count = static_cast<uint32_t>(stable_samples);
    const bool final_segment = segment && *segment + 1 == active_path_ids.size();
    const int32_t final_target_id = active_target_ids.back();
    const bool final_target_referenced = motion.previous_target_id == final_target_id ||
      motion.next_target_id == final_target_id;
    const bool legacy_zero_target_sentinel = motion.previous_target_id == 0 &&
      motion.next_target_id == 0 && observed_running_queue_evidence &&
      motion.sequence > latest_running_motion_sequence;
    fb.completion_blocker = completion_blocker(
      fb, error.has_value(), final_segment,
      final_target_referenced || legacy_zero_target_sentinel);
    fb.diagnostic_summary = diagnostic_summary(fb);
    const auto diagnostic_key = diagnostic_change_key(fb);
    if (diagnostic_key != last_diagnostic_key) {
      publish(fb);
      last_diagnostic_key = diagnostic_key;
    }
    if (stable_samples >= request.options.stable_samples) {
      result.code = MotionResultCode::kSuccess;
      result.sdk_error = kSdkSuccess;
      result.failure_stage = PathFailureStage::kNone;
      result.message = "path final target reached with matching queue and HAR evidence; last " +
        fb.diagnostic_summary;
      finish();
      return result;
    }
    backend_.sleep_for(std::chrono::milliseconds(50));
  }

  stop_and_classify(
    MotionResultCode::kTimeout, PathFailureStage::kExecution,
    "path timed out before conservative final completion was proven");
  finish();
  return result;
}

}  // namespace hikrobot_mpsdk_core
