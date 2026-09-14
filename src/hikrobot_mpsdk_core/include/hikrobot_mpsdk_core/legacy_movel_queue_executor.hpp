#pragma once

#include "hikrobot_mpsdk_core/motion_sequence_manager.hpp"
#include "hikrobot_mpsdk_core/types.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>

namespace hikrobot_mpsdk_core
{

constexpr std::size_t kMaxFollowCartesianPathTargets = 32;

struct GuardedStartResult
{
  bool attempted{false};
  int64_t sdk_error{0};
};

struct StopRequestResult
{
  int64_t sdk_error{0};
  std::chrono::steady_clock::time_point requested_at{};
  uint64_t motion_sequence_before_request{0};
};

class LegacyMoveLQueueBackend
{
public:
  virtual ~LegacyMoveLQueueBackend() = default;

  virtual int64_t clear_motion_targets() = 0;
  virtual int64_t add_move_l(
    int32_t path_id, int32_t target_id, const EulerPose & target,
    const MotionOptions & options) = 0;
  virtual GuardedStartResult start_motion_if_allowed(
    uint64_t expected_stop_generation, const std::atomic_bool * cancel_requested) = 0;
  virtual StopRequestResult request_stop() = 0;
  virtual uint64_t stop_generation() const = 0;
  virtual RobotState robot_state(std::chrono::milliseconds stale_timeout) const = 0;
  virtual MotionState motion_state() const = 0;
  virtual std::chrono::steady_clock::time_point now() const = 0;
  virtual void sleep_for(std::chrono::milliseconds duration) = 0;
};

class LegacyMoveLQueueExecutor
{
public:
  LegacyMoveLQueueExecutor(
    LegacyMoveLQueueBackend & backend, MotionSequenceManager & sequence,
    std::atomic_bool & motion_busy, std::atomic_bool & recovery_required,
    std::atomic<int32_t> & current_path_id, int collision_retries);

  CartesianPathResult execute(
    const FollowCartesianPathRequest & request, bool runtime_ready,
    const CartesianPathFeedbackCallback & feedback,
    const std::atomic_bool * cancel_requested);

private:
  LegacyMoveLQueueBackend & backend_;
  MotionSequenceManager & sequence_;
  std::atomic_bool & motion_busy_;
  std::atomic_bool & recovery_required_;
  std::atomic<int32_t> & current_path_id_;
  int collision_retries_{3};
};

}  // namespace hikrobot_mpsdk_core
