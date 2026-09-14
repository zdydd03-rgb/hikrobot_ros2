#include "hikrobot_mpsdk_core/legacy_movel_queue_executor.hpp"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <unistd.h>

#include "gtest/gtest.h"

namespace hc = hikrobot_mpsdk_core;
namespace
{

constexpr int64_t kCollision = static_cast<int64_t>(0x82068043ULL);

struct AddedTarget
{
  int32_t path_id;
  int32_t target_id;
  hc::EulerPose target;
  hc::MotionOptions options;
};

class FakeBackend : public hc::LegacyMoveLQueueBackend
{
public:
  int64_t clear_motion_targets() override
  {
    events.push_back("clear");
    queue_begin = added.size();
    const auto index = clear_calls++;
    return index < clear_results.size() ? clear_results[index] : 0;
  }

  int64_t add_move_l(
    int32_t path_id, int32_t target_id, const hc::EulerPose & target,
    const hc::MotionOptions & options) override
  {
    events.push_back("add");
    added.push_back({path_id, target_id, target, options});
    if (!sequence_file.empty()) {
      std::ifstream input(sequence_file);
      int32_t value = 0;
      input >> value;
      persisted_values_seen_by_add.push_back(value);
    }
    const auto index = add_calls++;
    const int64_t result = index < add_results.size() ? add_results[index] : 0;
    if (cancel_flag && index == cancel_after_add_call) {
      cancel_flag->store(true);
    }
    if (increment_stop_after_add_call == index) {
      ++stop_epoch;
    }
    return result;
  }

  hc::GuardedStartResult start_motion_if_allowed(
    uint64_t expected_stop_generation, const std::atomic_bool * cancel_requested) override
  {
    events.push_back("start");
    if ((cancel_requested && cancel_requested->load()) || stop_epoch != expected_stop_generation) {
      return {};
    }
    started = true;
    start_time = now_value;
    return {true, start_result};
  }

  hc::StopRequestResult request_stop() override
  {
    events.push_back("stop");
    ++stop_epoch;
    stop_requested = true;
    stop_motion_sequence_before_request = current_motion_sequence();
    stop_request_time = now_value;
    if (stop_callback_during_call) {
      stop_callback_sequence = stop_motion_sequence_before_request + 1;
    }
    return {stop_result, stop_request_time, stop_motion_sequence_before_request};
  }

  uint64_t stop_generation() const override {return stop_epoch;}

  hc::RobotState robot_state(std::chrono::milliseconds) const override
  {
    hc::RobotState state;
    state.valid = true;
    state.stale = force_stale;
    state.servo_status = 1;
    state.motion_status = started ? execution_motion_status : 0;
    state.joint_count = 6;
    state.sequence = freeze_robot_sequence ? 1 : sample_index + 1;
    state.received_at = repeat_robot_timestamp ? start_time + std::chrono::milliseconds(1) :
      now_value;
    if (started && sample_index >= 2 && !added.empty() && !never_reach_target) {
      state.base_pose = added.back().target;
    } else if (started && sample_index == 1 && !added.empty()) {
      state.base_pose = added.back().target;
      state.base_pose.x_mm *= 0.5;
    }
    return state;
  }

  hc::MotionState motion_state() const override
  {
    hc::MotionState state;
    if (stop_requested && stop_result == 0 && confirm_stop && !added.empty() &&
      (stop_callback_during_call ||
      now_value >= stop_request_time + stop_confirmation_delay))
    {
      state.valid = true;
      state.sequence = stop_callback_during_call ? stop_callback_sequence :
        stop_motion_sequence_before_request + 1;
      state.path_id = (wrong_callback_path || stop_callback_wrong_path) ?
        999999 : added.back().path_id;
      state.previous_target_id = (wrong_callback_path || stop_callback_wrong_path) ?
        999998 : added.back().target_id;
      state.stopped = true;
      state.left_path_count = 0;
      state.received_at = stop_callback_during_call ? stop_request_time :
        stop_request_time + stop_confirmation_delay;
      return state;
    }
    if (stop_requested) {
      state.sequence = stop_motion_sequence_before_request;
      if (pre_stop_cached_callback && !added.empty()) {
        state.valid = true;
        state.path_id = added.back().path_id;
        state.previous_target_id = added.back().target_id;
        state.stopped = true;
        state.left_path_count = 0;
        state.received_at = stop_request_time;
      }
      return state;
    }
    if (pre_stop_cached_callback && !added.empty()) {
      state.valid = true;
      state.sequence = 77;
      state.path_id = added.back().path_id;
      state.previous_target_id = added.back().target_id;
      state.stopped = true;
      state.left_path_count = 0;
      state.received_at = now_value;
      return state;
    }
    if (window_refill_mode && started && !added.empty() && sample_index > 0) {
      state.valid = true;
      state.sequence = window_reuse_callback_sequence ? 1 :
        (window_callback_sequences.size() >= sample_index ?
        window_callback_sequences[sample_index - 1] : sample_index);
      state.received_at = now_value;
      if (sample_index <= window_refill_terminal_after) {
        const std::size_t index = std::min(sample_index - 1, added.size() - 1);
        state.path_id = added[index].path_id;
        state.next_target_id = added[index].target_id;
        state.stopped = false;
        state.left_path_count = window_left_path_count;
        return state;
      }
      state.path_id = added.back().path_id;
      state.previous_target_id = added.back().target_id;
      state.stopped = true;
      state.left_path_count = 0;
      return state;
    }
    if (!started || added.empty() || sample_index == 0) {
      return state;
    }
    state.valid = true;
    state.sequence = current_motion_sequence();
    state.received_at = now_value;
    if (wrong_callback_path) {
      state.path_id = 999999;
      state.previous_target_id = 999998;
      state.stopped = sample_index >= 2;
      return state;
    }
    if ((sample_index == 1 && provide_running_evidence) || !provide_terminal_evidence) {
      state.path_id = added[queue_begin].path_id;
      state.next_target_id = added[queue_begin].target_id;
      state.stopped = false;
      state.left_path_count = static_cast<int32_t>(added.size());
      return state;
    }
    state.path_id = added.back().path_id;
    state.sequence = reuse_running_sequence_for_terminal ? 1 : state.sequence;
    state.previous_target_id = legacy_zero_target_sentinel ? 0 :
      (wrong_terminal_target_id ? added.back().target_id + 999 : added.back().target_id);
    state.next_target_id = 0;
    state.stopped = true;
    state.left_path_count = 0;
    return state;
  }

  std::chrono::steady_clock::time_point now() const override {return now_value;}

  void sleep_for(std::chrono::milliseconds duration) override
  {
    now_value += duration;
    ++sample_index;
    if (cancel_flag && sample_index == cancel_on_sample) {
      cancel_flag->store(true);
    }
  }

  uint64_t current_motion_sequence() const
  {
    if (pre_stop_cached_callback) {
      return 77;
    }
    return started ? sample_index : 0;
  }

  std::vector<std::string> events;
  std::vector<int64_t> clear_results;
  std::vector<int64_t> add_results;
  std::vector<AddedTarget> added;
  std::vector<int32_t> persisted_values_seen_by_add;
  std::string sequence_file;
  std::atomic_bool * cancel_flag{nullptr};
  std::size_t cancel_after_add_call{static_cast<std::size_t>(-1)};
  std::size_t increment_stop_after_add_call{static_cast<std::size_t>(-1)};
  std::size_t cancel_on_sample{static_cast<std::size_t>(-1)};
  std::size_t clear_calls{0};
  std::size_t add_calls{0};
  uint64_t stop_epoch{0};
  int64_t start_result{0};
  int64_t stop_result{0};
  bool started{false};
  bool stop_requested{false};
  bool confirm_stop{true};
  bool force_stale{false};
  bool provide_terminal_evidence{true};
  bool provide_running_evidence{true};
  bool wrong_callback_path{false};
  bool stop_callback_wrong_path{false};
  bool pre_stop_cached_callback{false};
  bool repeat_robot_timestamp{false};
  bool freeze_robot_sequence{false};
  bool never_reach_target{false};
  bool legacy_zero_target_sentinel{false};
  bool reuse_running_sequence_for_terminal{false};
  bool wrong_terminal_target_id{false};
  bool stop_callback_during_call{false};
  bool window_refill_mode{false};
  bool window_reuse_callback_sequence{false};
  std::vector<uint64_t> window_callback_sequences;
  int execution_motion_status{0};
  int32_t window_left_path_count{9};
  std::size_t sample_index{0};
  std::size_t queue_begin{0};
  std::size_t window_refill_terminal_after{0};
  std::chrono::steady_clock::time_point now_value{std::chrono::milliseconds(1000)};
  std::chrono::steady_clock::time_point start_time{};
  std::chrono::steady_clock::time_point stop_request_time{};
  uint64_t stop_motion_sequence_before_request{0};
  uint64_t stop_callback_sequence{0};
  std::chrono::milliseconds stop_confirmation_delay{50};
};

class QueueExecutorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static std::atomic_uint counter{0};
    directory = std::filesystem::temp_directory_path() /
      ("hikrobot_queue_test_" + std::to_string(::getpid()) + "_" +
      std::to_string(counter.fetch_add(1)));
    std::filesystem::create_directories(directory);
    state_file = directory / "path_id.state";
    std::ofstream(state_file) << "100\n";
    backend.sequence_file = state_file.string();
  }

  void TearDown() override {std::filesystem::remove_all(directory);}

  hc::FollowCartesianPathRequest request(std::size_t count = 2)
  {
    hc::FollowCartesianPathRequest value;
    value.tool.id = 2;
    value.tool.mount_position = 0;
    value.options.tool_id = 2;
    value.options.coordinate_type = 1;
    value.options.coordinate_id = 0;
    value.options.workpiece_id = -1;
    value.options.speed_percent = 1;
    value.options.blend_radius = 0;
    value.options.timeout = std::chrono::milliseconds(500);
    value.options.state_stale_timeout = std::chrono::milliseconds(100);
    value.options.stop_confirmation_timeout = std::chrono::milliseconds(300);
    value.options.stable_samples = 2;
    for (std::size_t i = 0; i < count; ++i) {
      hc::EulerPose pose;
      pose.x_mm = static_cast<double>((i + 1) * 10);
      value.targets.push_back(pose);
    }
    return value;
  }

  hc::CartesianPathResult run(const hc::FollowCartesianPathRequest & value, bool ready = true)
  {
    hc::MotionSequenceManager sequence(state_file.string());
    hc::LegacyMoveLQueueExecutor executor(
      backend, sequence, busy, recovery, current_path, 3);
    return executor.execute(value, ready, {}, &cancel);
  }

  hc::CartesianPathResult run_with_feedback(
    const hc::FollowCartesianPathRequest & value,
    std::vector<hc::CartesianPathFeedback> & feedback_values)
  {
    hc::MotionSequenceManager sequence(state_file.string());
    hc::LegacyMoveLQueueExecutor executor(
      backend, sequence, busy, recovery, current_path, 3);
    return executor.execute(
      value, true,
      [&feedback_values](const hc::CartesianPathFeedback & value) {
        feedback_values.push_back(value);
      },
      &cancel);
  }

  std::filesystem::path directory;
  std::filesystem::path state_file;
  FakeBackend backend;
  std::atomic_bool busy{false};
  std::atomic_bool recovery{false};
  std::atomic<int32_t> current_path{0};
  std::atomic_bool cancel{false};
};

TEST_F(QueueExecutorTest, RejectsInvalidRequestsWithoutSdkOrIdConsumption)
{
  auto value = request();
  value.targets.clear();
  EXPECT_EQ(run(value).code, hc::MotionResultCode::kInvalidGoal);
  value = request(33);
  EXPECT_EQ(run(value).code, hc::MotionResultCode::kInvalidGoal);
  value = request();
  value.options.speed_percent = 0;
  EXPECT_EQ(run(value).code, hc::MotionResultCode::kInvalidGoal);
  value = request();
  value.options.blend_radius = 11;
  EXPECT_EQ(run(value).code, hc::MotionResultCode::kInvalidGoal);
  value = request();
  value.options.timeout = std::chrono::milliseconds(0);
  EXPECT_EQ(run(value).code, hc::MotionResultCode::kInvalidGoal);
  value = request();
  value.options.stop_confirmation_timeout = std::chrono::milliseconds(0);
  EXPECT_EQ(run(value).code, hc::MotionResultCode::kInvalidGoal);
  value = request();
  value.targets[0].yaw_deg = std::numeric_limits<double>::infinity();
  EXPECT_EQ(run(value).code, hc::MotionResultCode::kInvalidGoal);
  EXPECT_TRUE(backend.events.empty());
  std::ifstream input(state_file);
  int value_on_disk = 0;
  input >> value_on_disk;
  EXPECT_EQ(value_on_disk, 100);
}

TEST_F(QueueExecutorTest, RejectsInvalidToolAndBusyOrRecoveryStates)
{
  auto value = request();
  value.tool.mount_position = 1;
  EXPECT_EQ(run(value).code, hc::MotionResultCode::kInvalidGoal);
  value = request();
  busy.store(true);
  EXPECT_EQ(run(value).code, hc::MotionResultCode::kBusy);
  busy.store(false);
  recovery.store(true);
  EXPECT_EQ(run(value).code, hc::MotionResultCode::kDriverNotReady);
  EXPECT_TRUE(backend.events.empty());
}

TEST_F(QueueExecutorTest, PersistsUniqueCompleteIdBlockBeforeAddsAndStartsOnce)
{
  const auto result = run(request());
  ASSERT_EQ(result.code, hc::MotionResultCode::kSuccess);
  ASSERT_EQ(result.path_ids, (std::vector<int32_t>{101, 103}));
  ASSERT_EQ(backend.added.size(), 2U);
  EXPECT_EQ(backend.added[0].target_id, 102);
  EXPECT_EQ(backend.added[1].target_id, 104);
  EXPECT_EQ(backend.persisted_values_seen_by_add, (std::vector<int32_t>{104, 104}));
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "clear"), 1);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 1);
  EXPECT_EQ(backend.events, (std::vector<std::string>{"clear", "add", "add", "start"}));
}

TEST_F(QueueExecutorTest, OneTargetUsesOneInitialAddAndNoRefill)
{
  const auto result = run(request(1));
  EXPECT_EQ(result.code, hc::MotionResultCode::kSuccess);
  EXPECT_EQ(backend.added.size(), 1U);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 1);
}

TEST_F(QueueExecutorTest, TenTargetsFillInitialWindowAndStartOnce)
{
  const auto result = run(request(10));
  EXPECT_EQ(result.code, hc::MotionResultCode::kSuccess);
  EXPECT_EQ(backend.added.size(), 10U);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 1);
}

TEST_F(QueueExecutorTest, TwentyTargetsRefillOneControllerWindowWithOneStart)
{
  backend.window_refill_mode = true;
  backend.window_refill_terminal_after = 10;
  auto value = request(20);
  value.options.timeout = std::chrono::milliseconds(1500);
  std::vector<hc::CartesianPathFeedback> feedback_values;
  const auto result = run_with_feedback(value, feedback_values);
  ASSERT_EQ(result.code, hc::MotionResultCode::kSuccess);
  ASSERT_EQ(backend.added.size(), 20U);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 1);
  const auto start = std::find(backend.events.begin(), backend.events.end(), "start");
  EXPECT_EQ(std::count(backend.events.begin(), start, "add"), 10);
  EXPECT_EQ(
    backend.persisted_values_seen_by_add,
    std::vector<int32_t>(20, 140));
  std::vector<int32_t> ids;
  for (const auto & target : backend.added) {
    ids.push_back(target.path_id);
  }
  std::sort(ids.begin(), ids.end());
  EXPECT_EQ(std::adjacent_find(ids.begin(), ids.end()), ids.end());
  EXPECT_TRUE(std::any_of(
    feedback_values.begin(), feedback_values.end(),
    [](const auto & item) {return item.queued_targets == 10; }));
  EXPECT_EQ(feedback_values.back().queued_targets, 20U);
}

TEST_F(QueueExecutorTest, ThirtyTwoTargetsStayWithinWindowAndStartOnce)
{
  backend.window_refill_mode = true;
  backend.window_refill_terminal_after = 22;
  auto value = request(32);
  value.options.timeout = std::chrono::milliseconds(2500);
  const auto result = run(value);
  ASSERT_EQ(result.code, hc::MotionResultCode::kSuccess);
  EXPECT_EQ(backend.added.size(), 32U);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 1);
  const auto start = std::find(backend.events.begin(), backend.events.end(), "start");
  EXPECT_EQ(std::count(backend.events.begin(), start, "add"), 10);
  EXPECT_EQ(
    backend.persisted_values_seen_by_add,
    std::vector<int32_t>(32, 164));
}

TEST_F(QueueExecutorTest, RepeatedMotionCallbackSequenceCannotRepeatRefill)
{
  backend.window_refill_mode = true;
  backend.window_reuse_callback_sequence = true;
  backend.window_refill_terminal_after = 100;
  auto value = request(20);
  value.options.timeout = std::chrono::milliseconds(300);
  value.options.stop_confirmation_timeout = std::chrono::milliseconds(100);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kTimeout);
  EXPECT_EQ(backend.added.size(), 11U);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 1);
}

TEST_F(QueueExecutorTest, OutOfOrderMotionCallbackCannotRepeatRefill)
{
  backend.window_refill_mode = true;
  backend.window_refill_terminal_after = 100;
  backend.window_callback_sequences = {1, 2, 1};
  auto value = request(20);
  value.options.timeout = std::chrono::milliseconds(200);
  value.options.stop_confirmation_timeout = std::chrono::milliseconds(100);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kTimeout);
  EXPECT_EQ(backend.added.size(), 12U);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 1);
}

TEST_F(QueueExecutorTest, RefillAddFailureStopsWithoutRetryingOrSecondStart)
{
  backend.window_refill_mode = true;
  backend.window_refill_terminal_after = 100;
  backend.add_results.assign(10, 0);
  backend.add_results.push_back(0x1234);
  auto value = request(20);
  value.options.timeout = std::chrono::milliseconds(1000);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kSdkError);
  EXPECT_EQ(result.sdk_error, 0x1234);
  EXPECT_EQ(result.failure_stage, hc::PathFailureStage::kAdd);
  EXPECT_TRUE(result.failure_target_index_valid);
  EXPECT_EQ(result.failure_target_index, 10U);
  EXPECT_EQ(backend.added.size(), 11U);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 1);
}

TEST_F(QueueExecutorTest, RefillCollisionDoesNotAllocateNewIdsOrRetryPath)
{
  backend.window_refill_mode = true;
  backend.window_refill_terminal_after = 100;
  backend.add_results.assign(10, 0);
  backend.add_results.push_back(kCollision);
  auto value = request(20);
  value.options.timeout = std::chrono::milliseconds(1000);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kSdkError);
  EXPECT_EQ(result.sdk_error, kCollision);
  EXPECT_EQ(result.path_ids.size(), 20U);
  EXPECT_EQ(backend.added.size(), 11U);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 1);
  std::ifstream input(state_file);
  int32_t high_water = 0;
  input >> high_water;
  EXPECT_EQ(high_water, 140);
}

TEST_F(QueueExecutorTest, CancelDuringRefillStopsWithoutSubmittingLaterTargets)
{
  backend.window_refill_mode = true;
  backend.window_refill_terminal_after = 100;
  backend.cancel_flag = &cancel;
  backend.cancel_after_add_call = 10;
  auto value = request(20);
  value.options.timeout = std::chrono::milliseconds(1000);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kCanceled);
  EXPECT_EQ(backend.added.size(), 11U);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "stop"), 1);
}

TEST_F(QueueExecutorTest, PrematureStoppedCallbackReportsQueueUnderflowImmediately)
{
  backend.window_refill_mode = true;
  backend.window_refill_terminal_after = 0;
  auto value = request(20);
  value.options.timeout = std::chrono::milliseconds(1000);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kSdkError);
  EXPECT_EQ(result.failure_stage, hc::PathFailureStage::kExecution);
  EXPECT_FALSE(recovery.load());
  EXPECT_LT(result.execution_time, value.options.timeout);
  EXPECT_NE(result.message.find("queue underflow/premature stop"), std::string::npos);
  EXPECT_NE(result.message.find("submitted_targets=10/20"), std::string::npos);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 1);
}

TEST_F(QueueExecutorTest, ExhaustionAndPersistenceFailureOccurBeforeAnySdkCall)
{
  std::ofstream(state_file, std::ios::trunc) << "1999999998\n";
  auto result = run(request(1));
  EXPECT_EQ(result.code, hc::MotionResultCode::kSdkError);
  EXPECT_TRUE(backend.events.empty());
  std::ifstream input(state_file);
  int64_t value_on_disk = 0;
  input >> value_on_disk;
  EXPECT_EQ(value_on_disk, 1999999998);

  std::ofstream(state_file, std::ios::trunc) << "100\n";
  std::filesystem::create_directory(state_file.string() + ".tmp");
  result = run(request(1));
  EXPECT_EQ(result.code, hc::MotionResultCode::kSdkError);
  EXPECT_TRUE(backend.events.empty());
}

TEST_F(QueueExecutorTest, AppliesOneToolSnapshotAndBaseOptionsToEveryAdd)
{
  auto value = request(3);
  ASSERT_EQ(run(value).code, hc::MotionResultCode::kSuccess);
  ASSERT_EQ(backend.added.size(), 3U);
  for (const auto & added : backend.added) {
    EXPECT_EQ(added.options.tool_id, 2);
    EXPECT_EQ(added.options.coordinate_type, 1);
    EXPECT_EQ(added.options.coordinate_id, 0);
    EXPECT_EQ(added.options.workpiece_id, -1);
  }
}

TEST_F(QueueExecutorTest, PartialAddFailureClearsQueueAndNeverStarts)
{
  backend.add_results = {0, 0x1234};
  const auto result = run(request(3));
  EXPECT_EQ(result.code, hc::MotionResultCode::kSdkError);
  EXPECT_EQ(result.sdk_error, 0x1234);
  EXPECT_TRUE(result.failure_target_index_valid);
  EXPECT_EQ(result.failure_target_index, 1U);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "clear"), 2);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 0);
  EXPECT_FALSE(recovery.load());
  EXPECT_FALSE(busy.load());
}

TEST_F(QueueExecutorTest, CleanupFailureBlocksSubsequentMotion)
{
  backend.add_results = {0, 0x1234};
  backend.clear_results = {0, 0x4321};
  auto result = run(request(2));
  EXPECT_EQ(result.sdk_error, 0x1234);
  EXPECT_TRUE(recovery.load());
  EXPECT_NE(result.message.find("0x4321"), std::string::npos);
  backend.events.clear();
  EXPECT_EQ(run(request()).code, hc::MotionResultCode::kDriverNotReady);
  EXPECT_TRUE(backend.events.empty());
}

TEST_F(QueueExecutorTest, CollisionRetriesWholeQueueWithFreshPersistedIds)
{
  backend.add_results = {kCollision, 0, 0};
  const auto result = run(request(2));
  ASSERT_EQ(result.code, hc::MotionResultCode::kSuccess);
  EXPECT_EQ(result.path_ids, (std::vector<int32_t>{105, 107}));
  ASSERT_EQ(backend.added.size(), 3U);
  EXPECT_EQ(backend.added[0].path_id, 101);
  EXPECT_EQ(backend.added[1].path_id, 105);
  EXPECT_EQ(backend.added[2].path_id, 107);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 1);
  std::ifstream input(state_file);
  int32_t high_water = 0;
  input >> high_water;
  EXPECT_EQ(high_water, 108);
}

TEST_F(QueueExecutorTest, StartFailureIsStoppedNotRetriedAndRequiresRecovery)
{
  backend.start_result = 0x5555;
  const auto result = run(request());
  EXPECT_EQ(result.code, hc::MotionResultCode::kSdkError);
  EXPECT_EQ(result.sdk_error, 0x5555);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 1);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "stop"), 1);
  EXPECT_TRUE(recovery.load());
  EXPECT_NE(result.message.find("not automatically retried"), std::string::npos);
}

TEST_F(QueueExecutorTest, CancellationBeforeWorkerConsumesNoIds)
{
  cancel.store(true);
  const auto result = run(request());
  EXPECT_EQ(result.code, hc::MotionResultCode::kCanceled);
  EXPECT_TRUE(result.path_ids.empty());
  EXPECT_TRUE(backend.events.empty());
}

TEST_F(QueueExecutorTest, CancellationBetweenAddsClearsAndNeverStarts)
{
  backend.cancel_flag = &cancel;
  backend.cancel_after_add_call = 0;
  const auto result = run(request(2));
  EXPECT_EQ(result.code, hc::MotionResultCode::kCanceled);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "clear"), 2);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 0);
}

TEST_F(QueueExecutorTest, StopAfterLastAddSuppressesStart)
{
  backend.increment_stop_after_add_call = 1;
  const auto result = run(request(2));
  EXPECT_EQ(result.code, hc::MotionResultCode::kCanceled);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 0);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "clear"), 2);
}

TEST_F(QueueExecutorTest, ExecutingCancelRequestsAndConfirmsStop)
{
  backend.cancel_flag = &cancel;
  backend.cancel_on_sample = 1;
  const auto result = run(request());
  EXPECT_EQ(result.code, hc::MotionResultCode::kCanceled);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "stop"), 1);
  EXPECT_FALSE(recovery.load());
  EXPECT_NE(result.message.find("stop confirmed"), std::string::npos);
}

TEST_F(QueueExecutorTest, StaleHarStopsExecution)
{
  backend.force_stale = true;
  const auto result = run(request());
  EXPECT_EQ(result.code, hc::MotionResultCode::kStateLost);
  EXPECT_EQ(std::count(backend.events.begin(), backend.events.end(), "start"), 0);
}

TEST_F(QueueExecutorTest, WrongPathCallbackCannotProveCompletion)
{
  backend.wrong_callback_path = true;
  auto value = request();
  value.options.timeout = std::chrono::milliseconds(200);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kSdkError);
  EXPECT_EQ(result.failure_stage, hc::PathFailureStage::kStop);
  EXPECT_TRUE(recovery.load());
  EXPECT_NE(result.message.find("queue underflow/premature stop"), std::string::npos);
}

TEST_F(QueueExecutorTest, RepeatedHarSampleCannotSupplyStableSamples)
{
  backend.repeat_robot_timestamp = true;
  backend.freeze_robot_sequence = true;
  auto value = request();
  value.options.timeout = std::chrono::milliseconds(200);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kTimeout);
}

TEST_F(QueueExecutorTest, MissingTerminalQueueEvidenceCannotSucceedAtFinalPose)
{
  backend.provide_terminal_evidence = false;
  auto value = request();
  value.options.timeout = std::chrono::milliseconds(200);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kTimeout);
}

TEST_F(QueueExecutorTest, Controller212ZeroTargetSentinelCompletesAfterRunningEvidence)
{
  backend.legacy_zero_target_sentinel = true;
  const auto result = run(request());
  EXPECT_EQ(result.code, hc::MotionResultCode::kSuccess);
  EXPECT_FALSE(recovery.load());
  EXPECT_NE(result.message.find("terminal_evidence=true"), std::string::npos);
  EXPECT_NE(result.message.find("pre_target=0, next_target=0"), std::string::npos);
}

TEST_F(QueueExecutorTest, ZeroTargetSentinelWithoutRunningEvidenceCannotComplete)
{
  backend.legacy_zero_target_sentinel = true;
  backend.provide_running_evidence = false;
  auto value = request();
  value.options.timeout = std::chrono::milliseconds(200);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kTimeout);
  EXPECT_NE(result.message.find("missing_running_queue_evidence"), std::string::npos);
}

TEST_F(QueueExecutorTest, ZeroTargetSentinelMustBeNewerThanRunningCallback)
{
  backend.legacy_zero_target_sentinel = true;
  backend.reuse_running_sequence_for_terminal = true;
  auto value = request();
  value.options.timeout = std::chrono::milliseconds(200);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kTimeout);
  EXPECT_NE(result.message.find("final_target_id_not_referenced"), std::string::npos);
}

TEST_F(QueueExecutorTest, WrongNonzeroFinalTargetCannotComplete)
{
  backend.wrong_terminal_target_id = true;
  auto value = request();
  value.options.timeout = std::chrono::milliseconds(200);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kTimeout);
  EXPECT_NE(result.message.find("final_target_id_not_referenced"), std::string::npos);
}

TEST_F(QueueExecutorTest, ExpiredMotionDeadlineUsesIndependentStopConfirmationWindow)
{
  backend.stop_confirmation_delay = std::chrono::milliseconds(100);
  auto value = request();
  value.options.timeout = std::chrono::milliseconds(100);
  value.options.stop_confirmation_timeout = std::chrono::milliseconds(300);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kTimeout);
  EXPECT_EQ(result.failure_stage, hc::PathFailureStage::kExecution);
  EXPECT_FALSE(recovery.load());
  EXPECT_EQ(result.execution_time, std::chrono::milliseconds(200));
  EXPECT_NE(result.message.find("stop confirmed"), std::string::npos);
}

TEST_F(QueueExecutorTest, StopConfirmationTimeoutSetsRecoveryLockWithoutUnboundedWait)
{
  backend.confirm_stop = false;
  auto value = request();
  value.options.timeout = std::chrono::milliseconds(100);
  value.options.stop_confirmation_timeout = std::chrono::milliseconds(150);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kTimeout);
  EXPECT_EQ(result.failure_stage, hc::PathFailureStage::kStop);
  EXPECT_TRUE(recovery.load());
  EXPECT_EQ(result.execution_time, std::chrono::milliseconds(250));
  EXPECT_NE(result.message.find("stop confirmation timed out after 150 ms"), std::string::npos);
}

TEST_F(QueueExecutorTest, StopSdkFailurePreservesErrorAndSetsRecoveryLock)
{
  backend.stop_result = 0x2468;
  auto value = request();
  value.options.timeout = std::chrono::milliseconds(100);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kTimeout);
  EXPECT_EQ(result.sdk_error, 0x2468);
  EXPECT_EQ(result.failure_stage, hc::PathFailureStage::kStop);
  EXPECT_TRUE(recovery.load());
  EXPECT_NE(result.message.find("0x2468"), std::string::npos);
}

TEST_F(QueueExecutorTest, CachedStoppedCallbackBeforeStopCannotConfirmNewRequest)
{
  backend.pre_stop_cached_callback = true;
  backend.confirm_stop = false;
  auto value = request();
  value.options.timeout = std::chrono::milliseconds(100);
  value.options.stop_confirmation_timeout = std::chrono::milliseconds(100);
  const auto result = run(value);
  EXPECT_EQ(result.failure_stage, hc::PathFailureStage::kStop);
  EXPECT_TRUE(recovery.load());
  EXPECT_NE(result.message.find("motion_seq=77"), std::string::npos);
}

TEST_F(QueueExecutorTest, FreshStoppedCallbackWithWrongPathCannotConfirmStop)
{
  backend.stop_callback_wrong_path = true;
  auto value = request();
  value.options.timeout = std::chrono::milliseconds(100);
  value.options.stop_confirmation_timeout = std::chrono::milliseconds(100);
  const auto result = run(value);
  EXPECT_EQ(result.failure_stage, hc::PathFailureStage::kStop);
  EXPECT_TRUE(recovery.load());
  EXPECT_NE(result.message.find("path_id=999999"), std::string::npos);
}

TEST_F(QueueExecutorTest, FreshMatchingStoppedCallbackConfirmsStop)
{
  backend.stop_confirmation_delay = std::chrono::milliseconds(50);
  auto value = request();
  value.options.timeout = std::chrono::milliseconds(100);
  value.options.stop_confirmation_timeout = std::chrono::milliseconds(100);
  const auto result = run(value);
  EXPECT_EQ(result.failure_stage, hc::PathFailureStage::kExecution);
  EXPECT_FALSE(recovery.load());
  EXPECT_NE(result.message.find("stop confirmed"), std::string::npos);
}

TEST_F(QueueExecutorTest, MatchingStoppedCallbackDuringBlockingStopCallConfirmsStop)
{
  backend.stop_callback_during_call = true;
  auto value = request();
  value.options.timeout = std::chrono::milliseconds(100);
  value.options.stop_confirmation_timeout = std::chrono::milliseconds(100);
  const auto result = run(value);
  EXPECT_EQ(result.code, hc::MotionResultCode::kTimeout);
  EXPECT_EQ(result.failure_stage, hc::PathFailureStage::kExecution);
  EXPECT_FALSE(recovery.load());
  EXPECT_NE(result.message.find("stop confirmed"), std::string::npos);
}

TEST_F(QueueExecutorTest, MotionCallbackSequenceAndSnapshotDiagnosticsAreConsistent)
{
  std::vector<hc::CartesianPathFeedback> values;
  ASSERT_EQ(run_with_feedback(request(), values).code, hc::MotionResultCode::kSuccess);
  uint64_t previous_sequence = 0;
  bool saw_callback = false;
  for (const auto & value : values) {
    if (value.stage != hc::PathExecutionStage::kExecuting ||
      !value.motion_callback_valid)
    {
      continue;
    }
    EXPECT_GE(value.motion_callback_sequence, previous_sequence);
    previous_sequence = value.motion_callback_sequence;
    EXPECT_EQ(value.motion_callback_path_matches, value.current_segment_valid);
    if (value.motion_callback_path_matches) {
      EXPECT_EQ(value.motion_callback_path_id, value.current_path_id);
    }
    saw_callback = true;
  }
  EXPECT_TRUE(saw_callback);
}

TEST_F(QueueExecutorTest, HarSequenceNotTimestampDrivesStableSampleCounting)
{
  backend.repeat_robot_timestamp = true;
  const auto result = run(request());
  EXPECT_EQ(result.code, hc::MotionResultCode::kSuccess);
}

TEST_F(QueueExecutorTest, CompletionDiagnosticsIdentifyEachBlockingCondition)
{
  const auto last_executing = [](const std::vector<hc::CartesianPathFeedback> & values) {
      for (auto it = values.rbegin(); it != values.rend(); ++it) {
        if (it->stage == hc::PathExecutionStage::kExecuting) {
          return *it;
        }
      }
      return hc::CartesianPathFeedback{};
    };

  auto run_case = [this, &last_executing]() {
      std::vector<hc::CartesianPathFeedback> values;
      auto value = request();
      value.options.timeout = std::chrono::milliseconds(200);
      (void)run_with_feedback(value, values);
      return last_executing(values);
    };

  backend.provide_running_evidence = false;
  auto diagnostic = run_case();
  EXPECT_EQ(diagnostic.completion_blocker, "missing_running_queue_evidence");

  backend = FakeBackend{};
  backend.sequence_file = state_file.string();
  backend.provide_terminal_evidence = false;
  diagnostic = run_case();
  EXPECT_FALSE(diagnostic.observed_terminal_queue_evidence);
  EXPECT_EQ(diagnostic.completion_blocker, "final_segment_not_observed");

  backend = FakeBackend{};
  backend.sequence_file = state_file.string();
  backend.never_reach_target = true;
  diagnostic = run_case();
  EXPECT_EQ(diagnostic.completion_blocker, "final_tcp_out_of_tolerance");

  backend = FakeBackend{};
  backend.sequence_file = state_file.string();
  backend.execution_motion_status = 1;
  diagnostic = run_case();
  EXPECT_EQ(diagnostic.completion_blocker, "har_motion_status_not_stopped");

  backend = FakeBackend{};
  backend.sequence_file = state_file.string();
  backend.freeze_robot_sequence = true;
  diagnostic = run_case();
  EXPECT_EQ(diagnostic.completion_blocker, "no_new_har_sample");

  backend = FakeBackend{};
  backend.sequence_file = state_file.string();
  std::vector<hc::CartesianPathFeedback> success_values;
  EXPECT_EQ(run_with_feedback(request(), success_values).code, hc::MotionResultCode::kSuccess);
  diagnostic = last_executing(success_values);
  EXPECT_EQ(diagnostic.completion_blocker, "complete");
  EXPECT_EQ(diagnostic.stable_sample_count, diagnostic.required_stable_samples);
}

}  // namespace
