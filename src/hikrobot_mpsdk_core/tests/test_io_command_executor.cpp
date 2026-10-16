#include "hikrobot_mpsdk_core/io_command_executor.hpp"

#include "hikrobot_mpsdk_core/command_channel_health.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

namespace hikrobot_mpsdk_core
{
namespace
{

using namespace std::chrono_literals;

IoResult success(bool value = false)
{
  IoResult out;
  out.success = true;
  out.value = value;
  out.message = "ok";
  return out;
}

TEST(IoCommandExecutorTest, NormalGetAndSetStyleTasksReturnValues)
{
  IoCommandExecutor executor;
  executor.resume();

  const auto get = executor.execute(
    [](const IoCommandExecutor::StartCallback & start) {
      EXPECT_TRUE(start());
      return success(true);
    }, 100ms, 100ms, {});
  const auto set = executor.execute(
    [](const IoCommandExecutor::StartCallback & start) {
      EXPECT_TRUE(start());
      return success(false);
    }, 100ms, 100ms, {});

  EXPECT_TRUE(get.success);
  EXPECT_TRUE(get.value);
  EXPECT_TRUE(set.success);
  EXPECT_FALSE(set.value);
}

TEST(IoCommandExecutorTest, MultipleRequestsUseExactlyOneSerialWorker)
{
  IoCommandExecutor executor;
  executor.resume();
  std::atomic<int> active{0};
  std::atomic<int> max_active{0};
  std::mutex ids_mutex;
  std::set<std::thread::id> worker_ids;
  std::vector<std::future<IoResult>> calls;

  for (int i = 0; i < 4; ++i) {
    calls.push_back(std::async(std::launch::async, [&]() {
      return executor.execute(
        [&](const IoCommandExecutor::StartCallback & start) {
          if (!start()) {
            return IoResult{};
          }
          const int now = active.fetch_add(1) + 1;
          int observed = max_active.load();
          while (now > observed && !max_active.compare_exchange_weak(observed, now)) {}
          {
            std::lock_guard<std::mutex> lock(ids_mutex);
            worker_ids.insert(std::this_thread::get_id());
          }
          std::this_thread::sleep_for(10ms);
          active.fetch_sub(1);
          return success();
        }, 500ms, 500ms, {});
    }));
  }

  for (auto & call : calls) {
    EXPECT_TRUE(call.get().success);
  }
  EXPECT_EQ(max_active.load(), 1);
  EXPECT_EQ(worker_ids.size(), 1U);
  EXPECT_EQ(executor.worker_start_count(), 1U);
}

TEST(IoCommandExecutorTest, QueuedTimeoutCancelsTaskBeforeItCanExecute)
{
  IoCommandExecutor executor;
  executor.resume();
  std::mutex block_mutex;
  std::condition_variable block_cv;
  bool first_started = false;
  bool release_first = false;
  std::atomic<int> second_sdk_calls{0};

  auto first = std::async(std::launch::async, [&]() {
    return executor.execute(
      [&](const IoCommandExecutor::StartCallback & start) {
        EXPECT_TRUE(start());
        std::unique_lock<std::mutex> lock(block_mutex);
        first_started = true;
        block_cv.notify_all();
        block_cv.wait(lock, [&]() {return release_first;});
        return success();
      }, 200ms, 1000ms, {});
  });
  {
    std::unique_lock<std::mutex> lock(block_mutex);
    ASSERT_TRUE(block_cv.wait_for(lock, 500ms, [&]() {return first_started;}));
  }

  const auto second = executor.execute(
    [&](const IoCommandExecutor::StartCallback & start) {
      if (start()) {
        ++second_sdk_calls;
      }
      return success();
    }, 30ms, 100ms, {});
  EXPECT_EQ(second.sdk_error, kLocalSdkCommandBusy);

  {
    std::lock_guard<std::mutex> lock(block_mutex);
    release_first = true;
  }
  block_cv.notify_all();
  EXPECT_TRUE(first.get().success);
  EXPECT_EQ(second_sdk_calls.load(), 0);
}

TEST(IoCommandExecutorTest, SharedSdkMutexWaitCanBeCancelledBeforeSdkEntry)
{
  IoCommandExecutor executor;
  executor.resume();
  std::timed_mutex sdk_mutex;
  sdk_mutex.lock();  // Simulates a motion command currently owning the same SDK mutex.
  std::atomic<int> sdk_calls{0};

  const auto started = std::chrono::steady_clock::now();
  const auto result = executor.execute(
    [&](const IoCommandExecutor::StartCallback & start) {
      std::unique_lock<std::timed_mutex> lock(sdk_mutex, std::defer_lock);
      if (!lock.try_lock_for(150ms)) {
        IoResult busy;
        busy.sdk_error = kLocalSdkCommandBusy;
        return busy;
      }
      if (start()) {
        ++sdk_calls;
      }
      return success();
    }, 30ms, 100ms, {});
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_EQ(result.sdk_error, kLocalSdkCommandBusy);
  EXPECT_LT(elapsed, 100ms);
  sdk_mutex.unlock();

  ASSERT_TRUE(executor.pause_and_wait_idle(500ms));
  EXPECT_EQ(sdk_calls.load(), 0);
}

TEST(IoCommandExecutorTest, RunningSdkTimeoutPoisonsAndLaterRequestsFailFast)
{
  IoCommandExecutor executor;
  executor.resume();
  CommandChannelHealth health;
  health.mark_configured_connected();
  std::mutex block_mutex;
  std::condition_variable block_cv;
  bool release = false;
  std::atomic<int> sdk_calls{0};

  auto blocked = std::async(std::launch::async, [&]() {
    return executor.execute(
      [&](const IoCommandExecutor::StartCallback & start) {
        EXPECT_TRUE(start());
        health.begin_call("FakeSdkGet", std::chrono::steady_clock::now());
        ++sdk_calls;
        std::unique_lock<std::mutex> lock(block_mutex);
        block_cv.wait(lock, [&]() {return release;});
        health.end_call(0, "FakeSdkGet", std::chrono::steady_clock::now());
        return success(true);
      }, 100ms, 40ms, [&]() {health.mark_call_timeout("FakeSdkGet", 40ms);});
  });

  const auto timed_out = blocked.get();
  EXPECT_EQ(timed_out.sdk_error, kLocalSdkCommandTimeout);
  EXPECT_TRUE(health.snapshot().recovery_required);
  EXPECT_EQ(health.snapshot().fault_code, kLocalSdkCommandTimeout);

  const auto rejected_started = std::chrono::steady_clock::now();
  const auto rejected = executor.execute(
    [&](const IoCommandExecutor::StartCallback &) {
      ++sdk_calls;
      return success();
    }, 100ms, 100ms, {});
  EXPECT_EQ(rejected.sdk_error, kLocalSdkRecoveryRequired);
  EXPECT_LT(std::chrono::steady_clock::now() - rejected_started, 50ms);

  {
    std::lock_guard<std::mutex> lock(block_mutex);
    release = true;
  }
  block_cv.notify_all();
  ASSERT_TRUE(executor.pause_and_wait_idle(500ms));
  EXPECT_EQ(sdk_calls.load(), 1);
  EXPECT_TRUE(health.snapshot().recovery_required);
  EXPECT_EQ(health.snapshot().fault_code, kLocalSdkCommandTimeout);
}

TEST(IoCommandExecutorTest, PauseCancelsDequeuedButNotStartedTaskSafely)
{
  IoCommandExecutor executor;
  executor.resume();
  std::timed_mutex sdk_mutex;
  sdk_mutex.lock();
  std::atomic<int> sdk_calls{0};

  auto call = std::async(std::launch::async, [&]() {
    return executor.execute(
      [&](const IoCommandExecutor::StartCallback & start) {
        std::unique_lock<std::timed_mutex> lock(sdk_mutex, std::defer_lock);
        if (!lock.try_lock_for(300ms)) {
          IoResult busy;
          busy.sdk_error = kLocalSdkCommandBusy;
          return busy;
        }
        if (start()) {
          ++sdk_calls;
        }
        return success();
      }, 500ms, 500ms, {});
  });
  std::this_thread::sleep_for(20ms);

  auto pause = std::async(std::launch::async, [&]() {
    return executor.pause_and_wait_idle(500ms);
  });
  std::this_thread::sleep_for(20ms);
  sdk_mutex.unlock();

  EXPECT_TRUE(pause.get());
  EXPECT_EQ(call.get().sdk_error, kLocalSdkRecoveryRequired);
  EXPECT_EQ(sdk_calls.load(), 0);
  EXPECT_TRUE(executor.idle());
}

TEST(IoCommandExecutorTest, NormalDestructionJoinsWorker)
{
  auto executor = std::make_unique<IoCommandExecutor>();
  executor->resume();
  EXPECT_TRUE(executor->execute(
      [](const IoCommandExecutor::StartCallback & start) {
        EXPECT_TRUE(start());
        return success();
      }, 100ms, 100ms, {}).success);
  executor.reset();
  SUCCEED();
}

}  // namespace
}  // namespace hikrobot_mpsdk_core
