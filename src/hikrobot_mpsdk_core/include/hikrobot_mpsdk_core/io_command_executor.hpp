#pragma once

#include "hikrobot_mpsdk_core/types.hpp"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace hikrobot_mpsdk_core
{

enum class IoTaskState
{
  kQueued,
  kRunning,
  kCompleted,
  kCancelled,
};

class IoCommandExecutor
{
public:
  using StartCallback = std::function<bool()>;
  using Operation = std::function<IoResult(const StartCallback &)>;
  using TimeoutCallback = std::function<void()>;

  IoCommandExecutor();
  ~IoCommandExecutor();

  IoCommandExecutor(const IoCommandExecutor &) = delete;
  IoCommandExecutor & operator=(const IoCommandExecutor &) = delete;

  IoResult execute(
    Operation operation, std::chrono::milliseconds queue_timeout,
    std::chrono::milliseconds execution_timeout, TimeoutCallback on_execution_timeout);

  void resume();
  bool pause_and_wait_idle(std::chrono::milliseconds timeout);
  bool idle() const;
  std::size_t worker_start_count() const;

private:
  struct Task;

  void worker_loop();
  void poison_and_cancel_queued();
  static void cancel_task(const std::shared_ptr<Task> & task, const IoResult & result);

  mutable std::mutex mutex_;
  std::condition_variable work_cv_;
  std::condition_variable idle_cv_;
  std::deque<std::shared_ptr<Task>> queue_;
  std::shared_ptr<Task> current_task_;
  bool accepting_{false};
  bool stopping_{false};
  bool worker_occupied_{false};
  std::size_t worker_start_count_{0};
  std::thread worker_;
};

}  // namespace hikrobot_mpsdk_core
