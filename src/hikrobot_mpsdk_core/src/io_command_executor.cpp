#include "hikrobot_mpsdk_core/io_command_executor.hpp"

#include "hikrobot_mpsdk_core/command_channel_health.hpp"

#include <utility>

namespace hikrobot_mpsdk_core
{

struct IoCommandExecutor::Task
{
  explicit Task(Operation operation_in) : operation(std::move(operation_in)) {}

  std::mutex mutex;
  std::condition_variable cv;
  IoTaskState state{IoTaskState::kQueued};
  std::chrono::steady_clock::time_point running_at{};
  Operation operation;
  IoResult result{};
};

namespace
{

IoResult local_failure(int64_t code, const std::string & message)
{
  IoResult result;
  result.sdk_error = code;
  result.message = message;
  return result;
}

}  // namespace

IoCommandExecutor::IoCommandExecutor()
: worker_(&IoCommandExecutor::worker_loop, this)
{
}

IoCommandExecutor::~IoCommandExecutor()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    accepting_ = false;
    stopping_ = true;
    const auto stopped = local_failure(
      kLocalSdkRecoveryRequired, "IO command executor is shutting down");
    for (const auto & task : queue_) {
      cancel_task(task, stopped);
    }
    if (current_task_) {
      cancel_task(current_task_, stopped);
    }
    queue_.clear();
  }
  work_cv_.notify_all();
  if (worker_.joinable()) {
    // Vendor SDK calls have no cancellation primitive. A permanently blocked
    // call therefore also makes destruction wait; detaching would permit the
    // worker to access a destroyed Runtime and SDK handle.
    worker_.join();
  }
}

IoResult IoCommandExecutor::execute(
  Operation operation, std::chrono::milliseconds queue_timeout,
  std::chrono::milliseconds execution_timeout, TimeoutCallback on_execution_timeout)
{
  if (!operation || queue_timeout.count() <= 0 || execution_timeout.count() <= 0) {
    return local_failure(kLocalSdkRecoveryRequired, "invalid IO command executor request");
  }

  const auto task = std::make_shared<Task>(std::move(operation));
  const auto queued_at = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!accepting_ || stopping_) {
      return local_failure(
        kLocalSdkRecoveryRequired,
        "IO command rejected because the command channel is unavailable");
    }
    queue_.push_back(task);
  }
  work_cv_.notify_one();

  std::unique_lock<std::mutex> task_lock(task->mutex);
  const auto queue_deadline = queued_at + queue_timeout;
  while (task->state == IoTaskState::kQueued) {
    if (task->cv.wait_until(task_lock, queue_deadline) == std::cv_status::timeout &&
      task->state == IoTaskState::kQueued)
    {
      task->state = IoTaskState::kCancelled;
      task->result = local_failure(
        kLocalSdkCommandBusy,
        "IO command timed out before entering the vendor SDK; task was cancelled");
      task_lock.unlock();
      task->cv.notify_all();
      return task->result;
    }
  }

  if (task->state == IoTaskState::kCancelled || task->state == IoTaskState::kCompleted) {
    return task->result;
  }

  const auto execution_deadline = task->running_at + execution_timeout;
  while (task->state == IoTaskState::kRunning) {
    if (task->cv.wait_until(task_lock, execution_deadline) == std::cv_status::timeout &&
      task->state == IoTaskState::kRunning)
    {
      task_lock.unlock();
      poison_and_cancel_queued();
      if (on_execution_timeout) {
        on_execution_timeout();
      }
      return local_failure(
        kLocalSdkCommandTimeout, "SDK command timeout; outcome unknown");
    }
  }
  return task->result;
}

void IoCommandExecutor::resume()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!stopping_) {
    accepting_ = true;
  }
}

bool IoCommandExecutor::pause_and_wait_idle(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  accepting_ = false;
  const auto paused = local_failure(
    kLocalSdkRecoveryRequired, "IO command cancelled because Runtime cleanup has started");
  for (const auto & task : queue_) {
    cancel_task(task, paused);
  }
  if (current_task_) {
    cancel_task(current_task_, paused);
  }
  queue_.clear();
  work_cv_.notify_all();
  return idle_cv_.wait_for(lock, timeout, [this]() {
    return !worker_occupied_ && queue_.empty();
  });
}

bool IoCommandExecutor::idle() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !worker_occupied_ && queue_.empty();
}

std::size_t IoCommandExecutor::worker_start_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return worker_start_count_;
}

void IoCommandExecutor::worker_loop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++worker_start_count_;
  }

  while (true) {
    std::shared_ptr<Task> task;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      work_cv_.wait(lock, [this]() {return stopping_ || !queue_.empty();});
      if (stopping_ && queue_.empty()) {
        return;
      }
      task = queue_.front();
      queue_.pop_front();
      current_task_ = task;
      worker_occupied_ = true;
    }

    bool cancelled = false;
    {
      std::lock_guard<std::mutex> task_lock(task->mutex);
      cancelled = task->state == IoTaskState::kCancelled;
    }

    if (!cancelled) {
      const auto mark_running = [task]() {
          std::lock_guard<std::mutex> lock(task->mutex);
          if (task->state != IoTaskState::kQueued) {
            return false;
          }
          task->state = IoTaskState::kRunning;
          task->running_at = std::chrono::steady_clock::now();
          task->cv.notify_all();
          return true;
        };
      IoResult result = task->operation(mark_running);
      {
        std::lock_guard<std::mutex> task_lock(task->mutex);
        if (task->state != IoTaskState::kCancelled) {
          task->result = std::move(result);
          task->state = IoTaskState::kCompleted;
        }
      }
      task->cv.notify_all();
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      worker_occupied_ = false;
      current_task_.reset();
    }
    idle_cv_.notify_all();
  }
}

void IoCommandExecutor::poison_and_cancel_queued()
{
  std::lock_guard<std::mutex> lock(mutex_);
  accepting_ = false;
  const auto poisoned = local_failure(
    kLocalSdkRecoveryRequired,
    "IO command rejected because SDK command-channel recovery is required");
  for (const auto & task : queue_) {
    cancel_task(task, poisoned);
  }
  if (current_task_) {
    cancel_task(current_task_, poisoned);
  }
  queue_.clear();
  work_cv_.notify_all();
}

void IoCommandExecutor::cancel_task(
  const std::shared_ptr<Task> & task, const IoResult & result)
{
  std::lock_guard<std::mutex> lock(task->mutex);
  if (task->state == IoTaskState::kQueued) {
    task->state = IoTaskState::kCancelled;
    task->result = result;
    task->cv.notify_all();
  }
}

}  // namespace hikrobot_mpsdk_core
