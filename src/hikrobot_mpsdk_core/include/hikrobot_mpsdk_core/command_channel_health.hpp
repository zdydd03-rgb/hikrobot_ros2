#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

namespace hikrobot_mpsdk_core
{

constexpr int64_t kLocalSdkCommandBusy = -10001;
constexpr int64_t kLocalSdkCommandTimeout = -10002;
constexpr int64_t kLocalSdkRecoveryRequired = -10003;
constexpr int64_t kLocalProtectedOutputReadSuppressed = -10004;

bool is_sdk_transport_error(int64_t code);

struct CommandChannelSnapshot
{
  bool connected{false};
  bool recovery_required{false};
  bool call_in_progress{false};
  std::chrono::milliseconds call_elapsed{0};
  int64_t fault_code{0};
  std::string fault_message;
  std::string active_operation;
};

class CommandChannelHealth
{
public:
  void mark_configured_connected();
  void mark_clean_disconnected();
  void on_connection_status(bool connected);
  void begin_call(const std::string & operation, std::chrono::steady_clock::time_point now);
  void end_call(
    int64_t sdk_result, const std::string & operation,
    std::chrono::steady_clock::time_point now);
  CommandChannelSnapshot poll(
    std::chrono::steady_clock::time_point now,
    std::chrono::milliseconds watchdog_timeout);
  CommandChannelSnapshot snapshot() const;

private:
  mutable std::mutex mutex_;
  bool connected_{false};
  bool recovery_required_{false};
  bool call_in_progress_{false};
  std::chrono::steady_clock::time_point call_started_at_{};
  int64_t fault_code_{0};
  std::string fault_message_;
  std::string active_operation_;
};

}  // namespace hikrobot_mpsdk_core
