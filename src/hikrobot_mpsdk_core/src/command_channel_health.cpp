#include "hikrobot_mpsdk_core/command_channel_health.hpp"

#include <sstream>

namespace hikrobot_mpsdk_core
{
namespace
{

constexpr int64_t kNetworkErrorBegin = 0x000020000;
constexpr int64_t kNetworkErrorEnd = 0x00002000B;

CommandChannelSnapshot make_snapshot(
  bool connected, bool recovery_required, bool call_in_progress,
  std::chrono::milliseconds call_elapsed, int64_t fault_code,
  const std::string & fault_message, const std::string & active_operation)
{
  CommandChannelSnapshot out;
  out.connected = connected && !recovery_required;
  out.recovery_required = recovery_required;
  out.call_in_progress = call_in_progress;
  out.call_elapsed = call_elapsed;
  out.fault_code = fault_code;
  out.fault_message = fault_message;
  out.active_operation = active_operation;
  return out;
}

}  // namespace

bool is_sdk_transport_error(int64_t code)
{
  return code >= kNetworkErrorBegin && code <= kNetworkErrorEnd;
}

void CommandChannelHealth::mark_configured_connected()
{
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = true;
  recovery_required_ = false;
  fault_code_ = 0;
  fault_message_.clear();
}

void CommandChannelHealth::mark_clean_disconnected()
{
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = false;
  if (!recovery_required_) {
    fault_code_ = 0;
    fault_message_.clear();
  }
}

void CommandChannelHealth::on_connection_status(bool connected)
{
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = connected;
  if (!connected) {
    recovery_required_ = true;
    fault_code_ = kLocalSdkRecoveryRequired;
    fault_message_ =
      "SDK connection callback reported disconnected; restart the Driver process before reuse";
  }
}

void CommandChannelHealth::begin_call(
  const std::string & operation, std::chrono::steady_clock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  call_in_progress_ = true;
  call_started_at_ = now;
  active_operation_ = operation;
}

void CommandChannelHealth::end_call(
  int64_t sdk_result, const std::string & operation,
  std::chrono::steady_clock::time_point)
{
  std::lock_guard<std::mutex> lock(mutex_);
  call_in_progress_ = false;
  active_operation_.clear();
  if (is_sdk_transport_error(sdk_result)) {
    connected_ = false;
    recovery_required_ = true;
    fault_code_ = sdk_result;
    std::ostringstream message;
    message << operation << " returned SDK transport error 0x" << std::hex << std::uppercase <<
      sdk_result << "; restart the Driver process before reuse";
    fault_message_ = message.str();
  }
}

CommandChannelSnapshot CommandChannelHealth::poll(
  std::chrono::steady_clock::time_point now,
  std::chrono::milliseconds watchdog_timeout)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::chrono::milliseconds elapsed{0};
  if (call_in_progress_) {
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - call_started_at_);
    if (watchdog_timeout.count() > 0 && elapsed >= watchdog_timeout && !recovery_required_) {
      connected_ = false;
      recovery_required_ = true;
      fault_code_ = kLocalSdkCommandTimeout;
      std::ostringstream message;
      message << active_operation_ << " exceeded the " << watchdog_timeout.count() <<
        " ms SDK command watchdog; outcome is unknown and the Driver process must be restarted";
      fault_message_ = message.str();
    }
  }
  return make_snapshot(
    connected_, recovery_required_, call_in_progress_, elapsed,
    fault_code_, fault_message_, active_operation_);
}

CommandChannelSnapshot CommandChannelHealth::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return make_snapshot(
    connected_, recovery_required_, call_in_progress_, std::chrono::milliseconds{0},
    fault_code_, fault_message_, active_operation_);
}

}  // namespace hikrobot_mpsdk_core
