#pragma once

#include <mutex>

namespace hikrobot_mpsdk_core
{

enum class ProtectedOutputState { UNKNOWN, LOW, ASSERTED };

class ProtectedArcOutputState
{
public:
  std::unique_lock<std::mutex> lock() const { return std::unique_lock<std::mutex>(mutex_); }
  ProtectedOutputState state_locked() const { return state_; }
  bool cached_value_locked() const { return cached_value_; }
  void read_result_locked(bool value);
  void write_result_locked(bool value, bool success);
  void mark_unknown_locked();
  void reset_unknown();
  ProtectedOutputState state() const;

private:
  mutable std::mutex mutex_;
  ProtectedOutputState state_{ProtectedOutputState::UNKNOWN};
  bool cached_value_{false};
};

}  // namespace hikrobot_mpsdk_core
