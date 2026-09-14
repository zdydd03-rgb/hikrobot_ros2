#include "hikrobot_mpsdk_core/protected_arc_output.hpp"

namespace hikrobot_mpsdk_core
{

void ProtectedArcOutputState::read_result_locked(bool value)
{
  cached_value_ = value;
  state_ = value ? ProtectedOutputState::ASSERTED : ProtectedOutputState::LOW;
}

void ProtectedArcOutputState::write_result_locked(bool value, bool success)
{
  if (!success) {
    state_ = ProtectedOutputState::UNKNOWN;
    return;
  }
  cached_value_ = value;
  state_ = value ? ProtectedOutputState::ASSERTED : ProtectedOutputState::LOW;
}

void ProtectedArcOutputState::mark_unknown_locked()
{
  state_ = ProtectedOutputState::UNKNOWN;
}

void ProtectedArcOutputState::reset_unknown()
{
  auto guard = lock();
  mark_unknown_locked();
}

ProtectedOutputState ProtectedArcOutputState::state() const
{
  auto guard = lock();
  return state_locked();
}

}  // namespace hikrobot_mpsdk_core
