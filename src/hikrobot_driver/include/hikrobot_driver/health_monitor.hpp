#pragma once

#include "hikrobot_mpsdk_core/robot_runtime.hpp"

#include <vector>

namespace hikrobot_driver
{

enum class HealthEvent
{
  kMonitorStarted,
  kCommunicationHealthy,
  kSdkDisconnected,
  kSdkReconnected,
  kHarTimedOut,
  kHarRestored,
  kEmergencyActivated,
  kEmergencyReleased,
  kCommandTimedOut,
  kRecoveryRequired,
};

class HealthTransitionTracker
{
public:
  std::vector<HealthEvent> update(
    const hikrobot_mpsdk_core::RuntimeHealthSnapshot & snapshot);
  void reset();

private:
  bool initialized_{false};
  bool sdk_connected_{false};
  bool har_timed_out_{false};
  bool emergency_active_{false};
  bool command_timed_out_{false};
  bool recovery_required_{false};
  bool communication_healthy_{false};
};

}  // namespace hikrobot_driver
