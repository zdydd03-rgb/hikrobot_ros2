#include "hikrobot_driver/health_monitor.hpp"

#include "hikrobot_mpsdk_core/command_channel_health.hpp"

namespace hikrobot_driver
{

std::vector<HealthEvent> HealthTransitionTracker::update(
  const hikrobot_mpsdk_core::RuntimeHealthSnapshot & snapshot)
{
  std::vector<HealthEvent> events;
  if (!snapshot.monitoring_active) {
    reset();
    return events;
  }

  const bool command_timed_out =
    snapshot.fault_code == hikrobot_mpsdk_core::kLocalSdkCommandTimeout;
  const bool healthy = snapshot.sdk_connected && snapshot.command_connected &&
    snapshot.har_received && !snapshot.har_timed_out && !snapshot.recovery_required;

  if (!initialized_) {
    initialized_ = true;
    events.push_back(HealthEvent::kMonitorStarted);
    if (!snapshot.sdk_connected) {
      events.push_back(HealthEvent::kSdkDisconnected);
    }
    if (snapshot.har_timed_out) {
      events.push_back(HealthEvent::kHarTimedOut);
    }
    if (snapshot.emergency_active) {
      events.push_back(HealthEvent::kEmergencyActivated);
    }
    if (command_timed_out) {
      events.push_back(HealthEvent::kCommandTimedOut);
    }
    if (snapshot.recovery_required) {
      events.push_back(HealthEvent::kRecoveryRequired);
    }
    if (healthy) {
      events.push_back(HealthEvent::kCommunicationHealthy);
    }
  } else {
    if (sdk_connected_ != snapshot.sdk_connected) {
      events.push_back(
        snapshot.sdk_connected ? HealthEvent::kSdkReconnected : HealthEvent::kSdkDisconnected);
    }
    if (har_timed_out_ != snapshot.har_timed_out) {
      events.push_back(
        snapshot.har_timed_out ? HealthEvent::kHarTimedOut : HealthEvent::kHarRestored);
    }
    if (emergency_active_ != snapshot.emergency_active) {
      events.push_back(
        snapshot.emergency_active ? HealthEvent::kEmergencyActivated :
        HealthEvent::kEmergencyReleased);
    }
    if (!command_timed_out_ && command_timed_out) {
      events.push_back(HealthEvent::kCommandTimedOut);
    }
    if (!recovery_required_ && snapshot.recovery_required) {
      events.push_back(HealthEvent::kRecoveryRequired);
    }
    if (!communication_healthy_ && healthy) {
      events.push_back(HealthEvent::kCommunicationHealthy);
    }
  }

  sdk_connected_ = snapshot.sdk_connected;
  har_timed_out_ = snapshot.har_timed_out;
  emergency_active_ = snapshot.emergency_active;
  command_timed_out_ = command_timed_out;
  recovery_required_ = snapshot.recovery_required;
  communication_healthy_ = healthy;
  return events;
}

void HealthTransitionTracker::reset()
{
  initialized_ = false;
  sdk_connected_ = false;
  har_timed_out_ = false;
  emergency_active_ = false;
  command_timed_out_ = false;
  recovery_required_ = false;
  communication_healthy_ = false;
}

}  // namespace hikrobot_driver
