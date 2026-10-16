#include "hikrobot_driver/health_monitor.hpp"
#include "hikrobot_mpsdk_core/command_channel_health.hpp"

#include <algorithm>

#include "gtest/gtest.h"

namespace hikrobot_driver
{
namespace
{

bool contains(const std::vector<HealthEvent> & events, HealthEvent event)
{
  return std::find(events.begin(), events.end(), event) != events.end();
}

hikrobot_mpsdk_core::RuntimeHealthSnapshot healthy_snapshot()
{
  hikrobot_mpsdk_core::RuntimeHealthSnapshot out;
  out.monitoring_active = true;
  out.sdk_connected = true;
  out.command_connected = true;
  out.har_received = true;
  return out;
}

TEST(HealthTransitionTrackerTest, ReportsHealthyStartupOnlyOnce)
{
  HealthTransitionTracker tracker;
  const auto state = healthy_snapshot();
  const auto first = tracker.update(state);
  EXPECT_TRUE(contains(first, HealthEvent::kMonitorStarted));
  EXPECT_TRUE(contains(first, HealthEvent::kCommunicationHealthy));
  EXPECT_TRUE(tracker.update(state).empty());
}

TEST(HealthTransitionTrackerTest, DistinguishesDisconnectAndReconnect)
{
  HealthTransitionTracker tracker;
  auto state = healthy_snapshot();
  (void)tracker.update(state);
  state.sdk_connected = false;
  state.command_connected = false;
  EXPECT_TRUE(contains(tracker.update(state), HealthEvent::kSdkDisconnected));
  state.sdk_connected = true;
  EXPECT_TRUE(contains(tracker.update(state), HealthEvent::kSdkReconnected));
}

TEST(HealthTransitionTrackerTest, ReportsHarTimeoutTransitionWithoutLogSpam)
{
  HealthTransitionTracker tracker;
  auto state = healthy_snapshot();
  (void)tracker.update(state);
  state.har_timed_out = true;
  state.command_connected = false;
  state.recovery_required = true;
  const auto failed = tracker.update(state);
  EXPECT_TRUE(contains(failed, HealthEvent::kHarTimedOut));
  EXPECT_TRUE(contains(failed, HealthEvent::kRecoveryRequired));
  EXPECT_TRUE(tracker.update(state).empty());
}

TEST(HealthTransitionTrackerTest, UsesExplicitEmergencyState)
{
  HealthTransitionTracker tracker;
  auto state = healthy_snapshot();
  state.emergency_status = 2;
  state.emergency_active = false;
  EXPECT_FALSE(contains(tracker.update(state), HealthEvent::kEmergencyActivated));
  state.emergency_status = 1;
  state.emergency_active = true;
  EXPECT_TRUE(contains(tracker.update(state), HealthEvent::kEmergencyActivated));
  state.emergency_status = 0;
  state.emergency_active = false;
  EXPECT_TRUE(contains(tracker.update(state), HealthEvent::kEmergencyReleased));
}

TEST(HealthTransitionTrackerTest, DistinguishesCommandTimeoutFromGenericRecovery)
{
  HealthTransitionTracker tracker;
  auto state = healthy_snapshot();
  (void)tracker.update(state);
  state.command_connected = false;
  state.recovery_required = true;
  state.fault_code = hikrobot_mpsdk_core::kLocalSdkCommandTimeout;
  const auto events = tracker.update(state);
  EXPECT_TRUE(contains(events, HealthEvent::kCommandTimedOut));
  EXPECT_TRUE(contains(events, HealthEvent::kRecoveryRequired));
}

TEST(HealthTransitionTrackerTest, UnmonitoredLifecycleStateResetsTransitions)
{
  HealthTransitionTracker tracker;
  auto state = healthy_snapshot();
  (void)tracker.update(state);
  state.monitoring_active = false;
  EXPECT_TRUE(tracker.update(state).empty());
  state = healthy_snapshot();
  EXPECT_TRUE(contains(tracker.update(state), HealthEvent::kMonitorStarted));
}

}  // namespace
}  // namespace hikrobot_driver
