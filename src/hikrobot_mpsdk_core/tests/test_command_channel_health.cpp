#include "hikrobot_mpsdk_core/command_channel_health.hpp"

#include <chrono>

#include "gtest/gtest.h"

namespace hikrobot_mpsdk_core
{
namespace
{

using namespace std::chrono_literals;

TEST(CommandChannelHealthTest, ClassifiesTheOfficialSdkNetworkErrorRange)
{
  EXPECT_TRUE(is_sdk_transport_error(0x000020000));
  EXPECT_TRUE(is_sdk_transport_error(0x000020004));
  EXPECT_TRUE(is_sdk_transport_error(0x000020005));
  EXPECT_TRUE(is_sdk_transport_error(0x00002000B));
  EXPECT_FALSE(is_sdk_transport_error(0));
  EXPECT_FALSE(is_sdk_transport_error(0x000010001));
  EXPECT_FALSE(is_sdk_transport_error(0x821D8004));
}

TEST(CommandChannelHealthTest, ReturnedTransportErrorRequiresProcessRecovery)
{
  CommandChannelHealth health;
  const auto now = std::chrono::steady_clock::time_point{1000ms};
  health.mark_configured_connected();
  health.begin_call("MP_GetConfigIOLevelStatus", now);
  health.end_call(0x000020004, "MP_GetConfigIOLevelStatus", now + 10ms);

  const auto state = health.snapshot();
  EXPECT_FALSE(state.connected);
  EXPECT_TRUE(state.recovery_required);
  EXPECT_EQ(state.fault_code, 0x000020004);
  EXPECT_NE(state.fault_message.find("restart"), std::string::npos);
}

TEST(CommandChannelHealthTest, WatchdogPoisonsUnknownOutcomeWithoutEndingCall)
{
  CommandChannelHealth health;
  const auto now = std::chrono::steady_clock::time_point{1000ms};
  health.mark_configured_connected();
  health.begin_call("MP_SetConfigIOLevelStatus", now);

  EXPECT_TRUE(health.poll(now + 2999ms, 3000ms).connected);
  const auto timed_out = health.poll(now + 3000ms, 3000ms);
  EXPECT_FALSE(timed_out.connected);
  EXPECT_TRUE(timed_out.recovery_required);
  EXPECT_TRUE(timed_out.call_in_progress);
  EXPECT_EQ(timed_out.fault_code, kLocalSdkCommandTimeout);
  EXPECT_NE(timed_out.fault_message.find("outcome is unknown"), std::string::npos);

  health.end_call(0, "MP_SetConfigIOLevelStatus", now + 4000ms);
  const auto completed_late = health.snapshot();
  EXPECT_TRUE(completed_late.recovery_required);
  EXPECT_FALSE(completed_late.connected);
}

TEST(CommandChannelHealthTest, DisconnectCallbackStaysPoisonedAfterReconnectCallback)
{
  CommandChannelHealth health;
  health.mark_configured_connected();
  health.on_connection_status(false);
  health.on_connection_status(true);

  const auto state = health.snapshot();
  EXPECT_FALSE(state.connected);
  EXPECT_TRUE(state.recovery_required);
}

TEST(CommandChannelHealthTest, CleanDisconnectDoesNotCreateRecoveryRequirement)
{
  CommandChannelHealth health;
  health.mark_configured_connected();
  health.mark_clean_disconnected();

  const auto state = health.snapshot();
  EXPECT_FALSE(state.connected);
  EXPECT_FALSE(state.recovery_required);
}

TEST(CommandChannelHealthTest, FeedbackTimeoutRequiresRecovery)
{
  CommandChannelHealth health;
  health.mark_configured_connected();
  health.mark_feedback_timeout(3001ms);

  const auto state = health.snapshot();
  EXPECT_FALSE(state.connected);
  EXPECT_TRUE(state.recovery_required);
  EXPECT_EQ(state.fault_code, kLocalSdkRecoveryRequired);
  EXPECT_NE(state.fault_message.find("HAR robot-state feedback"), std::string::npos);
}

TEST(CommandChannelHealthTest, FeedbackTimeoutDoesNotReplaceEarlierCommandTimeout)
{
  CommandChannelHealth health;
  const auto now = std::chrono::steady_clock::time_point{1000ms};
  health.mark_configured_connected();
  health.begin_call("blocked SDK call", now);
  const auto command_timeout = health.poll(now + 3000ms, 3000ms);
  health.mark_feedback_timeout(4000ms);

  const auto state = health.snapshot();
  EXPECT_EQ(state.fault_code, kLocalSdkCommandTimeout);
  EXPECT_EQ(state.fault_message, command_timeout.fault_message);
}

TEST(CommandChannelHealthTest, ServiceDeadlineCanPoisonAnInFlightCallDirectly)
{
  CommandChannelHealth health;
  const auto now = std::chrono::steady_clock::time_point{1000ms};
  health.mark_configured_connected();
  health.begin_call("FakeBlockingIO", now);
  health.mark_call_timeout("FakeBlockingIO", 3000ms);

  const auto timed_out = health.snapshot();
  EXPECT_TRUE(timed_out.recovery_required);
  EXPECT_TRUE(timed_out.call_in_progress);
  EXPECT_EQ(timed_out.fault_code, kLocalSdkCommandTimeout);

  health.end_call(0, "FakeBlockingIO", now + 4000ms);
  const auto late_success = health.snapshot();
  EXPECT_TRUE(late_success.recovery_required);
  EXPECT_EQ(late_success.fault_code, kLocalSdkCommandTimeout);
}

}  // namespace
}  // namespace hikrobot_mpsdk_core
