#include "hikrobot_mpsdk_core/protected_arc_output.hpp"

#include <atomic>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>

using hikrobot_mpsdk_core::ProtectedArcOutputState;
using hikrobot_mpsdk_core::ProtectedOutputState;

TEST(ProtectedArcOutput, StartsUnknownAndResetStaysUnknown)
{
  ProtectedArcOutputState state;
  EXPECT_EQ(state.state(), ProtectedOutputState::UNKNOWN);
  state.reset_unknown();
  EXPECT_EQ(state.state(), ProtectedOutputState::UNKNOWN);
}

TEST(ProtectedArcOutput, ReadsAndWritesPublishOnlySuccessfulState)
{
  ProtectedArcOutputState state;
  { auto lock = state.lock(); state.read_result_locked(false); }
  EXPECT_EQ(state.state(), ProtectedOutputState::LOW);
  { auto lock = state.lock(); state.read_result_locked(true); }
  EXPECT_EQ(state.state(), ProtectedOutputState::ASSERTED);
  { auto lock = state.lock(); state.write_result_locked(false, false); }
  EXPECT_EQ(state.state(), ProtectedOutputState::UNKNOWN);
  { auto lock = state.lock(); state.write_result_locked(true, true); }
  EXPECT_EQ(state.state(), ProtectedOutputState::ASSERTED);
  { auto lock = state.lock(); state.write_result_locked(false, true); }
  EXPECT_EQ(state.state(), ProtectedOutputState::LOW);
}

TEST(ProtectedArcOutput, AssertedReadIsSerializedAndSuppressed)
{
  ProtectedArcOutputState state;
  { auto lock = state.lock(); state.write_result_locked(true, true); }
  std::atomic_bool entered{false};
  std::atomic_bool suppressed{false};
  std::thread getter([&]() {
    auto lock = state.lock();
    entered = true;
    suppressed = state.state_locked() == ProtectedOutputState::ASSERTED;
  });
  for (int i = 0; i < 100 && !entered.load(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  getter.join();
  EXPECT_TRUE(suppressed.load());
}

TEST(ProtectedArcOutput, LowWriteCanFollowSuppressedGet)
{
  ProtectedArcOutputState state;
  { auto lock = state.lock(); state.write_result_locked(true, true); }
  auto get = [&]() {
    auto lock = state.lock();
    return state.state_locked() == ProtectedOutputState::ASSERTED;
  };
  EXPECT_TRUE(get());
  { auto lock = state.lock(); state.write_result_locked(false, true); }
  EXPECT_EQ(state.state(), ProtectedOutputState::LOW);
}

TEST(ProtectedArcOutput, FailedLowWriteDoesNotClearAssertedBeforeResult)
{
  ProtectedArcOutputState state;
  { auto lock = state.lock(); state.write_result_locked(true, true); }
  { auto lock = state.lock(); state.write_result_locked(false, false); }
  EXPECT_EQ(state.state(), ProtectedOutputState::UNKNOWN);
}

TEST(ProtectedArcOutput, FailedHighWriteIsUnknownAndReadMayRetry)
{
  ProtectedArcOutputState state;
  { auto lock = state.lock(); state.write_result_locked(true, false); }
  EXPECT_EQ(state.state(), ProtectedOutputState::UNKNOWN);
  { auto lock = state.lock(); state.read_result_locked(false); }
  EXPECT_EQ(state.state(), ProtectedOutputState::LOW);
}

TEST(ProtectedArcOutput, CachedCommandIsTrueOnlyAfterSuccessfulHigh)
{
  ProtectedArcOutputState state;
  { auto lock = state.lock(); EXPECT_FALSE(state.cached_value_locked()); }
  { auto lock = state.lock(); state.write_result_locked(true, true); }
  { auto lock = state.lock(); EXPECT_TRUE(state.cached_value_locked()); }
}

TEST(ProtectedArcOutput, NonProtectedAndReconnectDoNotAssumeLow)
{
  ProtectedArcOutputState state;
  { auto lock = state.lock(); state.write_result_locked(true, true); }
  state.reset_unknown();
  EXPECT_EQ(state.state(), ProtectedOutputState::UNKNOWN);
}
