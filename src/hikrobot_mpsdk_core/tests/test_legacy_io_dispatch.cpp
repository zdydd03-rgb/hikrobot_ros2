#include "hikrobot_mpsdk_core/legacy_io_dispatch.hpp"

#include <string>

#include "gtest/gtest.h"

namespace hikrobot_mpsdk_core
{
namespace
{

TEST(LegacyIoDispatchTest, GetDigitalInputCallsSdkOnce)
{
  int calls = 0;
  const auto result = dispatch_digital_get(
    "ready", DigitalIoType::kInput,
    [&calls](const std::string & name, DigitalIoType type, bool & value) {
      ++calls;
      EXPECT_EQ(name, "ready");
      EXPECT_EQ(type, DigitalIoType::kInput);
      value = true;
      return int64_t{0};
    });
  EXPECT_EQ(calls, 1);
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.value);
}

TEST(LegacyIoDispatchTest, ProtectedNameIsAnOrdinaryDigitalOutputRead)
{
  int calls = 0;
  const auto result = dispatch_digital_get(
    "start", DigitalIoType::kOutput,
    [&calls](const std::string & name, DigitalIoType type, bool & value) {
      ++calls;
      EXPECT_EQ(name, "start");
      EXPECT_EQ(type, DigitalIoType::kOutput);
      value = true;
      return int64_t{0};
    });
  EXPECT_EQ(calls, 1);
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.value);
}

TEST(LegacyIoDispatchTest, SetHighCallsOnlySetOnce)
{
  int set_calls = 0;
  int get_calls = 0;
  const auto result = dispatch_digital_set(
    "start", true,
    [&set_calls, &get_calls](const std::string & name, bool value) {
      ++set_calls;
      EXPECT_EQ(get_calls, 0);
      EXPECT_EQ(name, "start");
      EXPECT_TRUE(value);
      return int64_t{0};
    });
  EXPECT_EQ(set_calls, 1);
  EXPECT_EQ(get_calls, 0);
  EXPECT_TRUE(result.success);
}

TEST(LegacyIoDispatchTest, SetLowCallsOnlySetOnce)
{
  int calls = 0;
  const auto result = dispatch_digital_set(
    "start", false,
    [&calls](const std::string &, bool value) {
      ++calls;
      EXPECT_FALSE(value);
      return int64_t{0};
    });
  EXPECT_EQ(calls, 1);
  EXPECT_TRUE(result.success);
}

TEST(LegacyIoDispatchTest, SdkFailureIsReturnedWithoutRetry)
{
  int calls = 0;
  const auto result = dispatch_digital_set(
    "output", true,
    [&calls](const std::string &, bool) {
      ++calls;
      return int64_t{0x20004};
    });
  EXPECT_EQ(calls, 1);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.sdk_error, 0x20004);
}

}  // namespace
}  // namespace hikrobot_mpsdk_core
