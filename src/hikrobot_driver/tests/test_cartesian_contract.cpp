#include <cmath>
#include <limits>
#include <string>

#include "gtest/gtest.h"
#include "hikrobot_driver/cartesian_contract.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace hd = hikrobot_driver;
namespace hc = hikrobot_mpsdk_core;
namespace
{
constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::PoseStamped valid_pose()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.orientation.w = 1.0;
  return pose;
}
}  // namespace

TEST(CartesianContract, AcceptsBaseFrameAndConvertsMetresToMillimetres)
{
  auto pose = valid_pose();
  pose.pose.position.x = 0.123;
  hc::EulerPose sdk;
  std::string error;
  ASSERT_TRUE(hd::cartesian_pose_to_sdk(pose, "base_link", sdk, error));
  EXPECT_DOUBLE_EQ(sdk.x_mm, 123.0);
}

TEST(CartesianContract, RejectsEmptyFrame)
{
  auto pose = valid_pose();
  pose.header.frame_id.clear();
  hc::EulerPose sdk;
  std::string error;
  EXPECT_FALSE(hd::cartesian_pose_to_sdk(pose, "base_link", sdk, error));
  EXPECT_NE(error.find("received ''"), std::string::npos);
}

TEST(CartesianContract, RejectsNonBaseFrame)
{
  auto pose = valid_pose();
  pose.header.frame_id = "camera_link";
  hc::EulerPose sdk;
  std::string error;
  EXPECT_FALSE(hd::cartesian_pose_to_sdk(pose, "base_link", sdk, error));
  EXPECT_NE(error.find("camera_link"), std::string::npos);
}

TEST(CartesianContract, RejectsNonFinitePosition)
{
  auto pose = valid_pose();
  pose.pose.position.z = std::numeric_limits<double>::infinity();
  hc::EulerPose sdk;
  std::string error;
  EXPECT_FALSE(hd::cartesian_pose_to_sdk(pose, "base_link", sdk, error));
  pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(hd::cartesian_pose_to_sdk(pose, "base_link", sdk, error));
}

TEST(CartesianContract, RejectsPositionOutsideSdkNumericRange)
{
  auto pose = valid_pose();
  pose.pose.position.x = std::numeric_limits<double>::max();
  hc::EulerPose sdk;
  std::string error;
  EXPECT_FALSE(hd::cartesian_pose_to_sdk(pose, "base_link", sdk, error));
}

TEST(CartesianContract, RejectsNonFiniteQuaternion)
{
  auto pose = valid_pose();
  pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
  hc::EulerPose sdk;
  std::string error;
  EXPECT_FALSE(hd::cartesian_pose_to_sdk(pose, "base_link", sdk, error));
  pose.pose.orientation.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(hd::cartesian_pose_to_sdk(pose, "base_link", sdk, error));
}

TEST(CartesianContract, RejectsZeroQuaternion)
{
  auto pose = valid_pose();
  pose.pose.orientation.w = 0.0;
  hc::EulerPose sdk;
  std::string error;
  EXPECT_FALSE(hd::cartesian_pose_to_sdk(pose, "base_link", sdk, error));
}

TEST(CartesianContract, NormalizesQuaternionAndConvertsToZyxEulerDegrees)
{
  auto pose = valid_pose();
  tf2::Quaternion q;
  q.setRPY(0.1, -0.2, 0.3);
  pose.pose.orientation.x = q.x() * 4.0;
  pose.pose.orientation.y = q.y() * 4.0;
  pose.pose.orientation.z = q.z() * 4.0;
  pose.pose.orientation.w = q.w() * 4.0;
  hc::EulerPose sdk;
  std::string error;
  ASSERT_TRUE(hd::cartesian_pose_to_sdk(pose, "base_link", sdk, error));
  EXPECT_NEAR(sdk.roll_deg, 0.1 * 180.0 / kPi, 1e-9);
  EXPECT_NEAR(sdk.pitch_deg, -0.2 * 180.0 / kPi, 1e-9);
  EXPECT_NEAR(sdk.yaw_deg, 0.3 * 180.0 / kPi, 1e-9);
}

TEST(CartesianContract, SafelyNormalizesVeryLargeFiniteQuaternion)
{
  auto pose = valid_pose();
  pose.pose.orientation.x = std::numeric_limits<double>::max();
  pose.pose.orientation.y = std::numeric_limits<double>::max();
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 0.0;
  hc::EulerPose sdk;
  std::string error;
  ASSERT_TRUE(hd::cartesian_pose_to_sdk(pose, "base_link", sdk, error));
  EXPECT_TRUE(std::isfinite(sdk.roll_deg));
  EXPECT_TRUE(std::isfinite(sdk.pitch_deg));
  EXPECT_TRUE(std::isfinite(sdk.yaw_deg));
}

TEST(CartesianContract, ValidatesNormalizedOrZeroDuration)
{
  builtin_interfaces::msg::Duration duration;
  std::string error;
  EXPECT_TRUE(hd::valid_action_duration(duration, error));
  duration.sec = 1;
  duration.nanosec = 999999999U;
  EXPECT_TRUE(hd::valid_action_duration(duration, error));
  duration.sec = -1;
  EXPECT_FALSE(hd::valid_action_duration(duration, error));
  duration.sec = 0;
  duration.nanosec = 1000000000U;
  EXPECT_FALSE(hd::valid_action_duration(duration, error));
}

TEST(CartesianContract, UsesControllerActiveToolAndBaseCoordinate)
{
  hc::ToolInfo tool;
  tool.id = 2;
  tool.mount_position = 0;
  hc::MotionOptions options;
  options.tool_id = 7;
  options.coordinate_type = 4;
  options.coordinate_id = 3;
  options.workpiece_id = 8;
  std::string error;
  ASSERT_TRUE(hd::apply_base_cartesian_options(true, tool, options, error));
  EXPECT_EQ(options.tool_id, 2);
  EXPECT_EQ(options.coordinate_type, 1);
  EXPECT_EQ(options.coordinate_id, 0);
  EXPECT_EQ(options.workpiece_id, -1);
}

TEST(CartesianContract, RejectsInvalidToolCache)
{
  hc::ToolInfo tool;
  hc::MotionOptions options;
  std::string error;
  EXPECT_FALSE(hd::apply_base_cartesian_options(false, tool, options, error));
}

TEST(CartesianContract, RejectsExternalMountAndOutOfRangeId)
{
  hc::ToolInfo tool;
  tool.id = 2;
  tool.mount_position = 1;
  hc::MotionOptions options;
  std::string error;
  EXPECT_FALSE(hd::apply_base_cartesian_options(true, tool, options, error));
  tool.mount_position = 0;
  tool.id = -1;
  EXPECT_FALSE(hd::apply_base_cartesian_options(true, tool, options, error));
}

TEST(CartesianContract, MoveJOptionsAreOutsideCartesianContract)
{
  hc::MoveJRequest move_j;
  move_j.options.tool_id = 7;
  move_j.options.coordinate_type = 0;
  hc::ToolInfo tool;
  tool.id = 2;
  tool.mount_position = 0;
  hc::MotionOptions cartesian;
  std::string error;
  ASSERT_TRUE(hd::apply_base_cartesian_options(true, tool, cartesian, error));
  EXPECT_EQ(move_j.options.tool_id, 7);
  EXPECT_EQ(move_j.options.coordinate_type, 0);
}
