#include <cmath>

#include "gtest/gtest.h"
#include "hikrobot_mpsdk_core/types.hpp"

namespace hc = hikrobot_mpsdk_core;
namespace
{
constexpr double kEpsilon = 1e-9;
hc::RobotState valid_robot()
{
  hc::RobotState robot;
  robot.valid = true;
  robot.stale = false;
  robot.joint_count = 6;
  return robot;
}
double quaternion_norm(const hc::Quaternion & q)
{
  return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}
}  // namespace

TEST(ToolTransform, ConvertsMillimetresAndNormalizesQuaternion)
{
  const hc::EulerPose pose{1000.0, -2000.0, 500.0, 10.0, 20.0, 30.0};
  const auto transform = hc::euler_pose_to_transform(pose);
  EXPECT_DOUBLE_EQ(transform.x_m, 1.0);
  EXPECT_DOUBLE_EQ(transform.y_m, -2.0);
  EXPECT_DOUBLE_EQ(transform.z_m, 0.5);
  EXPECT_NEAR(quaternion_norm(transform.rotation), 1.0, kEpsilon);
}

TEST(ToolTransform, IdentityToolReturnsFlange)
{
  auto robot = valid_robot();
  robot.base_pose = {100.0, 200.0, 300.0, 0.0, 0.0, 0.0};
  hc::ToolInfo tool;
  tool.mount_position = 0;
  const auto tcp = hc::tcp_transform_if_valid(robot, true, tool);
  ASSERT_TRUE(tcp.has_value());
  EXPECT_DOUBLE_EQ(tcp->x_m, 0.1);
  EXPECT_DOUBLE_EQ(tcp->y_m, 0.2);
  EXPECT_DOUBLE_EQ(tcp->z_m, 0.3);
}

TEST(ToolTransform, IdentityFlangeReturnsTool)
{
  auto robot = valid_robot();
  hc::ToolInfo tool;
  tool.mount_position = 0;
  tool.flange_to_tool = {100.0, 200.0, 300.0, 0.0, 0.0, 0.0};
  const auto tcp = hc::tcp_transform_if_valid(robot, true, tool);
  ASSERT_TRUE(tcp.has_value());
  EXPECT_DOUBLE_EQ(tcp->x_m, 0.1);
  EXPECT_DOUBLE_EQ(tcp->y_m, 0.2);
  EXPECT_DOUBLE_EQ(tcp->z_m, 0.3);
}

TEST(ToolTransform, RotatesToolTranslationBeforeAdding)
{
  auto robot = valid_robot();
  robot.base_pose.yaw_deg = 90.0;
  hc::ToolInfo tool;
  tool.mount_position = 0;
  tool.flange_to_tool.x_mm = 1000.0;
  const auto tcp = hc::tcp_transform_if_valid(robot, true, tool);
  ASSERT_TRUE(tcp.has_value());
  EXPECT_NEAR(tcp->x_m, 0.0, kEpsilon);
  EXPECT_NEAR(tcp->y_m, 1.0, kEpsilon);
}

TEST(ToolTransform, ToolTwoRegressionComposition)
{
  auto robot = valid_robot();
  robot.base_pose = {500.0, -100.0, 800.0, 5.0, -10.0, 20.0};
  hc::ToolInfo tool;
  tool.mount_position = 0;
  tool.flange_to_tool = {137.521, 1.49678, 187.672, 0.0148433, -0.0961292, 0.0101835};
  const auto tcp = hc::tcp_transform_if_valid(robot, true, tool);
  ASSERT_TRUE(tcp.has_value());
  EXPECT_NEAR(quaternion_norm(tcp->rotation), 1.0, kEpsilon);
  EXPECT_NE(tcp->x_m, robot.base_pose.x_mm / 1000.0 + tool.flange_to_tool.x_mm / 1000.0);
}

TEST(ToolTransform, RejectsInvalidCacheAndExternalMount)
{
  auto robot = valid_robot();
  hc::ToolInfo tool;
  EXPECT_FALSE(hc::tcp_transform_if_valid(robot, false, tool).has_value());
  tool.mount_position = 1;
  EXPECT_FALSE(hc::tcp_transform_if_valid(robot, true, tool).has_value());
}

TEST(ToolTransform, ValidatesToolBoundsAndBoundedStrings)
{
  EXPECT_TRUE(hc::valid_tool_id(0));
  EXPECT_TRUE(hc::valid_tool_id(9));
  EXPECT_FALSE(hc::valid_tool_id(-1));
  EXPECT_FALSE(hc::valid_tool_id(10));
  EXPECT_TRUE(hc::valid_tool_count(0));
  EXPECT_TRUE(hc::valid_tool_count(10));
  EXPECT_FALSE(hc::valid_tool_count(-1));
  EXPECT_FALSE(hc::valid_tool_count(11));
  const char value[] = {'t', 'o', 'o', 'l'};
  EXPECT_EQ(hc::bounded_sdk_string(value, sizeof(value)), "tool");
}
