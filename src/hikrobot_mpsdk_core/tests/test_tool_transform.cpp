#include <cmath>
#include <limits>

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

TEST(CartesianStep, AcceptsTenMillimetreTranslation)
{
  hc::RigidTransform current;
  auto target = current;
  target.z_m = 0.01;
  std::string error;
  EXPECT_TRUE(hc::cartesian_path_within_limits(current, {target}, 0.05, 10.0, error));
}

TEST(CartesianStep, RejectsTranslationOverFiftyMillimetres)
{
  hc::RigidTransform current;
  auto target = current;
  target.x_m = 0.050001;
  std::string error;
  EXPECT_FALSE(hc::cartesian_path_within_limits(current, {target}, 0.05, 10.0, error));
}

TEST(CartesianStep, RejectsRotationOverTenDegrees)
{
  hc::RigidTransform current;
  const auto target = hc::euler_pose_to_transform({0.0, 0.0, 0.0, 0.0, 0.0, 10.01});
  std::string error;
  EXPECT_FALSE(hc::cartesian_path_within_limits(
      current, {target}, 1.0, 10.0 * std::acos(-1.0) / 180.0, error));
}

TEST(CartesianStep, QuaternionSignDoesNotChangeAngularDistance)
{
  const hc::Quaternion q{0.1, -0.2, 0.3, 0.9};
  const hc::Quaternion negative{-0.1, 0.2, -0.3, -0.9};
  EXPECT_NEAR(hc::quaternion_angular_distance_rad(q, negative), 0.0, kEpsilon);
}

TEST(CartesianStep, ChecksEveryMoveCAndMoveSSegment)
{
  hc::RigidTransform current;
  auto first = current;
  first.x_m = 0.04;
  auto second = first;
  second.y_m = 0.04;
  std::string error;
  EXPECT_TRUE(hc::cartesian_path_within_limits(current, {first, second}, 0.05, 1.0, error));
  second.y_m = 0.10;
  EXPECT_FALSE(hc::cartesian_path_within_limits(current, {first, second}, 0.05, 1.0, error));
  EXPECT_NE(error.find("segment 1"), std::string::npos);
}

TEST(CartesianFeedback, UsesTcpInsteadOfFlangeForActualAndError)
{
  auto robot = valid_robot();
  robot.base_pose = {100.0, 0.0, 0.0, 0.0, 0.0, 90.0};
  hc::ToolInfo tool;
  tool.id = 2;
  tool.mount_position = 0;
  tool.flange_to_tool = {100.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const hc::EulerPose target_tcp{100.0, 100.0, 0.0, 0.0, 0.0, 90.0};
  const auto error = hc::cartesian_tcp_error(robot, tool, target_tcp);
  ASSERT_TRUE(error.has_value());
  EXPECT_NEAR(error->actual_tcp.x_m, 0.1, kEpsilon);
  EXPECT_NEAR(error->actual_tcp.y_m, 0.1, kEpsilon);
  EXPECT_NEAR(error->position_error_m, 0.0, kEpsilon);
  EXPECT_NEAR(error->orientation_error_rad, 0.0, kEpsilon);
  EXPECT_NE(error->actual_tcp.y_m, robot.base_pose.y_mm / 1000.0);
}

TEST(CartesianFeedback, ToolTwoOffsetChangesCompletionError)
{
  auto robot = valid_robot();
  robot.base_pose = {500.0, -100.0, 800.0, 5.0, -10.0, 20.0};
  hc::ToolInfo tool;
  tool.id = 2;
  tool.mount_position = 0;
  tool.flange_to_tool = {137.5207242, 1.496775659, 187.672063961,
    0.0148433, -0.0961292, 0.0101835};
  const auto expected = hc::tcp_transform_if_valid(robot, true, tool);
  ASSERT_TRUE(expected.has_value());
  hc::EulerPose flange_target = robot.base_pose;
  const auto error_to_flange = hc::cartesian_tcp_error(robot, tool, flange_target);
  ASSERT_TRUE(error_to_flange.has_value());
  EXPECT_GT(error_to_flange->position_error_m, 0.20);
}

TEST(CartesianStep, RejectsNonPositiveLimits)
{
  hc::RigidTransform current;
  std::string error;
  EXPECT_FALSE(hc::cartesian_path_within_limits(current, {}, 0.0, 1.0, error));
  EXPECT_FALSE(hc::cartesian_path_within_limits(current, {}, 1.0, 0.0, error));
}

TEST(CartesianStep, RejectsNonFiniteOrZeroQuaternionTransforms)
{
  hc::RigidTransform current;
  current.x_m = std::numeric_limits<double>::quiet_NaN();
  std::string error;
  EXPECT_FALSE(hc::cartesian_path_within_limits(current, {}, 1.0, 1.0, error));
  current = {};
  hc::RigidTransform target;
  target.rotation = {0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(hc::cartesian_path_within_limits(current, {target}, 1.0, 1.0, error));
}
