#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace hikrobot_mpsdk_core
{

struct EulerPose
{
  double x_mm{0.0};
  double y_mm{0.0};
  double z_mm{0.0};
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double yaw_deg{0.0};
};

struct ToolInfo
{
  int32_t id{0};
  std::string name;
  std::string description;
  int mount_position{0};
  EulerPose flange_to_tool{};
  double mass{0.0};
  EulerPose mass_center{};
  bool convex_hull_enabled{false};
};

struct ToolResult
{
  bool success{false};
  int64_t sdk_error{0};
  ToolInfo tool{};
  std::string message;
};

struct ToolListResult
{
  bool success{false};
  int64_t sdk_error{0};
  std::vector<ToolInfo> tools;
  std::string message;
};

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct RigidTransform
{
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  Quaternion rotation{};
};

RigidTransform euler_pose_to_transform(const EulerPose & pose);
RigidTransform compose_transforms(const RigidTransform & lhs, const RigidTransform & rhs);
double transform_position_distance_m(const RigidTransform & a, const RigidTransform & b);
double quaternion_angular_distance_rad(const Quaternion & a, const Quaternion & b);
bool cartesian_path_within_limits(
  const RigidTransform & current, const std::vector<RigidTransform> & path,
  double max_translation_m, double max_rotation_rad, std::string & error);
bool valid_tool_id(int32_t id);
bool valid_tool_count(int32_t count);
std::string bounded_sdk_string(const char * value, std::size_t capacity);

struct RobotState
{
  bool valid{false};
  bool stale{true};
  int servo_status{0};
  int motion_status{0};
  int emergency_status{0};
  int controller_status{0};
  int safety_gate_status{0};
  int robot_motion_mode{0};
  int joint_count{0};
  std::array<double, 6> joints_deg{};
  EulerPose base_pose{};
  double motion_execution_percent{0.0};
  std::chrono::steady_clock::time_point received_at{};
};

std::optional<RigidTransform> tcp_transform_if_valid(
  const RobotState & robot, bool active_tool_valid, const ToolInfo & active_tool);

struct CartesianError
{
  RigidTransform actual_tcp{};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
};

std::optional<CartesianError> cartesian_tcp_error(
  const RobotState & robot, const ToolInfo & tool, const EulerPose & target_tcp);

struct MotionState
{
  bool valid{false};
  int32_t path_id{0};
  int32_t previous_target_id{0};
  int32_t next_target_id{0};
  bool stopped{true};
  int32_t left_path_count{0};
  double percent{0.0};
  std::chrono::steady_clock::time_point received_at{};
};

struct MotionOptions
{
  int speed_percent{5};
  int blend_radius{0};
  int tool_id{0};
  int workpiece_id{-1};
  int coordinate_type{1};  // SDK: 1 = base coordinate system
  int coordinate_id{0};
  std::chrono::milliseconds timeout{30000};
  double joint_tolerance_deg{0.20};
  double position_tolerance_mm{1.0};
  double orientation_tolerance_deg{1.0};
  int stable_samples{5};
  std::chrono::milliseconds state_stale_timeout{1000};
};

struct MoveJRequest
{
  std::array<double, 6> joints_deg{};
  MotionOptions options{};
};

struct MoveLRequest
{
  EulerPose target{};
  ToolInfo tool{};
  MotionOptions options{};
};

struct MoveCRequest
{
  EulerPose via{};
  EulerPose target{};
  ToolInfo tool{};
  MotionOptions options{};
};

struct MoveSRequest
{
  std::vector<EulerPose> via{};
  EulerPose target{};
  ToolInfo tool{};
  MotionOptions options{};
};

enum class MotionResultCode : uint8_t
{
  kSuccess = 0,
  kInvalidGoal = 1,
  kDriverNotReady = 2,
  kRobotNotReady = 3,
  kBusy = 4,
  kSdkError = 5,
  kTimeout = 6,
  kCanceled = 7,
  kStateLost = 8,
};

struct MotionFeedback
{
  int32_t path_id{0};
  double progress_percent{0.0};
  RobotState robot{};
  MotionState motion{};
  double position_error_mm{0.0};
  double orientation_error_deg{0.0};
  std::array<double, 6> joint_error_deg{};
  bool actual_tcp_valid{false};
  RigidTransform actual_tcp{};
};

struct MotionResult
{
  MotionResultCode code{MotionResultCode::kSdkError};
  int64_t sdk_error{0};
  int32_t path_id{0};
  std::chrono::milliseconds execution_time{0};
  std::string message{};
};

struct IoResult
{
  bool success{false};
  bool value{false};
  int64_t sdk_error{0};
  std::string message{};
};

using FeedbackCallback = std::function<void(const MotionFeedback &)>;

}  // namespace hikrobot_mpsdk_core
