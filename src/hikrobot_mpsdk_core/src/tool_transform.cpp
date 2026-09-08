#include "hikrobot_mpsdk_core/types.hpp"

#include <algorithm>
#include <cmath>

#include "MP_Public_Defines.h"

namespace hikrobot_mpsdk_core
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

Quaternion normalize(Quaternion q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm > 0.0) {
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    q.w /= norm;
  }
  return q;
}

Quaternion multiply(const Quaternion & a, const Quaternion & b)
{
  return normalize({
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z});
}

void rotate(const Quaternion & q, double x, double y, double z, double & ox, double & oy, double & oz)
{
  const double tx = 2.0 * (q.y * z - q.z * y);
  const double ty = 2.0 * (q.z * x - q.x * z);
  const double tz = 2.0 * (q.x * y - q.y * x);
  ox = x + q.w * tx + (q.y * tz - q.z * ty);
  oy = y + q.w * ty + (q.z * tx - q.x * tz);
  oz = z + q.w * tz + (q.x * ty - q.y * tx);
}

bool finite_transform(const RigidTransform & transform)
{
  const auto & q = transform.rotation;
  const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(transform.x_m) && std::isfinite(transform.y_m) &&
         std::isfinite(transform.z_m) && std::isfinite(norm_squared) && norm_squared > 1e-12;
}
}  // namespace

RigidTransform euler_pose_to_transform(const EulerPose & pose)
{
  const double roll = pose.roll_deg * kPi / 180.0;
  const double pitch = pose.pitch_deg * kPi / 180.0;
  const double yaw = pose.yaw_deg * kPi / 180.0;
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  return {pose.x_mm / 1000.0, pose.y_mm / 1000.0, pose.z_mm / 1000.0,
    normalize({
      cy * cp * sr - sy * sp * cr,
      sy * cp * sr + cy * sp * cr,
      sy * cp * cr - cy * sp * sr,
      cy * cp * cr + sy * sp * sr})};
}

RigidTransform compose_transforms(const RigidTransform & lhs, const RigidTransform & rhs)
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  rotate(lhs.rotation, rhs.x_m, rhs.y_m, rhs.z_m, x, y, z);
  return {lhs.x_m + x, lhs.y_m + y, lhs.z_m + z, multiply(lhs.rotation, rhs.rotation)};
}

double transform_position_distance_m(const RigidTransform & a, const RigidTransform & b)
{
  const double dx = a.x_m - b.x_m;
  const double dy = a.y_m - b.y_m;
  const double dz = a.z_m - b.z_m;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double quaternion_angular_distance_rad(const Quaternion & a, const Quaternion & b)
{
  const Quaternion qa = normalize(a);
  const Quaternion qb = normalize(b);
  double dot = std::abs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w);
  dot = std::clamp(dot, 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

bool cartesian_path_within_limits(
  const RigidTransform & current, const std::vector<RigidTransform> & path,
  double max_translation_m, double max_rotation_rad, std::string & error)
{
  if (max_translation_m <= 0.0 || max_rotation_rad <= 0.0) {
    error = "Cartesian step limits must be positive";
    return false;
  }
  if (!finite_transform(current)) {
    error = "current Cartesian pose is invalid";
    return false;
  }
  const RigidTransform * previous = &current;
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!finite_transform(path[i])) {
      error = "Cartesian path contains an invalid pose at segment " + std::to_string(i);
      return false;
    }
    const double translation = transform_position_distance_m(*previous, path[i]);
    const double rotation = quaternion_angular_distance_rad(previous->rotation, path[i].rotation);
    if (translation > max_translation_m || rotation > max_rotation_rad) {
      error = "Cartesian path segment " + std::to_string(i) +
        " exceeds configured translation/rotation step limit";
      return false;
    }
    previous = &path[i];
  }
  return true;
}

bool valid_tool_id(int32_t id)
{
  return id >= 0 && id < MP_TOOL_NUM_MAX;
}

bool valid_tool_count(int32_t count)
{
  return count >= 0 && count <= MP_TOOL_NUM_MAX;
}

std::string bounded_sdk_string(const char * value, std::size_t capacity)
{
  const char * const end = value + capacity;
  const char * const terminator = std::find(value, end, '\0');
  return std::string(value, terminator);
}

std::optional<RigidTransform> tcp_transform_if_valid(
  const RobotState & robot, bool active_tool_valid, const ToolInfo & active_tool)
{
  if (!robot.valid || robot.stale || robot.joint_count < 6 || !active_tool_valid ||
    active_tool.mount_position != 0)
  {
    return std::nullopt;
  }
  return compose_transforms(
    euler_pose_to_transform(robot.base_pose), euler_pose_to_transform(active_tool.flange_to_tool));
}

std::optional<CartesianError> cartesian_tcp_error(
  const RobotState & robot, const ToolInfo & tool, const EulerPose & target_tcp)
{
  const auto actual = tcp_transform_if_valid(robot, true, tool);
  if (!actual) {
    return std::nullopt;
  }
  const auto target = euler_pose_to_transform(target_tcp);
  return CartesianError{
    *actual, transform_position_distance_m(*actual, target),
    quaternion_angular_distance_rad(actual->rotation, target.rotation)};
}
}  // namespace hikrobot_mpsdk_core
