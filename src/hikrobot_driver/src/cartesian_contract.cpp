#include "hikrobot_driver/cartesian_contract.hpp"

#include <cmath>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace hikrobot_driver
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
}  // namespace

bool cartesian_pose_to_sdk(
  const geometry_msgs::msg::PoseStamped & input, const std::string & base_frame,
  hikrobot_mpsdk_core::EulerPose & output, std::string & error)
{
  if (input.header.frame_id != base_frame) {
    error = "Cartesian target frame must be '" + base_frame + "'; received '" +
      input.header.frame_id + "'";
    return false;
  }
  const auto & p = input.pose.position;
  const auto & o = input.pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(o.x) || !std::isfinite(o.y) || !std::isfinite(o.z) || !std::isfinite(o.w))
  {
    error = "Cartesian target contains NaN or Inf";
    return false;
  }

  tf2::Quaternion q;
  tf2::fromMsg(input.pose.orientation, q);
  if (q.length2() < 1e-12) {
    error = "target quaternion has zero length";
    return false;
  }
  q.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  output.x_mm = p.x * 1000.0;
  output.y_mm = p.y * 1000.0;
  output.z_mm = p.z * 1000.0;
  output.roll_deg = roll * kRadToDeg;
  output.pitch_deg = pitch * kRadToDeg;
  output.yaw_deg = yaw * kRadToDeg;
  if (!std::isfinite(output.x_mm) || !std::isfinite(output.y_mm) ||
    !std::isfinite(output.z_mm))
  {
    error = "Cartesian target is outside the representable SDK range";
    return false;
  }
  return true;
}

bool apply_base_cartesian_options(
  bool active_tool_valid, const hikrobot_mpsdk_core::ToolInfo & active_tool,
  hikrobot_mpsdk_core::MotionOptions & options, std::string & error)
{
  if (!active_tool_valid) {
    error = "active Tool cache is invalid";
    return false;
  }
  if (!hikrobot_mpsdk_core::valid_tool_id(active_tool.id)) {
    error = "controller Active Tool ID is outside the SDK range";
    return false;
  }
  if (active_tool.mount_position != 0) {
    error = "controller Active Tool is not flange-mounted";
    return false;
  }
  options.tool_id = active_tool.id;
  options.coordinate_type = 1;
  options.coordinate_id = 0;
  options.workpiece_id = -1;
  return true;
}

}  // namespace hikrobot_driver
