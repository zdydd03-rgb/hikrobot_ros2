#pragma once

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hikrobot_mpsdk_core/types.hpp"

namespace hikrobot_driver
{

bool cartesian_pose_to_sdk(
  const geometry_msgs::msg::PoseStamped & input, const std::string & base_frame,
  hikrobot_mpsdk_core::EulerPose & output, std::string & error);

bool apply_base_cartesian_options(
  bool active_tool_valid, const hikrobot_mpsdk_core::ToolInfo & active_tool,
  hikrobot_mpsdk_core::MotionOptions & options, std::string & error);

}  // namespace hikrobot_driver
