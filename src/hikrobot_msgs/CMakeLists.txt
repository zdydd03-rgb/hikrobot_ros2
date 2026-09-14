cmake_minimum_required(VERSION 3.8)
project(hikrobot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveJ.action"
  "action/MoveL.action"
  "action/MoveC.action"
  "action/MoveS.action"
  "action/FollowCartesianPath.action"
  "msg/RobotStatus.msg"
  "msg/ToolInfo.msg"
  "srv/SetDigitalOutput.srv"
  "srv/GetDigitalOutput.srv"
  "srv/GetDigitalInput.srv"
  "srv/StopMotion.srv"
  "srv/GetActiveTool.srv"
  "srv/GetToolInfoList.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
