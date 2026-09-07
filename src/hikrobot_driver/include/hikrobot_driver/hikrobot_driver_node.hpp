#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "hikrobot_msgs/action/move_j.hpp"
#include "hikrobot_msgs/action/move_l.hpp"
#include "hikrobot_msgs/action/move_c.hpp"
#include "hikrobot_msgs/action/move_s.hpp"
#include "hikrobot_msgs/msg/robot_status.hpp"
#include "hikrobot_msgs/srv/set_digital_output.hpp"
#include "hikrobot_msgs/srv/get_digital_output.hpp"
#include "hikrobot_msgs/srv/get_digital_input.hpp"
#include "hikrobot_msgs/srv/stop_motion.hpp"
#include "hikrobot_mpsdk_core/robot_runtime.hpp"

namespace hikrobot_driver
{

class HikrobotDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit HikrobotDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HikrobotDriverNode() override;

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  using MoveJ = hikrobot_msgs::action::MoveJ;
  using MoveL = hikrobot_msgs::action::MoveL;
  using MoveC = hikrobot_msgs::action::MoveC;
  using MoveS = hikrobot_msgs::action::MoveS;
  using GoalHandleMoveJ = rclcpp_action::ServerGoalHandle<MoveJ>;
  using GoalHandleMoveL = rclcpp_action::ServerGoalHandle<MoveL>;
  using GoalHandleMoveC = rclcpp_action::ServerGoalHandle<MoveC>;
  using GoalHandleMoveS = rclcpp_action::ServerGoalHandle<MoveS>;

  void declare_parameters();
  void create_ros_interfaces();
  void publish_state();
  void join_motion_worker();
  void release_goal_reservation();

  rclcpp_action::GoalResponse handle_move_j_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJ::Goal> goal);
  rclcpp_action::GoalResponse handle_move_l_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveL::Goal> goal);
  rclcpp_action::GoalResponse handle_move_c_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveC::Goal> goal);
  rclcpp_action::GoalResponse handle_move_s_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveS::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel();
  void accept_move_j(const std::shared_ptr<GoalHandleMoveJ> goal_handle);
  void accept_move_l(const std::shared_ptr<GoalHandleMoveL> goal_handle);
  void accept_move_c(const std::shared_ptr<GoalHandleMoveC> goal_handle);
  void accept_move_s(const std::shared_ptr<GoalHandleMoveS> goal_handle);

  hikrobot_mpsdk_core::MotionOptions options_from_goal(
    uint8_t speed_percent, uint8_t blend_radius,
    const builtin_interfaces::msg::Duration & timeout) const;
  bool resolve_joint_goal(
    const MoveJ::Goal & goal, std::array<double, 6> & joints_deg, std::string & error) const;
  bool pose_to_sdk(
    const geometry_msgs::msg::PoseStamped & input,
    hikrobot_mpsdk_core::EulerPose & output, std::string & error) const;
  geometry_msgs::msg::PoseStamped state_pose_to_ros(
    const hikrobot_mpsdk_core::RobotState & state, const rclcpp::Time & stamp) const;

  uint8_t map_result_code(hikrobot_mpsdk_core::MotionResultCode code) const;

  std::unique_ptr<hikrobot_mpsdk_core::RobotRuntime> core_;
  std::atomic_bool ros_active_{false};
  std::atomic_bool goal_reserved_{false};
  std::atomic_bool cancel_requested_{false};
  std::thread motion_worker_;
  mutable std::mutex worker_mutex_;

  std::vector<std::string> joint_names_;
  std::string base_frame_;
  std::string tool_frame_;
  std::string programmer_password_env_;
  double default_timeout_sec_{30.0};
  double joint_tolerance_rad_{0.0034906585};
  double position_tolerance_m_{0.001};
  double orientation_tolerance_rad_{0.0174532925};
  int stable_samples_{5};
  int state_stale_timeout_ms_{1000};
  int tool_id_{0};
  int workpiece_id_{-1};

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr flange_pub_;
  rclcpp_lifecycle::LifecyclePublisher<hikrobot_msgs::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  rclcpp_action::Server<MoveJ>::SharedPtr move_j_server_;
  rclcpp_action::Server<MoveL>::SharedPtr move_l_server_;
  rclcpp_action::Server<MoveC>::SharedPtr move_c_server_;
  rclcpp_action::Server<MoveS>::SharedPtr move_s_server_;

  rclcpp::Service<hikrobot_msgs::srv::SetDigitalOutput>::SharedPtr set_do_service_;
  rclcpp::Service<hikrobot_msgs::srv::GetDigitalOutput>::SharedPtr get_do_service_;
  rclcpp::Service<hikrobot_msgs::srv::GetDigitalInput>::SharedPtr get_di_service_;
  rclcpp::Service<hikrobot_msgs::srv::StopMotion>::SharedPtr stop_service_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace hikrobot_driver
