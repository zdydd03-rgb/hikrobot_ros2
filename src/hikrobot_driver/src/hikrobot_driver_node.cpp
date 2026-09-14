#include "hikrobot_driver/hikrobot_driver_node.hpp"
#include "hikrobot_driver/cartesian_contract.hpp"
#include "hikrobot_mpsdk_core/legacy_movel_queue_executor.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <sstream>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace hikrobot_driver
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;

template<typename Fn>
class ScopeExit
{
public:
  explicit ScopeExit(Fn fn) : fn_(std::move(fn)) {}
  ScopeExit(const ScopeExit &) = delete;
  ScopeExit & operator=(const ScopeExit &) = delete;
  ~ScopeExit() {fn_();}

private:
  Fn fn_;
};

template<typename Fn>
ScopeExit<Fn> make_scope_exit(Fn fn)
{
  return ScopeExit<Fn>(std::move(fn));
}

builtin_interfaces::msg::Duration duration_from_ms(std::chrono::milliseconds value)
{
  builtin_interfaces::msg::Duration out;
  const int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(value).count();
  out.sec = static_cast<int32_t>(ns / 1000000000LL);
  out.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return out;
}

std::chrono::milliseconds duration_to_ms(
  const builtin_interfaces::msg::Duration & value, double fallback_seconds)
{
  const int64_t ns = static_cast<int64_t>(value.sec) * 1000000000LL + value.nanosec;
  if (ns <= 0) {
    return std::chrono::milliseconds(static_cast<int64_t>(fallback_seconds * 1000.0));
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::nanoseconds(ns));
}

}  // namespace

HikrobotDriverNode::HikrobotDriverNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("hikrobot_driver", options),
  core_(std::make_unique<hikrobot_mpsdk_core::RobotRuntime>())
{
  declare_parameters();
  create_ros_interfaces();
}

HikrobotDriverNode::~HikrobotDriverNode()
{
  ros_active_.store(false);
  request_motion_cancel();
  if (core_ && !core_->command_recovery_required()) {
    (void)core_->stop_motion();
  }
  join_motion_worker();
  if (core_) {
    core_->cleanup();
  }
}

void HikrobotDriverNode::declare_parameters()
{
  declare_parameter<std::string>("robot_ip", "192.168.2.64");
  declare_parameter<int>("robot_port", 9000);
  declare_parameter<int>("har_publish_ms", 50);
  declare_parameter<std::vector<std::string>>(
    "joint_names", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<std::string>("tool_frame", "tool0");
  declare_parameter<std::string>("programmer_password_env", "HIKROBOT_PROGRAMMER_PASSWORD");
  declare_parameter<std::string>("sequence_state_file", "");
  declare_parameter<int>("sequence_collision_retries", 3);
  declare_parameter<double>("default_timeout_sec", 30.0);
  declare_parameter<double>("joint_tolerance_rad", 0.0034906585);  // 0.2 deg
  declare_parameter<double>("position_tolerance_m", 0.001);
  declare_parameter<double>("orientation_tolerance_rad", 0.0174532925);  // 1 deg
  declare_parameter<int>("stable_samples", 5);
  declare_parameter<int>("state_stale_timeout_ms", 1000);
  declare_parameter<int>("stop_confirmation_timeout_ms", 3000);
  declare_parameter<int>("sdk_command_lock_timeout_ms", 1000);
  declare_parameter<int>("sdk_command_watchdog_ms", 3000);
  declare_parameter<std::string>("arc_output_name", "start");
  declare_parameter<int>("tool_id", 0);
  declare_parameter<int>("workpiece_id", -1);
  declare_parameter<double>("max_cartesian_translation_step_m", 0.05);
  declare_parameter<double>("max_cartesian_rotation_step_rad", 0.1745329252);
  declare_parameter<double>("state_publish_rate_hz", 20.0);
}

void HikrobotDriverNode::create_ros_interfaces()
{
  state_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sdk_service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  stop_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());
  flange_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/flange_pose", 10);
  tcp_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/tcp_pose", 10);
  status_pub_ = create_publisher<hikrobot_msgs::msg::RobotStatus>("~/status", 10);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  const double hz = std::max(1.0, get_parameter("state_publish_rate_hz").as_double());
  state_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
    std::bind(&HikrobotDriverNode::publish_state, this), state_callback_group_);

  const auto base = get_node_base_interface();
  const auto clock = get_node_clock_interface();
  const auto logging = get_node_logging_interface();
  const auto waitables = get_node_waitables_interface();

  move_j_server_ = rclcpp_action::create_server<MoveJ>(
    base, clock, logging, waitables, "~/move_j",
    std::bind(&HikrobotDriverNode::handle_move_j_goal, this, std::placeholders::_1, std::placeholders::_2),
    [this](const std::shared_ptr<GoalHandleMoveJ>) {return handle_cancel();},
    std::bind(&HikrobotDriverNode::accept_move_j, this, std::placeholders::_1));

  move_l_server_ = rclcpp_action::create_server<MoveL>(
    base, clock, logging, waitables, "~/move_l",
    std::bind(&HikrobotDriverNode::handle_move_l_goal, this, std::placeholders::_1, std::placeholders::_2),
    [this](const std::shared_ptr<GoalHandleMoveL>) {return handle_cancel();},
    std::bind(&HikrobotDriverNode::accept_move_l, this, std::placeholders::_1));

  move_c_server_ = rclcpp_action::create_server<MoveC>(
    base, clock, logging, waitables, "~/move_c",
    std::bind(&HikrobotDriverNode::handle_move_c_goal, this, std::placeholders::_1, std::placeholders::_2),
    [this](const std::shared_ptr<GoalHandleMoveC>) {return handle_cancel();},
    std::bind(&HikrobotDriverNode::accept_move_c, this, std::placeholders::_1));

  move_s_server_ = rclcpp_action::create_server<MoveS>(
    base, clock, logging, waitables, "~/move_s",
    std::bind(&HikrobotDriverNode::handle_move_s_goal, this, std::placeholders::_1, std::placeholders::_2),
    [this](const std::shared_ptr<GoalHandleMoveS>) {return handle_cancel();},
    std::bind(&HikrobotDriverNode::accept_move_s, this, std::placeholders::_1));

  follow_cartesian_path_server_ = rclcpp_action::create_server<FollowCartesianPath>(
    base, clock, logging, waitables, "~/follow_cartesian_path",
    std::bind(
      &HikrobotDriverNode::handle_follow_cartesian_path_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    [this](const std::shared_ptr<GoalHandleFollowCartesianPath>) {return handle_cancel();},
    std::bind(
      &HikrobotDriverNode::accept_follow_cartesian_path, this,
      std::placeholders::_1));

  set_do_service_ = create_service<hikrobot_msgs::srv::SetDigitalOutput>(
    "~/set_digital_output",
    [this](
      const std::shared_ptr<hikrobot_msgs::srv::SetDigitalOutput::Request> req,
      std::shared_ptr<hikrobot_msgs::srv::SetDigitalOutput::Response> res)
    {
      if (!ros_active_.load() || !core_->active()) {
        res->success = false;
        res->message = "driver must be ACTIVE and command-connected for DO writes";
        return;
      }
      const auto result = core_->set_digital_output(req->name, req->value);
      res->success = result.success;
      res->sdk_error_code = result.sdk_error;
      res->message = result.message;
    }, rmw_qos_profile_services_default, sdk_service_callback_group_);

  get_do_service_ = create_service<hikrobot_msgs::srv::GetDigitalOutput>(
    "~/get_digital_output",
    [this](
      const std::shared_ptr<hikrobot_msgs::srv::GetDigitalOutput::Request> req,
      std::shared_ptr<hikrobot_msgs::srv::GetDigitalOutput::Response> res)
    {
      const auto result = core_->get_digital_output(req->name);
      res->success = result.success;
      res->value = result.value;
      res->sdk_error_code = result.sdk_error;
      res->message = result.message;
    }, rmw_qos_profile_services_default, sdk_service_callback_group_);

  get_di_service_ = create_service<hikrobot_msgs::srv::GetDigitalInput>(
    "~/get_digital_input",
    [this](
      const std::shared_ptr<hikrobot_msgs::srv::GetDigitalInput::Request> req,
      std::shared_ptr<hikrobot_msgs::srv::GetDigitalInput::Response> res)
    {
      const auto result = core_->get_digital_input(req->name);
      res->success = result.success;
      res->value = result.value;
      res->sdk_error_code = result.sdk_error;
      res->message = result.message;
    }, rmw_qos_profile_services_default, sdk_service_callback_group_);

  stop_service_ = create_service<hikrobot_msgs::srv::StopMotion>(
    "~/stop_motion",
    [this](
      const std::shared_ptr<hikrobot_msgs::srv::StopMotion::Request>,
      std::shared_ptr<hikrobot_msgs::srv::StopMotion::Response> res)
    {
      request_motion_cancel();
      const int64_t ret = core_->stop_motion();
      res->success = ret == 0;
      res->sdk_error_code = ret;
      res->message = res->success ? "stop requested" : core_->last_error_message();
      if (!res->success && res->message.empty()) {
        res->message = "MP_StopRealTimeMotion failed";
      }
    }, rmw_qos_profile_services_default, stop_callback_group_);

  get_active_tool_service_ = create_service<hikrobot_msgs::srv::GetActiveTool>(
    "~/get_active_tool",
    [this](
      const std::shared_ptr<hikrobot_msgs::srv::GetActiveTool::Request>,
      std::shared_ptr<hikrobot_msgs::srv::GetActiveTool::Response> res)
    {
      const auto result = core_->get_active_tool();
      res->success = result.success;
      res->sdk_error_code = result.sdk_error;
      res->message = result.message;
      if (result.success) {
        {
          std::lock_guard<std::mutex> lock(active_tool_mutex_);
          active_tool_cache_ = result.tool;
          active_tool_valid_ = true;
        }
        res->tool = tool_info_to_ros(result.tool);
      }
    }, rmw_qos_profile_services_default, sdk_service_callback_group_);

  get_tool_info_list_service_ = create_service<hikrobot_msgs::srv::GetToolInfoList>(
    "~/get_tool_info_list",
    [this](
      const std::shared_ptr<hikrobot_msgs::srv::GetToolInfoList::Request>,
      std::shared_ptr<hikrobot_msgs::srv::GetToolInfoList::Response> res)
    {
      const auto result = core_->get_tool_info_list();
      res->success = result.success;
      res->sdk_error_code = result.sdk_error;
      res->message = result.message;
      if (result.success) {
        res->tools.reserve(result.tools.size());
        for (const auto & tool : result.tools) {
          res->tools.push_back(tool_info_to_ros(tool));
        }
      }
    }, rmw_qos_profile_services_default, sdk_service_callback_group_);
}

HikrobotDriverNode::CallbackReturn HikrobotDriverNode::on_configure(const rclcpp_lifecycle::State &)
{
  joint_names_ = get_parameter("joint_names").as_string_array();
  if (joint_names_.size() != 6) {
    RCLCPP_ERROR(get_logger(), "joint_names must contain exactly 6 names");
    return CallbackReturn::FAILURE;
  }
  base_frame_ = get_parameter("base_frame").as_string();
  tool_frame_ = get_parameter("tool_frame").as_string();
  programmer_password_env_ = get_parameter("programmer_password_env").as_string();
  default_timeout_sec_ = get_parameter("default_timeout_sec").as_double();
  joint_tolerance_rad_ = get_parameter("joint_tolerance_rad").as_double();
  position_tolerance_m_ = get_parameter("position_tolerance_m").as_double();
  orientation_tolerance_rad_ = get_parameter("orientation_tolerance_rad").as_double();
  stable_samples_ = get_parameter("stable_samples").as_int();
  state_stale_timeout_ms_ = get_parameter("state_stale_timeout_ms").as_int();
  stop_confirmation_timeout_ms_ = get_parameter("stop_confirmation_timeout_ms").as_int();
  sdk_command_lock_timeout_ms_ = get_parameter("sdk_command_lock_timeout_ms").as_int();
  sdk_command_watchdog_ms_ = get_parameter("sdk_command_watchdog_ms").as_int();
  arc_output_name_ = get_parameter("arc_output_name").as_string();
  tool_id_ = get_parameter("tool_id").as_int();
  workpiece_id_ = get_parameter("workpiece_id").as_int();
  max_cartesian_translation_step_m_ = get_parameter("max_cartesian_translation_step_m").as_double();
  max_cartesian_rotation_step_rad_ = get_parameter("max_cartesian_rotation_step_rad").as_double();
  if (max_cartesian_translation_step_m_ <= 0.0 || max_cartesian_rotation_step_rad_ <= 0.0 ||
    stop_confirmation_timeout_ms_ <= 0 || sdk_command_lock_timeout_ms_ <= 0 ||
    sdk_command_watchdog_ms_ <= sdk_command_lock_timeout_ms_ || arc_output_name_.empty())
  {
    RCLCPP_ERROR(
      get_logger(),
      "Cartesian/stop limits must be positive and SDK lock timeout must be less than watchdog");
    return CallbackReturn::FAILURE;
  }

  std::string sequence_file = get_parameter("sequence_state_file").as_string();
  if (sequence_file.empty()) {
    const char * home = std::getenv("HOME");
    sequence_file = home ? std::string(home) + "/.ros/hikrobot/path_id.state" :
      "/tmp/hikrobot_path_id.state";
  }

  hikrobot_mpsdk_core::RuntimeConfig cfg;
  cfg.robot_ip = get_parameter("robot_ip").as_string();
  cfg.robot_port = get_parameter("robot_port").as_int();
  cfg.har_publish_ms = static_cast<uint32_t>(get_parameter("har_publish_ms").as_int());
  cfg.sdk_command_lock_timeout_ms = sdk_command_lock_timeout_ms_;
  cfg.sdk_command_watchdog_ms = sdk_command_watchdog_ms_;
  cfg.arc_output_name = arc_output_name_;
  cfg.sequence_state_file = sequence_file;
  cfg.sequence_collision_retries = get_parameter("sequence_collision_retries").as_int();

  std::string error;
  if (!core_->configure(cfg, error)) {
    RCLCPP_ERROR(get_logger(), "Core configure failed: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }
  if (!refresh_active_tool_cache(error)) {
    RCLCPP_ERROR(get_logger(), "Active Tool load during configure failed: %s", error.c_str());
    core_->cleanup();
    clear_active_tool_cache();
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    get_logger(), "Configured: controller=%s:%d, PathID state=%s",
    cfg.robot_ip.c_str(), cfg.robot_port, cfg.sequence_state_file.c_str());
  return CallbackReturn::SUCCESS;
}

HikrobotDriverNode::CallbackReturn HikrobotDriverNode::on_activate(const rclcpp_lifecycle::State &)
{
  const char * password = std::getenv(programmer_password_env_.c_str());
  if (!password || std::string(password).empty()) {
    RCLCPP_ERROR(
      get_logger(), "Environment variable %s is not set; refusing automatic PROGRAMMER login",
      programmer_password_env_.c_str());
    return CallbackReturn::FAILURE;
  }

  std::string error;
  if (!core_->activate(password, error)) {
    RCLCPP_ERROR(get_logger(), "Core activate failed: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }
  if (!refresh_active_tool_cache(error)) {
    RCLCPP_ERROR(get_logger(), "Active Tool load during activate failed: %s", error.c_str());
    core_->deactivate();
    clear_active_tool_cache();
    return CallbackReturn::FAILURE;
  }

  joint_pub_->on_activate();
  flange_pub_->on_activate();
  tcp_pub_->on_activate();
  status_pub_->on_activate();
  diagnostics_pub_->on_activate();
  cancel_requested_.store(false);
  ros_active_.store(true, std::memory_order_release);
  RCLCPP_INFO(get_logger(), "ACTIVE: AUTO + MotorOn + legacy realtime motion mode ready");
  {
    std::lock_guard<std::mutex> lock(active_tool_mutex_);
    RCLCPP_INFO(
      get_logger(), "Base Cartesian contract: frame=%s, coordinate=base(1:0), active_tool=%d",
      base_frame_.c_str(), active_tool_cache_.id);
  }
  return CallbackReturn::SUCCESS;
}

HikrobotDriverNode::CallbackReturn HikrobotDriverNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  ros_active_.store(false, std::memory_order_release);
  request_motion_cancel();
  (void)core_->stop_motion();
  join_motion_worker();
  core_->deactivate();
  goal_reserved_.store(false, std::memory_order_release);

  diagnostics_pub_->on_deactivate();
  status_pub_->on_deactivate();
  tcp_pub_->on_deactivate();
  flange_pub_->on_deactivate();
  joint_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "INACTIVE: realtime exited, MotorOff, original work mode restored");
  return CallbackReturn::SUCCESS;
}

HikrobotDriverNode::CallbackReturn HikrobotDriverNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  ros_active_.store(false);
  request_motion_cancel();
  join_motion_worker();
  core_->cleanup();
  clear_active_tool_cache();
  goal_reserved_.store(false);
  return CallbackReturn::SUCCESS;
}

HikrobotDriverNode::CallbackReturn HikrobotDriverNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  ros_active_.store(false);
  request_motion_cancel();
  if (core_) {
    (void)core_->stop_motion();
  }
  join_motion_worker();
  if (core_) {
    core_->cleanup();
  }
  return CallbackReturn::SUCCESS;
}

void HikrobotDriverNode::join_motion_worker()
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  if (motion_worker_.joinable()) {
    motion_worker_.join();
  }
}

void HikrobotDriverNode::release_goal_reservation()
{
  goal_reserved_.store(false, std::memory_order_release);
}

void HikrobotDriverNode::request_motion_cancel()
{
  cancel_requested_.store(true, std::memory_order_release);
  std::lock_guard<std::mutex> lock(path_cancel_mutex_);
  if (path_cancel_token_) {
    path_cancel_token_->store(true, std::memory_order_release);
  }
}

rclcpp_action::GoalResponse HikrobotDriverNode::handle_move_j_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJ::Goal> goal)
{
  if (!ros_active_.load() || !core_->active()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->positions.size() != 6 ||
    (!goal->joint_names.empty() && goal->joint_names.size() != goal->positions.size()) ||
    goal->speed_percent < 1 || goal->speed_percent > 100 || goal->blend_radius > 10)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!goal_reserved_.compare_exchange_strong(expected, true)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse HikrobotDriverNode::handle_move_l_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveL::Goal> goal)
{
  if (!ros_active_.load() || !core_->active() || goal->speed_percent < 1 ||
    goal->speed_percent > 100 || goal->blend_radius > 10)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!goal_reserved_.compare_exchange_strong(expected, true)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse HikrobotDriverNode::handle_move_c_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveC::Goal> goal)
{
  if (!ros_active_.load() || !core_->active() || goal->speed_percent < 1 ||
    goal->speed_percent > 100 || goal->blend_radius > 10)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!goal_reserved_.compare_exchange_strong(expected, true)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse HikrobotDriverNode::handle_move_s_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveS::Goal> goal)
{
  if (!ros_active_.load() || !core_->active() || goal->via_poses.size() > 49 ||
    goal->speed_percent < 1 || goal->speed_percent > 100 || goal->blend_radius > 10)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!goal_reserved_.compare_exchange_strong(expected, true)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse HikrobotDriverNode::handle_follow_cartesian_path_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const FollowCartesianPath::Goal> goal)
{
  std::string duration_error;
  if (!ros_active_.load() || !core_->active() || goal->poses.empty() ||
    goal->poses.size() > hikrobot_mpsdk_core::kMaxFollowCartesianPathTargets ||
    goal->speed_percent < 1 || goal->speed_percent > 100 || goal->blend_radius > 10 ||
    !valid_action_duration(goal->timeout, duration_error))
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!goal_reserved_.compare_exchange_strong(expected, true)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  {
    std::lock_guard<std::mutex> lock(path_cancel_mutex_);
    path_cancel_token_ = std::make_shared<std::atomic_bool>(false);
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HikrobotDriverNode::handle_cancel()
{
  request_motion_cancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

hikrobot_mpsdk_core::MotionOptions HikrobotDriverNode::options_from_goal(
  uint8_t speed_percent, uint8_t blend_radius,
  const builtin_interfaces::msg::Duration & timeout) const
{
  hikrobot_mpsdk_core::MotionOptions out;
  out.speed_percent = speed_percent;
  out.blend_radius = blend_radius;
  out.tool_id = tool_id_;
  out.workpiece_id = workpiece_id_;
  out.coordinate_type = 1;
  out.coordinate_id = 0;
  out.timeout = duration_to_ms(timeout, default_timeout_sec_);
  out.joint_tolerance_deg = joint_tolerance_rad_ * kRadToDeg;
  out.position_tolerance_mm = position_tolerance_m_ * 1000.0;
  out.orientation_tolerance_deg = orientation_tolerance_rad_ * kRadToDeg;
  out.stable_samples = stable_samples_;
  out.state_stale_timeout = std::chrono::milliseconds(state_stale_timeout_ms_);
  out.stop_confirmation_timeout =
    std::chrono::milliseconds(stop_confirmation_timeout_ms_);
  return out;
}

bool HikrobotDriverNode::resolve_joint_goal(
  const MoveJ::Goal & goal, std::array<double, 6> & joints_deg, std::string & error) const
{
  if (goal.positions.size() != 6) {
    error = "MoveJ requires exactly 6 positions";
    return false;
  }

  if (goal.joint_names.empty()) {
    for (std::size_t i = 0; i < 6; ++i) {
      joints_deg[i] = goal.positions[i] * kRadToDeg;
    }
    return true;
  }
  if (goal.joint_names.size() != goal.positions.size()) {
    error = "joint_names and positions sizes differ";
    return false;
  }

  for (std::size_t target_index = 0; target_index < joint_names_.size(); ++target_index) {
    const auto it = std::find(goal.joint_names.begin(), goal.joint_names.end(), joint_names_[target_index]);
    if (it == goal.joint_names.end()) {
      error = "MoveJ goal does not contain configured joint " + joint_names_[target_index];
      return false;
    }
    const std::size_t source_index = static_cast<std::size_t>(std::distance(goal.joint_names.begin(), it));
    joints_deg[target_index] = goal.positions[source_index] * kRadToDeg;
  }
  return true;
}

bool HikrobotDriverNode::pose_to_sdk(
  const geometry_msgs::msg::PoseStamped & input,
  hikrobot_mpsdk_core::EulerPose & output, std::string & error) const
{
  return cartesian_pose_to_sdk(input, base_frame_, output, error);
}

bool HikrobotDriverNode::prepare_cartesian_goal(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  hikrobot_mpsdk_core::MotionOptions & options,
  hikrobot_mpsdk_core::ToolInfo & tool, int64_t & sdk_error, std::string & error)
{
  sdk_error = 0;
  if (!refresh_active_tool_cache(error, &sdk_error)) {
    return false;
  }
  bool tool_valid = false;
  {
    std::lock_guard<std::mutex> lock(active_tool_mutex_);
    tool_valid = active_tool_valid_;
    tool = active_tool_cache_;
  }
  if (!apply_base_cartesian_options(tool_valid, tool, options, error)) {
    return false;
  }
  const auto state = core_->robot_state();
  const auto current = hikrobot_mpsdk_core::tcp_transform_if_valid(state, true, tool);
  if (!current) {
    error = "current TCP unavailable from valid non-stale HAR state";
    return false;
  }
  std::vector<hikrobot_mpsdk_core::RigidTransform> path;
  path.reserve(poses.size());
  for (const auto & stamped : poses) {
    hikrobot_mpsdk_core::EulerPose converted;
    if (!cartesian_pose_to_sdk(stamped, base_frame_, converted, error)) {
      return false;
    }
    path.push_back(hikrobot_mpsdk_core::euler_pose_to_transform(converted));
  }
  return hikrobot_mpsdk_core::cartesian_path_within_limits(
    *current, path, max_cartesian_translation_step_m_, max_cartesian_rotation_step_rad_, error);
}

geometry_msgs::msg::PoseStamped HikrobotDriverNode::state_pose_to_ros(
  const hikrobot_mpsdk_core::RobotState & state, const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::PoseStamped out;
  out.header.stamp = stamp;
  out.header.frame_id = base_frame_;
  out.pose = tool_pose_to_ros(state.base_pose);
  return out;
}

geometry_msgs::msg::Pose HikrobotDriverNode::tool_pose_to_ros(
  const hikrobot_mpsdk_core::EulerPose & pose) const
{
  geometry_msgs::msg::Pose out;
  out.position.x = pose.x_mm / 1000.0;
  out.position.y = pose.y_mm / 1000.0;
  out.position.z = pose.z_mm / 1000.0;
  tf2::Quaternion q;
  q.setRPY(pose.roll_deg * kDegToRad, pose.pitch_deg * kDegToRad, pose.yaw_deg * kDegToRad);
  q.normalize();
  out.orientation = tf2::toMsg(q);
  return out;
}

geometry_msgs::msg::Pose HikrobotDriverNode::transform_to_ros(
  const hikrobot_mpsdk_core::RigidTransform & transform) const
{
  geometry_msgs::msg::Pose out;
  out.position.x = transform.x_m;
  out.position.y = transform.y_m;
  out.position.z = transform.z_m;
  out.orientation.x = transform.rotation.x;
  out.orientation.y = transform.rotation.y;
  out.orientation.z = transform.rotation.z;
  out.orientation.w = transform.rotation.w;
  return out;
}

hikrobot_msgs::msg::ToolInfo HikrobotDriverNode::tool_info_to_ros(
  const hikrobot_mpsdk_core::ToolInfo & tool) const
{
  hikrobot_msgs::msg::ToolInfo out;
  out.id = tool.id;
  out.name = tool.name;
  out.description = tool.description;
  out.mount_position = tool.mount_position;
  out.flange_to_tool = tool_pose_to_ros(tool.flange_to_tool);
  out.mass = tool.mass;
  out.mass_center = tool_pose_to_ros(tool.mass_center);
  out.convex_hull_enabled = tool.convex_hull_enabled;
  return out;
}

bool HikrobotDriverNode::refresh_active_tool_cache(std::string & error, int64_t * sdk_error)
{
  const auto result = core_->get_active_tool();
  if (sdk_error) {
    *sdk_error = result.sdk_error;
  }
  if (!result.success) {
    const auto low32 = static_cast<uint32_t>(static_cast<uint64_t>(result.sdk_error));
    std::ostringstream oss;
    oss << result.message << " (sdk=" << result.sdk_error << ", low32=0x" << std::hex << low32 << ')';
    error = oss.str();
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(active_tool_mutex_);
    active_tool_cache_ = result.tool;
    active_tool_valid_ = true;
  }
  RCLCPP_INFO(
    get_logger(), "Active Tool loaded: id=%d, name=%s, mount=%s", result.tool.id,
    result.tool.name.c_str(), result.tool.mount_position == 0 ? "flange" : "external");
  return true;
}

void HikrobotDriverNode::clear_active_tool_cache()
{
  std::lock_guard<std::mutex> lock(active_tool_mutex_);
  active_tool_cache_ = {};
  active_tool_valid_ = false;
}

uint8_t HikrobotDriverNode::map_result_code(hikrobot_mpsdk_core::MotionResultCode code) const
{
  return static_cast<uint8_t>(code);
}

void HikrobotDriverNode::accept_move_j(const std::shared_ptr<GoalHandleMoveJ> goal_handle)
{
  join_motion_worker();
  cancel_requested_.store(false);
  std::lock_guard<std::mutex> lock(worker_mutex_);
  motion_worker_ = std::thread([this, goal_handle]() {
    const auto goal = goal_handle->get_goal();
    hikrobot_mpsdk_core::MoveJRequest request;
    std::string error;
    if (!resolve_joint_goal(*goal, request.joints_deg, error)) {
      auto result = std::make_shared<MoveJ::Result>();
      result->result_code = MoveJ::Result::INVALID_GOAL;
      result->message = error;
      goal_handle->abort(result);
      release_goal_reservation();
      return;
    }
    request.options = options_from_goal(goal->speed_percent, goal->blend_radius, goal->timeout);

    const auto result_core = core_->execute_move_j(
      request,
      [goal_handle, &request](const hikrobot_mpsdk_core::MotionFeedback & fb) {
        auto msg = std::make_shared<MoveJ::Feedback>();
        msg->path_id = fb.path_id;
        msg->progress_percent = fb.progress_percent;
        msg->target_positions.resize(6);
        msg->actual_positions.resize(6);
        msg->position_error.resize(6);
        for (std::size_t i = 0; i < 6; ++i) {
          msg->target_positions[i] = request.joints_deg[i] * kDegToRad;
          msg->actual_positions[i] = fb.robot.joints_deg[i] * kDegToRad;
          msg->position_error[i] = fb.joint_error_deg[i] * kDegToRad;
        }
        goal_handle->publish_feedback(msg);
      },
      &cancel_requested_);

    auto result = std::make_shared<MoveJ::Result>();
    result->result_code = map_result_code(result_core.code);
    result->sdk_error_code = result_core.sdk_error;
    result->path_id = result_core.path_id;
    result->execution_time = duration_from_ms(result_core.execution_time);
    result->message = result_core.message;
    if (result_core.code == hikrobot_mpsdk_core::MotionResultCode::kSuccess) {
      goal_handle->succeed(result);
    } else if (result_core.code == hikrobot_mpsdk_core::MotionResultCode::kCanceled) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
    } else {
      goal_handle->abort(result);
    }
    release_goal_reservation();
  });
}

void HikrobotDriverNode::accept_move_l(const std::shared_ptr<GoalHandleMoveL> goal_handle)
{
  join_motion_worker();
  cancel_requested_.store(false);
  std::lock_guard<std::mutex> lock(worker_mutex_);
  motion_worker_ = std::thread([this, goal_handle]() {
    const auto goal = goal_handle->get_goal();
    hikrobot_mpsdk_core::MoveLRequest request;
    std::string error;
    if (!pose_to_sdk(goal->target_pose, request.target, error)) {
      auto result = std::make_shared<MoveL::Result>();
      result->result_code = MoveL::Result::INVALID_GOAL;
      result->message = error;
      goal_handle->abort(result);
      release_goal_reservation();
      return;
    }
    request.options = options_from_goal(goal->speed_percent, goal->blend_radius, goal->timeout);
    int64_t sdk_error = 0;
    if (!prepare_cartesian_goal(
        {goal->target_pose}, request.options, request.tool, sdk_error, error))
    {
      auto result = std::make_shared<MoveL::Result>();
      result->result_code = MoveL::Result::INVALID_GOAL;
      result->sdk_error_code = sdk_error;
      result->message = error;
      goal_handle->abort(result);
      release_goal_reservation();
      return;
    }

    hikrobot_mpsdk_core::RobotState target_state;
    target_state.base_pose = request.target;
    const auto result_core = core_->execute_move_l(
      request,
      [this, goal_handle, target_state](const hikrobot_mpsdk_core::MotionFeedback & fb) {
        auto msg = std::make_shared<MoveL::Feedback>();
        msg->path_id = fb.path_id;
        msg->progress_percent = fb.progress_percent;
        const auto stamp = now();
        msg->target_pose = state_pose_to_ros(target_state, stamp);
        msg->actual_pose.header.stamp = stamp;
        msg->actual_pose.header.frame_id = base_frame_;
        if (fb.actual_tcp_valid) {
          msg->actual_pose.pose = transform_to_ros(fb.actual_tcp);
        }
        msg->position_error_m = fb.position_error_mm / 1000.0;
        msg->orientation_error_rad = fb.orientation_error_deg * kDegToRad;
        goal_handle->publish_feedback(msg);
      }, &cancel_requested_);

    auto result = std::make_shared<MoveL::Result>();
    result->result_code = map_result_code(result_core.code);
    result->sdk_error_code = result_core.sdk_error;
    result->path_id = result_core.path_id;
    result->execution_time = duration_from_ms(result_core.execution_time);
    result->message = result_core.message;
    if (result_core.code == hikrobot_mpsdk_core::MotionResultCode::kSuccess) {
      goal_handle->succeed(result);
    } else if (result_core.code == hikrobot_mpsdk_core::MotionResultCode::kCanceled) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
    } else {
      goal_handle->abort(result);
    }
    release_goal_reservation();
  });
}

void HikrobotDriverNode::accept_move_c(const std::shared_ptr<GoalHandleMoveC> goal_handle)
{
  join_motion_worker();
  cancel_requested_.store(false);
  std::lock_guard<std::mutex> lock(worker_mutex_);
  motion_worker_ = std::thread([this, goal_handle]() {
    const auto goal = goal_handle->get_goal();
    hikrobot_mpsdk_core::MoveCRequest request;
    std::string error;
    if (!pose_to_sdk(goal->via_pose, request.via, error) ||
      !pose_to_sdk(goal->target_pose, request.target, error))
    {
      auto result = std::make_shared<MoveC::Result>();
      result->result_code = MoveC::Result::INVALID_GOAL;
      result->message = error;
      goal_handle->abort(result);
      release_goal_reservation();
      return;
    }
    request.options = options_from_goal(goal->speed_percent, goal->blend_radius, goal->timeout);
    int64_t sdk_error = 0;
    if (!prepare_cartesian_goal(
        {goal->via_pose, goal->target_pose}, request.options, request.tool, sdk_error, error))
    {
      auto result = std::make_shared<MoveC::Result>();
      result->result_code = MoveC::Result::INVALID_GOAL;
      result->sdk_error_code = sdk_error;
      result->message = error;
      goal_handle->abort(result);
      release_goal_reservation();
      return;
    }
    hikrobot_mpsdk_core::RobotState target_state;
    target_state.base_pose = request.target;

    const auto result_core = core_->execute_move_c(
      request,
      [this, goal_handle, target_state](const hikrobot_mpsdk_core::MotionFeedback & fb) {
        auto msg = std::make_shared<MoveC::Feedback>();
        msg->path_id = fb.path_id;
        msg->progress_percent = fb.progress_percent;
        const auto stamp = now();
        msg->target_pose = state_pose_to_ros(target_state, stamp);
        msg->actual_pose.header.stamp = stamp;
        msg->actual_pose.header.frame_id = base_frame_;
        if (fb.actual_tcp_valid) {
          msg->actual_pose.pose = transform_to_ros(fb.actual_tcp);
        }
        msg->position_error_m = fb.position_error_mm / 1000.0;
        msg->orientation_error_rad = fb.orientation_error_deg * kDegToRad;
        goal_handle->publish_feedback(msg);
      }, &cancel_requested_);

    auto result = std::make_shared<MoveC::Result>();
    result->result_code = map_result_code(result_core.code);
    result->sdk_error_code = result_core.sdk_error;
    result->path_id = result_core.path_id;
    result->execution_time = duration_from_ms(result_core.execution_time);
    result->message = result_core.message;
    if (result_core.code == hikrobot_mpsdk_core::MotionResultCode::kSuccess) {
      goal_handle->succeed(result);
    } else if (result_core.code == hikrobot_mpsdk_core::MotionResultCode::kCanceled) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
    } else {
      goal_handle->abort(result);
    }
    release_goal_reservation();
  });
}

void HikrobotDriverNode::accept_move_s(const std::shared_ptr<GoalHandleMoveS> goal_handle)
{
  join_motion_worker();
  cancel_requested_.store(false);
  std::lock_guard<std::mutex> lock(worker_mutex_);
  motion_worker_ = std::thread([this, goal_handle]() {
    const auto goal = goal_handle->get_goal();
    hikrobot_mpsdk_core::MoveSRequest request;
    std::string error;
    request.via.reserve(goal->via_poses.size());
    for (const auto & pose : goal->via_poses) {
      hikrobot_mpsdk_core::EulerPose converted;
      if (!pose_to_sdk(pose, converted, error)) {
        auto result = std::make_shared<MoveS::Result>();
        result->result_code = MoveS::Result::INVALID_GOAL;
        result->message = error;
        goal_handle->abort(result);
        release_goal_reservation();
        return;
      }
      request.via.push_back(converted);
    }
    if (!pose_to_sdk(goal->target_pose, request.target, error)) {
      auto result = std::make_shared<MoveS::Result>();
      result->result_code = MoveS::Result::INVALID_GOAL;
      result->message = error;
      goal_handle->abort(result);
      release_goal_reservation();
      return;
    }
    request.options = options_from_goal(goal->speed_percent, goal->blend_radius, goal->timeout);
    std::vector<geometry_msgs::msg::PoseStamped> path = goal->via_poses;
    path.push_back(goal->target_pose);
    int64_t sdk_error = 0;
    if (!prepare_cartesian_goal(path, request.options, request.tool, sdk_error, error)) {
      auto result = std::make_shared<MoveS::Result>();
      result->result_code = MoveS::Result::INVALID_GOAL;
      result->sdk_error_code = sdk_error;
      result->message = error;
      goal_handle->abort(result);
      release_goal_reservation();
      return;
    }
    hikrobot_mpsdk_core::RobotState target_state;
    target_state.base_pose = request.target;

    const auto result_core = core_->execute_move_s(
      request,
      [this, goal_handle, target_state](const hikrobot_mpsdk_core::MotionFeedback & fb) {
        auto msg = std::make_shared<MoveS::Feedback>();
        msg->path_id = fb.path_id;
        msg->progress_percent = fb.progress_percent;
        const auto stamp = now();
        msg->target_pose = state_pose_to_ros(target_state, stamp);
        msg->actual_pose.header.stamp = stamp;
        msg->actual_pose.header.frame_id = base_frame_;
        if (fb.actual_tcp_valid) {
          msg->actual_pose.pose = transform_to_ros(fb.actual_tcp);
        }
        msg->position_error_m = fb.position_error_mm / 1000.0;
        msg->orientation_error_rad = fb.orientation_error_deg * kDegToRad;
        goal_handle->publish_feedback(msg);
      }, &cancel_requested_);

    auto result = std::make_shared<MoveS::Result>();
    result->result_code = map_result_code(result_core.code);
    result->sdk_error_code = result_core.sdk_error;
    result->path_id = result_core.path_id;
    result->execution_time = duration_from_ms(result_core.execution_time);
    result->message = result_core.message;
    if (result_core.code == hikrobot_mpsdk_core::MotionResultCode::kSuccess) {
      goal_handle->succeed(result);
    } else if (result_core.code == hikrobot_mpsdk_core::MotionResultCode::kCanceled) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
    } else {
      goal_handle->abort(result);
    }
    release_goal_reservation();
  });
}

void HikrobotDriverNode::accept_follow_cartesian_path(
  const std::shared_ptr<GoalHandleFollowCartesianPath> goal_handle)
{
  join_motion_worker();
  std::shared_ptr<std::atomic_bool> cancel_token;
  {
    std::lock_guard<std::mutex> lock(path_cancel_mutex_);
    cancel_token = path_cancel_token_;
  }
  std::lock_guard<std::mutex> lock(worker_mutex_);
  motion_worker_ = std::thread([this, goal_handle, cancel_token]() {
    const auto reservation_guard = make_scope_exit([this, cancel_token]() {
        {
          std::lock_guard<std::mutex> cancel_lock(path_cancel_mutex_);
          if (path_cancel_token_ == cancel_token) {
            path_cancel_token_.reset();
          }
        }
        release_goal_reservation();
      });

    const auto complete = [goal_handle](
      const std::shared_ptr<FollowCartesianPath::Result> & result,
      hikrobot_mpsdk_core::MotionResultCode code)
      {
        if (code == hikrobot_mpsdk_core::MotionResultCode::kSuccess) {
          goal_handle->succeed(result);
        } else if (code == hikrobot_mpsdk_core::MotionResultCode::kCanceled &&
          goal_handle->is_canceling())
        {
          goal_handle->canceled(result);
        } else {
          goal_handle->abort(result);
        }
      };

    try {
      if (!cancel_token) {
        auto result = std::make_shared<FollowCartesianPath::Result>();
        result->result_code = FollowCartesianPath::Result::DRIVER_NOT_READY;
        result->failure_stage = FollowCartesianPath::Result::STAGE_VALIDATION;
        result->message = "path cancellation state was not initialized";
        complete(result, hikrobot_mpsdk_core::MotionResultCode::kDriverNotReady);
        return;
      }
      if (cancel_token->load(std::memory_order_acquire)) {
        auto result = std::make_shared<FollowCartesianPath::Result>();
        result->result_code = FollowCartesianPath::Result::CANCELED;
        result->failure_stage = FollowCartesianPath::Result::STAGE_VALIDATION;
        result->message = "path canceled after acceptance and before worker execution";
        complete(result, hikrobot_mpsdk_core::MotionResultCode::kCanceled);
        return;
      }

      const auto goal = goal_handle->get_goal();
      hikrobot_mpsdk_core::FollowCartesianPathRequest request;
      request.targets.reserve(goal->poses.size());
      for (std::size_t i = 0; i < goal->poses.size(); ++i) {
        hikrobot_mpsdk_core::EulerPose converted;
        std::string error;
        if (!pose_to_sdk(goal->poses[i], converted, error)) {
          auto result = std::make_shared<FollowCartesianPath::Result>();
          result->result_code = FollowCartesianPath::Result::INVALID_GOAL;
          result->failure_stage = FollowCartesianPath::Result::STAGE_VALIDATION;
          result->failure_target_index_valid = true;
          result->failure_target_index = static_cast<uint32_t>(i);
          result->message = error;
          complete(result, hikrobot_mpsdk_core::MotionResultCode::kInvalidGoal);
          return;
        }
        request.targets.push_back(converted);
      }
      request.options = options_from_goal(
        goal->speed_percent, goal->blend_radius, goal->timeout);

      if (cancel_token->load(std::memory_order_acquire)) {
        auto result = std::make_shared<FollowCartesianPath::Result>();
        result->result_code = FollowCartesianPath::Result::CANCELED;
        result->failure_stage = FollowCartesianPath::Result::STAGE_VALIDATION;
        result->message = "path canceled before Active Tool refresh";
        complete(result, hikrobot_mpsdk_core::MotionResultCode::kCanceled);
        return;
      }

      int64_t sdk_error = 0;
      std::string error;
      if (!prepare_cartesian_goal(
          goal->poses, request.options, request.tool, sdk_error, error))
      {
        auto result = std::make_shared<FollowCartesianPath::Result>();
        result->result_code = sdk_error == 0 ? FollowCartesianPath::Result::INVALID_GOAL :
          FollowCartesianPath::Result::SDK_ERROR;
        result->sdk_error_code = sdk_error;
        result->failure_stage = FollowCartesianPath::Result::STAGE_VALIDATION;
        result->message = error;
        complete(
          result, sdk_error == 0 ? hikrobot_mpsdk_core::MotionResultCode::kInvalidGoal :
          hikrobot_mpsdk_core::MotionResultCode::kSdkError);
        return;
      }
      if (cancel_token->load(std::memory_order_acquire)) {
        auto result = std::make_shared<FollowCartesianPath::Result>();
        result->result_code = FollowCartesianPath::Result::CANCELED;
        result->failure_stage = FollowCartesianPath::Result::STAGE_VALIDATION;
        result->message = "path canceled after validation and before ID reservation";
        complete(result, hikrobot_mpsdk_core::MotionResultCode::kCanceled);
        return;
      }

      const auto core_result = core_->execute_follow_cartesian_path(
        request,
        [this, goal_handle](const hikrobot_mpsdk_core::CartesianPathFeedback & source) {
          auto feedback = std::make_shared<FollowCartesianPath::Feedback>();
          feedback->stage = static_cast<uint8_t>(source.stage);
          feedback->total_targets = static_cast<uint32_t>(source.total_targets);
          feedback->queued_targets = static_cast<uint32_t>(source.queued_targets);
          feedback->current_segment_valid = source.current_segment_valid;
          feedback->current_segment_index = static_cast<uint32_t>(source.current_segment_index);
          feedback->current_path_id = source.current_path_id;
          feedback->actual_tcp_valid = source.actual_tcp_valid;
          if (source.actual_tcp_valid) {
            feedback->actual_tcp.header.stamp = now();
            feedback->actual_tcp.header.frame_id = base_frame_;
            feedback->actual_tcp.pose = transform_to_ros(source.actual_tcp);
          }
          feedback->final_position_error_m = source.final_position_error_mm / 1000.0;
          feedback->final_orientation_error_rad =
            source.final_orientation_error_deg * kDegToRad;
          feedback->motion_callback_sequence = source.motion_callback_sequence;
          feedback->motion_callback_post_start = source.motion_callback_post_start;
          feedback->motion_callback_valid = source.motion_callback_valid;
          feedback->motion_callback_path_id = source.motion_callback_path_id;
          feedback->motion_callback_previous_target_id =
            source.motion_callback_previous_target_id;
          feedback->motion_callback_next_target_id = source.motion_callback_next_target_id;
          feedback->motion_callback_stopped = source.motion_callback_stopped;
          feedback->motion_callback_left_path_count = source.motion_callback_left_path_count;
          feedback->motion_callback_age_ms = source.motion_callback_age_ms;
          feedback->motion_callback_path_matches = source.motion_callback_path_matches;
          feedback->observed_running_queue_evidence = source.observed_running_queue_evidence;
          feedback->observed_terminal_queue_evidence = source.observed_terminal_queue_evidence;
          feedback->final_in_tolerance = source.final_in_tolerance;
          feedback->har_motion_status = source.har_motion_status;
          feedback->har_sample_sequence = source.har_sample_sequence;
          feedback->new_har_sample = source.new_har_sample;
          feedback->stable_sample_count = source.stable_sample_count;
          feedback->required_stable_samples = source.required_stable_samples;
          feedback->completion_blocker = source.completion_blocker;
          feedback->diagnostic_summary = source.diagnostic_summary;
          if (!source.diagnostic_summary.empty()) {
            RCLCPP_INFO(
              get_logger(), "FollowCartesianPath %s", source.diagnostic_summary.c_str());
          }
          goal_handle->publish_feedback(feedback);
        }, cancel_token.get());

      auto result = std::make_shared<FollowCartesianPath::Result>();
      result->result_code = map_result_code(core_result.code);
      result->sdk_error_code = core_result.sdk_error;
      result->path_ids = core_result.path_ids;
      result->failure_stage = static_cast<uint8_t>(core_result.failure_stage);
      result->failure_target_index_valid = core_result.failure_target_index_valid;
      result->failure_target_index = static_cast<uint32_t>(core_result.failure_target_index);
      result->execution_time = duration_from_ms(core_result.execution_time);
      result->message = core_result.message;
      complete(result, core_result.code);
    } catch (const std::exception & error) {
      auto result = std::make_shared<FollowCartesianPath::Result>();
      result->result_code = FollowCartesianPath::Result::SDK_ERROR;
      result->failure_stage = FollowCartesianPath::Result::STAGE_NONE;
      result->message = std::string("FollowCartesianPath worker exception: ") + error.what();
      complete(result, hikrobot_mpsdk_core::MotionResultCode::kSdkError);
    } catch (...) {
      auto result = std::make_shared<FollowCartesianPath::Result>();
      result->result_code = FollowCartesianPath::Result::SDK_ERROR;
      result->failure_stage = FollowCartesianPath::Result::STAGE_NONE;
      result->message = "FollowCartesianPath worker caught an unknown exception";
      complete(result, hikrobot_mpsdk_core::MotionResultCode::kSdkError);
    }
  });
}

void HikrobotDriverNode::publish_state()
{
  if (!ros_active_.load(std::memory_order_acquire) || !core_) {
    return;
  }

  const auto state = core_->robot_state();
  const auto motion = core_->motion_state();
  const auto stamp = now();

  if (state.valid && state.joint_count >= 6) {
    sensor_msgs::msg::JointState joint;
    joint.header.stamp = stamp;
    joint.name = joint_names_;
    joint.position.resize(6);
    for (std::size_t i = 0; i < 6; ++i) {
      joint.position[i] = state.joints_deg[i] * kDegToRad;
    }
    joint_pub_->publish(joint);
  }

  if (state.valid && !state.stale && state.joint_count >= 6) {
    flange_pub_->publish(state_pose_to_ros(state, stamp));
    hikrobot_mpsdk_core::ToolInfo tool;
    bool tool_valid = false;
    {
      std::lock_guard<std::mutex> lock(active_tool_mutex_);
      tool = active_tool_cache_;
      tool_valid = active_tool_valid_;
    }
    const auto tcp = hikrobot_mpsdk_core::tcp_transform_if_valid(state, tool_valid, tool);
    if (tcp) {
      geometry_msgs::msg::PoseStamped tcp_pose;
      tcp_pose.header.stamp = stamp;
      tcp_pose.header.frame_id = base_frame_;
      tcp_pose.pose = transform_to_ros(*tcp);
      tcp_pub_->publish(tcp_pose);
    } else if (!tool_valid || tool.mount_position != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "TCP pose not published: active Tool cache is %s",
        !tool_valid ? "invalid" : "not flange-mounted");
    }
  }

  hikrobot_msgs::msg::RobotStatus status;
  status.header.stamp = stamp;
  status.connected = core_->connected();
  status.active = core_->active();
  status.state_valid = state.valid;
  status.state_stale = state.stale;
  status.servo_status = state.servo_status;
  status.motion_status = state.motion_status;
  status.emergency_status = state.emergency_status;
  status.controller_status = state.controller_status;
  status.safety_gate_status = state.safety_gate_status;
  status.robot_motion_mode = state.robot_motion_mode;
  status.joint_count = state.joint_count;
  status.motion_execution_percent = state.motion_execution_percent;
  status.current_path_id = motion.valid ? motion.path_id : 0;
  status.last_sdk_error = core_->last_sdk_error();
  status.last_error_message = core_->last_error_message();
  status_pub_->publish(status);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus diag;
  diag.name = "hikrobot_driver";
  diag.hardware_id = get_parameter("robot_ip").as_string();
  if (core_->command_recovery_required()) {
    diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag.message = "SDK command channel recovery required; restart Driver process";
  } else if (!status.connected) {
    diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag.message = "SDK command channel disconnected";
  } else if (!state.valid || state.stale) {
    diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag.message = "HAR state stale/unavailable";
  } else if (state.emergency_status != 0 || state.servo_status == 2) {
    diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag.message = "robot safety/servo error state";
  } else {
    diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag.message = "driver active";
  }
  array.status.push_back(diag);
  diagnostics_pub_->publish(array);
}

}  // namespace hikrobot_driver
