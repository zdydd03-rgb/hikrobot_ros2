#include "hikrobot_driver/hikrobot_driver_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <sstream>
#include <utility>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"

namespace hikrobot_driver
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;

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
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

HikrobotDriverNode::~HikrobotDriverNode()
{
  ros_active_.store(false);
  cancel_requested_.store(true);
  if (core_) {
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
  declare_parameter<int>("tool_id", 0);
  declare_parameter<int>("workpiece_id", -1);
  declare_parameter<double>("state_publish_rate_hz", 20.0);
}

void HikrobotDriverNode::create_ros_interfaces()
{
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());
  flange_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/flange_pose", 10);
  tcp_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/tcp_pose", 10);
  status_pub_ = create_publisher<hikrobot_msgs::msg::RobotStatus>("~/status", 10);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  const double hz = std::max(1.0, get_parameter("state_publish_rate_hz").as_double());
  state_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
    std::bind(&HikrobotDriverNode::publish_state, this));

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

  set_do_service_ = create_service<hikrobot_msgs::srv::SetDigitalOutput>(
    "~/set_digital_output",
    [this](
      const std::shared_ptr<hikrobot_msgs::srv::SetDigitalOutput::Request> req,
      std::shared_ptr<hikrobot_msgs::srv::SetDigitalOutput::Response> res)
    {
      if (!ros_active_.load()) {
        res->success = false;
        res->message = "driver must be ACTIVE for DO writes";
        return;
      }
      const auto result = core_->set_digital_output(req->name, req->value);
      res->success = result.success;
      res->sdk_error_code = result.sdk_error;
      res->message = result.message;
    });

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
    });

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
    });

  stop_service_ = create_service<hikrobot_msgs::srv::StopMotion>(
    "~/stop_motion",
    [this](
      const std::shared_ptr<hikrobot_msgs::srv::StopMotion::Request>,
      std::shared_ptr<hikrobot_msgs::srv::StopMotion::Response> res)
    {
      cancel_requested_.store(true, std::memory_order_release);
      const int64_t ret = core_->stop_motion();
      res->success = ret == 0;
      res->sdk_error_code = ret;
      res->message = res->success ? "stop requested" : "MP_StopRealTimeMotion failed";
    });

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
    });

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
    });
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
  tool_id_ = get_parameter("tool_id").as_int();
  workpiece_id_ = get_parameter("workpiece_id").as_int();

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
  return CallbackReturn::SUCCESS;
}

HikrobotDriverNode::CallbackReturn HikrobotDriverNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  ros_active_.store(false, std::memory_order_release);
  cancel_requested_.store(true, std::memory_order_release);
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
  cancel_requested_.store(true);
  join_motion_worker();
  core_->cleanup();
  clear_active_tool_cache();
  goal_reserved_.store(false);
  return CallbackReturn::SUCCESS;
}

HikrobotDriverNode::CallbackReturn HikrobotDriverNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  ros_active_.store(false);
  cancel_requested_.store(true);
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

rclcpp_action::GoalResponse HikrobotDriverNode::handle_move_j_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJ::Goal> goal)
{
  if (!ros_active_.load()) {
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
  if (!ros_active_.load() || goal->speed_percent < 1 || goal->speed_percent > 100 || goal->blend_radius > 10) {
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
  if (!ros_active_.load() || goal->speed_percent < 1 || goal->speed_percent > 100 || goal->blend_radius > 10) {
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
  if (!ros_active_.load() || goal->via_poses.size() > 49 ||
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

rclcpp_action::CancelResponse HikrobotDriverNode::handle_cancel()
{
  cancel_requested_.store(true, std::memory_order_release);
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
  geometry_msgs::msg::PoseStamped pose = input;
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = base_frame_;
  }

  if (pose.header.frame_id != base_frame_) {
    try {
      pose = tf_buffer_->transform(pose, base_frame_, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & e) {
      error = std::string("cannot transform pose to ") + base_frame_ + ": " + e.what();
      return false;
    }
  }

  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  if (q.length2() < 1e-12) {
    error = "target quaternion has zero length";
    return false;
  }
  q.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  output.x_mm = pose.pose.position.x * 1000.0;
  output.y_mm = pose.pose.position.y * 1000.0;
  output.z_mm = pose.pose.position.z * 1000.0;
  output.roll_deg = roll * kRadToDeg;
  output.pitch_deg = pitch * kRadToDeg;
  output.yaw_deg = yaw * kRadToDeg;
  return true;
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

bool HikrobotDriverNode::refresh_active_tool_cache(std::string & error)
{
  const auto result = core_->get_active_tool();
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
        msg->actual_pose = state_pose_to_ros(fb.robot, stamp);
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
        msg->actual_pose = state_pose_to_ros(fb.robot, stamp);
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
        msg->actual_pose = state_pose_to_ros(fb.robot, stamp);
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
  status.connected = core_->configured();
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
  if (!state.valid || state.stale) {
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
