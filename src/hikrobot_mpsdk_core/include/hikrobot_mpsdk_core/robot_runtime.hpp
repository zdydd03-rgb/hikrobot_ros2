#pragma once

#include "hikrobot_mpsdk_core/motion_sequence_manager.hpp"
#include "hikrobot_mpsdk_core/types.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace hikrobot_mpsdk_core
{

struct RuntimeConfig
{
  std::string robot_ip{"192.168.2.64"};
  int robot_port{9000};
  uint32_t har_publish_ms{50};
  int sdk_command_lock_timeout_ms{1000};
  int sdk_command_watchdog_ms{3000};
  std::string sequence_state_file{"/tmp/hikrobot_path_id.state"};
  int sequence_collision_retries{3};
};

struct RuntimeHealthSnapshot
{
  bool monitoring_active{false};
  bool sdk_connected{false};
  bool command_connected{false};
  bool recovery_required{false};
  bool sdk_call_in_progress{false};
  std::chrono::milliseconds sdk_call_elapsed{0};
  int64_t fault_code{0};
  std::string fault_message;
  std::string active_operation;
  bool har_received{false};
  uint64_t har_sequence{0};
  std::chrono::milliseconds har_age{0};
  bool har_timed_out{false};
  int emergency_status{0};
  bool emergency_active{false};
};

class RobotRuntime
{
public:
  RobotRuntime();
  ~RobotRuntime();

  RobotRuntime(const RobotRuntime &) = delete;
  RobotRuntime & operator=(const RobotRuntime &) = delete;

  bool configure(const RuntimeConfig & config, std::string & error);
  bool activate(const std::string & programmer_password, std::string & error);
  void deactivate();
  void cleanup();

  bool configured() const;
  bool connected() const;
  bool active() const;
  bool busy() const;
  bool command_recovery_required() const;
  RuntimeHealthSnapshot monitor_health(std::chrono::milliseconds communication_timeout);

  RobotState robot_state() const;
  MotionState motion_state() const;
  ToolResult get_active_tool();
  ToolListResult get_tool_info_list();

  MotionResult execute_move_j(
    const MoveJRequest & request,
    const FeedbackCallback & feedback,
    const std::atomic_bool * cancel_requested);
  MotionResult execute_move_l(
    const MoveLRequest & request,
    const FeedbackCallback & feedback,
    const std::atomic_bool * cancel_requested);
  MotionResult execute_move_c(
    const MoveCRequest & request,
    const FeedbackCallback & feedback,
    const std::atomic_bool * cancel_requested);
  MotionResult execute_move_s(
    const MoveSRequest & request,
    const FeedbackCallback & feedback,
    const std::atomic_bool * cancel_requested);
  CartesianPathResult execute_follow_cartesian_path(
    const FollowCartesianPathRequest & request,
    const CartesianPathFeedbackCallback & feedback,
    const std::atomic_bool * cancel_requested);

  int64_t stop_motion();

  IoResult set_digital_output(const std::string & name, bool value);
  IoResult get_digital_output(const std::string & name) const;
  IoResult get_digital_input(const std::string & name) const;
  std::vector<std::string> list_digital_outputs(int64_t & sdk_error) const;

  int64_t last_sdk_error() const;
  std::string last_error_message() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hikrobot_mpsdk_core
