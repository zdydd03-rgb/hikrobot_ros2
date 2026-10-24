#include <arpa/inet.h>

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>

#include "MP_HAR_API.h"
#include "MP_HAR_Struct.h"
#include "MP_Public_API.h"
#include "MP_Public_Error.h"
#include "MP_Public_Struct.h"

namespace
{

std::string bounded_string(const char * value, std::size_t capacity)
{
  const char * const end = value + capacity;
  const char * const terminator = std::find(value, end, '\0');
  return std::string(value, terminator);
}

void report_call(const char * api, int64_t value, bool success)
{
  const auto low32 = static_cast<uint32_t>(static_cast<uint64_t>(value));
  std::cout << api << ": return=" << value << " low32=0x"
            << std::hex << std::setw(8) << std::setfill('0') << low32
            << std::dec << std::setfill(' ') << " " << (success ? "PASS" : "FAIL") << '\n';
}

bool report_status(const char * api, int64_t value)
{
  const bool success = value == MP_SUCCESS;
  report_call(api, value, success);
  return success;
}

const char * mount_position_name(E_MP_MOUNT_POSITION value)
{
  switch (value) {
    case E_MP_MOUNT_FLANGE:
      return "flange";
    case E_MP_MOUNT_EXTERNAL:
      return "external";
    default:
      return "unknown";
  }
}

const char * common_status_name(E_MP_COMMON_STATUS value)
{
  switch (value) {
    case E_MP_STATUS_DISABLE:
      return "disabled";
    case E_MP_STATUS_ENABLE:
      return "enabled";
    default:
      return "unknown";
  }
}

void print_pose(const ST_HAR_EULER & pose, const std::string & indent)
{
  std::cout << indent << "XYZ_mm: " << pose.dX << ", " << pose.dY << ", " << pose.dZ << '\n';
  std::cout << indent << "RPY_deg: " << pose.dRoll << ", " << pose.dPitch << ", " << pose.dYaw << '\n';
}

void print_tool_summary(const ST_HAR_TOOL & tool, const std::string & prefix)
{
  std::cout << prefix << "id: " << tool.nID << '\n';
  std::cout << prefix << "name: " << bounded_string(tool.szName, sizeof(tool.szName)) << '\n';
  std::cout << prefix << "mount_position: " << mount_position_name(tool.eMountPosition)
            << " (" << static_cast<int>(tool.eMountPosition) << ")\n";
  print_pose(tool.stBody.stPoseE, prefix);
}

void print_tool_detail(const ST_HAR_TOOL & tool)
{
  std::cout << "Active tool detail:\n";
  print_tool_summary(tool, "  ");
  std::cout << "  description: "
            << bounded_string(tool.szDescription, sizeof(tool.szDescription)) << '\n';
  std::cout << "  mass_sdk_dMass: " << tool.stBody.dMass << '\n';
  std::cout << "  mass_center:\n";
  print_pose(tool.stBody.stMassCenter, "    ");
  std::cout << "  convex_hull: " << common_status_name(tool.eCvxEnable)
            << " (" << static_cast<int>(tool.eCvxEnable) << ")\n";
}

bool parse_port(std::string_view text, int & port)
{
  if (text.empty()) {
    return false;
  }
  int parsed = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    return false;
  }
  if (parsed < 1 || parsed > 65535) {
    return false;
  }
  port = parsed;
  return true;
}

class SdkSession
{
public:
  bool create()
  {
    handle_ = MP_CreateHandle();
    const bool success = handle_ != MP_INVALID_HANDLE;
    report_call("MP_CreateHandle", static_cast<int64_t>(handle_), success);
    return success;
  }

  bool connect(const std::string & ip, int port)
  {
    ST_ROBOT_CONNECT_INFO connection{};
    if (ip.size() >= sizeof(connection.stCtrlServerInfo.szIpAddr)) {
      std::cerr << "IP address exceeds SDK buffer capacity\n";
      return false;
    }
    std::memcpy(connection.stCtrlServerInfo.szIpAddr, ip.data(), ip.size());
    connection.stCtrlServerInfo.nPort = port;

    const int64_t result = MP_ConnectRobotWithIP(handle_, &connection);
    connected_ = result == MP_SUCCESS;
    return report_status("MP_ConnectRobotWithIP", result);
  }

  MP_HANDLE handle() const
  {
    return handle_;
  }

  int finish(int exit_code)
  {
    bool cleanup_success = true;
    if (connected_) {
      const int64_t result = MP_DisconnectRobot(handle_);
      cleanup_success = report_status("MP_DisconnectRobot", result) && cleanup_success;
      connected_ = false;
    }
    if (handle_ != MP_INVALID_HANDLE) {
      const int64_t result = MP_DestroyHandle(handle_);
      cleanup_success = report_status("MP_DestroyHandle", result) && cleanup_success;
      handle_ = MP_INVALID_HANDLE;
    }
    return exit_code == 0 && cleanup_success ? 0 : 1;
  }

  ~SdkSession()
  {
    (void)finish(1);
  }

private:
  MP_HANDLE handle_{MP_INVALID_HANDLE};
  bool connected_{false};
};

int run_probe(const std::string & ip, int port)
{
  SdkSession session;
  if (!session.create()) {
    return session.finish(1);
  }
  if (!session.connect(ip, port)) {
    return session.finish(1);
  }

  ST_TOOL_SIMPLE_INFO active_tool{};
  if (!report_status("MP_GetActivateTool", MP_GetActivateTool(session.handle(), &active_tool))) {
    return session.finish(1);
  }
  std::cout << "Active tool: id=" << active_tool.nToolID
            << " name=" << bounded_string(active_tool.szName, sizeof(active_tool.szName)) << '\n';
  if (active_tool.nToolID < 0 || active_tool.nToolID >= MP_TOOL_NUM_MAX) {
    std::cerr << "MP_GetActivateTool returned invalid tool ID: " << active_tool.nToolID << '\n';
    return session.finish(1);
  }

  ST_HAR_TOOL_LIST tool_list{};
  if (!report_status("MP_HAR_GetToolInfoList", MP_HAR_GetToolInfoList(session.handle(), &tool_list))) {
    return session.finish(1);
  }
  if (tool_list.nNum < 0 || tool_list.nNum > MP_TOOL_NUM_MAX) {
    std::cerr << "MP_HAR_GetToolInfoList returned invalid tool count: " << tool_list.nNum << '\n';
    return session.finish(1);
  }
  std::cout << "Tool list count: " << tool_list.nNum << '\n';
  for (int32_t index = 0; index < tool_list.nNum; ++index) {
    std::cout << "Tool list entry " << index << ":\n";
    print_tool_summary(tool_list.szTool[index], "  ");
  }

  ST_TOOL_INDEX tool_index{};
  tool_index.nToolID = active_tool.nToolID;
  ST_HAR_TOOL active_tool_detail{};
  if (!report_status(
      "MP_HAR_GetToolInfo",
      MP_HAR_GetToolInfo(session.handle(), &tool_index, &active_tool_detail)))
  {
    return session.finish(1);
  }
  print_tool_detail(active_tool_detail);
  return session.finish(0);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <controller_ip> <controller_port>\n";
    return 2;
  }

  const std::string ip = argv[1];
  in_addr parsed_ip{};
  if (::inet_pton(AF_INET, ip.c_str(), &parsed_ip) != 1) {
    std::cerr << "Invalid IPv4 address: " << ip << '\n';
    return 2;
  }

  int port = 0;
  if (!parse_port(argv[2], port)) {
    std::cerr << "Invalid controller port: " << argv[2] << " (expected 1..65535)\n";
    return 2;
  }
  return run_probe(ip, port);
}
