#include "hikrobot_mpsdk_core/legacy_io_dispatch.hpp"

#include <iomanip>
#include <sstream>

namespace hikrobot_mpsdk_core
{
namespace
{

std::string failure_message(const char * operation, int64_t code)
{
  std::ostringstream out;
  out << operation << " failed: 0x" << std::hex << std::uppercase << code;
  return out.str();
}

}  // namespace

IoResult dispatch_digital_get(
  const std::string & name, DigitalIoType type, const DigitalGetCall & sdk_get)
{
  IoResult out;
  bool value = false;
  const int64_t ret = sdk_get(name, type, value);
  out.success = ret == 0;
  out.value = value;
  out.sdk_error = ret;
  out.message = out.success ? "ok" : failure_message("MP_GetConfigIOLevelStatus", ret);
  return out;
}

IoResult dispatch_digital_set(
  const std::string & name, bool value, const DigitalSetCall & sdk_set)
{
  IoResult out;
  const int64_t ret = sdk_set(name, value);
  out.success = ret == 0;
  out.value = value;
  out.sdk_error = ret;
  out.message = out.success ? "ok" : failure_message("MP_SetConfigIOLevelStatus", ret);
  return out;
}

}  // namespace hikrobot_mpsdk_core
