#pragma once

#include "hikrobot_mpsdk_core/types.hpp"

#include <cstdint>
#include <functional>
#include <string>

namespace hikrobot_mpsdk_core
{

enum class DigitalIoType
{
  kInput,
  kOutput,
};

using DigitalGetCall = std::function<int64_t(const std::string &, DigitalIoType, bool &)>;
using DigitalSetCall = std::function<int64_t(const std::string &, bool)>;

IoResult dispatch_digital_get(
  const std::string & name, DigitalIoType type, const DigitalGetCall & sdk_get);
IoResult dispatch_digital_set(
  const std::string & name, bool value, const DigitalSetCall & sdk_set);

}  // namespace hikrobot_mpsdk_core
