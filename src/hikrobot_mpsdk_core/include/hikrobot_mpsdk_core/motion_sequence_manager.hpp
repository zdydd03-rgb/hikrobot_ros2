#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

namespace hikrobot_mpsdk_core
{

class MotionSequenceManager
{
public:
  explicit MotionSequenceManager(std::string state_file);

  // Reserves a contiguous positive ID block and persists the high-water mark
  // before returning. The first ID is used as nPathID; subsequent IDs can be
  // used as target IDs for the same SDK command.
  int32_t reserve_block(std::size_t count);

  int32_t high_water_mark() const;

private:
  int32_t load_or_seed() const;
  void persist_atomic(int32_t value) const;

  std::string state_file_;
  mutable std::mutex mutex_;
  int32_t high_water_mark_{0};
};

}  // namespace hikrobot_mpsdk_core
