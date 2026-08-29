#include "hikrobot_mpsdk_core/motion_sequence_manager.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>

int main()
{
  const std::string path = "/tmp/hikrobot_sequence_manager_test.state";
  std::filesystem::remove(path);
  std::filesystem::remove(path + ".tmp");

  int first = 0;
  {
    hikrobot_mpsdk_core::MotionSequenceManager manager(path);
    first = manager.reserve_block(3);
    const int second = manager.reserve_block(2);
    assert(second == first + 3);
    assert(manager.high_water_mark() == second + 1);
  }

  {
    hikrobot_mpsdk_core::MotionSequenceManager manager(path);
    const int after_restart = manager.reserve_block(1);
    assert(after_restart == first + 5);
  }

  std::ifstream in(path);
  int persisted = 0;
  in >> persisted;
  assert(persisted == first + 5);

  std::filesystem::remove(path);
  std::cout << "MotionSequenceManager persistence test: PASS\n";
  return 0;
}
