#include "hikrobot_mpsdk_core/motion_sequence_manager.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

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

  constexpr int kMaxSequence = 2000000000;
  {
    std::ofstream out(path);
    out << kMaxSequence - 2 << '\n';
  }
  {
    hikrobot_mpsdk_core::MotionSequenceManager manager(path);
    assert(manager.reserve_block(1) == kMaxSequence - 1);
    assert(manager.high_water_mark() == kMaxSequence - 1);

    bool exhausted = false;
    try {
      (void)manager.reserve_block(1);
    } catch (const std::overflow_error & error) {
      const std::string message = error.what();
      assert(message.find("PathID") != std::string::npos);
      assert(message.find("refusing to wrap") != std::string::npos);
      exhausted = true;
    }
    assert(exhausted);
    assert(manager.high_water_mark() == kMaxSequence - 1);
  }
  {
    std::ifstream boundary_state(path);
    int boundary_persisted = 0;
    boundary_state >> boundary_persisted;
    assert(boundary_persisted == kMaxSequence - 1);
  }

  {
    std::ofstream out(path);
    out << kMaxSequence - 10 << '\n';
  }
  {
    hikrobot_mpsdk_core::MotionSequenceManager manager(path);
    bool exhausted = false;
    try {
      (void)manager.reserve_block(20);
    } catch (const std::overflow_error &) {
      exhausted = true;
    }
    assert(exhausted);
    assert(manager.high_water_mark() == kMaxSequence - 10);
  }
  {
    std::ifstream boundary_state(path);
    int boundary_persisted = 0;
    boundary_state >> boundary_persisted;
    assert(boundary_persisted == kMaxSequence - 10);
  }

  std::filesystem::remove(path);
  std::cout << "MotionSequenceManager persistence test: PASS\n";
  return 0;
}
