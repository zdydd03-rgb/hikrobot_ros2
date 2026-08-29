#include "hikrobot_mpsdk_core/motion_sequence_manager.hpp"

#include <chrono>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>

#include <fcntl.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace hikrobot_mpsdk_core
{
namespace
{
constexpr int32_t kMaxSequence = 2000000000;
}

MotionSequenceManager::MotionSequenceManager(std::string state_file)
: state_file_(std::move(state_file)), high_water_mark_(load_or_seed())
{
}

int32_t MotionSequenceManager::load_or_seed() const
{
  std::ifstream in(state_file_);
  int64_t value = 0;
  if (in >> value && value >= 0 && value < kMaxSequence) {
    return static_cast<int32_t>(value);
  }

  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now).count();
  return static_cast<int32_t>(100000 + (seconds % 1000000000));
}

void MotionSequenceManager::persist_atomic(int32_t value) const
{
  fs::path path(state_file_);
  if (path.has_parent_path()) {
    fs::create_directories(path.parent_path());
  }

  const fs::path tmp = path.string() + ".tmp";
  const std::string payload = std::to_string(value) + "\n";

  int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
  if (fd < 0) {
    throw std::runtime_error("cannot open sequence temp file: " + std::string(std::strerror(errno)));
  }

  const char * data = payload.data();
  std::size_t left = payload.size();
  while (left > 0) {
    const ssize_t written = ::write(fd, data, left);
    if (written < 0) {
      const int saved = errno;
      ::close(fd);
      throw std::runtime_error("cannot write sequence state: " + std::string(std::strerror(saved)));
    }
    data += written;
    left -= static_cast<std::size_t>(written);
  }

  if (::fsync(fd) != 0) {
    const int saved = errno;
    ::close(fd);
    throw std::runtime_error("cannot fsync sequence state: " + std::string(std::strerror(saved)));
  }
  ::close(fd);

  fs::rename(tmp, path);

  if (path.has_parent_path()) {
    int dirfd = ::open(path.parent_path().c_str(), O_RDONLY | O_DIRECTORY);
    if (dirfd >= 0) {
      (void)::fsync(dirfd);
      ::close(dirfd);
    }
  }
}

int32_t MotionSequenceManager::reserve_block(std::size_t count)
{
  if (count == 0 || count > 64) {
    throw std::invalid_argument("sequence block size must be in [1, 64]");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  int64_t first = static_cast<int64_t>(high_water_mark_) + 1;
  int64_t next_high = first + static_cast<int64_t>(count) - 1;

  if (next_high >= kMaxSequence) {
    first = 1;
    next_high = static_cast<int64_t>(count);
  }

  persist_atomic(static_cast<int32_t>(next_high));
  high_water_mark_ = static_cast<int32_t>(next_high);
  return static_cast<int32_t>(first);
}

int32_t MotionSequenceManager::high_water_mark() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return high_water_mark_;
}

}  // namespace hikrobot_mpsdk_core
