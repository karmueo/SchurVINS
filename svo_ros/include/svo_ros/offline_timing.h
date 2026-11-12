#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <utility>

namespace svo_ros {
// Only explicitly measured algorithm calls contribute to the FPS denominator.
class ComputationTimer
{
public:
  template<class Function>
  auto measure(Function&& function)
  {
    const auto start = std::chrono::steady_clock::now();
    struct Accumulate
    {
      double& seconds;
      std::chrono::steady_clock::time_point start;
      ~Accumulate() { seconds += std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(); }
    } accumulate{seconds_, start};
    return std::forward<Function>(function)();
  }
  double seconds() const { return seconds_; }
  double fps(uint64_t frames) const { return seconds_ > 0 ? frames / seconds_ : 0.0; }
private:
  double seconds_ = 0.0;
};

class ReplaySchedule
{
public:
  using Clock = std::chrono::steady_clock;
  explicit ReplaySchedule(bool realtime) : realtime_(realtime) {}
  Clock::time_point target(int64_t stamp, Clock::time_point now)
  {
    if (!realtime_) return now;
    if (!first_stamp_) { first_stamp_ = stamp; start_ = now; }
    return start_ + std::chrono::nanoseconds(stamp - *first_stamp_);
  }
private:
  bool realtime_;
  std::optional<int64_t> first_stamp_;
  Clock::time_point start_;
};
}  // namespace svo_ros
