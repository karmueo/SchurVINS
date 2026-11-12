#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <stdexcept>
#include <vector>

namespace svo_ros {
// Caller owns synchronization. No wall-clock timeout: paused replay retains state.
template<class Image, class Imu>
class InputScheduler
{
public:
  struct Counters
  {
    uint64_t image_messages = 0, image_groups = 0, imu_received = 0;
    uint64_t imu_consumed = 0, leading_skipped = 0, overflow = 0;
    uint64_t time_jumps = 0, duplicates = 0;
  } counters;
  struct Bundle { int64_t stamp; std::vector<Image> images; };
  InputScheduler(size_t cameras, bool use_imu, size_t image_limit = 64,
                 size_t imu_limit = 10000)
    : cameras_(cameras), use_imu_(use_imu), image_limit_(image_limit),
      imu_limit_(imu_limit), last_image_(cameras, -1)
  {
    if (cameras == 0 || image_limit == 0 || imu_limit == 0)
      throw std::invalid_argument("Queue capacities and camera count must be positive");
  }
  bool image(size_t camera, int64_t stamp, Image image)
  {
    if (stopped_) return false;
    ++counters.image_messages;
    if (stamp < last_image_.at(camera)) return timeJump();
    if (stamp == last_image_.at(camera)) { ++counters.duplicates; return false; }
    last_image_[camera] = stamp;
    auto it = images_.find(stamp);
    if (it == images_.end())
    {
      if (images_.size() >= image_limit_) return overflow();
      it = images_.emplace(stamp, std::vector<Image>(cameras_)).first;
    }
    it->second[camera] = std::move(image);
    return true;
  }
  bool imu(int64_t stamp, Imu sample)
  {
    if (stopped_) return false;
    ++counters.imu_received;
    if (stamp < last_imu_) return timeJump();
    if (stamp == last_imu_) { ++counters.duplicates; return false; }
    if (imu_.size() >= imu_limit_) return overflow();
    if (first_imu_ < 0) first_imu_ = stamp;
    last_imu_ = stamp;
    imu_.emplace_back(stamp, std::move(sample));
    return true;
  }
  bool canTakeImu(bool initialization_or_final_drain = false) const
  {
    if (imu_.empty()) return false;
    if (initialization_or_final_drain) return true;
    return !images_.empty() && covered_ < images_.begin()->first;
  }
  bool takeImu(Imu& result, bool initialization_or_final_drain = false)
  {
    if (!canTakeImu(initialization_or_final_drain)) return false;
    covered_ = imu_.front().first;
    result = std::move(imu_.front().second);
    imu_.pop_front();
    ++counters.imu_consumed;
    return true;
  }
  bool ready() const
  {
    if (stopped_ || images_.empty()) return false;
    for (const auto& image : images_.begin()->second) if (!image) return false;
    return !use_imu_ || (first_imu_ >= 0 && covered_ >= images_.begin()->first);
  }
  bool takeImage(Bundle& result, bool& leading)
  {
    if (!ready()) return false;
    auto it = images_.begin();
    result = Bundle{it->first, std::move(it->second)};
    images_.erase(it);
    ++counters.image_groups;
    leading = use_imu_ && result.stamp < first_imu_;
    if (leading) ++counters.leading_skipped;
    return true;
  }
  int64_t nextStamp() const { return images_.begin()->first; }
  bool leadingNext() const { return use_imu_ && first_imu_ >= 0 && nextStamp() < first_imu_; }
  size_t pendingImages() const { return images_.size(); }
  size_t pendingImu() const { return imu_.size(); }
  bool hasImu() const { return !imu_.empty(); }
  bool stopped() const { return stopped_; }
private:
  bool overflow() { ++counters.overflow; stopped_ = true; return false; }
  bool timeJump() { ++counters.time_jumps; stopped_ = true; return false; }
  size_t cameras_, image_limit_, imu_limit_;
  bool use_imu_, stopped_ = false;
  std::vector<int64_t> last_image_;
  int64_t first_imu_ = -1, last_imu_ = -1, covered_ = -1;
  std::map<int64_t, std::vector<Image>> images_;
  std::deque<std::pair<int64_t, Imu>> imu_;
};
}  // namespace svo_ros
