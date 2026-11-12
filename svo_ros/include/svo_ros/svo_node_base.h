#pragma once
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <svo_ros/input_scheduler.h>
#include <svo_ros/svo_interface.h>

namespace svo_ros {
class SvoNodeBase : public rclcpp::Node
{
public:
  explicit SvoNodeBase(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SvoNodeBase() override;
  bool stopped() { std::lock_guard<std::mutex> lock(mutex_); return quit_ || failed_; }
private:
  using Image = sensor_msgs::msg::Image::ConstSharedPtr;
  using Imu = sensor_msgs::msg::Imu::ConstSharedPtr;
  using Scheduler = InputScheduler<Image, Imu>;
  void image(size_t camera, Image msg);
  void imu(Imu msg);
  void worker();
  void createEstimator(std::ofstream trajectory = std::ofstream());
  void closeRun();
  void saveConfiguration();
  void publishStatus();
  std::string statusLocked() const;
  svo::PipelineType type_;
  std::unique_ptr<svo::SvoInterface> interface_;
  std::unique_ptr<Scheduler> scheduler_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> images_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::CallbackGroup::SharedPtr image_group_, imu_group_, control_group_;
  std::mutex mutex_;
  std::condition_variable changed_;
  std::deque<char> commands_;
  std::thread worker_;
  bool quit_ = false, idle_ = false, failed_ = false, use_imu_ = true;
  bool draining_ = false, inflight_ = false;
  double initialization_seconds_ = -1.0;
  int64_t first_processed_stamp_ = -1;
  size_t cameras_ = 2;
  uint64_t invalid_images_ = 0, invalid_imu_ = 0, idle_images_ = 0, idle_imu_ = 0;
  uint64_t processed_ = 0, poses_ = 0, resets_ = 0, tracking_lost_ = 0;
  int previous_stage_ = -1;
  std::string output_root_, run_dir_, error_;
  std::ofstream frames_;
  std::vector<double> durations_ms_;
  std::chrono::steady_clock::time_point start_;
};
} // namespace svo_ros
