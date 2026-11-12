#pragma once
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <svo_ros/ego_processing.h>
namespace svo_ros {
class EgoInput : public rclcpp::Node
{
public:
  explicit EgoInput(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~EgoInput() override;
private:
  void image(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  void decodeImages();
  void decodeImage(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  void imu(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  EgoProcessing processing_;
  rclcpp::CallbackGroup::SharedPtr image_group_, imu_group_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_, right_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr corrected_imu_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<uint64_t> received_images_{0}, decoded_images_{0}, received_imu_{0}, published_imu_{0}, invalid_images_{0};
  std::atomic<uint64_t> image_queue_overflow_{0};
  std::mutex image_mutex_;
  std::condition_variable image_changed_;
  std::deque<sensor_msgs::msg::CompressedImage::ConstSharedPtr> pending_images_;
  size_t image_queue_capacity_ = 64;
  bool stopping_ = false, decoder_inflight_ = false;
  std::thread decoder_;
};
} // namespace svo_ros
