#pragma once
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Core>
namespace svo_ros {
class EgoInput : public rclcpp::Node
{
public:
  explicit EgoInput(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
private:
  void image(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  void imu(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  Eigen::Vector3d acc_bias_, gyro_bias_;
  rclcpp::CallbackGroup::SharedPtr image_group_, imu_group_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_, right_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr corrected_imu_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<uint64_t> received_images_{0}, decoded_images_{0}, received_imu_{0}, published_imu_{0}, invalid_images_{0};
};
} // namespace svo_ros
