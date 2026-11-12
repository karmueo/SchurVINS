#pragma once

#include <array>
#include <string>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <opencv2/core.hpp>
#include <Eigen/Core>

namespace svo_ros {
// Shared by live callbacks and the synchronous MCAP runner. Inputs stay read-only.
class EgoProcessing
{
public:
  explicit EgoProcessing(const std::string& cameras);
  std::array<cv::Mat, 2> decode(const sensor_msgs::msg::CompressedImage& image) const;
  sensor_msgs::msg::Imu correct(const sensor_msgs::msg::Imu& imu) const;
private:
  Eigen::Vector3d acc_bias_, gyro_bias_;
};
}  // namespace svo_ros
