#include <svo_ros/ego_processing.h>
#include <svo_ros/ego_math.h>
#include <rclcpp/time.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <stdexcept>

namespace svo_ros {
EgoProcessing::EgoProcessing(const std::string& cameras)
{
  if (cameras.empty()) throw std::invalid_argument("cameras is required");
  const auto calibration = YAML::LoadFile(cameras);
  for (int i = 0; i < 3; ++i)
  {
    acc_bias_[i] = calibration["imu"]["accelerometer_bias"][i].as<double>();
    gyro_bias_[i] = calibration["imu"]["gyroscope_bias"][i].as<double>();
  }
  gyro_bias_ = gyroBiasRadians(gyro_bias_);
  if (!acc_bias_.allFinite() || !gyro_bias_.allFinite())
    throw std::invalid_argument("Non-finite EGO IMU bias");
}

std::array<cv::Mat, 2> EgoProcessing::decode(const sensor_msgs::msg::CompressedImage& image) const
{
  if (image.format.find("jpeg") == std::string::npos && image.format.find("jpg") == std::string::npos)
    throw std::invalid_argument("Expected JPEG compressed image");
  const cv::Mat gray = cv::imdecode(image.data, cv::IMREAD_GRAYSCALE);
  if (gray.cols != 3840 || gray.rows != 1080 || gray.type() != CV_8UC1)
    throw std::invalid_argument("Expected decoded 3840x1080 JPEG");
  return {gray(cv::Rect(0, 0, 1920, 1080)), gray(cv::Rect(1920, 0, 1920, 1080))};
}

sensor_msgs::msg::Imu EgoProcessing::correct(const sensor_msgs::msg::Imu& imu) const
{
  auto corrected = imu;
  const auto stamp = correctedEgoStamp(rclcpp::Time(imu.header.stamp).nanoseconds());
  if (stamp < 0) throw std::invalid_argument("Negative corrected EGO timestamp");
  corrected.header.stamp = rclcpp::Time(stamp);
  corrected.header.frame_id = "imu";
  corrected.angular_velocity.x -= gyro_bias_.x();
  corrected.angular_velocity.y -= gyro_bias_.y();
  corrected.angular_velocity.z -= gyro_bias_.z();
  corrected.linear_acceleration.x -= acc_bias_.x();
  corrected.linear_acceleration.y -= acc_bias_.y();
  corrected.linear_acceleration.z -= acc_bias_.z();
  const auto& a = corrected.linear_acceleration;
  const auto& w = corrected.angular_velocity;
  if (!(std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z) &&
        std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.z)))
    throw std::invalid_argument("Non-finite EGO IMU sample");
  return corrected;
}
}  // namespace svo_ros
