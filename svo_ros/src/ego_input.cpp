#include <svo_ros/ego_input.h>
#include <svo_ros/ego_math.h>
#include <vikit/params_helper.h>
#include <yaml-cpp/yaml.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath>
namespace svo_ros {
EgoInput::EgoInput(const rclcpp::NodeOptions& options) : Node("ego_input", options)
{
  const auto file = vk::param<std::string>(*this, "source_calibration", "");
  if (file.empty()) throw std::invalid_argument("source_calibration is required");
  const auto calibration = YAML::LoadFile(file);
  for (int i = 0; i < 3; ++i)
  {
    acc_bias_[i] = calibration["imu"]["accelerometer_bias"][i].as<double>();
    gyro_bias_[i] = calibration["imu"]["gyroscope_bias"][i].as<double>();
  }
  gyro_bias_ = gyroBiasRadians(gyro_bias_);
  image_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  imu_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions image_options, imu_options;
  image_options.callback_group = image_group_; imu_options.callback_group = imu_group_;
  left_ = create_publisher<sensor_msgs::msg::Image>("/ego/cam0/image_raw", rclcpp::QoS(64).reliable());
  right_ = create_publisher<sensor_msgs::msg::Image>("/ego/cam1/image_raw", rclcpp::QoS(64).reliable());
  corrected_imu_ = create_publisher<sensor_msgs::msg::Imu>("/ego/imu", rclcpp::QoS(10000).reliable());
  image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>("/stereo/image_raw", rclcpp::QoS(64).reliable(),
    [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) { image(std::move(msg)); }, image_options);
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu", rclcpp::QoS(10000).reliable(),
    [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { imu(std::move(msg)); }, imu_options);
  status_ = create_publisher<std_msgs::msg::String>("/ego/runtime_status", rclcpp::QoS(1).transient_local());
  timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {
    std_msgs::msg::String msg;
    msg.data = "{\"raw_images\":" + std::to_string(received_images_) +
      ",\"decoded_images\":" + std::to_string(decoded_images_) + ",\"invalid_images\":" + std::to_string(invalid_images_) +
      ",\"raw_imu\":" + std::to_string(received_imu_) + ",\"published_imu\":" + std::to_string(published_imu_) +
      ",\"matched_image_publishers\":" + std::to_string(image_sub_->get_publisher_count()) +
      ",\"matched_imu_publishers\":" + std::to_string(imu_sub_->get_publisher_count()) +
      ",\"matched_left_subscribers\":" + std::to_string(left_->get_subscription_count()) +
      ",\"matched_right_subscribers\":" + std::to_string(right_->get_subscription_count()) +
      ",\"matched_imu_subscribers\":" + std::to_string(corrected_imu_->get_subscription_count()) + "}";
    status_->publish(msg);
  });
}
void EgoInput::image(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  ++received_images_;
  try
  {
    if (msg->format.find("jpeg") == std::string::npos && msg->format.find("jpg") == std::string::npos)
      throw std::invalid_argument("Expected JPEG compressed image");
    const cv::Mat gray = cv::imdecode(msg->data, cv::IMREAD_GRAYSCALE);
    if (gray.cols != 3840 || gray.rows != 1080 || gray.type() != CV_8UC1)
      throw std::invalid_argument("Expected decoded 3840x1080 JPEG");
    auto header = msg->header;
    header.frame_id = "cam0";
    auto left = cv_bridge::CvImage(header, "mono8", gray(cv::Rect(0,0,1920,1080))).toImageMsg();
    header.frame_id = "cam1";
    auto right = cv_bridge::CvImage(header, "mono8", gray(cv::Rect(1920,0,1920,1080))).toImageMsg();
    left_->publish(*left); right_->publish(*right); ++decoded_images_;
  }
  catch (const std::exception& error)
  {
    ++invalid_images_; RCLCPP_ERROR(get_logger(), "EGO decode failed: %s", error.what());
  }
}
void EgoInput::imu(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  ++received_imu_;
  auto corrected = *msg;
  const auto stamp = correctedEgoStamp(rclcpp::Time(msg->header.stamp).nanoseconds());
  if (stamp < 0) { RCLCPP_ERROR(get_logger(), "Negative corrected EGO timestamp"); return; }
  corrected.header.stamp = rclcpp::Time(stamp);
  corrected.header.frame_id = "imu";
  corrected.angular_velocity.x -= gyro_bias_.x();
  corrected.angular_velocity.y -= gyro_bias_.y();
  corrected.angular_velocity.z -= gyro_bias_.z();
  corrected.linear_acceleration.x -= acc_bias_.x();
  corrected.linear_acceleration.y -= acc_bias_.y();
  corrected.linear_acceleration.z -= acc_bias_.z();
  if (!std::isfinite(corrected.angular_velocity.x + corrected.angular_velocity.y + corrected.angular_velocity.z +
                     corrected.linear_acceleration.x + corrected.linear_acceleration.y + corrected.linear_acceleration.z))
    return;
  corrected_imu_->publish(corrected); ++published_imu_;
}
} // namespace svo_ros
RCLCPP_COMPONENTS_REGISTER_NODE(svo_ros::EgoInput)
