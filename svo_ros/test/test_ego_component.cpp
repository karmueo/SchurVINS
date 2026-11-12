#include <gtest/gtest.h>
#include <svo_ros/ego_input.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>

TEST(EgoComponent, ActualCallbacksCorrectAndSplitWithoutAccumulation)
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  const auto fixture = std::filesystem::temp_directory_path() /
    ("ego-component-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
  {
    std::ofstream calibration(fixture);
    calibration << "imu:\n  accelerometer_bias: [1.0, 2.0, 3.0]\n"
                   "  gyroscope_bias: [180.0, 90.0, -180.0]\n";
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("cameras", fixture.string())});
  auto adapter = std::make_shared<svo_ros::EgoInput>(options);
  auto source = std::make_shared<rclcpp::Node>("ego_component_fixture");
  auto image_publisher = source->create_publisher<sensor_msgs::msg::CompressedImage>("/stereo/image_raw", rclcpp::QoS(64).reliable());
  auto imu_publisher = source->create_publisher<sensor_msgs::msg::Imu>("/imu", rclcpp::QoS(10000).reliable());
  std::mutex mutex; std::condition_variable changed;
  std::vector<sensor_msgs::msg::Image> left, right;
  std::vector<sensor_msgs::msg::Imu> imus;
  std::string status_snapshot;
  auto status_sub = source->create_subscription<std_msgs::msg::String>("/ego/runtime_status", rclcpp::QoS(1).transient_local(),
    [&](std_msgs::msg::String::ConstSharedPtr msg) { std::lock_guard<std::mutex> lock(mutex); status_snapshot = msg->data; changed.notify_all(); });
  auto left_sub = source->create_subscription<sensor_msgs::msg::Image>("/ego/cam0/image_raw", rclcpp::QoS(64).reliable(),
    [&](sensor_msgs::msg::Image::ConstSharedPtr msg) { std::lock_guard<std::mutex> lock(mutex); left.push_back(*msg); changed.notify_all(); });
  auto right_sub = source->create_subscription<sensor_msgs::msg::Image>("/ego/cam1/image_raw", rclcpp::QoS(64).reliable(),
    [&](sensor_msgs::msg::Image::ConstSharedPtr msg) { std::lock_guard<std::mutex> lock(mutex); right.push_back(*msg); changed.notify_all(); });
  auto imu_sub = source->create_subscription<sensor_msgs::msg::Imu>("/ego/imu", rclcpp::QoS(10000).reliable(),
    [&](sensor_msgs::msg::Imu::ConstSharedPtr msg) { std::lock_guard<std::mutex> lock(mutex); imus.push_back(*msg); changed.notify_all(); });
  sensor_msgs::msg::CompressedImage image;
  image.header.stamp.sec = 10; image.header.stamp.nanosec = 50; image.format = "jpeg";
  cv::Mat stereo(1080, 3840, CV_8UC1, cv::Scalar(40));
  stereo(cv::Rect(1920,0,1920,1080)).setTo(200);
  ASSERT_TRUE(cv::imencode(".jpg", stereo, image.data, {cv::IMWRITE_JPEG_QUALITY, 100}));
  sensor_msgs::msg::Imu imu;
  imu.header = image.header;
  imu.angular_velocity.x = 4.0; imu.angular_velocity.y = 5.0; imu.angular_velocity.z = 6.0;
  imu.linear_acceleration.x = 10.0; imu.linear_acceleration.y = 20.0; imu.linear_acceleration.z = 30.0;
  bool sent = false;
  auto ready = source->create_wall_timer(std::chrono::milliseconds(10), [&] {
    if (!sent && image_publisher->get_subscription_count() && imu_publisher->get_subscription_count() &&
        left_sub->get_publisher_count() && right_sub->get_publisher_count() && imu_sub->get_publisher_count())
    {
      sent = true;
      // Reuse identical inputs to catch mutation and repeated bias/time application.
      for (int i = 0; i < 2; ++i) { image_publisher->publish(image); imu_publisher->publish(imu); }
    }
  });
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(adapter); executor.add_node(source);
  std::thread spin([&] { executor.spin(); });
  bool complete;
  {
    std::unique_lock<std::mutex> lock(mutex);
    complete = changed.wait_for(lock, std::chrono::seconds(15), [&] {
      return left.size() >= 2 && right.size() >= 2 && imus.size() >= 2 &&
             status_snapshot.find("\"decoded_images\":2") != std::string::npos &&
             status_snapshot.find("\"pending_decode_images\":0") != std::string::npos &&
             status_snapshot.find("\"decoder_inflight\":false") != std::string::npos;
    });
  }
  executor.cancel(); spin.join();
  EXPECT_TRUE(complete);
  EXPECT_NE(status_snapshot.find("\"image_queue_overflow\":0"), std::string::npos);
  ASSERT_EQ(left.size(), 2u); ASSERT_EQ(right.size(), 2u); ASSERT_EQ(imus.size(), 2u);
  constexpr double pi = 3.14159265358979323846;
  for (size_t i = 0; i < 2; ++i)
  {
    EXPECT_EQ(rclcpp::Time(imus[i].header.stamp).nanoseconds(), 9993000050LL);
    EXPECT_EQ(imus[i].header.frame_id, "imu");
    EXPECT_NEAR(imus[i].angular_velocity.x, 4.0-pi, 1e-12);
    EXPECT_NEAR(imus[i].angular_velocity.y, 5.0-pi/2, 1e-12);
    EXPECT_NEAR(imus[i].angular_velocity.z, 6.0+pi, 1e-12);
    EXPECT_DOUBLE_EQ(imus[i].linear_acceleration.x, 9.0);
    EXPECT_DOUBLE_EQ(imus[i].linear_acceleration.y, 18.0);
    EXPECT_DOUBLE_EQ(imus[i].linear_acceleration.z, 27.0);
    for (const auto* camera : {&left[i], &right[i]})
    {
      EXPECT_EQ(camera->width, 1920u); EXPECT_EQ(camera->height, 1080u);
      EXPECT_EQ(camera->encoding, "mono8");
      EXPECT_EQ(camera->header.stamp, image.header.stamp);
    }
    EXPECT_EQ(left[i].header.frame_id, "cam0"); EXPECT_EQ(right[i].header.frame_id, "cam1");
    EXPECT_NEAR(left[i].data.at(540*left[i].step+960), 40, 1);
    EXPECT_NEAR(right[i].data.at(540*right[i].step+960), 200, 1);
  }
  EXPECT_DOUBLE_EQ(imu.angular_velocity.x, 4.0);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.x, 10.0);
  EXPECT_EQ(rclcpp::Time(imu.header.stamp).nanoseconds(), 10000000050LL);
  executor.remove_node(adapter); adapter.reset();
  std::filesystem::remove(fixture);
  rclcpp::shutdown();
}
