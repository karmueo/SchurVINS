#include <gtest/gtest.h>
#include <svo_ros/svo_node_base.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <filesystem>
#include <future>
#include <thread>

TEST(RuntimeLifecycle, ReadOnlyResetAndComponentExit)
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  const auto output = std::filesystem::temp_directory_path() /
    ("svo-lifecycle-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("calib_file", ament_index_cpp::get_package_share_directory("svo_ros") + "/param/calib/euroc_stereo.yaml"),
    rclcpp::Parameter("output_dir", output.string()), rclcpp::Parameter("use_imu", true)});
  auto component = std::make_shared<svo_ros::SvoNodeBase>(options);
  EXPECT_FALSE(component->set_parameter(rclcpp::Parameter("use_imu", false)).successful);
  auto observer = std::make_shared<rclcpp::Node>("runtime_lifecycle_observer");
  std::promise<void> reset_seen, stop_seen; bool delivered = false, stopped = false;
  auto subscription = observer->create_subscription<std_msgs::msg::String>("/svo/runtime_status", rclcpp::QoS(1).transient_local(),
    [&](std_msgs::msg::String::ConstSharedPtr msg) {
      if (!delivered && msg->data.find("\"resets\":1") != std::string::npos)
      { delivered = true; reset_seen.set_value(); }
      if (!stopped && msg->data.find("\"stopped\":true") != std::string::npos)
      { stopped = true; stop_seen.set_value(); }
    });
  auto control = observer->create_publisher<std_msgs::msg::String>("/svo/remote_key", 10);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(component); executor.add_node(observer);
  bool reset_sent = false;
  auto ready = observer->create_wall_timer(std::chrono::milliseconds(10), [&] {
    if (!reset_sent && control->get_subscription_count())
    { reset_sent = true; std_msgs::msg::String message; message.data = "r"; control->publish(message); }
  });
  std::thread spin([&] { executor.spin(); });
  std_msgs::msg::String command;
  const bool reset = reset_seen.get_future().wait_for(std::chrono::seconds(10)) == std::future_status::ready;
  EXPECT_TRUE(reset);
  command.data = "q"; control->publish(command);
  EXPECT_EQ(stop_seen.get_future().wait_for(std::chrono::seconds(10)), std::future_status::ready);
  EXPECT_TRUE(component->stopped());
  EXPECT_TRUE(rclcpp::ok()); // q must leave the component container alive.
  executor.cancel(); spin.join(); executor.remove_node(component); component.reset();
  size_t summaries = 0;
  for (const auto& entry : std::filesystem::directory_iterator(output))
    if (std::filesystem::exists(entry.path()/"summary.json")) ++summaries;
  EXPECT_EQ(summaries, 2u);
  std::filesystem::remove_all(output);
  rclcpp::shutdown();
}
