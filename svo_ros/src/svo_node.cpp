#include <svo_ros/svo_node_base.h>
#include <glog/logging.h>
int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<svo_ros::SvoNodeBase>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  auto exit_timer = node->create_wall_timer(std::chrono::milliseconds(100), [&] { if (node->stopped()) executor.cancel(); });
  executor.spin();
  exit_timer.reset();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
