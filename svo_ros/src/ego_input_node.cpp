#include <svo_ros/ego_input.h>
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<svo_ros::EgoInput>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node); executor.spin();
  rclcpp::shutdown(); return 0;
}
