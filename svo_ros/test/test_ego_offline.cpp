#include <gtest/gtest.h>
#include <svo_ros/offline_timing.h>
#include <svo_ros/visualizer.h>
#include <filesystem>
#include <fstream>
#include <thread>

TEST(EgoReplaySchedule, FastModeAndRealtimeUseRecordingTime)
{
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  svo_ros::ReplaySchedule fast(false), realtime(true);
  EXPECT_EQ(fast.target(1000000000, start), start);
  EXPECT_EQ(fast.target(9000000000, start), start);
  EXPECT_EQ(realtime.target(1000000000, start), start);
  EXPECT_EQ(realtime.target(1500000000, start), start + std::chrono::milliseconds(500));
  // Falling behind does not shift the schedule or discard an input frame.
  EXPECT_EQ(realtime.target(2000000000, start + std::chrono::seconds(3)), start + std::chrono::seconds(1));
}

TEST(EgoComputationTimer, ExcludesWorkOutsideMeasuredAlgorithmCalls)
{
  svo_ros::ComputationTimer timer;
  EXPECT_DOUBLE_EQ(timer.fps(0), 0.0);
  EXPECT_EQ(timer.measure([] { return 42; }), 42);
  timer.measure([] { std::this_thread::sleep_for(std::chrono::milliseconds(5)); });
  const auto seconds = timer.seconds();
  std::this_thread::sleep_for(std::chrono::milliseconds(20)); // IO, decode, publish or replay wait.
  EXPECT_DOUBLE_EQ(timer.seconds(), seconds);
  EXPECT_GE(seconds, .005);
  EXPECT_DOUBLE_EQ(timer.fps(10), 10/seconds);
  EXPECT_THROW(timer.measure([] { throw std::runtime_error("algorithm failed"); }), std::runtime_error);
  EXPECT_GE(timer.seconds(), seconds);
}

TEST(EgoTrajectoryOutput, SinkOdomPathAndOptionalTumShareBodyPose)
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  const auto output = std::filesystem::temp_directory_path() /
    ("ego-trajectory-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directory(output);
  const auto trajectory = output / "schurvins_fixture_trajectory.txt";
  for (const bool save : {false, true})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("save_trajectory", save),
      rclcpp::Parameter("trajectory_file", trajectory.string())});
    auto node = std::make_shared<rclcpp::Node>("trajectory_output_test", options);
    {
      svo::Visualizer visualizer(output.string(), *node, 2);
      size_t calls = 0;
      visualizer.pose_sink_ = [&](const auto& odom, const auto& path) {
        ++calls;
        EXPECT_EQ(odom.header.frame_id, "world");
        EXPECT_EQ(odom.child_frame_id, "imu");
        EXPECT_EQ(odom.pose.pose, path.poses.back().pose);
        EXPECT_EQ(odom.header, path.header);
        EXPECT_EQ(path.poses.size(), calls);
      };
      visualizer.publishImuPose(svo::Transformation(), Eigen::Matrix<double,6,6>::Zero(), 1793850028412198799LL);
      visualizer.publishImuPose(svo::Transformation(), Eigen::Matrix<double,6,6>::Zero(), 1793850028428865465LL);
      EXPECT_EQ(calls, 2u);
    }
    EXPECT_EQ(std::filesystem::exists(trajectory), save);
  }
  std::ifstream stream(trajectory);
  std::string timestamp;
  double value;
  ASSERT_TRUE(stream >> timestamp);
  EXPECT_EQ(timestamp, "1793850028.412198799");
  for (int i = 0; i < 7; ++i)
  {
    ASSERT_TRUE(stream >> value);
    EXPECT_DOUBLE_EQ(value, i == 6 ? 1.0 : 0.0);
  }
  std::filesystem::remove_all(output);
  rclcpp::shutdown();
}
