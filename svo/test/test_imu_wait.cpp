#include <gtest/gtest.h>
#include <svo/imu_handler.h>
#include <svo/common/conversions.h>
#include <chrono>
#include <cmath>
#include <future>
#include <thread>

TEST(ImuCoverage, EmptyAndStaleInputHaveABoundedTimeout)
{
  svo::ImuHandler handler(svo::ImuCalibration{}, svo::ImuInitialization{}, svo::IMUHandlerOptions{});
  EXPECT_FALSE(std::isfinite(handler.getLatestTimestamp()));
  auto started = std::chrono::steady_clock::now();
  EXPECT_FALSE(handler.waitTill(2.0, .02));
  EXPECT_LT(std::chrono::steady_clock::now()-started, std::chrono::seconds(1));
  handler.addImuMeasurement(svo::ImuMeasurement(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  started = std::chrono::steady_clock::now();
  EXPECT_FALSE(handler.waitTill(2.0, .02));
  EXPECT_LT(std::chrono::steady_clock::now()-started, std::chrono::seconds(1));
}

TEST(ImuCoverage, ArrivalWakesWaiterAndExactEpochBoundaryIsCovered)
{
  svo::ImuHandler handler(svo::ImuCalibration{}, svo::ImuInitialization{}, svo::IMUHandlerOptions{});
  const int64_t timestamp_ns = 1403715271812143000LL;
  const double timestamp = static_cast<double>(timestamp_ns) * svo::common::conversions::kNanoSecondsToSeconds;
  auto result = std::async(std::launch::async, [&] { return handler.waitTill(timestamp, .5); });
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  handler.addImuMeasurement(svo::ImuMeasurement(timestamp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  EXPECT_EQ(result.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_TRUE(result.get());
  EXPECT_TRUE(handler.waitTill(timestamp, 0.0));
}
