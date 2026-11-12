#include <gtest/gtest.h>
#include <svo_ros/ego_math.h>
TEST(EgoMath, MillimetersAndExtrinsicDirection)
{
  Eigen::Matrix4d source = Eigen::Matrix4d::Identity();
  source.block<3,3>(0,0) = Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
  source(0,3) = 100; source(1,3) = -20; source(2,3) = 3;
  auto result = svo_ros::egoBodyCamera(source);
  source.block<3,1>(0,3) *= .001;
  EXPECT_LT((source * result - Eigen::Matrix4d::Identity()).norm(), 1e-12);
  EXPECT_NEAR(result.block<3,1>(0,3).norm(), std::sqrt(.01+.0004+.000009), 1e-12);
}
TEST(EgoMath, BiasConversionAndSingleTimeCorrection)
{
  const auto bias = svo_ros::gyroBiasRadians(Eigen::Vector3d(180,90,-180));
  EXPECT_NEAR(bias.x(), 3.141592653589793, 1e-14);
  EXPECT_NEAR(bias.y(), 1.5707963267948966, 1e-14);
  EXPECT_EQ(svo_ros::correctedEgoStamp(1000000000), 993000000);
  // The adapter consumes raw topics; core receives the already corrected stamp.
  const int64_t core_stamp = svo_ros::correctedEgoStamp(1000000000);
  EXPECT_EQ(core_stamp, 993000000);
}
