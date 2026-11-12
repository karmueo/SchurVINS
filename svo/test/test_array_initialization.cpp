#include <gtest/gtest.h>
#include <svo/frame_handler_array.h>
#include <svo/initialization.h>
#include <svo/direct/depth_filter.h>
#include <svo/direct/feature_detection.h>
#include <svo/reprojector.h>
#include <svo/tracker/feature_tracker.h>
#include <vikit/cameras/camera_factory.h>
#include <vikit/cameras/ncamera.h>

namespace {
class WaitingInitializer : public svo::AbstractInitialization
{
public:
  explicit WaitingInitializer(const svo::CameraBundle::Ptr& cameras)
    : AbstractInitialization(svo::InitializationOptions(), svo::FeatureTrackerOptions(),
                             svo::DetectorOptions(), cameras) {}
  svo::InitResult addFrameBundle(const svo::FrameBundlePtr&) override
  {
    ++calls;
    return svo::InitResult::kTracking;
  }
  int calls = 0;
};

class ArrayHandler : public svo::FrameHandlerArray
{
public:
  explicit ArrayHandler(const svo::CameraBundle::Ptr& cameras)
    : FrameHandlerArray(svo::BaseOptions(), svo::DepthFilterOptions(), svo::DetectorOptions(),
                        svo::InitializationOptions(), svo::ReprojectorOptions(),
                        svo::FeatureTrackerOptions(), cameras) {}
  svo::UpdateResult processInitialization()
  {
    stage_ = svo::Stage::kInitializing;
    return processFrameBundle();
  }
};
}

TEST(ArrayInitialization, WaitsForGeometryWithoutCreatingAFalseKeyframe)
{
  Eigen::VectorXd intrinsics(4);
  intrinsics << 200, 200, 160, 120;
  std::vector<svo::CameraPtr> cameras;
  vk::cameras::TransformationVector extrinsics;
  for (int i = 0; i < 4; ++i)
  {
    cameras.push_back(vk::cameras::factory::makePinholeCamera(intrinsics, 320, 240));
    extrinsics.emplace_back(svo::Quaternion(), Eigen::Vector3d(-0.1 * i, 0, 0));
  }
  auto rig = std::make_shared<svo::CameraBundle>(extrinsics, cameras, "four_camera_fixture");
  ArrayHandler handler(rig);
  auto initializer = std::make_unique<WaitingInitializer>(rig);
  const auto* observed = initializer.get();
  handler.initializer_ = std::move(initializer);
  EXPECT_EQ(handler.processInitialization(), svo::UpdateResult::kDefault);
  EXPECT_EQ(observed->calls, 1);
  EXPECT_EQ(handler.stage(), svo::Stage::kInitializing);
  EXPECT_FALSE(observed->frames_ref_);
}
