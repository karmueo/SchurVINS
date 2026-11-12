#include <gtest/gtest.h>
#include <svo/direct/depth_filter.h>
#include <svo/direct/feature_detection.h>
#include <svo/common/frame.h>
#include <vikit/cameras/camera_factory.h>
#include <vikit/cameras/ncamera.h>
#include <chrono>
#include <thread>

namespace {
class ObservableFilter : public svo::DepthFilter
{
public:
  using svo::DepthFilter::DepthFilter;
  bool queueEmpty() { ulock_t lock(jobs_mut_); return jobs_.empty(); }
};
template<class Predicate> bool waitUntil(Predicate condition)
{
  const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (condition()) return true;
    std::this_thread::yield();
  }
  return false;
}
}

TEST(DepthFilterConcurrency, FrontendLockAndResetCancelAlreadyDequeuedJob)
{
  Eigen::VectorXd intrinsics(4); intrinsics << 200,200,160,120;
  const auto camera = vk::cameras::factory::makePinholeCamera(intrinsics,320,240);
  auto rig = std::make_shared<svo::CameraBundle>(
      vk::cameras::TransformationVector(1,svo::Transformation()),
      std::vector<svo::CameraPtr>{camera},"concurrency");
  svo::DepthFilterOptions options; options.use_threaded_depthfilter = true;
  svo::DetectorOptions detector; detector.cell_size = 20;
  ObservableFilter filter(options,detector,rig);
  cv::Mat image(240,320,CV_8UC1); cv::RNG rng(42); rng.fill(image,cv::RNG::UNIFORM,0,256);
  auto old_frame = std::make_shared<svo::Frame>(camera,image,1000000000LL,4);
  auto new_frame = std::make_shared<svo::Frame>(camera,image,2000000000LL,4);
  {
    std::lock_guard<std::recursive_mutex> frontend(filter.frame_state_mutex_);
    filter.addKeyframe(old_frame,5,1,10);
    // The worker has removed the job, but cannot touch frame storage until the
    // frontend releases its lock. Reset must invalidate this in-flight job too.
    ASSERT_TRUE(waitUntil([&] { return filter.queueEmpty(); }));
    EXPECT_EQ(old_frame->numFeatures(),0u);
    filter.reset();
    filter.addKeyframe(new_frame,5,1,10);
  }
  ASSERT_TRUE(waitUntil([&] {
    std::lock_guard<std::recursive_mutex> lock(filter.frame_state_mutex_);
    return new_frame->numFeatures() > 0;
  }));
  {
    std::lock_guard<std::recursive_mutex> lock(filter.frame_state_mutex_);
    EXPECT_EQ(old_frame->numFeatures(),0u);
    EXPECT_GT(new_frame->numFeatures(),0u);
  }
  filter.stopThread();
  filter.startThread();
  auto restarted = std::make_shared<svo::Frame>(camera,image,3000000000LL,4);
  filter.addKeyframe(restarted,5,1,10);
  EXPECT_TRUE(waitUntil([&] {
    std::lock_guard<std::recursive_mutex> lock(filter.frame_state_mutex_);
    return restarted->numFeatures() > 0;
  }));
  filter.stopThread();
}
