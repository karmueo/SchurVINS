#include <gtest/gtest.h>
#include <svo_ros/input_scheduler.h>
#include <memory>
namespace {
using Ptr = std::shared_ptr<int>;
using Queue = svo_ros::InputScheduler<Ptr, Ptr>;
Ptr sample() { return std::make_shared<int>(1); }
TEST(InputScheduler, ExactPairAndCoverage)
{
  Queue q(2, true); Queue::Bundle b; bool leading;
  q.image(0, 20, sample()); q.image(1, 21, sample());
  q.imu(10, sample()); q.imu(20, sample()); Ptr imu;
  EXPECT_FALSE(q.ready()); q.takeImu(imu); q.takeImu(imu);
  EXPECT_FALSE(q.ready()); q.image(1, 20, sample());
  // Per-camera rollback is a hard failure rather than approximate pairing.
  EXPECT_TRUE(q.stopped()); EXPECT_EQ(q.counters.time_jumps, 1u);
}
TEST(InputScheduler, WaitPauseAndLeadingFrames)
{
  Queue q(2, true); Queue::Bundle b; bool leading; Ptr imu;
  q.image(0, 10, sample()); q.image(1, 10, sample());
  EXPECT_FALSE(q.takeImage(b, leading));
  EXPECT_EQ(q.pendingImages(), 1u); // An arbitrary replay pause preserves the bundle.
  q.imu(15, sample()); q.takeImu(imu);
  ASSERT_TRUE(q.takeImage(b, leading)); EXPECT_TRUE(leading);
  q.image(0, 20, sample()); q.image(1, 20, sample());
  EXPECT_FALSE(q.takeImage(b, leading));
  q.imu(25, sample()); q.takeImu(imu);
  ASSERT_TRUE(q.takeImage(b, leading)); EXPECT_FALSE(leading); EXPECT_EQ(b.stamp, 20);
}
TEST(InputScheduler, OverflowIsVisibleAndFatal)
{
  Queue q(1, false, 1, 1); q.image(0, 10, sample());
  EXPECT_FALSE(q.image(0, 20, sample())); EXPECT_TRUE(q.stopped()); EXPECT_EQ(q.counters.overflow, 1u);
  Queue imu(1, true, 1, 1); imu.imu(10, sample()); EXPECT_FALSE(imu.imu(20, sample())); EXPECT_EQ(imu.counters.overflow, 1u);
}
TEST(InputScheduler, CompleteReconstructionClearsCoverage)
{
  auto q = std::make_unique<Queue>(4, true); Ptr imu;
  q->imu(100, sample()); q->takeImu(imu, true);
  q = std::make_unique<Queue>(4, true);
  for (size_t camera = 0; camera < 4; ++camera) q->image(camera, 200, sample());
  EXPECT_FALSE(q->ready()); EXPECT_EQ(q->counters.imu_received, 0u);
}
TEST(InputScheduler, FourCameraVisualPipeline)
{
  Queue q(4, false); Queue::Bundle b; bool leading;
  for (size_t camera = 0; camera < 4; ++camera) q.image(camera, 100, sample());
  ASSERT_TRUE(q.takeImage(b, leading)); EXPECT_EQ(b.images.size(), 4u); EXPECT_FALSE(leading);
}
TEST(InputScheduler, ImuRollbackStopsRun)
{
  Queue q(2, true); q.imu(20, sample()); EXPECT_FALSE(q.imu(19, sample()));
  EXPECT_EQ(q.counters.time_jumps, 1u); EXPECT_TRUE(q.stopped());
}
} // namespace

TEST(InputScheduler, LateImagesRetainTheNeededImuHistory)
{
  Queue q(2, true); Ptr imu; Queue::Bundle bundle; bool leading;
  for (int stamp = 1; stamp <= 100; ++stamp) q.imu(stamp, sample());
  EXPECT_FALSE(q.takeImu(imu)); // No camera time is known yet.
  q.image(0, 20, sample());
  for (int i = 0; i < 20; ++i) ASSERT_TRUE(q.takeImu(imu));
  EXPECT_FALSE(q.takeImu(imu)); // Wait for delayed right image, keep future IMU queued.
  EXPECT_EQ(q.pendingImu(), 80u);
  q.image(1, 20, sample()); ASSERT_TRUE(q.takeImage(bundle, leading));
  EXPECT_EQ(bundle.stamp, 20); EXPECT_FALSE(leading);
  EXPECT_FALSE(q.takeImu(imu));
  q.image(0, 40, sample()); q.image(1, 40, sample());
  for (int i = 0; i < 20; ++i) ASSERT_TRUE(q.takeImu(imu));
  EXPECT_FALSE(q.takeImu(imu)); ASSERT_TRUE(q.takeImage(bundle, leading));
  EXPECT_EQ(q.pendingImu(), 60u);
  while (q.takeImu(imu, true)) {} // End-of-input drain is explicit.
  EXPECT_EQ(q.pendingImu(), 0u); EXPECT_EQ(q.counters.imu_consumed, 100u);
}
