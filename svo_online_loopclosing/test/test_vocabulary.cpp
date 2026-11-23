#include <gtest/gtest.h>
#include <svo/online_loopclosing/bow.h>
#include <svo/online_loopclosing/loop_closing.h>
#include <vikit/cameras/camera_factory.h>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <cmath>

TEST(LoopVocabulary, MissingResourceIsAnExplicitError)
{
  EXPECT_THROW(svo::loadVoc("/nonexistent/schurvins-vocabulary.yml.gz"), std::runtime_error);
}

TEST(LoopVocabulary, OfficialVocabularyTransformsScoresAndRetrievesRevisitedFrame)
{
  const char* resource = std::getenv("SVO_VOCABULARY");
  if (!resource || !*resource || !std::filesystem::is_regular_file(resource))
    GTEST_SKIP() << "Set SVO_VOCABULARY to the locked official voc_MH01_9X3.yml.gz";
  const auto vocabulary = svo::loadVoc(resource);
  ASSERT_FALSE(vocabulary.empty());
  ASSERT_GT(vocabulary.size(), 1u);
  // Synthetic ORB-shaped binary descriptors are deterministic and independent
  // of any vocabulary training. A revisit keeps 7/8 of the original features.
  cv::Mat original(512,32,CV_8UC1), unrelated(512,32,CV_8UC1);
  cv::RNG random(0x564f4341);
  random.fill(original,cv::RNG::UNIFORM,0,256);
  random.fill(unrelated,cv::RNG::UNIFORM,0,256);
  cv::Mat revisit = original.clone();
  unrelated.rowRange(0,64).copyTo(revisit.rowRange(448,512));
  std::vector<cv::Mat> original_features, unrelated_features, revisit_features;
  svo::changeStructure(original,&original_features);
  svo::changeStructure(unrelated,&unrelated_features);
  svo::changeStructure(revisit,&revisit_features);
  DBoW2::BowVector original_bow, unrelated_bow, revisit_bow;
  std::vector<int> word_ids;
  svo::createBOW(original_features,vocabulary,&original_bow,&word_ids);
  svo::createBOW(unrelated_features,vocabulary,&unrelated_bow,nullptr);
  svo::createBOW(revisit_features,vocabulary,&revisit_bow,nullptr);
  ASSERT_FALSE(original_bow.empty()); ASSERT_FALSE(unrelated_bow.empty()); ASSERT_FALSE(revisit_bow.empty());
  EXPECT_EQ(word_ids.size(),original_features.size());
  for (const int id : word_ids) EXPECT_GE(id,0);
  const double self = svo::compareBOWs(original_bow,original_bow,vocabulary);
  const double matching = svo::compareBOWs(original_bow,revisit_bow,vocabulary);
  const double different = svo::compareBOWs(original_bow,unrelated_bow,vocabulary);
  EXPECT_TRUE(std::isfinite(self)); EXPECT_TRUE(std::isfinite(matching)); EXPECT_TRUE(std::isfinite(different));
  EXPECT_NEAR(self,1.0,1e-8);
  EXPECT_GT(matching,different+.1);
  EXPECT_GT(matching,.5);
  OrbDatabase database(vocabulary,false,0);
  const auto distractor_id = database.add(unrelated_features);
  const auto original_id = database.add(original_features);
  DBoW2::QueryResults results;
  database.query(revisit_features,results,2);
  ASSERT_EQ(results.size(),2u);
  EXPECT_EQ(results[0].Id,original_id);
  EXPECT_EQ(results[1].Id,distractor_id);
  EXPECT_GT(results[0].Score,results[1].Score);
  EXPECT_NEAR(results[0].Score,matching,1e-8);
}

class LoopLifecycle : public testing::Test
{
protected:
  void SetUp() override
  {
    const char* resource = std::getenv("SVO_VOCABULARY");
    if (!resource || !std::filesystem::is_regular_file(resource))
      GTEST_SKIP() << "Set SVO_VOCABULARY to a locked official vocabulary";
    Eigen::VectorXd intrinsics(4);
    intrinsics << 200, 200, 160, 120;
    camera_ = vk::cameras::factory::makePinholeCamera(intrinsics, 320, 240);
    vk::cameras::TransformationVector extrinsics;
    extrinsics.emplace_back(svo::Quaternion(), Eigen::Vector3d::Zero());
    rig_ = std::make_shared<svo::CameraBundle>(
        extrinsics, std::vector<svo::CameraPtr>{camera_}, "loop_lifecycle");
    const std::filesystem::path vocabulary(resource);
    options_.voc_path = vocabulary.parent_path().string();
    options_.voc_name = vocabulary.filename().string();
    options_.scale_ret_app = "None";
    options_.global_map_type = "BuiltInPoseGraph";
  }

  svo::CameraPtr camera_;
  svo::CameraBundle::Ptr rig_;
  svo::LoopClosureOptions options_{};
};

TEST_F(LoopLifecycle, FinishingWithoutSubmittedFramesIsSafeAndRepeatable)
{
  svo::LoopClosing loop(options_, rig_);
  EXPECT_TRUE(loop.lastFinished());
  EXPECT_TRUE(loop.finishPending(0.0));
  EXPECT_TRUE(loop.finishPending(0.1));
  EXPECT_EQ(loop.corrections_applied_, 0u);
  EXPECT_EQ(loop.pgo_completed_, 0u);
}

TEST_F(LoopLifecycle, DrainingSubmitsBothConsecutiveLoopConstraints)
{
  options_.ignored_past_frames = 4;
  svo::LoopClosing loop(options_, rig_);
  cv::Mat image(240, 320, CV_8UC1);
  cv::RNG(42).fill(image, cv::RNG::UNIFORM, 0, 256);
  std::vector<svo::FramePtr> frames;
  for (int i = 0; i < 4; ++i)
  {
    auto frame = std::make_shared<svo::Frame>(camera_, image, (i + 1) * 100000000LL, 1);
    frame->T_f_w_ = svo::Transformation(svo::Quaternion(), Eigen::Vector3d(-i, 0, 0));
    frame->accumulated_w_T_correction_ =
        svo::Transformation(svo::Quaternion(), Eigen::Vector3d(0.1 * i, 0, 0));
    frames.push_back(frame);
    loop.addFrameToPR(std::make_shared<svo::FrameBundle>(std::vector<svo::FramePtr>{frame}));
    ASSERT_TRUE(loop.finishPending(5.0));
  }

  ASSERT_EQ(loop.kf_list_.size(), frames.size());
  loop.cur_kf_to_lc_kf_bundle_id_map_[frames[2]->bundleId()] = frames[0]->bundleId();
  loop.cur_kf_to_lc_kf_bundle_id_map_[frames[3]->bundleId()] = frames[0]->bundleId();
  for (const auto& frame : frames)
    loop.updateKeyframe(frame);

  ASSERT_TRUE(loop.finishPending(5.0));
  EXPECT_EQ(loop.pgo_completed_, 2u);
  EXPECT_EQ(loop.pgo_->poses_->size(), frames.size());
  EXPECT_TRUE(loop.finishPending(0.0));
  EXPECT_EQ(loop.pgo_completed_, 2u);
}
