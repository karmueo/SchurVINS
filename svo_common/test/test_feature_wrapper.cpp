#include <gtest/gtest.h>
#include <svo/common/feature_wrapper.h>

TEST(SvoCommonTestSuite, FeatureWrapperReferencesLiveStorage)
{
  svo::FeatureType type = svo::FeatureType::kCorner;
  svo::Keypoint px(12.0, 25.0);
  svo::BearingVector bearing(0.0, 0.0, 1.0);
  svo::GradientVector gradient(1.0, 0.0);
  svo::Score score = 0.75;
  svo::Level level = 1;
  svo::PointPtr landmark;
  svo::SeedRef seed;
  int track = 7;
  svo::FeatureWrapper feature(type, px, bearing, gradient, score, level,
                              landmark, seed, track);
  EXPECT_EQ(feature.type, type);
  EXPECT_EQ(feature.track_id, 7);
  EXPECT_TRUE(feature.px.isApprox(px));
  EXPECT_TRUE(feature.f.isApprox(bearing));
  feature.px.x() = 99.0;
  feature.type = svo::FeatureType::kEdgelet;
  feature.track_id = 11;
  feature.level = 2;
  EXPECT_DOUBLE_EQ(px.x(), 99.0);
  EXPECT_EQ(type, svo::FeatureType::kEdgelet);
  EXPECT_EQ(track, 11);
  EXPECT_EQ(level, 2);
  auto copied = feature;
  copied.score = 0.5;
  EXPECT_DOUBLE_EQ(score, 0.5);
}
