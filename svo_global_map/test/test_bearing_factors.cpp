#include <gtest/gtest.h>
#include <svo/gtsam/gtsam_optimizer.h>
#include <svo/gtsam/graph_manager.h>
#include <gtsam/base/numericalDerivative.h>
#include <svo/gtsam/camera_bearing_factor.h>
#include <svo/gtsam/camera_bearing_extrinsics_factor.h>

TEST(BearingFactors, BodyAndLandmarkJacobiansAndIndependentRequests)
{
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.1, -0.2, 0.3),
                          gtsam::Point3(1, -2, 0.5));
  const gtsam::Pose3 extrinsic(gtsam::Rot3::RzRyRx(-0.1, 0.05, 0.2),
                               gtsam::Point3(0.2, -0.05, 0.1));
  const gtsam::Point3 point(2, 1, 6);
  const auto measurement = pose.compose(extrinsic).transformTo(point).normalized().eval();
  gtsam::CameraBearingFactor<gtsam::Pose3, gtsam::Point3> factor(
      measurement, gtsam::noiseModel::Isotropic::Sigma(3, 1), 0, 1, extrinsic);
  gtsam::Matrix Hp, Hl;
  EXPECT_LT(factor.evaluateError(pose, point, Hp, Hl).norm(), 1e-12);
  const std::function<gtsam::Vector3(const gtsam::Pose3&, const gtsam::Point3&)> error =
      [&factor](const auto& p, const auto& l) { return factor.evaluateError(p, l); };
  EXPECT_LT((Hp - gtsam::numericalDerivative21(error, pose, point)).norm(), 1e-7);
  EXPECT_LT((Hl - gtsam::numericalDerivative22(error, pose, point)).norm(), 1e-7);
  gtsam::Matrix landmark_only, pose_only;
  factor.evaluateError(pose, point, boost::none, landmark_only);
  factor.evaluateError(pose, point, pose_only, boost::none);
  EXPECT_LT((landmark_only - Hl).norm(), 1e-12);
  EXPECT_LT((pose_only - Hp).norm(), 1e-12);
}

TEST(BearingFactors, ExtrinsicJacobianAndIndependentRequests)
{
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.1, -0.2, 0.3),
                          gtsam::Point3(1, -2, 0.5));
  const gtsam::Pose3 extrinsic(gtsam::Rot3::RzRyRx(-0.1, 0.05, 0.2),
                               gtsam::Point3(0.2, -0.05, 0.1));
  const gtsam::Point3 point(2, 1, 6);
  const auto measurement = pose.compose(extrinsic).transformTo(point).normalized().eval();
  gtsam::CameraBearingExtrinsicsFactor<gtsam::Pose3, gtsam::Point3> factor(
      measurement, gtsam::noiseModel::Isotropic::Sigma(3, 1), 0, 1, 2);
  gtsam::Matrix Hp, He, Hl;
  EXPECT_LT(factor.evaluateError(pose, extrinsic, point, Hp, He, Hl).norm(), 1e-12);
  const std::function<gtsam::Vector3(const gtsam::Pose3&, const gtsam::Pose3&, const gtsam::Point3&)> error =
      [&factor](const auto& p, const auto& e, const auto& l) { return factor.evaluateError(p, e, l); };
  EXPECT_LT((Hp - gtsam::numericalDerivative31(error, pose, extrinsic, point)).norm(), 1e-7);
  EXPECT_LT((He - gtsam::numericalDerivative32(error, pose, extrinsic, point)).norm(), 1e-7);
  EXPECT_LT((Hl - gtsam::numericalDerivative33(error, pose, extrinsic, point)).norm(), 1e-7);
  gtsam::Matrix extrinsic_only, landmark_only;
  factor.evaluateError(pose, extrinsic, point, boost::none, extrinsic_only, boost::none);
  factor.evaluateError(pose, extrinsic, point, boost::none, boost::none, landmark_only);
  EXPECT_LT((extrinsic_only - He).norm(), 1e-12);
  EXPECT_LT((landmark_only - Hl).norm(), 1e-12);
}

TEST(GlobalMapOptimizer, StartsAndJoinsRepeatedlyWithoutLostShutdown)
{
  auto graph = std::make_shared<svo::GraphManager>(svo::GraphManagerOptions());
  svo::GTSAMOptimizer optimizer(svo::GTSAMOptimizerOptions(), graph);
  optimizer.initialize();
  for (int i = 0; i < 20; ++i)
  {
    optimizer.startThread();
    optimizer.optimize();
    optimizer.quitThread();
    EXPECT_FALSE(optimizer.thread_);
  }
}
