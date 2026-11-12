#include <gtest/gtest.h>
#include <svo/pgo/pgo.h>

TEST(PoseGraph, SolvesMetricTranslationAndQuaternionConstraint)
{
  svo::Pgo graph;
  const svo::Transformation origin;
  const svo::Transformation initial(svo::Quaternion(), Eigen::Vector3d(1.3, 0.2, -0.1));
  const svo::Quaternion rotation(Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())));
  const svo::Transformation expected(rotation, Eigen::Vector3d(1, 0, 0));
  graph.addPoseToPgoProblem(origin, 0);
  graph.addPoseToPgoProblem(initial, 1);
  auto information = Eigen::Matrix<double, 6, 6>::Identity().eval();
  graph.addConstraint(expected, 0, 1, information);
  graph.solve();
  const auto& result = graph.poses_->at(1);
  EXPECT_LT((result.p - expected.getPosition()).norm(), 1e-6);
  EXPECT_NEAR(std::abs(result.q.dot(expected.getRotation().toImplementation())), 1.0, 1e-8);
}
