// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <vikit/eigen_checks.h>
#include <gtest/gtest.h>

#include <aslam/common/entrypoint.h>
#include <svo/vio_common/test_utils.hpp>
#include <svo/common/transformation.h>
#include <svo/common/types.h>

#include "svo/ceres_backend/pose_local_parameterization.hpp"

TEST(poseLocalParametrizationTests, PlusTest)
{
  Eigen::Matrix<double, 7,1> x =
      svo::test_utils::randomVectorNormalDistributed<7>(true, 0.0, 10.0);
  x.tail<4>().normalize();
  Eigen::Matrix<svo::FloatType, 6, 1> dx =
      svo::test_utils::randomVectorNormalDistributed<6>(true, 0.0, 0.1);
  Eigen::Matrix<double, 7,1> x_dist;
  svo::ceres_backend::PoseLocalParameterization::plus(x.data(), dx.data(), x_dist.data());
  svo::Transformation T(
        Eigen::Vector3d(x[0], x[1], x[2]),
      Eigen::Quaterniond(x[6], x[3], x[4], x[5]));
  svo::Transformation T_dist(
        Eigen::Vector3d(x_dist[0], x_dist[1], x_dist[2]),
      Eigen::Quaterniond(x_dist[6], x_dist[3], x_dist[4], x_dist[5]));
  svo::Quaternion q_dist_with_ze =
      svo::Quaternion::exp(dx.tail<3>()) * T.getRotation();
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(T_dist.getRotationMatrix(),
                                        q_dist_with_ze.getRotationMatrix()));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(T_dist.getPosition(),
                                        T.getPosition() + dx.head<3>()));
}

TEST(poseLocalParametrizationTests, MinusTest)
{
  //! @todo this test fails. check minus implementation...
  Eigen::Matrix<double, 7,1> x =
      svo::test_utils::randomVectorNormalDistributed<7>(true, 0.0, 10.0);
  x.tail<4>().normalize();
  Eigen::Matrix<svo::FloatType, 6, 1> dx =
      svo::test_utils::randomVectorNormalDistributed<6>(true, 0.0, 0.1);
  Eigen::Matrix<double, 7,1> x_dist;
  svo::ceres_backend::PoseLocalParameterization::plus(x.data(), dx.data(), x_dist.data());

  Eigen::Matrix<svo::FloatType, 6, 1> dx_from_minus;
  svo::ceres_backend::PoseLocalParameterization::minus(x.data(), x_dist.data(),
                                              dx_from_minus.data());
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(dx, dx_from_minus));
}

TEST(poseLocalParametrizationTests, PlusJacobianTest)
{
  Eigen::Matrix<double, 7,1> x =
      svo::test_utils::randomVectorNormalDistributed<7>(false, 0.0, 10.0);
  x.tail<4>().normalize();
  Eigen::Matrix<svo::FloatType, 6, 1> dx =
      Eigen::Matrix<svo::FloatType, 6, 1>::Zero();
  Eigen::Matrix<double, 7, 6, Eigen::RowMajor> jacobian_analytic;
  svo::ceres_backend::PoseLocalParameterization::plusJacobian(
        x.data(), jacobian_analytic.data());

  const double delta = 1e-3;
  Eigen::Matrix<double, 7,1> result_plus, result_minus;
  Eigen::Matrix<double, 7, 6> jacobian_numeric;
  for (size_t i = 0; i < 6; ++i)
  {
    dx[i] = delta;
    svo::ceres_backend::PoseLocalParameterization::plus(
          x.data(), dx.data(), result_plus.data());
    dx[i] = -delta;
    svo::ceres_backend::PoseLocalParameterization::plus(
          x.data(), dx.data(), result_minus.data());
    jacobian_numeric.col(i) = (result_plus - result_minus) * 1.0 / (2.0 * delta);
  }
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(jacobian_analytic,
                                jacobian_numeric, 1e-7));

}

TEST(poseLocalParametrizationTests, LiftJacobianTest)
{
  Eigen::Matrix<double, 7, 1> x =
      svo::test_utils::randomVectorNormalDistributed<7>(false, 0.0, 10.0);
  x.tail<4>().normalize();

  Eigen::Matrix<double, 7, 6, Eigen::RowMajor> plus_jacobian;
  Eigen::Matrix<double, 6, 7, Eigen::RowMajor> lift_jacobian;

  svo::ceres_backend::PoseLocalParameterization::plusJacobian(x.data(),
                                                     plus_jacobian.data());
  svo::ceres_backend::PoseLocalParameterization::liftJacobian(x.data(),
                                                     lift_jacobian.data());
  Eigen::Matrix<double, 6, 6> identity;
  identity.setIdentity();
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(lift_jacobian * plus_jacobian, identity));
}

VIKIT_UNITTEST_ENTRYPOINT

// Exercise the Ceres 2 Manifold interface, whose Minus argument order differs
// from the legacy static minus(x, y) helper used by marginalization.
TEST(PoseManifold, PreservesLeftAxisAngleUpdateAndInverse)
{
  svo::ceres_backend::PoseLocalParameterization manifold;
  Eigen::Matrix<double, 7, 1> x;
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()));
  x << 1, -2, 3, q.x(), q.y(), q.z(), q.w();
  Eigen::Matrix<double, 6, 1> delta;
  delta << 0.1, -0.2, 0.3, 0.12, -0.08, 0.04;
  Eigen::Matrix<double, 7, 1> y;
  ASSERT_TRUE(manifold.Plus(x.data(), delta.data(), y.data()));
  Eigen::Quaterniond expected = Eigen::Quaterniond(Eigen::AngleAxisd(
      delta.tail<3>().norm(), delta.tail<3>().normalized())) * q;
  EXPECT_LT((y.tail<4>() - expected.coeffs()).norm(), 1e-12);
  Eigen::Matrix<double, 6, 1> recovered;
  ASSERT_TRUE(manifold.Minus(y.data(), x.data(), recovered.data()));
  EXPECT_LT((recovered - delta).norm(), 1e-12);

  Eigen::Matrix<double, 7, 6, Eigen::RowMajor> plus;
  Eigen::Matrix<double, 6, 7, Eigen::RowMajor> minus;
  ASSERT_TRUE(manifold.PlusJacobian(x.data(), plus.data()));
  ASSERT_TRUE(manifold.MinusJacobian(x.data(), minus.data()));
  EXPECT_LT((minus * plus - Eigen::Matrix<double, 6, 6>::Identity()).norm(), 1e-12);
  const double step = 1e-7;
  for (int i = 0; i < 6; ++i)
  {
    Eigen::Matrix<double, 6, 1> perturb = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 7, 1> yp, ym;
    perturb[i] = step;
    manifold.Plus(x.data(), perturb.data(), yp.data());
    perturb[i] = -step;
    manifold.Plus(x.data(), perturb.data(), ym.data());
    EXPECT_LT(((yp - ym) / (2 * step) - plus.col(i)).norm(), 1e-8);
  }
  for (int i = 0; i < 7; ++i)
  {
    auto yp = x.eval();
    auto ym = x.eval();
    yp[i] += step;
    ym[i] -= step;
    // Ambient quaternion derivatives are evaluated on the unit sphere.
    yp.tail<4>().normalize();
    ym.tail<4>().normalize();
    Eigen::Matrix<double, 6, 1> dp, dm;
    manifold.Minus(yp.data(), x.data(), dp.data());
    manifold.Minus(ym.data(), x.data(), dm.data());
    EXPECT_LT(((dp - dm) / (2 * step) - minus.col(i)).norm(), 1e-8);
  }
}
