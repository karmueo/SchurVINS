#pragma once
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <limits>
namespace vk {
template<class A, class B>
::testing::AssertionResult eigenMatrixNear(const Eigen::MatrixBase<A>& a,
                                          const Eigen::MatrixBase<B>& b, double eps)
{
  if (a.rows() != b.rows() || a.cols() != b.cols())
    return ::testing::AssertionFailure() << "Matrix dimensions differ";
  if (!a.allFinite() || !b.allFinite())
    return ::testing::AssertionFailure() << "Non-finite matrix";
  const double error = (a.template cast<double>() - b.template cast<double>()).cwiseAbs().maxCoeff();
  if (error <= eps) return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << "Maximum error " << error << " exceeds " << eps;
}
}
#define EIGEN_MATRIX_NEAR(a,b,eps) ::vk::eigenMatrixNear((a),(b),(eps))
#define EIGEN_MATRIX_EQUAL_DOUBLE(a,b) ::vk::eigenMatrixNear((a),(b),1e-12)
