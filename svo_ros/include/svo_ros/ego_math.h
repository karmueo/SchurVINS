#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <cstdint>
namespace svo_ros {
constexpr int64_t kEgoImuOffsetNs = -7000000;
inline int64_t correctedEgoStamp(int64_t raw) { return raw + kEgoImuOffsetNs; }
inline Eigen::Vector3d gyroBiasRadians(const Eigen::Vector3d& degrees)
{
  return degrees * (3.14159265358979323846 / 180.0);
}
inline Eigen::Matrix4d egoBodyCamera(const Eigen::Matrix4d& camera_imu_mm)
{
  Eigen::Matrix4d metric = camera_imu_mm;
  metric.block<3,1>(0,3) *= 0.001;
  // Full inverse also tolerates the finite precision rotation in the source YAML.
  return metric.inverse();
}
} // namespace svo_ros
