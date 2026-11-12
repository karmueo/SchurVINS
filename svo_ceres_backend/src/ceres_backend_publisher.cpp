#include "svo/ceres_backend_publisher.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>

namespace
{
template <typename T>
void normalizeVector(const std::vector<T>& in, std::vector<float>* out)
{
  auto res = std::minmax_element(in.begin(), in.end());
  const T min = *res.first;
  const T max = *res.second;
  const float dist = std::max(1.0f, static_cast<float>(max - min));

  out->resize(in.size());
  for (size_t i = 0; i < in.size(); i++)
  {
    (*out)[i] = (in[i] - min) / dist;
  }
}
}

namespace svo
{
CeresBackendPublisher::CeresBackendPublisher(
    rclcpp::Node& node,
    const std::shared_ptr<ceres_backend::Map>& map_ptr)
  : node_(node)
  , map_ptr_(map_ptr)
{
  pub_imu_pose_ = node_.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "backend_pose_imu", 10);
  pub_imu_pose_viz_ =
      node_.create_publisher<geometry_msgs::msg::PoseStamped>("backend_pose_imu_viz", 10);
  pub_points_ =
      node_.create_publisher<sensor_msgs::msg::PointCloud2>("backend_points", 10);
}

void CeresBackendPublisher::publish(const ViNodeState& state,
                                    const int64_t timestamp,
                                    const int32_t seq)
{
  publishImuPose(state, timestamp, seq);
  publishBackendLandmarks(timestamp);
}

void CeresBackendPublisher::publishImuPose(const ViNodeState& state,
                                           const int64_t timestamp,
                                           const int32_t seq)
{
  // Trace state
  state_ = state;

  {
    std::lock_guard<std::mutex> lock(mutex_frame_id_);
    state_frame_id_ = BundleId(seq);
  }

  size_t n_pose_sub = pub_imu_pose_->get_subscription_count();
  size_t n_pose_viz_sub = pub_imu_pose_viz_->get_subscription_count();
  if (n_pose_sub == 0 && n_pose_viz_sub == 0)
  {
    return;
  }
  VLOG(100) << "Publish IMU Pose";

  Eigen::Quaterniond q = state.get_T_W_B().getRotation().toImplementation();
  Eigen::Vector3d p = state.get_T_W_B().getPosition();
  const rclcpp::Time time(timestamp);

  if (n_pose_sub > 0)
  {
    auto msg_pose = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg_pose->header.stamp = time;
    msg_pose->header.frame_id = kWorldFrame;
    msg_pose->pose.pose.position.x = p[0];
    msg_pose->pose.pose.position.y = p[1];
    msg_pose->pose.pose.position.z = p[2];
    msg_pose->pose.pose.orientation.x = q.x();
    msg_pose->pose.pose.orientation.y = q.y();
    msg_pose->pose.pose.orientation.z = q.z();
    msg_pose->pose.pose.orientation.w = q.w();
    for (size_t i = 0; i < 36; ++i)
      msg_pose->pose.covariance[i] = 0;
    pub_imu_pose_->publish(std::move(msg_pose));
  }

  if (n_pose_viz_sub > 0)
  {
    auto msg_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();
    msg_pose->header.stamp = time;
    msg_pose->header.frame_id = kWorldFrame;
    msg_pose->pose.position.x = p[0];
    msg_pose->pose.position.y = p[1];
    msg_pose->pose.position.z = p[2];
    msg_pose->pose.orientation.x = q.x();
    msg_pose->pose.orientation.y = q.y();
    msg_pose->pose.orientation.z = q.z();
    msg_pose->pose.orientation.w = q.w();
    pub_imu_pose_viz_->publish(std::move(msg_pose));
  }
}

void CeresBackendPublisher::publishBackendLandmarks(
    const int64_t timestamp) const
{
  if (pub_points_->get_subscription_count() == 0)
  {
    return;
  }

  // get all landmarks
  const std::unordered_map<
      uint64_t, std::shared_ptr<ceres_backend::ParameterBlock> >& idmap =
      map_ptr_->idToParameterBlockMap();
  size_t n_pts = 0;
  std::vector<const double*> landmark_pointers;
  std::vector<uint64_t> point_ids;
  for (auto& it : idmap)
  {
    if (it.second->typeInfo() == "HomogeneousPointParameterBlock" &&
        !it.second->fixed())
    {
      n_pts++;
      landmark_pointers.push_back(it.second->parameters());
      point_ids.push_back(it.second->id());
    }
  }

  if (n_pts < 5)
  {
    return;
  }

  std::vector<float> intensities;
  normalizeVector(point_ids, &intensities);

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = rclcpp::Time(timestamp);
  cloud.header.frame_id = kWorldFrame;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                               "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                               "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                               "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(n_pts);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"),
      z(cloud, "z"), intensity(cloud, "intensity");
  for (size_t i = 0; i < landmark_pointers.size(); ++i, ++x, ++y, ++z, ++intensity)
  {
    const auto p = landmark_pointers[i];
    *x = p[0];
    *y = p[1];
    *z = p[2];
    *intensity = intensities[i];
  }
  pub_points_->publish(cloud);
}

}  // namespace svo
