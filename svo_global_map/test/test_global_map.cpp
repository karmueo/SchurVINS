// This file is part of SVO - Semi-direct Visual Odometry.
// This file is subject to the terms and conditions defined in LICENSE.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <svo/common/camera.h>
#include <svo/common/frame.h>
#include <svo/common/point.h>
#include <svo/global_map.h>
#include <svo/gtsam/graph_manager.h>
#include <svo/gtsam/gtsam_optimizer.h>
#include <vikit/cameras/camera_factory.h>
#include <vikit/cameras/ncamera.h>

namespace
{
constexpr int kFirstFrameId = 100;
constexpr int kFirstPointId = 1000;
constexpr size_t kInitialFrames = 3;

struct Scene
{
  svo::CameraBundlePtr rig;
  std::vector<svo::Transformation> truth_poses;
  std::vector<Eigen::Vector3d> truth_points;
  std::vector<svo::PointPtr> points;
  std::vector<svo::FramePtr> frames;
};

Scene makeScene(const Eigen::Vector3d& point_offset = Eigen::Vector3d::Zero())
{
  Scene scene;
  Eigen::VectorXd intrinsics(4);
  intrinsics << 200.0, 200.0, 320.0, 240.0;
  const auto camera = vk::cameras::factory::makePinholeCamera(intrinsics, 640, 480);
  vk::cameras::TransformationVector extrinsics(1, svo::Transformation());
  scene.rig = std::make_shared<svo::CameraBundle>(
      extrinsics, std::vector<svo::CameraPtr>{camera}, "global_map_fixture");

  // A deterministic non-coplanar scene; no random sampling or dataset is needed.
  for (int row = 0; row < 4; ++row)
  {
    for (int column = 0; column < 6; ++column)
    {
      scene.truth_points.emplace_back(-1.2 + 0.48 * column,
          -0.6 + 0.4 * row, 4.5 + 0.3 * ((3 * row + 2 * column) % 5));
      const int id = kFirstPointId + static_cast<int>(scene.points.size());
      scene.points.emplace_back(std::make_shared<svo::Point>(
          id, scene.truth_points.back() + point_offset));
    }
  }

  for (size_t frame_index = 0; frame_index < 4; ++frame_index)
  {
    const Eigen::Vector3d position(0.8 * frame_index, 0.05 * frame_index, 0.0);
    scene.truth_poses.emplace_back(svo::Quaternion(), position);
    svo::Transformation initial = scene.truth_poses.back();
    if (frame_index != 0)
    {
      initial = svo::Transformation(
          svo::Quaternion(Eigen::Quaterniond(Eigen::AngleAxisd(
              0.025, Eigen::Vector3d(1.0, -2.0, 1.0).normalized()))),
          position + Eigen::Vector3d(0.12, -0.08, 0.05));
    }
    const int id = kFirstFrameId + static_cast<int>(frame_index);
    auto frame = std::make_shared<svo::Frame>(
        id, 1000000000LL * (frame_index + 1), camera, initial);
    frame->bundle_id_ = id;
    frame->nframe_index_ = 0;
    frame->set_T_cam_imu(svo::Transformation());
    frame->resizeFeatureStorage(scene.points.size());
    frame->num_features_ = scene.points.size();
    frame->grad_vec_.setZero();
    for (size_t point_index = 0; point_index < scene.points.size(); ++point_index)
    {
      const Eigen::Vector3d in_camera =
          scene.truth_poses.back().inverse() * scene.truth_points[point_index];
      frame->f_vec_.col(point_index) = in_camera.normalized();
      frame->px_vec_.col(point_index) = Eigen::Vector2d(
          200.0 * in_camera.x() / in_camera.z() + 320.0,
          200.0 * in_camera.y() / in_camera.z() + 240.0);
      frame->landmark_vec_[point_index] = scene.points[point_index];
      frame->track_id_vec_(point_index) = scene.points[point_index]->id();
    }
    // GlobalMap owns admission and observation registration. Pre-registering
    // Point::obs_ here would prevent the production cache from admitting it.
    frame->setKeyframe();
    scene.frames.push_back(frame);
  }
  return scene;
}

double bearingError(const Scene& scene, size_t frame_count)
{
  double error = 0.0;
  for (size_t i = 0; i < frame_count; ++i)
  {
    const auto& frame = scene.frames[i];
    for (size_t j = 0; j < scene.points.size(); ++j)
    {
      if (!frame->landmark_vec_[j])
        return std::numeric_limits<double>::infinity();
      const Eigen::Vector3d predicted = frame->T_f_w_ * frame->landmark_vec_[j]->pos();
      if (!predicted.allFinite() || predicted.norm() == 0.0)
        return std::numeric_limits<double>::infinity();
      error += (predicted.normalized() - frame->f_vec_.col(j)).squaredNorm();
    }
  }
  return error;
}

double positionError(const Scene& scene, size_t frame_count)
{
  double error = 0.0;
  for (size_t i = 0; i < frame_count; ++i)
    error += (scene.frames[i]->T_world_imu().getPosition() -
              scene.truth_poses[i].getPosition()).squaredNorm();
  return std::sqrt(error / frame_count);
}

// An assertion or solver exception must not leave an incomplete initial graph
// for GlobalMap's destructor to finish indefinitely. Success is asserted before
// this cleanup, including the normal finishAndQuit() path.
struct ClearMapOnExit
{
  svo::GlobalMap& map;
  ~ClearMapOnExit() { map.reset(); }
};

void expectRetrievedPoses(const svo::BundleIdToIMUPose& poses,
                          const Scene& scene, size_t count)
{
  ASSERT_EQ(poses.size(), count);
  for (size_t i = 0; i < count; ++i)
  {
    const auto found = poses.find(scene.frames[i]->bundleId());
    ASSERT_NE(found, poses.end());
    EXPECT_TRUE(found->second.getTransformationMatrix().allFinite());
    EXPECT_LT((found->second.getPosition() - scene.truth_poses[i].getPosition()).norm(), 0.02);
    const auto rotation = found->second.getRotation().toImplementation();
    EXPECT_LT(rotation.angularDistance(scene.truth_poses[i].getRotation().toImplementation()), 0.005);
  }
}
}  // namespace

TEST(GlobalMapFeature, AdmitsOptimizesAndRetrievesRealMultiFrameLandmarks)
{
  auto scene = makeScene();
  const double initial_bearing_error = bearingError(scene, kInitialFrames);
  const double initial_position_error = positionError(scene, kInitialFrames);
  ASSERT_GT(initial_bearing_error, 1e-4);
  ASSERT_GT(initial_position_error, 0.05);

  svo::GlobalMapOptions options;
  options.use_imu = false;  // This fixture supplies visual factors, not synthetic IMU.
  options.use_thread = false;  // Exercise the real solver deterministically.
  options.print_stdcout = false;
  options.init_min_frame_before_opt = kInitialFrames;
  // Point conditioning is a separate admission policy. Use a short fixture
  // baseline without requiring the default dataset-scale conditioning gate.
  options.pt_cond_thresh = -1.0;
  svo::GlobalMap map(options, svo::GraphManagerOptions(),
                     svo::GTSAMOptimizerOptions(), scene.rig);
  ClearMapOnExit cleanup{map};

  ASSERT_NO_THROW({
    for (size_t i = 0; i < kInitialFrames; ++i)
      map.addKeyframe(scene.frames[i]);
  });
  ASSERT_TRUE(map.hasInitialBA());
  ASSERT_EQ(map.numFramesInMap(), kInitialFrames);
  ASSERT_TRUE(map.isAllOptimizationDone());
  EXPECT_LT(bearingError(scene, kInitialFrames), initial_bearing_error * 0.05);
  EXPECT_LT(positionError(scene, kInitialFrames), initial_position_error * 0.1);

  svo::BundleIdToIMUPose poses;
  map.getBundleIDIMUPosesMap(&poses);
  expectRetrievedPoses(poses, scene, kInitialFrames);

  rpg::PositionVec points;
  std::vector<int> point_ids;
  map.getPointPositionAndIds(&points, &point_ids);
  ASSERT_EQ(points.size(), scene.points.size());
  ASSERT_EQ(point_ids.size(), points.size());
  for (size_t i = 0; i < points.size(); ++i)
  {
    const auto original = std::find_if(scene.points.begin(), scene.points.end(),
        [&](const auto& point) { return point->id() == point_ids[i]; });
    ASSERT_NE(original, scene.points.end());
    EXPECT_TRUE(points[i].allFinite());
    EXPECT_LT((points[i] - scene.truth_points[original - scene.points.begin()]).norm(), 0.01);
    EXPECT_TRUE(map.pointInGraph(point_ids[i]));
    EXPECT_TRUE(map.isPointOptimized(point_ids[i]));
  }
  EXPECT_TRUE(map.hasNewEstimate());
  map.setResultsPublished();
  EXPECT_FALSE(map.hasNewEstimate());

  // A fourth, initially perturbed pose must update the existing map rather than
  // merely replay the initial estimate or populate a fresh native GTSAM graph.
  const double fourth_error_before = bearingError(scene, scene.frames.size());
  ASSERT_NO_THROW(map.addKeyframe(scene.frames.back()));
  ASSERT_EQ(map.numFramesInMap(), scene.frames.size());
  EXPECT_TRUE(map.hasNewEstimate());
  EXPECT_TRUE(map.isAllOptimizationDone());
  EXPECT_LT(bearingError(scene, scene.frames.size()), fourth_error_before * 0.05);
  map.getBundleIDIMUPosesMap(&poses);
  expectRetrievedPoses(poses, scene, scene.frames.size());
  ASSERT_NO_THROW(map.finishAndQuit());
  EXPECT_TRUE(map.isAllOptimizationDone());
}

TEST(GlobalMapGraphFeature, ReducesNonlinearCostAndProcessesIncrementalState)
{
  const auto scene = makeScene(Eigen::Vector3d(0.06, -0.04, 0.08));
  auto graph = std::make_shared<svo::GraphManager>(svo::GraphManagerOptions());
  graph->initialize(200.0);
  for (size_t j = 0; j < scene.points.size(); ++j)
  {
    graph->addLandmark(*scene.points[j]);
    graph->addPointPriorFactor(scene.points[j]->id(), scene.truth_points[j], 0.01);
  }
  for (size_t i = 0; i < kInitialFrames; ++i)
  {
    graph->augmentStateWithPose(scene.frames[i]->bundleId(), scene.frames[i]->T_world_imu());
    for (size_t j = 0; j < scene.points.size(); ++j)
      graph->addObservationToLandmark(*scene.frames[i], j, false);
  }
  graph->addPosePriorFactor(scene.frames.front()->bundleId(),
                            scene.truth_poses.front(), 1e-4, 1e-4);
  const auto relative_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.01);
  for (size_t i = 1; i < kInitialFrames; ++i)
    graph->addRelativePosePriorFactor(scene.frames[i - 1]->bundleId(), scene.frames[i]->bundleId(),
        scene.truth_poses[i - 1].inverse() * scene.truth_poses[i], relative_noise);

  const double initial_cost = graph->new_factors_.error(graph->new_values_);
  ASSERT_TRUE(std::isfinite(initial_cost));
  ASSERT_GT(initial_cost, 1.0);
  svo::GTSAMOptimizer optimizer(svo::GTSAMOptimizerOptions(), graph);
  optimizer.initialize();
  ASSERT_NO_THROW(optimizer.optimize());
  ASSERT_EQ(optimizer.latest_estimate_state_index_.load(), scene.frames[kInitialFrames - 1]->bundleId());
  ASSERT_EQ(optimizer.estimate_.size(), scene.points.size() + kInitialFrames);
  const double optimized_cost = optimizer.isam_->getFactorsUnsafe().error(optimizer.estimate_);
  EXPECT_TRUE(std::isfinite(optimized_cost));
  EXPECT_LT(optimized_cost, initial_cost * 0.01);
  for (size_t j = 0; j < scene.points.size(); ++j)
  {
    const auto point = optimizer.estimate_.at<gtsam::Point3>(gtsam::Symbol('l', scene.points[j]->id()));
    EXPECT_TRUE(point.allFinite());
    EXPECT_LT((point - scene.truth_points[j]).norm(), 0.01);
  }
  for (size_t i = 0; i < kInitialFrames; ++i)
  {
    const auto pose = optimizer.estimate_.at<gtsam::Pose3>(gtsam::Symbol('x', scene.frames[i]->bundleId()));
    EXPECT_TRUE(pose.matrix().allFinite());
    EXPECT_LT((pose.translation() - scene.truth_poses[i].getPosition()).norm(), 0.02);
  }

  const auto& next = scene.frames.back();
  graph->augmentStateWithPose(next->bundleId(), next->T_world_imu());
  for (size_t j = 0; j < scene.points.size(); ++j)
    graph->addObservationToLandmark(*next, j, false);
  graph->addRelativePosePriorFactor(scene.frames[kInitialFrames - 1]->bundleId(), next->bundleId(),
      scene.truth_poses[kInitialFrames - 1].inverse() * scene.truth_poses.back(), relative_noise);
  auto before_update = optimizer.estimate_;
  before_update.insert(graph->new_values_);
  const double incremental_cost = graph->new_factors_.error(before_update) +
      optimizer.isam_->getFactorsUnsafe().error(before_update);
  ASSERT_GT(incremental_cost, 1.0);
  ASSERT_NO_THROW(optimizer.optimize());
  EXPECT_EQ(optimizer.latest_estimate_state_index_.load(), next->bundleId());
  EXPECT_EQ(optimizer.estimate_.size(), scene.points.size() + scene.frames.size());
  const double final_cost = optimizer.isam_->getFactorsUnsafe().error(optimizer.estimate_);
  EXPECT_TRUE(std::isfinite(final_cost));
  EXPECT_LT(final_cost, incremental_cost * 0.01);
  const auto final_pose = optimizer.estimate_.at<gtsam::Pose3>(gtsam::Symbol('x', next->bundleId()));
  EXPECT_TRUE(final_pose.matrix().allFinite());
  EXPECT_LT((final_pose.translation() - scene.truth_poses.back().getPosition()).norm(), 0.02);
}
