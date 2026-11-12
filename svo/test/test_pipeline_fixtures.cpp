// Geometry-consistent synthetic fixtures for the unmodified traditional pipelines.
#include <gtest/gtest.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
#include <svo/frame_handler_array.h>
#include <svo/initialization.h>
#include <svo/map.h>
#include <svo/reprojector.h>
#include <svo/stereo_triangulation.h>
#include <svo/direct/depth_filter.h>
#include <svo/direct/feature_detection.h>
#include <svo/tracker/feature_tracker.h>
#include <svo/common/frame.h>
#include <vikit/cameras/camera_factory.h>
#include <vikit/cameras/ncamera.h>
#include <opencv2/imgproc.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <vector>

namespace {
constexpr int kWidth = 640, kHeight = 480;
constexpr double kFocal = 400.0;
constexpr int kFrames = 24;

struct Tile
{
  Eigen::Vector3d center;
  double width, height;
  cv::Mat texture;
};

// Every texture belongs to a fixed physical plane in world coordinates. Camera
// motion and rig extrinsics only change the pinhole homography, not the texture.
class TexturedScene
{
public:
  TexturedScene()
  {
    cv::RNG random(0x53434855);
    addTile(Eigen::Vector3d(0,0,9), 20.0, 16.0, 768, random);
    for (int y = -4; y <= 4; ++y)
      for (int x = -6; x <= 6; ++x)
      {
        const int layer = ((x+6)*13+(y+4)*7) % 6;
        addTile(Eigen::Vector3d(.75*x,.75*y,4.0+.4*layer), .70, .70, 40, random);
      }
    std::stable_sort(tiles_.begin(), tiles_.end(), [](const Tile& a, const Tile& b) {
      return a.center.z() > b.center.z();
    });
  }

  cv::Mat render(const svo::Transformation& world_camera) const
  {
    cv::Mat image(kHeight, kWidth, CV_8UC1, cv::Scalar(127));
    const auto camera_world = world_camera.inverse();
    for (const auto& tile : tiles_)
    {
      const std::vector<Eigen::Vector3d> corners = {
        tile.center+Eigen::Vector3d(-tile.width/2,-tile.height/2,0),
        tile.center+Eigen::Vector3d(tile.width/2,-tile.height/2,0),
        tile.center+Eigen::Vector3d(tile.width/2,tile.height/2,0),
        tile.center+Eigen::Vector3d(-tile.width/2,tile.height/2,0)};
      std::vector<cv::Point2f> pixels;
      for (const auto& corner : corners)
      {
        const auto point = camera_world * corner;
        if (point.z() <= .1) break;
        pixels.emplace_back(kFocal*point.x()/point.z()+kWidth/2.0,
                            kFocal*point.y()/point.z()+kHeight/2.0);
      }
      if (pixels.size() != 4) continue;
      const auto roi = cv::boundingRect(pixels) & cv::Rect(0,0,kWidth,kHeight);
      if (roi.empty()) continue;
      for (auto& pixel : pixels) { pixel.x -= roi.x; pixel.y -= roi.y; }
      const float last_x = static_cast<float>(tile.texture.cols-1);
      const float last_y = static_cast<float>(tile.texture.rows-1);
      const std::vector<cv::Point2f> source = {{0,0},{last_x,0},{last_x,last_y},{0,last_y}};
      const auto homography = cv::getPerspectiveTransform(source, pixels);
      cv::Mat projected, mask;
      cv::warpPerspective(tile.texture, projected, homography, roi.size(), cv::INTER_LINEAR);
      cv::warpPerspective(cv::Mat(tile.texture.size(), CV_8UC1, cv::Scalar(255)),
                          mask, homography, roi.size(), cv::INTER_NEAREST);
      projected.copyTo(image(roi), mask);
    }
    return image;
  }
private:
  void addTile(const Eigen::Vector3d& center, double width, double height,
               int resolution, cv::RNG& random)
  {
    Tile tile{center,width,height,cv::Mat(resolution,resolution,CV_8UC1)};
    random.fill(tile.texture, cv::RNG::UNIFORM, 10, 245);
    cv::GaussianBlur(tile.texture, tile.texture, cv::Size(3,3), .5);
    tiles_.push_back(std::move(tile));
  }
  std::vector<Tile> tiles_;
};

struct Rig
{
  svo::CameraBundle::Ptr cameras;
  vk::cameras::TransformationVector body_camera;
};

Rig makeRig(size_t count)
{
  Rig result;
  std::vector<svo::CameraPtr> cameras;
  vk::cameras::TransformationVector camera_body;
  Eigen::VectorXd intrinsics(4); intrinsics << kFocal,kFocal,kWidth/2.0,kHeight/2.0;
  for (size_t camera = 0; camera < count; ++camera)
  {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double yaw = 0, pitch = 0;
    if (count == 2) center.x() = camera == 0 ? -.15 : .15;
    if (count == 4)
    {
      // Non-collinear centers and distinct viewing directions constrain metric
      // noncentral geometry. No stereo pair is duplicated for the array case.
      const Eigen::Vector3d centers[] = {{-.18,-.12,0},{.17,-.10,.03},
                                        {-.12,.15,-.02},{.20,.13,.01}};
      center = centers[camera];
      yaw = camera % 2 == 0 ? -.08 : .09;
      pitch = camera < 2 ? -.04 : .05;
    }
    const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitX())).toRotationMatrix();
    result.body_camera.emplace_back(svo::Quaternion(rotation), center);
    camera_body.push_back(result.body_camera.back().inverse());
    cameras.push_back(vk::cameras::factory::makePinholeCamera(intrinsics,kWidth,kHeight));
  }
  result.cameras = std::make_shared<svo::CameraBundle>(camera_body,cameras,"textured_multidepth_fixture");
  return result;
}

svo::Transformation bodyPose(int frame)
{
  const double time = static_cast<double>(frame);
  const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(.006*time,Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(.002*std::sin(.3*time),Eigen::Vector3d::UnitZ())).toRotationMatrix();
  return svo::Transformation(svo::Quaternion(rotation),
      Eigen::Vector3d(.045*time,.008*std::sin(.3*time),.002*time));
}

enum class Pipeline { Mono, Stereo, Array };

void exercisePipeline(Pipeline pipeline)
{
  std::srand(7); cv::setRNGSeed(7);
  if (!google::IsGoogleLoggingInitialized()) google::InitGoogleLogging("pipeline_fixtures");
  const size_t count = pipeline == Pipeline::Mono ? 1 : (pipeline == Pipeline::Stereo ? 2 : 4);
  const Rig rig = makeRig(count);
  TexturedScene scene;
  svo::BaseOptions base;
  // Test-local settings: small, synchronous, noise-free scene. Production
  // defaults are untouched; no supplied poses, map points or IMU priors are used.
  base.use_imu = false;
  base.trace_statistics = false;
  base.max_n_kfs = 20;
  base.init_map_scale = 5.0;
  base.img_align_max_level = 3;
  base.img_align_min_level = 0;
  base.quality_min_fts = 20;
  base.quality_max_fts_drop = 1000;
  base.kfselect_min_dist = .035;
  svo::DepthFilterOptions depth;
  depth.use_threaded_depthfilter = false;
  depth.max_n_seeds_per_frame = 240;
  svo::DetectorOptions detector;
  detector.cell_size = 20;
  detector.max_level = 2;
  svo::ReprojectorOptions reprojector;
  reprojector.cell_size = detector.cell_size;
  reprojector.max_n_features_per_frame = 240;
  svo::FeatureTrackerOptions tracker;
  tracker.klt_max_level = 3;
  svo::InitializationOptions initialization;
  initialization.init_type = pipeline == Pipeline::Mono ? svo::InitializerType::kFivePoint :
      (pipeline == Pipeline::Stereo ? svo::InitializerType::kStereo : svo::InitializerType::kArrayGeometric);
  initialization.init_min_features = pipeline == Pipeline::Stereo ? 30 : 60;
  initialization.init_min_tracked = 40;
  initialization.init_min_inliers = 30;
  initialization.init_min_disparity = 12;
  std::unique_ptr<svo::FrameHandlerBase> handler;
  if (pipeline == Pipeline::Mono)
    handler = std::make_unique<svo::FrameHandlerMono>(base,depth,detector,initialization,reprojector,tracker,rig.cameras);
  else if (pipeline == Pipeline::Stereo)
    handler = std::make_unique<svo::FrameHandlerStereo>(base,depth,detector,initialization,
        svo::StereoTriangulationOptions(),reprojector,tracker,rig.cameras);
  else handler = std::make_unique<svo::FrameHandlerArray>(base,depth,detector,initialization,reprojector,tracker,rig.cameras);
  handler->setInitialImuPose(svo::Transformation());
  handler->start();
  int tracking_frames = 0, first_tracking_frame = -1;
  int64_t previous_timestamp = -1;
  size_t maximum_keyframes = 0;
  Eigen::Vector3d first_position = Eigen::Vector3d::Zero(), last_position = first_position;
  for (int frame = 0; frame < kFrames; ++frame)
  {
    SCOPED_TRACE(::testing::Message() << "camera_count=" << count << ", input_frame=" << frame);
    std::vector<cv::Mat> images;
    for (size_t camera = 0; camera < count; ++camera)
      images.push_back(scene.render(bodyPose(frame)*rig.body_camera[camera]));
    if (frame == 0 && count > 1)
      for (size_t camera = 1; camera < count; ++camera)
        EXPECT_GT(cv::norm(images.front(),images[camera],cv::NORM_L1), 10000.0);
    const int64_t timestamp = 1000000000LL + 50000000LL*frame;
    ASSERT_TRUE(handler->addImageBundle(images,timestamp));
    const auto frames = handler->getLastFrames();
    ASSERT_TRUE(frames);
    ASSERT_EQ(frames->size(), count);
    EXPECT_EQ(frames->getMinTimestampNanoseconds(), timestamp);
    EXPECT_GT(timestamp,previous_timestamp); previous_timestamp = timestamp;
    maximum_keyframes = std::max(maximum_keyframes,handler->map()->numKeyframes());
    if (handler->stage() != svo::Stage::kTracking) continue;
    const auto pose = frames->get_T_W_B();
    ASSERT_TRUE(pose.getTransformationMatrix().allFinite());
    EXPECT_GT(frames->numLandmarks(), 15u);
    if (first_tracking_frame < 0) { first_tracking_frame = frame; first_position = pose.getPosition(); }
    last_position = pose.getPosition(); ++tracking_frames;
    EXPECT_LT(last_position.norm(), 10.0); // Reject finite but runaway poses.
  }
  ASSERT_GE(tracking_frames, 6) << "The real initializer must enter tracking and remain useful";
  EXPECT_EQ(handler->stage(), svo::Stage::kTracking);
  EXPECT_GE(maximum_keyframes, count == 4 ? 8u : count);
  const Eigen::Vector3d estimated = last_position-first_position;
  const Eigen::Vector3d expected = bodyPose(kFrames-1).getPosition()-bodyPose(first_tracking_frame).getPosition();
  ASSERT_GT(estimated.norm(), .02) << "A stationary identity output is not successful tracking";
  EXPECT_GT(estimated.normalized().dot(expected.normalized()), .7);
  if (pipeline != Pipeline::Mono)
  {
    // The known baselines must recover a physical scale; this generous bound
    // detects missing metric scale without claiming dataset-level accuracy.
    const double metric_ratio = estimated.norm()/expected.norm();
    EXPECT_GT(metric_ratio,.35); EXPECT_LT(metric_ratio,2.5);
  }
}
} // namespace

TEST(PipelineFixtures, MonocularFivePointInitializationAndTracking)
{
#ifdef USE_SCHUR_VINS
  GTEST_SKIP() << "Traditional monocular fixture requires USE_SCHUR_VINS=OFF";
#else
  exercisePipeline(Pipeline::Mono);
#endif
}
TEST(PipelineFixtures, StereoPureVisionTracksWithMetricBaseline)
{
#ifdef USE_SCHUR_VINS
  GTEST_SKIP() << "Pure-vision fixture requires USE_SCHUR_VINS=OFF";
#else
  exercisePipeline(Pipeline::Stereo);
#endif
}
TEST(PipelineFixtures, FourDistinctCamerasInitializeAndTrackMetricMotion)
{
#ifdef USE_SCHUR_VINS
  GTEST_SKIP() << "Traditional four-camera fixture requires USE_SCHUR_VINS=OFF";
#else
  exercisePipeline(Pipeline::Array);
#endif
}
