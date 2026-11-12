// Modification Note: 
// This file may have been modified by the authors of SchurVINS.
// (All authors of SchurVINS are with PICO department of ByteDance Corporation)
#include <svo_ros/svo_interface.h>



#include <svo_ros/svo_factory.h>
#include <svo_ros/visualizer.h>
#include <svo/common/frame.h>
#include <svo/map.h>
#include <svo/imu_handler.h>
#include <svo/common/camera.h>
#include <svo/common/conversions.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
#include <svo/frame_handler_array.h>
#include <svo/initialization.h>
#include <svo/direct/depth_filter.h>

#include <vikit/params_helper.h>
#include <vikit/timer.h>
#include <svo_ros/ceres_backend_factory.h>
#include <svo/ceres_backend_publisher.hpp>



#ifdef SVO_LOOP_CLOSING
#include <svo/online_loopclosing/loop_closing.h>
#endif

#ifdef SVO_GLOBAL_MAP
#include <svo/global_map.h>
#endif

namespace svo {

SvoInterface::SvoInterface(
    const PipelineType& pipeline_type,
    rclcpp::Node& nh,
    rclcpp::Node& private_nh, const std::string& run_dir)
  : pnh_(private_nh)
  , pipeline_type_(pipeline_type)
  , set_initial_attitude_from_gravity_(
      vk::param<bool>(pnh_, "set_initial_attitude_from_gravity", true))
  , automatic_reinitialization_(
      vk::param<bool>(pnh_, "automatic_reinitialization", false))
{
  if (!google::IsGoogleLoggingInitialized()) google::InitGoogleLogging("svo");
  switch (pipeline_type)
  {
    case PipelineType::kMono:
      svo_ = factory::makeMono(pnh_, nullptr, run_dir);
      break;
    case PipelineType::kStereo:
      svo_ = factory::makeStereo(pnh_, nullptr, run_dir);
      break;
    case PipelineType::kArray:
      svo_ = factory::makeArray(pnh_, nullptr, run_dir);
      break;
    default:
      LOG(FATAL) << "Unknown pipeline";
      break;
  }
  ncam_ = svo_->getNCamera();
  const size_t expected = pipeline_type == PipelineType::kMono ? 1 : (pipeline_type == PipelineType::kStereo ? 2 : 4);
  if (ncam_->numCameras() != expected)
    throw std::invalid_argument("Calibration camera count does not match selected pipeline");

  visualizer_.reset(
        new Visualizer(svo_->options_.trace_dir, pnh_, ncam_->getNumCameras()));

  if(vk::param<bool>(pnh_, "use_imu", false))
  {
    imu_handler_ = factory::getImuHandler(pnh_);
    svo_->setImuHandler(imu_handler_);
    // svo_->imu_handler_ = imu_handler_;
  }

  if(vk::param<bool>(pnh_, "use_ceres_backend", false))
  {
    ceres_backend_interface_ = ceres_backend_factory::makeBackend(pnh_,ncam_);
    if(imu_handler_){
      svo_->setBundleAdjuster(ceres_backend_interface_);
      ceres_backend_interface_->setImu(imu_handler_);
      ceres_backend_interface_->makePublisher(pnh_, ceres_backend_publisher_);
    }
    else
    {
      SVO_ERROR_STREAM("Cannot use ceres backend without using imu");
    }
  }
  if(vk::param<bool>(pnh_, "runlc", false))
  {
#ifdef SVO_LOOP_CLOSING
    LoopClosingPtr loop_closing_ptr =
        factory::getLoopClosingModule(pnh_, svo_->getNCamera(), run_dir);
    svo_->lc_ = std::move(loop_closing_ptr);
    CHECK(svo_->depth_filter_->options_.extra_map_points)
        << "The depth filter seems to be initialized without extra map points.";
#else
    LOG(FATAL) << "You have to enable loop closing in svo_cmake.";
#endif
  }

  if(vk::param<bool>(pnh_, "use_global_map", false))
  {
#ifdef SVO_GLOBAL_MAP
    svo_->global_map_ = factory::getGlobalMap(pnh_, svo_->getNCamera(), run_dir);
    if (imu_handler_)
    {
      svo_->global_map_->initializeIMUParams(imu_handler_->imu_calib_,
                                             imu_handler_->imu_init_);
    }
#else
    LOG(FATAL) << "You have to enable global map in cmake";
#endif
  }

  svo_->start();
}

SvoInterface::~SvoInterface() = default;

void SvoInterface::processImageBundle(
    const std::vector<cv::Mat>& images,
    const int64_t timestamp_nanoseconds)
{
  if (!svo_->isBackendValid())
  {
    if (vk::param<bool>(pnh_, "use_ceres_backend", false, true))
    {
      ceres_backend_interface_ =
          ceres_backend_factory::makeBackend(pnh_, ncam_);
      if (imu_handler_)
      {
        svo_->setBundleAdjuster(ceres_backend_interface_);
        ceres_backend_interface_->setImu(imu_handler_);
        ceres_backend_interface_->makePublisher(pnh_, ceres_backend_publisher_);
      }
      else
      {
        SVO_ERROR_STREAM("Cannot use ceres backend without using imu");
      }
    }
  }
  svo_->addImageBundle(images, timestamp_nanoseconds);
}

void SvoInterface::publishResults(
    const std::vector<cv::Mat>& images,
    const int64_t timestamp_nanoseconds)
{
  CHECK_NOTNULL(svo_.get());
  CHECK_NOTNULL(visualizer_.get());

  visualizer_->img_caption_.clear();
  if (svo_->isBackendValid())
  {
    std::string static_str = ceres_backend_interface_->getStationaryStatusStr();
    visualizer_->img_caption_ = static_str;
  }

  visualizer_->publishSvoInfo(svo_.get(), timestamp_nanoseconds);
  switch (svo_->stage())
  {
    case Stage::kTracking: {
      Eigen::Matrix<double, 6, 6> covariance;
      covariance.setZero();
      visualizer_->publishImuPose(
            svo_->getLastFrames()->get_T_W_B(), covariance, timestamp_nanoseconds);
      visualizer_->publishCameraPoses(svo_->getLastFrames(), timestamp_nanoseconds);
      visualizer_->visualizeMarkers(
            svo_->getLastFrames(), svo_->closeKeyframes(), svo_->map());
      visualizer_->exportToDense(svo_->getLastFrames());
      bool draw_boundary = false;
      if (svo_->isBackendValid())
      {
        draw_boundary = svo_->getBundleAdjuster()->isFixedToGlobalMap();
      }
      visualizer_->publishImagesWithFeatures(
            svo_->getLastFrames(), timestamp_nanoseconds,
            draw_boundary);
#ifdef SVO_LOOP_CLOSING
      // detections
      if (svo_->lc_)
      {
        visualizer_->publishLoopClosureInfo(
              svo_->lc_->cur_loop_check_viz_info_,
              std::string("loop_query"),
              Eigen::Vector3f(0.0f, 0.0f, 1.0f), 0.5);
        visualizer_->publishLoopClosureInfo(
              svo_->lc_->loop_detect_viz_info_, std::string("loop_detection"),
              Eigen::Vector3f(1.0f, 0.0f, 0.0f), 1.0);
        if (svo_->isBackendValid())
        {
          visualizer_->publishLoopClosureInfo(
                svo_->lc_->loop_correction_viz_info_,
                std::string("loop_correction"),
                Eigen::Vector3f(0.0f, 1.0f, 0.0f), 3.0);
        }
        if (svo_->getLastFrames()->at(0)->isKeyframe())
        {
          bool pc_recalculated = visualizer_->publishPoseGraph(
                svo_->lc_->kf_list_,
                svo_->lc_->need_to_update_pose_graph_viz_,
                static_cast<size_t>(svo_->lc_->options_.ignored_past_frames));
          if(pc_recalculated)
          {
            svo_->lc_->need_to_update_pose_graph_viz_ = false;
          }
        }
      }
#endif
#ifdef SVO_GLOBAL_MAP
      if (svo_->global_map_)
      {
        visualizer_->visualizeGlobalMap(*(svo_->global_map_),
                                        std::string("global_vis"),
                                        Eigen::Vector3f(0.0f, 0.0f, 1.0f),
                                        0.3);
        visualizer_->visualizeFixedLandmarks(svo_->getLastFrames()->at(0));
      }
#endif
      break;
    }
    case Stage::kInitializing: {
      visualizer_->publishBundleFeatureTracks(
            svo_->initializer_->frames_ref_, svo_->getLastFrames(),
            timestamp_nanoseconds);
      break;
    }
    case Stage::kPaused:
    case Stage::kRelocalization:
      visualizer_->publishImages(images, timestamp_nanoseconds);
      break;
    default:
      LOG(FATAL) << "Unknown stage";
      break;
  }

}

bool SvoInterface::setImuPrior(const int64_t timestamp_nanoseconds)
{
  if(svo_->getBundleAdjuster())
  {
    //if we use backend, this will take care of setting priors
    if(!svo_->hasStarted())
    {
      //when starting up, make sure we already have IMU measurements
      if(imu_handler_->getMeasurementsCopy().size() < 10u)
      {
        return false;
      }
    }
    return true;
  }

  if(imu_handler_ && !svo_->hasStarted() && set_initial_attitude_from_gravity_)
  {
    // set initial orientation
    Quaternion R_imu_world;
    if(imu_handler_->getInitialAttitude(
         timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds,
         R_imu_world))
    {
      VLOG(3) << "Set initial orientation from accelerometer measurements.";
      svo_->setRotationPrior(R_imu_world);
    }
    else
    {
      return false;
    }
  }
  else if(imu_handler_ && svo_->getLastFrames())
  {
    // set incremental rotation prior
    Quaternion R_lastimu_newimu;
    if(imu_handler_->getRelativeRotationPrior(
         svo_->getLastFrames()->getMinTimestampNanoseconds() *
         common::conversions::kNanoSecondsToSeconds,
         timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds,
         false, R_lastimu_newimu))
    {
      VLOG(3) << "Set incremental rotation prior from IMU.";
      svo_->setRotationIncrementPrior(R_lastimu_newimu);
    }
  }
  return true;
}

} // namespace svo
