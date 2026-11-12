#pragma once

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>    // user-input
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>


#include <svo/common/types.h>
#include <svo/common/camera_fwd.h>
#include <svo/common/transformation.h>

namespace svo {

// forward declarations
class FrameHandlerBase;
class Visualizer;
class ImuHandler;
class BackendInterface;
class CeresBackendInterface;
class CeresBackendPublisher;

enum class PipelineType {
  kMono,
  kStereo,
  kArray
};

/// SVO Interface
class SvoInterface
{
public:

  rclcpp::Node& pnh_;
  PipelineType pipeline_type_;
  // SVO modules.
  std::shared_ptr<FrameHandlerBase> svo_;
  std::shared_ptr<Visualizer> visualizer_;
  std::shared_ptr<ImuHandler> imu_handler_;
  std::shared_ptr<BackendInterface> backend_interface_;
  std::shared_ptr<CeresBackendInterface> ceres_backend_interface_;
  std::shared_ptr<CeresBackendPublisher> ceres_backend_publisher_;

  CameraBundlePtr ncam_;

  // Parameters
  bool set_initial_attitude_from_gravity_ = true;

  // System state.
  bool quit_ = false;
  bool idle_ = false;
  bool automatic_reinitialization_ = false;

  SvoInterface(const PipelineType& pipeline_type,
          rclcpp::Node& nh,
          rclcpp::Node& private_nh, const std::string& run_dir = "");

  virtual ~SvoInterface();

  // Processing
  void processImageBundle(
      const std::vector<cv::Mat>& images,
      int64_t timestamp_nanoseconds);

  bool setImuPrior(const int64_t timestamp_nanoseconds);

  void publishResults(
      const std::vector<cv::Mat>& images,
      const int64_t timestamp_nanoseconds);

  virtual void imageCallbackPreprocessing(int64_t) {}
  virtual void imageCallbackPostprocessing() {}
};
} // namespace svo
