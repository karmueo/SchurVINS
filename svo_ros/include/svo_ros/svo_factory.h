#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <svo/common/camera_fwd.h>

namespace svo {

// forward declarations
class ImuHandler;
class LoopClosing;
class GlobalMap;
class FrameHandlerMono;
class FrameHandlerStereo;
class FrameHandlerArray;
class FrameHandlerDenseMono;

namespace factory {

/// Get IMU Handler.
std::shared_ptr<ImuHandler> getImuHandler(
    rclcpp::Node& pnh);

#ifdef SVO_LOOP_CLOSING
/// Create loop closing module
std::shared_ptr<LoopClosing> getLoopClosingModule(
    rclcpp::Node& pnh,
    const CameraBundlePtr& cam=nullptr, const std::string& trace_dir = "");
#endif

#ifdef SVO_GLOBAL_MAP
std::shared_ptr<GlobalMap> getGlobalMap(
    rclcpp::Node& pnh,
    const CameraBundlePtr& ncams = nullptr, const std::string& trace_dir = "");
#endif

/// Factory for Mono-SVO.
std::shared_ptr<FrameHandlerMono> makeMono(
    rclcpp::Node& pnh,
    const CameraBundlePtr& cam = nullptr, const std::string& trace_dir = "");

/// Factory for Stereo-SVO.
std::shared_ptr<FrameHandlerStereo> makeStereo(
    rclcpp::Node& pnh,
    const CameraBundlePtr& cam = nullptr, const std::string& trace_dir = "");

/// Factory for Camera-Array-SVO.
std::shared_ptr<FrameHandlerArray> makeArray(
    rclcpp::Node& pnh,
    const CameraBundlePtr& cam = nullptr, const std::string& trace_dir = "");

/// Factory for Camera-Array-SVO
std::shared_ptr<FrameHandlerDenseMono> makeDenseMono(
    rclcpp::Node& pnh);

} // namespace factory
} // namespace mono
