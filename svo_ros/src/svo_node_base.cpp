// Native ROS2 runtime. Estimator access is confined to worker_.
#include <svo_ros/svo_node_base.h>
#include <svo_ros/visualizer.h>
#include <svo/frame_handler_base.h>
#include <svo/common/frame.h>
#include <svo/imu_handler.h>
#include <vikit/params_helper.h>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <sys/resource.h>
#include <yaml-cpp/yaml.h>

namespace svo_ros {
SvoNodeBase::SvoNodeBase(const rclcpp::NodeOptions& options) : Node("svo", options)
{
  const auto pipeline = vk::param<std::string>(*this, "pipeline", "stereo");
  if (pipeline == "mono") { type_ = svo::PipelineType::kMono; cameras_ = 1; }
  else if (pipeline == "stereo") { type_ = svo::PipelineType::kStereo; cameras_ = 2; }
  else if (pipeline == "array") { type_ = svo::PipelineType::kArray; cameras_ = 4; }
  else throw std::invalid_argument("pipeline must be mono, stereo or array");
#ifdef USE_SCHUR_VINS
  if (cameras_ != 2) throw std::invalid_argument("mono/array require USE_SCHUR_VINS=OFF");
#endif
  use_imu_ = vk::param<bool>(*this, "use_imu", true);
  output_root_ = vk::param<std::string>(*this, "output_dir", "/tmp/schurvins-results");
  createEstimator();
  image_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  imu_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions image_options, imu_options, control_options;
  image_options.callback_group = image_group_;
  imu_options.callback_group = imu_group_;
  control_options.callback_group = control_group_;
  auto image_qos = rclcpp::QoS(rclcpp::KeepLast(64)).reliable();
  for (size_t camera = 0; camera < cameras_; ++camera)
  {
    const auto topic = vk::param<std::string>(*this, "cam" + std::to_string(camera) + "_topic",
                                             "/cam" + std::to_string(camera) + "/image_raw");
    images_.push_back(create_subscription<sensor_msgs::msg::Image>(topic, image_qos,
      [this, camera](Image msg) { image(camera, std::move(msg)); }, image_options));
  }
  if (use_imu_)
    imu_ = create_subscription<sensor_msgs::msg::Imu>(vk::param<std::string>(*this, "imu_topic", "/imu"),
      rclcpp::QoS(rclcpp::KeepLast(10000)).reliable(),
      [this](Imu msg) { imu(std::move(msg)); }, imu_options);
  control_ = create_subscription<std_msgs::msg::String>(
    vk::param<std::string>(*this, "remote_key_topic", "/svo/remote_key"), 10,
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      if (msg->data.empty()) return;
      std::lock_guard<std::mutex> lock(mutex_);
      commands_.push_back(msg->data.front()); changed_.notify_one();
    }, control_options);
  status_ = create_publisher<std_msgs::msg::String>("/svo/runtime_status", rclcpp::QoS(1).transient_local());
  status_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { publishStatus(); });
  saveConfiguration();
  worker_ = std::thread(&SvoNodeBase::worker, this);
}
SvoNodeBase::~SvoNodeBase()
{
  { std::lock_guard<std::mutex> lock(mutex_); draining_ = true; idle_ = false; }
  changed_.notify_all();
  if (worker_.joinable()) worker_.join();
}
void SvoNodeBase::createEstimator()
{
  const auto image_limit = vk::param<int>(*this, "image_queue_capacity", 64);
  const auto imu_limit = vk::param<int>(*this, "imu_queue_capacity", 10000);
  if (image_limit < 1 || imu_limit < 1) throw std::invalid_argument("Queue capacities must be positive");
  scheduler_ = std::make_unique<Scheduler>(cameras_, use_imu_, image_limit, imu_limit);
  std::filesystem::create_directories(output_root_);
  const auto id = std::chrono::system_clock::now().time_since_epoch().count();
  run_dir_ = (std::filesystem::path(output_root_) / ("run_" + std::to_string(id))).string();
  if (!std::filesystem::create_directory(run_dir_)) throw std::runtime_error("Run directory exists");
  interface_ = std::make_unique<svo::SvoInterface>(type_, *this, *this, run_dir_);
  frames_.open(run_dir_ + "/frames.csv");
  if (!frames_) throw std::runtime_error("Cannot open frame statistics");
  frames_ << "timestamp_ns,status,stage,processing_ms,pose_available\n";
  invalid_images_ = invalid_imu_ = idle_images_ = idle_imu_ = 0;
  processed_ = poses_ = tracking_lost_ = 0;
  initialization_seconds_ = -1.0; first_processed_stamp_ = -1;
  previous_stage_ = -1; durations_ms_.clear(); start_ = std::chrono::steady_clock::now();
}
void SvoNodeBase::saveConfiguration()
{
  YAML::Node root;
  auto parameters = root[get_fully_qualified_name()]["ros__parameters"];
  for (const auto& name : list_parameters({}, 100).names)
  {
    const auto parameter = get_parameter(name);
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      parameters[name] = parameter.as_string();
    else parameters[name] = YAML::Load(parameter.value_to_string());
  }
  parameters["trace_dir"] = run_dir_;
  std::ofstream config(run_dir_ + "/effective_config.yaml");
  if (!config) throw std::runtime_error("Cannot open effective configuration");
  config << root;

}
void SvoNodeBase::image(size_t camera, Image msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (quit_ || failed_ || draining_) return;
  if (idle_) { ++idle_images_; return; }
  if (!msg || msg->header.stamp.sec < 0 || msg->width == 0 || msg->height == 0 || msg->data.empty())
  {
    ++invalid_images_; ++scheduler_->counters.image_messages;
    failed_ = true; error_ = "invalid image input"; changed_.notify_one(); return;
  }
  const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
  scheduler_->image(camera, stamp, std::move(msg));
  changed_.notify_one();
}
void SvoNodeBase::imu(Imu msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (quit_ || failed_ || draining_) return;
  if (idle_) { ++idle_imu_; return; }
  bool valid = msg && msg->header.stamp.sec >= 0;
  if (valid)
  {
    const auto& a = msg->linear_acceleration; const auto& w = msg->angular_velocity;
    valid = std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z) &&
            std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.z);
  }
  if (!valid)
  {
    ++invalid_imu_; ++scheduler_->counters.imu_received;
    failed_ = true; error_ = "invalid IMU input"; changed_.notify_one(); return;
  }
  const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
  scheduler_->imu(stamp, std::move(msg));
  changed_.notify_one();
}
std::string SvoNodeBase::statusLocked() const
{
  const auto& c = scheduler_->counters;
  size_t matched_inputs = 0;
  for (const auto& input : images_) if (input->get_publisher_count()) ++matched_inputs;
  if (imu_ && imu_->get_publisher_count()) ++matched_inputs;
  std::ostringstream out;
  out << "{\"image_messages\":" << c.image_messages << ",\"image_groups\":" << c.image_groups
      << ",\"imu_received\":" << c.imu_received << ",\"imu_consumed\":" << c.imu_consumed
      << ",\"matched_inputs\":" << matched_inputs
      << ",\"processed\":" << processed_ << ",\"poses\":" << poses_
      << ",\"leading_skipped\":" << c.leading_skipped << ",\"overflow\":" << c.overflow
      << ",\"time_jumps\":" << c.time_jumps << ",\"duplicates\":" << c.duplicates
      << ",\"pending_images\":" << scheduler_->pendingImages() << ",\"pending_imu\":" << scheduler_->pendingImu()
      << ",\"awaiting_images\":" << (scheduler_->pendingImages() == 0 && scheduler_->pendingImu() != 0 ? "true" : "false")
      << ",\"invalid_images\":" << invalid_images_ << ",\"invalid_imu\":" << invalid_imu_
      << ",\"idle_images\":" << idle_images_ << ",\"idle_imu\":" << idle_imu_
      << ",\"idle\":" << (idle_ ? "true" : "false")
      << ",\"inflight\":" << (inflight_ ? "true" : "false")
      << ",\"initialization_seconds\":" << initialization_seconds_
      << ",\"resets\":" << resets_ << ",\"tracking_lost\":" << tracking_lost_
      << ",\"stopped\":" << (quit_ || failed_ ? "true" : "false")
      << ",\"failed\":" << (failed_ ? "true" : "false") << ",\"error\":\"" << error_
      << "\",\"run_dir\":\"" << run_dir_ << "\"}";
  return out.str();
}
void SvoNodeBase::publishStatus()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std_msgs::msg::String msg; msg.data = statusLocked(); status_->publish(msg);
}
void SvoNodeBase::closeRun()
{
  frames_.flush();
  std::ofstream summary(run_dir_ + "/summary.json");
  auto sorted = durations_ms_; std::sort(sorted.begin(), sorted.end());
  auto quantile = [&](double q) { return sorted.empty() ? 0.0 : sorted[static_cast<size_t>(q*(sorted.size()-1))]; };
  struct rusage usage {}; getrusage(RUSAGE_SELF, &usage);
  auto status = statusLocked(); status.pop_back();
  summary << status << ",\"processing_ms_p50\":" << quantile(.5) << ",\"processing_ms_p95\":" << quantile(.95)
          << ",\"processing_ms_p99\":" << quantile(.99) << ",\"elapsed_seconds\":"
          << std::chrono::duration<double>(std::chrono::steady_clock::now()-start_).count()
          << ",\"peak_rss_kb\":" << usage.ru_maxrss << "}\n";
}
void SvoNodeBase::worker()
{
  std::unique_lock<std::mutex> lock(mutex_);
  bool waiting_for_prior = false;
  try
  {
    while (!quit_)
    {
      changed_.wait(lock, [this, &waiting_for_prior] { return quit_ || failed_ || draining_ || scheduler_->stopped() || !commands_.empty() || scheduler_->canTakeImu(waiting_for_prior || (draining_ && scheduler_->pendingImages() == 0)) || (!idle_ && !waiting_for_prior && scheduler_->ready()); });
      if (quit_ || failed_) break;
      if (scheduler_->stopped())
      {
        failed_ = true;
        error_ = scheduler_->counters.time_jumps ? "timestamp rollback: restart required" : "queue overflow: replay more slowly";
        RCLCPP_ERROR(get_logger(), "%s", error_.c_str()); break;
      }
      if (!commands_.empty())
      {
        const char command = commands_.front(); commands_.pop_front();
        if (command == 'q') { draining_ = true; idle_ = false; continue; }
        if (command == 'r')
        {
          closeRun(); frames_.close(); interface_.reset(); ++resets_; createEstimator(); saveConfiguration(); waiting_for_prior = false; idle_ = true;
        }
        else if (command == 's') { interface_->svo_->start(); idle_ = false; }
        else if (command == 'c' || command == 'C') interface_->svo_->setCompensation(command == 'c');
        continue;
      }
      if (draining_ && !scheduler_->canTakeImu(waiting_for_prior || scheduler_->pendingImages() == 0) && !scheduler_->ready())
      {
        if (scheduler_->pendingImages())
        {
          failed_ = true; error_ = "unpaired images or missing final IMU coverage at exit";
        }
        quit_ = true; break;
      }
      Imu imu_msg;
      // Feed only through the next camera timestamp plus its first covering sample.
      // The IMU handler retains the samples needed by the estimator.
      if (scheduler_->takeImu(imu_msg, waiting_for_prior || (draining_ && scheduler_->pendingImages() == 0)))
      {
        lock.unlock();
        const auto& w = imu_msg->angular_velocity; const auto& a = imu_msg->linear_acceleration;
        interface_->imu_handler_->addImuMeasurement(svo::ImuMeasurement(
          rclcpp::Time(imu_msg->header.stamp).seconds(), Eigen::Vector3d(w.x,w.y,w.z), Eigen::Vector3d(a.x,a.y,a.z)));
        lock.lock();
        waiting_for_prior = false;
      }
      if (!idle_ && scheduler_->ready() && !scheduler_->leadingNext() &&
          !interface_->setImuPrior(scheduler_->nextStamp()))
      {
        waiting_for_prior = true;
        if (draining_ && !scheduler_->hasImu())
        { failed_ = true; error_ = "IMU initialization prior unavailable at exit"; break; }
        continue;
      }
      Scheduler::Bundle bundle; bool leading = false;
      if (idle_ || !scheduler_->takeImage(bundle, leading)) continue;
      if (leading) { frames_ << bundle.stamp << ",leading_imu_coverage,-1,0,0\n"; continue; }
      inflight_ = true;
      lock.unlock();
      const auto started = std::chrono::steady_clock::now();
      std::vector<cv::Mat> images;
      for (const auto& msg : bundle.images) images.push_back(cv_bridge::toCvCopy(msg, "mono8")->image);
      interface_->processImageBundle(images, bundle.stamp);
      const int stage = static_cast<int>(interface_->svo_->stage());
      bool pose = interface_->svo_->stage() == svo::Stage::kTracking;
      if (pose)
      {
        const auto transform = interface_->svo_->getLastFrames()->get_T_W_B();
        if (!transform.getPosition().allFinite() || !transform.getRotation().toImplementation().coeffs().allFinite())
          throw std::runtime_error("Non-finite estimated pose");
      }
      interface_->publishResults(images, bundle.stamp);
      const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-started).count();
      if (interface_->svo_->stage() == svo::Stage::kPaused && interface_->automatic_reinitialization_) interface_->svo_->start();
      lock.lock();
      inflight_ = false;
      if (first_processed_stamp_ < 0) first_processed_stamp_ = bundle.stamp;
      ++processed_; if (pose) { ++poses_; if (initialization_seconds_ < 0) initialization_seconds_ = (bundle.stamp-first_processed_stamp_)*1e-9; }
      if (previous_stage_ == static_cast<int>(svo::Stage::kTracking) && stage != previous_stage_) ++tracking_lost_;
      previous_stage_ = stage; durations_ms_.push_back(elapsed);
      frames_ << bundle.stamp << ',' << "processed" << ',' << stage << ',' << elapsed << ',' << pose << '\n';
    }
  }
  catch (const std::exception& error)
  {
    if (!lock.owns_lock()) lock.lock();
    inflight_ = false; failed_ = true; error_ = "estimator exception";
    RCLCPP_ERROR(get_logger(), "Runtime stopped: %s", error.what());
  }
  closeRun();
}
} // namespace svo_ros
RCLCPP_COMPONENTS_REGISTER_NODE(svo_ros::SvoNodeBase)
