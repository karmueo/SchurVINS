// Synchronous EGO replay: bounded input queues, no DDS transport in the estimator path.
#include <svo_ros/ego_processing.h>
#include <svo_ros/input_scheduler.h>
#include <svo_ros/offline_timing.h>
#include <svo_ros/svo_interface.h>
#include <svo_ros/visualizer.h>
#include <svo/frame_handler_base.h>
#include <svo/common/frame.h>
#include <svo/imu_handler.h>
#include <vikit/params_helper.h>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <map>
#include <thread>
#include <sys/resource.h>

namespace svo_ros {
namespace fs = std::filesystem;
namespace {
using BagMessage = rosbag2_storage::SerializedBagMessage;
using Imu = sensor_msgs::msg::Imu;
struct ImageRecord
{
  sensor_msgs::msg::CompressedImage image;
  int64_t record_stamp;
};
using Scheduler = InputScheduler<std::shared_ptr<ImageRecord>, Imu::SharedPtr>;

template<class Message>
Message deserialize(const BagMessage& record)
{
  rclcpp::SerializedMessage serialized(*record.serialized_data);
  Message message;
  rclcpp::Serialization<Message>().deserialize_message(&serialized, &message);
  return message;
}

std::string jsonString(const std::string& value)
{
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : value)
  {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
    else out << c;
  }
  out << '"';
  return out.str();
}
}  // namespace

class EgoBagRunner
{
public:
  explicit EgoBagRunner(rclcpp::Node& node)
    : node_(node), input_(vk::param<std::string>(node, "inbag", "")),
      output_(vk::param<std::string>(node, "output_dir", "")),
      schedule_(vk::param<bool>(node, "realtime", false)), scheduler_(2, true, 64, 10000)
  {
    if (input_.extension() != ".mcap" || !fs::is_regular_file(input_))
      throw std::invalid_argument("inbag must be an existing .mcap file");
    if (output_.empty()) throw std::invalid_argument("output_dir is required");
    const auto stem = "schurvins_" + input_.stem().string();
    bag_path_ = output_ / (stem + ".mcap");
    staging_ = output_ / ("." + stem + ".bag");
  }

  int run()
  {
    const auto started = std::chrono::steady_clock::now();
    bool complete = false;
    std::string error;
    try
    {
      prepare();
      while (rclcpp::ok() && reader_.has_next())
      {
        auto record = reader_.read_next();
        const auto target = schedule_.target(record->recv_timestamp, std::chrono::steady_clock::now());
        while (rclcpp::ok() && std::chrono::steady_clock::now() < target)
          std::this_thread::sleep_until(std::min(target, std::chrono::steady_clock::now()+std::chrono::milliseconds(10)));
        if (!rclcpp::ok()) break;
        // Original CDR and both log/publish timestamps are copied without type loading.
        writer_->write(record);
        ++copied_;
        rosgraph_msgs::msg::Clock clock;
        clock.clock = rclcpp::Time(record->recv_timestamp);
        clock_->publish(clock);
        if (record->topic_name == "/stereo/image_raw")
        {
          auto image = std::make_shared<ImageRecord>();
          image->image = deserialize<sensor_msgs::msg::CompressedImage>(*record);
          image->record_stamp = record->recv_timestamp;
          const auto stamp = rclcpp::Time(image->image.header.stamp).nanoseconds();
          scheduler_.image(0, stamp, image);
          scheduler_.image(1, stamp, image);
        }
        else if (record->topic_name == "/imu")
        {
          auto imu = std::make_shared<Imu>(processing_->correct(deserialize<Imu>(*record)));
          const auto stamp = rclcpp::Time(imu->header.stamp).nanoseconds();
          scheduler_.imu(stamp, std::move(imu));
        }
        pump(false);
      }
      if (!rclcpp::ok()) throw std::runtime_error("MCAP processing interrupted");
      pump(true);
      if (scheduler_.pendingImages() || scheduler_.pendingImu())
        throw std::runtime_error("Unpaired images or missing final IMU coverage at EOF");
      if (!processed_ || !poses_) throw std::runtime_error("MCAP processing never entered tracking");
      complete = true;
    }
    catch (const std::exception& exception)
    {
      error = exception.what();
      RCLCPP_ERROR(node_.get_logger(), "EGO MCAP processing failed: %s", error.c_str());
    }
    // Close streams and the MCAP footer before advertising completion or exiting launch.
    try
    {
      if (interface_ && interface_->visualizer_->save_traj_.is_open())
      {
        interface_->visualizer_->save_traj_.flush();
        if (!interface_->visualizer_->save_traj_) throw std::runtime_error("Cannot flush trajectory");
        interface_->visualizer_->save_traj_.close();
      }
      frames_.flush();
      if (frames_.is_open() && !frames_) throw std::runtime_error("Cannot flush frame statistics");
      if (writer_)
      {
        writer_->close(); writer_.reset();
        fs::path mcap;
        for (const auto& entry : fs::directory_iterator(staging_))
          if (entry.path().extension() == ".mcap")
          {
            if (!mcap.empty()) throw std::runtime_error("Unexpected split MCAP output");
            mcap = entry.path();
          }
        if (mcap.empty()) throw std::runtime_error("MCAP output is missing");
        if (fs::exists(bag_path_)) throw std::runtime_error("Refusing to overwrite " + bag_path_.string());
        fs::rename(mcap, bag_path_);
        fs::remove_all(staging_);
      }
    }
    catch (const std::exception& exception)
    {
      complete = false; error += std::string(error.empty() ? "" : "; ") + exception.what();
      RCLCPP_ERROR(node_.get_logger(), "Cannot finalize output: %s", exception.what());
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    if (prepared_)
    {
      try { writeSummary(complete, error, wall); }
      catch (const std::exception& exception)
      { complete = false; RCLCPP_ERROR(node_.get_logger(), "%s", exception.what()); }
    }
    std::cout << "Average VIO FPS (excluding IO, decode, publishing and waits): "
              << std::fixed << std::setprecision(3) << computation_.fps(processed_)
              << " (frames=" << processed_ << ", compute_seconds=" << computation_.seconds()
              << ", total_seconds=" << wall << ", complete=" << (complete ? "true" : "false") << ")\n";
    return complete ? 0 : 1;
  }

private:
  void prepare()
  {
    fs::create_directories(output_);
    for (const auto& path : {bag_path_, staging_, output_/"summary.json", output_/"frames.csv", output_/"effective_config.yaml"})
      if (fs::exists(path)) throw std::runtime_error("Refusing to overwrite " + path.string());
    rosbag2_storage::StorageOptions options;
    options.uri = input_.string(); options.storage_id = "mcap";
    reader_.open(options);
    if (!reader_.set_read_order({rosbag2_storage::ReadOrder::ReceivedTimestamp, false}))
      throw std::runtime_error("MCAP reader cannot order messages by recording time");
    const auto topics = reader_.get_all_topics_and_types();
    bool images = false, imu = false;
    for (const auto& topic : topics)
    {
      if (topic.name == "/svo/odometry" || topic.name == "/svo/path_imu")
        throw std::runtime_error("Input already contains reserved VIO output topic: " + topic.name);
      if (topic.name == "/stereo/image_raw")
        images = topic.type == "sensor_msgs/msg/CompressedImage" && topic.serialization_format == "cdr";
      if (topic.name == "/imu") imu = topic.type == "sensor_msgs/msg/Imu" && topic.serialization_format == "cdr";
    }
    if (!images || !imu) throw std::runtime_error("MCAP requires CDR /stereo/image_raw (CompressedImage) and /imu (Imu)");
    processing_ = std::make_unique<EgoProcessing>(vk::param<std::string>(node_, "cameras", ""));
    prepared_ = true;
    interface_ = std::make_unique<svo::SvoInterface>(svo::PipelineType::kStereo, node_, node_, output_.string());
    if (!interface_->imu_handler_) throw std::runtime_error("EGO MCAP requires use_imu=true");
    frames_.open(output_/"frames.csv");
    if (!frames_) throw std::runtime_error("Cannot open frame statistics");
    frames_ << "timestamp_ns,status,stage,processing_ms,pose_available\n";
    saveConfiguration();
    clock_ = node_.create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    std::vector<rosbag2_storage::MessageDefinition> definitions;
    reader_.get_all_message_definitions(definitions);
    std::map<std::string, rosbag2_storage::MessageDefinition> schemas;
    for (const auto& definition : definitions) schemas.emplace(definition.topic_type, definition);
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    options.uri = staging_.string();
    writer_->open(options);
    for (const auto& topic : topics)
    {
      const auto schema = schemas.find(topic.type);
      writer_->create_topic(topic, schema == schemas.end() ?
        rosbag2_storage::MessageDefinition::empty_message_definition_for(topic.type) : schema->second);
    }
    rosbag2_storage::TopicMetadata odom, path;
    odom.name = "/svo/odometry"; odom.type = "nav_msgs/msg/Odometry"; odom.serialization_format = "cdr";
    path.name = "/svo/path_imu"; path.type = "nav_msgs/msg/Path"; path.serialization_format = "cdr";
    writer_->create_topic(odom); writer_->create_topic(path);
    interface_->visualizer_->pose_sink_ = [this](const auto& odometry, const auto& path_message) {
      writeResult(odometry, "/svo/odometry"); writeResult(path_message, "/svo/path_imu");
    };
    RCLCPP_INFO(node_.get_logger(), "EGO input: %s; output: %s", input_.c_str(), bag_path_.c_str());
  }

  template<class Message>
  void writeResult(const Message& message, const std::string& topic)
  {
    auto serialized = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<Message>().serialize_message(&message, serialized.get());
    auto record = std::make_shared<BagMessage>();
    const auto data = serialized->release_rcl_serialized_message();
    record->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(new rcutils_uint8_array_t(data), [](auto* bytes) {
      const auto result = rcutils_uint8_array_fini(bytes);
      (void)result;
      delete bytes;
    });
    record->topic_name = topic;
    record->recv_timestamp = current_record_stamp_;
    record->send_timestamp = rclcpp::Time(message.header.stamp).nanoseconds();
    writer_->write(record);
  }

  void pump(bool eof)
  {
    while (rclcpp::ok())
    {
      if (scheduler_.stopped()) throw std::runtime_error("EGO input queue overflow or timestamp rollback");
      Imu::SharedPtr imu;
      const bool drain = waiting_for_prior_ || (eof && scheduler_.pendingImages() == 0);
      if (scheduler_.takeImu(imu, drain))
      {
        computation_.measure([&] {
          const auto& w = imu->angular_velocity; const auto& a = imu->linear_acceleration;
          interface_->imu_handler_->addImuMeasurement(svo::ImuMeasurement(
            rclcpp::Time(imu->header.stamp).nanoseconds()*1e-9,
            Eigen::Vector3d(w.x, w.y, w.z), Eigen::Vector3d(a.x, a.y, a.z)));
        });
        waiting_for_prior_ = false;
        continue;
      }
      if (!scheduler_.ready()) return;
      if (!scheduler_.leadingNext() && !computation_.measure([&] { return interface_->setImuPrior(scheduler_.nextStamp()); }))
      {
        waiting_for_prior_ = true;
        if (eof && !scheduler_.hasImu()) throw std::runtime_error("IMU initialization prior unavailable at EOF");
        if (!scheduler_.hasImu()) return;
        continue;
      }
      Scheduler::Bundle bundle; bool leading = false;
      if (!scheduler_.takeImage(bundle, leading)) return;
      const auto decoded = processing_->decode(bundle.images[0]->image);
      ++decoded_;
      if (leading)
      {
        frames_ << bundle.stamp << ",leading_imu_coverage,-1,0,0\n";
        continue;
      }
      std::vector<cv::Mat> images{decoded[0], decoded[1]};
      const double before = computation_.seconds();
      computation_.measure([&] { interface_->processImageBundle(images, bundle.stamp); });
      const double elapsed_ms = (computation_.seconds()-before)*1000;
      const auto stage = interface_->svo_->stage();
      const bool pose = stage == svo::Stage::kTracking;
      if (pose && !interface_->svo_->getLastFrames()->get_T_W_B().getTransformationMatrix().allFinite())
        throw std::runtime_error("Non-finite estimated pose");
      if (first_processed_stamp_ < 0) first_processed_stamp_ = bundle.stamp;
      if (pose)
      {
        if (first_tracking_stamp_ < 0) first_tracking_stamp_ = bundle.stamp;
        ++poses_;
      }
      if (previous_tracking_ && !pose) ++tracking_lost_;
      previous_tracking_ = pose;
      ++processed_;
      current_record_stamp_ = bundle.images[0]->record_stamp;
      interface_->publishResults(images, bundle.stamp);
      if (stage == svo::Stage::kPaused && interface_->automatic_reinitialization_)
      { computation_.measure([&] { interface_->svo_->start(); }); ++resets_; }
      frames_ << bundle.stamp << ",processed," << static_cast<int>(stage) << ',' << elapsed_ms << ',' << pose << '\n';
      rclcpp::spin_some(node_.get_node_base_interface());
    }
  }

  void saveConfiguration()
  {
    YAML::Node root;
    auto parameters = root[node_.get_fully_qualified_name()]["ros__parameters"];
    for (const auto& name : node_.list_parameters({}, 100).names)
    {
      const auto parameter = node_.get_parameter(name);
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) parameters[name] = parameter.as_string();
      else parameters[name] = YAML::Load(parameter.value_to_string());
    }
    std::ofstream config(output_/"effective_config.yaml");
    config << root;
    if (!config) throw std::runtime_error("Cannot write effective configuration");
  }

  void writeSummary(bool complete, const std::string& error, double wall)
  {
    const auto& c = scheduler_.counters;
    struct rusage usage {}; getrusage(RUSAGE_SELF, &usage);
    std::ofstream out(output_/"summary.json");
    out << std::setprecision(17) << "{\"complete\":" << (complete ? "true" : "false")
        << ",\"error\":" << jsonString(error) << ",\"input\":" << jsonString(input_.string())
        << ",\"realtime\":" << (node_.get_parameter("realtime").as_bool() ? "true" : "false")
        << ",\"cameras\":" << jsonString(node_.get_parameter("cameras").as_string())
        << ",\"output_bag\":" << jsonString(bag_path_.string()) << ",\"start_offset_seconds\":0"
        << ",\"copied_messages\":" << copied_ << ",\"decoded_images\":" << decoded_
        << ",\"image_messages\":" << c.image_messages << ",\"image_groups\":" << c.image_groups
        << ",\"imu_received\":" << c.imu_received << ",\"imu_consumed\":" << c.imu_consumed
        << ",\"leading_skipped\":" << c.leading_skipped << ",\"overflow\":" << c.overflow
        << ",\"duplicates\":" << c.duplicates << ",\"time_jumps\":" << c.time_jumps
        << ",\"pending_images\":" << scheduler_.pendingImages() << ",\"pending_imu\":" << scheduler_.pendingImu()
        << ",\"processed\":" << processed_ << ",\"poses\":" << poses_ << ",\"tracking_lost\":" << tracking_lost_
        << ",\"resets\":" << resets_ << ",\"trajectory_coverage\":" << (processed_ ? double(poses_)/processed_ : 0)
        << ",\"initialization_seconds\":" << (first_tracking_stamp_ < 0 ? -1.0 : (first_tracking_stamp_-first_processed_stamp_)*1e-9)
        << ",\"compute_seconds\":" << computation_.seconds() << ",\"average_fps\":" << computation_.fps(processed_)
        << ",\"elapsed_seconds\":" << wall << ",\"peak_rss_kb\":" << usage.ru_maxrss << "}\n";
    out.flush();
    if (!out) throw std::runtime_error("Cannot write MCAP run summary");
  }

  rclcpp::Node& node_;
  fs::path input_, output_, bag_path_, staging_;
  ReplaySchedule schedule_;
  ComputationTimer computation_;
  Scheduler scheduler_;
  rosbag2_cpp::Reader reader_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::unique_ptr<EgoProcessing> processing_;
  std::unique_ptr<svo::SvoInterface> interface_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  std::ofstream frames_;
  bool waiting_for_prior_ = false, previous_tracking_ = false, prepared_ = false;
  int64_t current_record_stamp_ = 0, first_processed_stamp_ = -1, first_tracking_stamp_ = -1;
  uint64_t copied_ = 0, decoded_ = 0, processed_ = 0, poses_ = 0, tracking_lost_ = 0, resets_ = 0;
};
}  // namespace svo_ros

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  int result = 1;
  try
  {
    auto node = std::make_shared<rclcpp::Node>("svo");
    svo_ros::EgoBagRunner runner(*node);
    result = runner.run();
  }
  catch (const std::exception& error) { std::cerr << "EGO MCAP: " << error.what() << '\n'; }
  rclcpp::shutdown();
  return result;
}
