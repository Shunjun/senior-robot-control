#include "native_capture.hpp"
#include "snapshot.hpp"
#include "staged_graph.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "rclcpp/rclcpp.hpp"
#include "senior_slam_bridge/srv/capture_graph.hpp"
#include "senior_slam_bridge/srv/read_graph.hpp"
#include "senior_slam_bridge/srv/read_loaded_graph.hpp"
//Compatible: the Humble slam_toolbox 2.6.10 subclass exposes the mapper and its mutex.
#include "slam_toolbox/slam_toolbox_async.hpp"
#include "slam_toolbox/serialization.hpp"

namespace senior_slam_bridge
{
class SlamGraphNode final : public slam_toolbox::AsynchronousSlamToolbox
{
public:
  explicit SlamGraphNode(const rclcpp::NodeOptions & options)
  : AsynchronousSlamToolbox(options) {}

  ~SlamGraphNode() override
  {
    if (capture_directory_) {
      //Compatible: join Humble's dedicated TF listener before retiring the ingress lock.
      tfL_.reset();
      scan_filter_.reset();
      scan_filter_sub_.reset();
    }
  }

  void start()
  {
    const auto prefix = declare_parameter<std::string>("candidate_graph_prefix", "");
    const auto digest = declare_parameter<std::string>("candidate_graph_digest", "");
    const auto attempt = declare_parameter<std::string>("candidate_load_attempt", "");
    const auto capture_root = declare_parameter<std::string>("capture_directory", "");
    if (prefix.empty() && digest.empty() && attempt.empty()) {
      if (!capture_root.empty()) {
        prepare_owned_capture(capture_root);
      }
      configure();
      if (capture_directory_) {
        start_owned_capture();
      }
    } else {
      if (!capture_root.empty()) {
        throw std::invalid_argument("Candidate load and owned capture are separate modes");
      }
      load_candidate(prefix, digest, attempt);
    }
    start_graph_service();
  }

private:
  using CaptureGraph = srv::CaptureGraph;
  using ReadGraph = srv::ReadGraph;
  using ReadLoadedGraph = srv::ReadLoadedGraph;

  void prepare_owned_capture(const std::string & root)
  {
    const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
    const auto saver = overrides.find("use_map_saver");
    const auto interactive = overrides.find("enable_interactive_mode");
    if (saver == overrides.end() || saver->second.get<bool>() ||
      (interactive != overrides.end() && interactive->second.get<bool>()))
    {
      throw std::invalid_argument("Owned capture requires use_map_saver=false and interactive off");
    }
    capture_directory_ = std::make_unique<CaptureDirectory>(root);
  }

  void start_owned_capture()
  {
    ssDesserialize_.reset();
    ssSerialize_.reset();
    ssPauseMeasurements_.reset();
    capture_graph_ = create_service<CaptureGraph>(
      "/senior/slam/capture_graph",
      [this](const CaptureGraph::Request::SharedPtr, CaptureGraph::Response::SharedPtr response) {
        response->boot_id = boot_id_;
        response->snapshot_sequence = ++snapshot_sequence_;
        response->map_frame = map_frame_;
        response->odom_frame = odom_frame_;
        response->status = CaptureGraph::Response::UNAVAILABLE;
        try {
          capture_locked_observation(*response);
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "Graph capture failed: %s", error.what());
        } catch (...) {
          RCLCPP_WARN(get_logger(), "Native graph capture failed");
        }
      });
  }

  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override
  {
    if (!capture_directory_) {
      AsynchronousSlamToolbox::laserCallback(std::move(scan));
      return;
    }
    //Compatible: Humble's TF filter can dispatch off the executor, and getLaser()
    // registers a new sensor in the dataset before addScan acquires smapper_mutex_.
    const std::lock_guard<std::mutex> ingress(scan_ingress_mutex_);
    AsynchronousSlamToolbox::laserCallback(std::move(scan));
  }

  void capture_locked_observation(CaptureGraph::Response & response)
  {
    capture_directory_->require_free_slot();
    const auto limit = response.keyframes.max_size();
    std::vector<Keyframe> frames;
    std::vector<char> graph_bytes;
    std::vector<char> dataset_bytes;
    {
      const std::lock_guard<std::mutex> ingress(scan_ingress_mutex_);
      boost::mutex::scoped_lock lock(smapper_mutex_);
      response.sample_stamp = now();
      auto * mapper = smapper_ ? smapper_->getMapper() : nullptr;
      auto copied = copy_keyframes(mapper, limit);
      if (!copied) {
        return;
      }
      frames = std::move(*copied);
      switch (validate_snapshot(frames, limit)) {
        case SnapshotStatus::empty:
          response.status = CaptureGraph::Response::EMPTY;
          return;
        case SnapshotStatus::unavailable:
          return;
        case SnapshotStatus::ok:
          break;
      }
      graph_bytes = capture_native_archive(
        [mapper](const std::string & path) {mapper->SaveToFile(path);});
      dataset_bytes = capture_native_archive(
        [this](const std::string & path) {dataset_->SaveToFile(path);});
    }
    // Build the response before reserving the single output slot.
    decltype(response.keyframes) keyframes;
    fill_keyframes(frames, keyframes);
    auto artifact = capture_directory_->write_pair(graph_bytes, dataset_bytes);
    response.keyframes = std::move(keyframes);
    response.relative_prefix = std::move(artifact.relative_prefix);
    response.pair_digest = std::move(artifact.pair_digest);
    response.status = CaptureGraph::Response::OK;
  }

  void load_candidate(
    const std::string & prefix, const std::string & digest, const std::string & attempt)
  {
    if (attempt.empty() || attempt.size() > 64 ||
      attempt.find_first_not_of(
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_") != std::string::npos)
    {
      throw std::runtime_error("Expected a bounded candidate attempt ID");
    }

    const StagedGraph staged(prefix, digest);
    auto mapper = std::make_unique<karto::Mapper>();
    auto dataset = std::make_unique<karto::Dataset>();
    //Compatible: use the native Humble 2.6.10 decoder with a private, verified pair.
    if (!serialization::read(staged.prefix().string(), *mapper, *dataset, shared_from_this())) {
      throw std::runtime_error("Could not decode a native graph and laser dataset");
    }
    //Compatible: Humble restores the Dataset lookup during decoding, but Clear() expects
    // those sensors to be registered in the process-wide manager before it unregisters them.
    bool has_laser = false;
    for (auto * object : dataset->GetLasers()) {
      if (auto * sensor = dynamic_cast<karto::Sensor *>(object)) {
        karto::SensorManager::GetInstance()->RegisterSensor(sensor, true);
        has_laser = has_laser || dynamic_cast<karto::LaserRangeFinder *>(sensor) != nullptr;
      }
    }
    if (!mapper->GetGraph() || !has_laser) {
      throw std::runtime_error("Decoded candidate requires a graph and laser sensor");
    }

    // Candidate mode deliberately creates no scan, TF, solver or mutation interfaces.
    // Startup is single threaded and the read services are created after installation.
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    if (map_frame_.empty() || odom_frame_.empty() || map_frame_ == odom_frame_) {
      throw std::runtime_error("Expected distinct nonempty candidate frames");
    }
    {
      boost::mutex::scoped_lock lock(smapper_mutex_);
      smapper_->setMapper(mapper.release());
      dataset_ = std::move(dataset);
    }
    const auto limit = ReadGraph::Response{}.keyframes.max_size();
    rclcpp::Time sample_stamp;
    auto frames = copy_graph(limit, sample_stamp);
    if (!frames || validate_snapshot(*frames, limit) != SnapshotStatus::ok) {
      throw std::runtime_error("Installed candidate graph is invalid or empty");
    }

    candidate_receipt_ = create_service<ReadLoadedGraph>(
      "/senior/slam/read_loaded_graph",
      [this, attempt, loaded_digest = staged.digest()](
        const ReadLoadedGraph::Request::SharedPtr, ReadLoadedGraph::Response::SharedPtr response)
      {
        response->boot_id = boot_id_;
        response->attempt_id = attempt;
        response->pair_digest = loaded_digest;
      });
  }

  void start_graph_service()
  {
    read_graph_ = create_service<ReadGraph>(
      "/senior/slam/read_graph",
      [this](const ReadGraph::Request::SharedPtr, ReadGraph::Response::SharedPtr response) {
        response->boot_id = boot_id_;
        response->snapshot_sequence = ++snapshot_sequence_;
        response->map_frame = map_frame_;
        response->odom_frame = odom_frame_;

        const auto limit = response->keyframes.max_size();
        rclcpp::Time sample_stamp;
        auto frames = copy_graph(limit, sample_stamp);
        response->sample_stamp = sample_stamp;
        if (!frames) {
          response->status = ReadGraph::Response::UNAVAILABLE;
          return;
        }
        const auto status = validate_snapshot(*frames, limit);
        switch (status) {
          case SnapshotStatus::empty:
            response->status = ReadGraph::Response::EMPTY;
            return;
          case SnapshotStatus::unavailable:
            response->status = ReadGraph::Response::UNAVAILABLE;
            return;
          case SnapshotStatus::ok:
            response->status = ReadGraph::Response::OK;
            break;
        }

        fill_keyframes(*frames, response->keyframes);
      });
  }

  std::optional<std::vector<Keyframe>> copy_graph(std::size_t limit, rclcpp::Time & sample_stamp)
  {
    boost::mutex::scoped_lock lock(smapper_mutex_);
    sample_stamp = now();
    return copy_keyframes(smapper_ ? smapper_->getMapper() : nullptr, limit);
  }

  static void fill_keyframes(
    const std::vector<Keyframe> & frames, ReadGraph::Response::_keyframes_type & keyframes)
  {
    keyframes.reserve(frames.size());
    for (const auto & frame : frames) {
      msg::Keyframe output;
      output.id = frame.id;
      output.x = frame.x;
      output.y = frame.y;
      output.yaw = frame.yaw;
      keyframes.push_back(output);
    }
  }

  // Requires smapper_mutex_ for the entire access.
  static std::optional<std::vector<Keyframe>> copy_keyframes(
    const karto::Mapper * mapper, std::size_t limit)
  {
    std::vector<Keyframe> frames;
    if (!mapper) {
      return std::nullopt;
    }
    const auto * graph = mapper->GetGraph();
    // Karto initializes its graph on the first usable scan.
    if (!graph) {
      return frames;
    }

    for (const auto & sensor : graph->GetVertices()) {
      for (const auto & entry : sensor.second) {
        const auto * scan = entry.second ? entry.second->GetObject() : nullptr;
        if (!scan || frames.size() == limit) {
          return std::nullopt;
        }
        const auto & pose = scan->GetCorrectedPose();
        frames.push_back({scan->GetUniqueId(), pose.GetX(), pose.GetY(), pose.GetHeading()});
      }
    }
    return frames;
  }

  const std::string boot_id_{boost::uuids::to_string(boost::uuids::random_generator{}())};
  std::uint64_t snapshot_sequence_{0};
  std::mutex scan_ingress_mutex_;
  std::unique_ptr<CaptureDirectory> capture_directory_;
  rclcpp::Service<CaptureGraph>::SharedPtr capture_graph_;
  rclcpp::Service<ReadGraph>::SharedPtr read_graph_;
  rclcpp::Service<ReadLoadedGraph>::SharedPtr candidate_receipt_;
};
}  // namespace senior_slam_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<senior_slam_bridge::SlamGraphNode>(rclcpp::NodeOptions{});
  try {
    node->start();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
