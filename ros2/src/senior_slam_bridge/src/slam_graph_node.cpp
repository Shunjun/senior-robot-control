#include "snapshot.hpp"
#include "staged_graph.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "rclcpp/rclcpp.hpp"
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

  void start()
  {
    const auto prefix = declare_parameter<std::string>("candidate_graph_prefix", "");
    const auto digest = declare_parameter<std::string>("candidate_graph_digest", "");
    const auto attempt = declare_parameter<std::string>("candidate_load_attempt", "");
    if (prefix.empty() && digest.empty() && attempt.empty()) {
      configure();
    } else {
      load_candidate(prefix, digest, attempt);
    }
    start_graph_service();
  }

private:
  using ReadGraph = srv::ReadGraph;
  using ReadLoadedGraph = srv::ReadLoadedGraph;

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

        response->keyframes.reserve(frames->size());
        for (const auto & frame : *frames) {
          msg::Keyframe output;
          output.id = frame.id;
          output.x = frame.x;
          output.y = frame.y;
          output.yaw = frame.yaw;
          response->keyframes.push_back(output);
        }
      });
  }

  std::optional<std::vector<Keyframe>> copy_graph(std::size_t limit, rclcpp::Time & sample_stamp)
  {
    std::vector<Keyframe> frames;
    boost::mutex::scoped_lock lock(smapper_mutex_);
    sample_stamp = now();
    const auto * mapper = smapper_ ? smapper_->getMapper() : nullptr;
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
