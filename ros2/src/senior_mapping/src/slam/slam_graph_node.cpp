#include "bounded_scan_ingress.hpp"
#include "candidate_stationary_match.hpp"
#include "graph_commit_history.hpp"
#include "native_capture.hpp"
#include "mapping_navigation.hpp"
#include <tbb/global_control.h>
#include "snapshot.hpp"
#include "staged_graph.hpp"
#include "static_lidar_tf_ingress.hpp"

#include <algorithm>
#include <atomic>
#include <deque>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "rclcpp/rclcpp.hpp"
#include "senior_interfaces/msg/qualified_mapping_scan.hpp"
#include "senior_interfaces/srv/capture_graph.hpp"
#include "senior_interfaces/srv/match_stationary_scan.hpp"
#include "senior_interfaces/srv/read_graph.hpp"
#include "senior_interfaces/srv/mapping_gate.hpp"
#include "senior_interfaces/srv/read_graph_commit.hpp"
#include "senior_interfaces/srv/read_loaded_graph.hpp"
//Compatible: the Humble slam_toolbox 2.6.10 subclass exposes the mapper and its mutex.
#include "slam_toolbox/slam_toolbox_async.hpp"
#include "slam_toolbox/serialization.hpp"

namespace senior_mapping
{
class SlamGraphNode final : public slam_toolbox::AsynchronousSlamToolbox
{
public:
  explicit SlamGraphNode(const rclcpp::NodeOptions & options)
  : AsynchronousSlamToolbox(options) {}

  ~SlamGraphNode() override
  {
    accepting_.store(false);
    stopping_.store(true);
    if (bounded_ingress_) {bounded_ingress_->retire_source();}
    if (ingress_executor_) {ingress_executor_->cancel();}
    if (ingress_thread_.joinable()) {ingress_thread_.join();}
    // No ingress work can retain graph/cache or TF state after this barrier.
    {const std::lock_guard<std::mutex> lock(queue_mutex_); graph_queue_.clear();}
    if (capture_directory_ || bounded_ingress_) {
      //Compatible: join Humble's dedicated TF listener before retiring the ingress lock.
      tfL_.reset();
      scan_filter_.reset();
      scan_filter_sub_.reset();
    }
  }

  void start()
  {
    const bool production = declare_parameter<bool>("production_session", false);
    const auto production_boot = declare_parameter<std::string>("production_boot_id", "");
    if (production) {
      if (production_boot.size() != 36) {
        throw std::invalid_argument("Production owner must supply its new boot identity");
      }
      boot_id_ = production_boot;
      const auto threads = declare_parameter<int>("mapping_worker_threads", 1);
      if (threads < 1 || threads > 4) {throw std::invalid_argument("Invalid mapping thread budget");}
      compute_budget_ = std::make_unique<tbb::global_control>(
        tbb::global_control::max_allowed_parallelism, threads);
    }
    const auto live_prefix = declare_parameter<std::string>("production_graph_prefix", "");
    const auto live_digest = declare_parameter<std::string>("production_graph_digest", "");
    if ((!live_prefix.empty() || !live_digest.empty()) && !production) {
      throw std::invalid_argument("Production graph requires an explicit session owner");
    }
    const auto prefix = declare_parameter<std::string>("candidate_graph_prefix", "");
    const auto digest = declare_parameter<std::string>("candidate_graph_digest", "");
    const auto attempt = declare_parameter<std::string>("candidate_load_attempt", "");
    const bool stationary_diagnostic = declare_parameter<bool>("candidate_stationary_match", false);
    const auto capture_root = declare_parameter<std::string>("capture_directory", "");
    const auto bounded_topic = declare_parameter<std::string>("bounded_scan_topic", "");
    const auto static_lidar_topic = declare_parameter<std::string>(
      "verified_static_lidar_tf_topic", "");
    if (!static_lidar_topic.empty() && bounded_topic.empty()) {
      throw std::invalid_argument("Verified static lidar TF requires bounded scan ingress");
    }
    if (!bounded_topic.empty() &&
      (!prefix.empty() || !digest.empty() || !attempt.empty()))
    {
      throw std::invalid_argument("Bounded scan ingress is separate from candidate load");
    }
    if (prefix.empty() && digest.empty() && attempt.empty()) {
      if (stationary_diagnostic) {
        throw std::invalid_argument("Stationary matching requires an isolated candidate load");
      }
      if (!capture_root.empty() || !bounded_topic.empty()) {
        prepare_owned_graph();
      }
      if (!capture_root.empty()) {
        capture_directory_ = std::make_unique<CaptureDirectory>(capture_root);
      }
      configure();
      if (production) {
        if (bounded_topic.empty() || capture_root.empty()) {
          throw std::invalid_argument("Production mapping requires bounded input and capture");
        }
        accepting_ = false;
        production_ = true;
        if (!live_prefix.empty()) {
          const StagedGraph staged(live_prefix, live_digest);
          auto request = std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Request>();
          auto response = std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Response>();
          request->filename = staged.prefix().string();
          request->match_type = request->START_AT_GIVEN_POSE;
          const auto seed = declare_parameter<std::vector<double>>("production_seed", std::vector<double>{0., 0., 0.});
          if (seed.size() != 3 || !std::all_of(seed.begin(), seed.end(),
              [](double value) {return std::isfinite(value);})) {
            throw std::invalid_argument("Invalid production map seed");
          }
          request->initial_pose.x = seed[0];
          request->initial_pose.y = seed[1];
          request->initial_pose.theta = seed[2];
          //Compatible: use the supported Humble loader before spinning any scan callback.
          if (!deserializePoseGraphCallback(nullptr, request, response)) {
            throw std::runtime_error("Could not load production graph");
          }
          rclcpp::Time stamp;
          const auto frames = copy_graph(ReadGraph::Response{}.keyframes.max_size(), stamp);
          if (!frames || frames->empty()) {
            throw std::runtime_error("Production graph load returned no vertices");
          }
        }
        mapping_gate_ = create_service<senior_interfaces::srv::MappingGate>("/senior/slam/mapping_gate",
          [this](const senior_interfaces::srv::MappingGate::Request::SharedPtr request,
                 senior_interfaces::srv::MappingGate::Response::SharedPtr response) {
            const std::lock_guard<std::mutex> lock(scan_ingress_mutex_);
            if (request->boot_id != boot_id_) {return;}
            accepting_ = request->accepting;
            input_floor_ns_.store(now().nanoseconds());
            response->graph_commit_sequence = graph_commit_.sequence;
            response->ok = true;
          });
      }
      if (!capture_root.empty() || !bounded_topic.empty()) {
        disable_stock_mutations();
      }
      if (!bounded_topic.empty()) {
        ingress_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        if (!static_lidar_topic.empty()) {
          start_static_lidar_tf(static_lidar_topic);
        }
        start_bounded_ingress(bounded_topic);
        start_graph_commit_service();
      }
      if (capture_directory_) {
        start_owned_capture();
      }
    } else {
      if (!capture_root.empty()) {
        throw std::invalid_argument("Candidate load and owned capture are separate modes");
      }
      load_candidate(prefix, digest, attempt, stationary_diagnostic);
    }
    start_graph_service();
    if (production_) {
      const auto maximum = declare_parameter<int>("mapping_max_grid_cells", 1048576);
      if (maximum < 1 || maximum > 1048576) {throw std::invalid_argument("Invalid map raster budget");}
      navigation_ = std::make_unique<MappingNavigation>(*this, boot_id_,
        [this] {return graph_commit_.sequence > 0 && bounded_ingress_->active_snapshot();},
        [this, maximum](std::uint64_t sequence) -> std::optional<nav_msgs::msg::OccupancyGrid> {
          const std::lock_guard<std::mutex> ingress(scan_ingress_mutex_);
          boost::mutex::scoped_lock lock(smapper_mutex_);
          if (sequence != graph_commit_.sequence) {return std::nullopt;}
          auto grid = bounded_native_raster(*smapper_->getMapper(), resolution_, maximum);
          if (grid) {grid->header.frame_id = map_frame_; grid->header.stamp = now();}
          return grid;
        });
    }
    if (ingress_group_) {
      ingress_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      ingress_executor_->add_callback_group(ingress_group_, get_node_base_interface());
      ingress_thread_ = std::thread([this] {
          try {ingress_executor_->spin();}
          catch (const std::exception & error) {
            bounded_ingress_->retire_source();
            try {publish_invalid(true, "ingress_failed");}
            catch (const std::exception &) {}  // The ROS context may already be shutting down.
            RCLCPP_ERROR(get_logger(), "Qualified ingress stopped: %s", error.what());
          }
        });
    }
  }

private:
  using CaptureGraph = senior_interfaces::srv::CaptureGraph;
  using ReadGraph = senior_interfaces::srv::ReadGraph;
  using ReadGraphCommit = senior_interfaces::srv::ReadGraphCommit;
  using ReadLoadedGraph = senior_interfaces::srv::ReadLoadedGraph;
  using MatchStationaryScan = senior_interfaces::srv::MatchStationaryScan;

  struct GraphCommitWatermark
  {
    std::uint64_t sequence{0};
    std::int32_t last_keyframe_id{0};
  };

  using StaticReceipt = BoundedScanIngress::StaticReceipt;

  struct InFlightScan
  {
    BoundedScanIngress::BoundedScan::ConstSharedPtr scan;
    LaserTransform mount;
    StaticReceipt::ConstSharedPtr static_receipt;
  };

  using Qualified = senior_interfaces::msg::QualifiedMappingScan;
  struct GeometryWitness
  {
    NativeLaserOffset offset;
    LaserRayLayout rays;
    double angle_max, range_min, range_max, height;
    std::int8_t sign;
    bool matches(const sensor_msgs::msg::LaserScan & scan, LaserTransform mount) const
    {
      return scan.angle_min == rays.angle_min && scan.angle_increment == rays.angle_increment &&
        scan.angle_max == angle_max && scan.range_min == range_min && scan.range_max == range_max &&
        scan.ranges.size() == rays.ray_count && std::abs(mount.z-height) <= 1e-5 &&
        matches_native_laser_geometry(mount, offset, rays);
    }
  };
  struct GraphCandidate
  {
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    BoundedScanIngress::BoundedScan::ConstSharedPtr original;
    LaserTransform mount;
    StaticReceipt::ConstSharedPtr static_receipt;
  };

  struct CommittedIngressReceipt
  {
    BoundedScanIngress::Receipt scan;
    StaticReceipt::ConstSharedPtr static_receipt;
  };

  struct CommittedScanCut
  {
    std::int32_t keyframe_id;
    builtin_interfaces::msg::Time sample_stamp;
    NativeLaserOffset corrected_sensor;
    std::int8_t raw_ray_angle_sign;
    BoundedScanIngress::BoundedScan::ConstSharedPtr scan;
    StaticReceipt::ConstSharedPtr static_receipt;
    std::vector<Keyframe> keyframes;
    NativeLaserOffset original_odom;
  };

  template<typename Response>
  static void fill_commit_watermark(Response & response, const GraphCommitWatermark & watermark)
  {
    response.graph_commit_watermark_valid = true;
    response.graph_commit_sequence = watermark.sequence;
    response.last_committed_keyframe_id = watermark.last_keyframe_id;
  }

  void prepare_owned_graph()
  {
    const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
    const auto saver = overrides.find("use_map_saver");
    const auto interactive = overrides.find("enable_interactive_mode");
    const auto initial_graph = overrides.find("map_file_name");
    if (saver == overrides.end() || saver->second.get<bool>() ||
      (interactive != overrides.end() && interactive->second.get<bool>()))
    {
      throw std::invalid_argument("Owned graph requires use_map_saver=false and interactive off");
    }
    if (initial_graph != overrides.end() && !initial_graph->second.get<std::string>().empty()) {
      throw std::invalid_argument("Owned graph must start a fresh graph");
    }
  }

  void disable_stock_mutations()
  {
    ssDesserialize_.reset();
    ssSerialize_.reset();
    ssPauseMeasurements_.reset();
  }

  void start_static_lidar_tf(const std::string & topic)
  {
    const auto frame = declare_parameter<std::string>("verified_static_lidar_frame", "laser_link");
    const auto relay_node = declare_parameter<std::string>(
      "verified_static_lidar_relay_node", "/slam_static_lidar_tf_receipt");
    const auto raw_topic = declare_parameter<std::string>(
      "verified_static_lidar_tf_source_topic", "/tf_static");
    const auto raw_node = declare_parameter<std::string>(
      "verified_static_lidar_tf_source_node", "/robot_state_publisher");
    static_lidar_tf_ = std::make_unique<StaticLidarTfIngress>(
      *this, topic, relay_node, raw_topic, raw_node, base_frame_, frame, ingress_group_);
  }

  void start_bounded_ingress(const std::string & topic)
  {
    if (topic == scan_topic_) {
      throw std::invalid_argument("Bounded scan topic must differ from the raw scan topic");
    }
    //Compatible: the Humble base configure() installs a LaserScan MessageFilter.
    // No node callbacks have spun yet, so discard it before it can create TF futures.
    scan_filter_.reset();
    scan_filter_sub_.reset();
    qualified_publisher_ = create_publisher<Qualified>(
      declare_parameter<std::string>("qualified_scan_topic", "/senior/slam/qualified_scan"),
      rclcpp::QoS(1).reliable().durability_volatile());
    bounded_ingress_ = std::make_unique<BoundedScanIngress>(
      *this, *tf_, topic, scan_topic_, odom_frame_, base_frame_,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan,
        BoundedScanIngress::BoundedScan::ConstSharedPtr original, LaserTransform mount,
        StaticReceipt::ConstSharedPtr static_receipt)
      {
        if (stopping_.load()) {return;}
        const auto witness = std::atomic_load(&geometry_witness_);
        if (witness) {
          if (!witness->matches(*scan, mount)) {
            bounded_ingress_->retire_source();
            publish_invalid(true, "geometry_changed");
            return;
          }
          publish_qualified(*witness, original, mount, static_receipt);
        }
        // Bootstrap must still reach getLaser/addScan before a cached witness exists.
        const std::lock_guard<std::mutex> queue_lock(queue_mutex_);
        if (graph_queue_.size() == 8) {graph_queue_.pop_front();}
        graph_queue_.push_back({std::move(scan), std::move(original), mount,
          std::move(static_receipt)});
      }, static_lidar_tf_.get(), ingress_group_);
    graph_drain_ = create_wall_timer(std::chrono::milliseconds(5), [this] {drain_graph_scan();});
    qualified_expiry_ = create_wall_timer(std::chrono::milliseconds(20), [this] {
        if (bounded_ingress_->retired_snapshot()) {
          if (!retirement_published_) {
            publish_invalid(true, "source_retired"); retirement_published_ = true;
          }
        } else if (!bounded_ingress_->active_snapshot()) {
          if (current_valid_) {publish_invalid(false, "source_unavailable");}
        } else if (current_valid_ && senior_sensor_ingress::time::monotonic_ns() >= current_deadline_) {
          publish_invalid(false, "expired");
        }
      }, ingress_group_);
  }

  void publish_qualified(const GeometryWitness & witness,
    BoundedScanIngress::BoundedScan::ConstSharedPtr original, LaserTransform mount,
    StaticReceipt::ConstSharedPtr static_receipt)
  {
    try {
      const auto exact = tf_->lookupTransform(odom_frame_, base_frame_,
        rclcpp::Time(original->capture.stamp, get_clock()->get_clock_type())).transform;
      Qualified result;
      result.backend_boot = boot_id_; result.status = Qualified::QUALIFIED;
      result.scan = *original;
      if (static_receipt) {
        result.static_lidar_tf_receipt = *static_receipt;
        result.static_receipt_valid = true;
      }
      result.odom_base.x = exact.translation.x;
      result.odom_base.y = exact.translation.y;
      const auto & q = exact.rotation;
      result.odom_base.theta = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
      result.base_lidar.translation.x = mount.x;
      result.base_lidar.translation.y = mount.y;
      result.base_lidar.translation.z = mount.z;
      result.base_lidar.rotation.x = mount.qx; result.base_lidar.rotation.y = mount.qy;
      result.base_lidar.rotation.z = mount.qz; result.base_lidar.rotation.w = mount.qw;
      result.raw_ray_angle_sign = witness.sign;
      result.geometry_id = boot_id_ + ":mount-1";
      if (std::isfinite(result.odom_base.x) && std::isfinite(result.odom_base.y) &&
        std::isfinite(result.odom_base.theta) &&
        scan_receipt_admission::fresh(original->capture, scan_receipt_admission::steady_seconds()) &&
        bounded_ingress_->active_snapshot())
      {
        qualified_publisher_->publish(result);
        current_deadline_ = original->capture.sample_steady_ns + 500000000ULL;
        current_valid_ = true;
      }
    } catch (const tf2::TransformException &) {
      publish_invalid(false, "exact_odom_missing");
    }
  }

  void publish_invalid(bool retired, const std::string & reason)
  {
    Qualified result;
    result.backend_boot = boot_id_;
    result.status = retired ? Qualified::RETIRED : Qualified::INVALID;
    result.reason = reason;
    qualified_publisher_->publish(result);
    current_valid_ = false;
  }

  void drain_graph_scan()
  {
    std::optional<GraphCandidate> candidate;
    {
      const std::lock_guard<std::mutex> lock(queue_mutex_);
      if (graph_queue_.empty()) {return;}
      candidate = std::move(graph_queue_.front()); graph_queue_.pop_front();
    }
    const auto & scan = candidate->scan;
    if (stopping_.load() || !bounded_ingress_->active_snapshot() ||
      !scan_receipt_admission::fresh(candidate->original->capture, scan_receipt_admission::steady_seconds()) ||
      (production_ && (!accepting_.load() ||
      rclcpp::Time(scan->header.stamp).nanoseconds() <= input_floor_ns_.load()))) {return;}
    const std::lock_guard<std::mutex> ingress(scan_ingress_mutex_);
    const auto cached = lasers_.find(scan->header.frame_id);
    if (cached != lasers_.end() &&
      !match_native_mount(cached->second.getLaser(), *scan, candidate->mount))
    {
      retire_geometry(); return;
    }
    in_flight_ = InFlightScan{candidate->original, candidate->mount, candidate->static_receipt};
    try {
      AsynchronousSlamToolbox::laserCallback(scan);
      karto::Pose2 odometry;
      if (navigation_ && pose_helper_->getOdomPose(odometry, in_flight_->scan->capture.stamp)) {
        navigation_->observe(in_flight_->scan->capture, odometry,
          in_flight_->static_receipt != nullptr);
      }
    } catch (...) {in_flight_.reset(); throw;}
    in_flight_.reset();
  }

  void start_owned_capture()
  {
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
    if (bounded_ingress_) {
      return;
    }
    if (!capture_directory_) {
      AsynchronousSlamToolbox::laserCallback(std::move(scan));
      return;
    }
    //Compatible: Humble's TF filter can dispatch off the executor, and getLaser()
    // registers a new sensor in the dataset before addScan acquires smapper_mutex_.
    const std::lock_guard<std::mutex> ingress(scan_ingress_mutex_);
    AsynchronousSlamToolbox::laserCallback(std::move(scan));
  }

  std::optional<std::int8_t> match_native_mount(
    karto::LaserRangeFinder * laser, const sensor_msgs::msg::LaserScan & scan,
    LaserTransform mount)
  {
    const auto cached = lasers_.find(scan.header.frame_id);
    if (!laser || cached == lasers_.end() || cached->second.getLaser() != laser ||
      laser->GetMinimumAngle() != scan.angle_min || laser->GetMaximumAngle() != scan.angle_max ||
      laser->GetAngularResolution() != scan.angle_increment ||
      laser->GetNumberOfRangeReadings() != scan.ranges.size() ||
      (mount_height_ && std::abs(mount.z - *mount_height_) > 1e-5))
    {
      return std::nullopt;
    }
    const auto offset = laser->GetOffsetPose();
    const bool inverted = cached->second.isInverted();
    if (!matches_native_laser_geometry(
        mount, {offset.GetX(), offset.GetY(), offset.GetHeading()},
        {scan.angle_min, scan.angle_increment, scan.ranges.size(), inverted}))
    {
      return std::nullopt;
    }
    return inverted ? -1 : 1;
  }

  void retire_geometry()
  {
    bounded_ingress_->retire_source();
    RCLCPP_WARN(get_logger(), "Bounded scan mount disagrees with native laser geometry");
  }

  karto::LocalizedRangeScan * addScan(
    karto::LaserRangeFinder * laser,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
    karto::Pose2 & odom_pose) override
  {
    //Compatible: Humble's async laser callback invokes this virtual while the
    // owned ingress lock is held. The base returns null for an uncommitted scan.
    std::int8_t ray_angle_sign{0};
    if (in_flight_) {
      const auto matched = match_native_mount(laser, *scan, in_flight_->mount);
      if (!matched) {
        retire_geometry();
        return nullptr;
      }
      if (!mount_height_) {
        mount_height_ = in_flight_->mount.z;
      }
      ray_angle_sign = *matched;
      const auto offset = laser->GetOffsetPose();
      std::shared_ptr<const GeometryWitness> witness = std::make_shared<GeometryWitness>(GeometryWitness{
        {offset.GetX(), offset.GetY(), offset.GetHeading()},
        {scan->angle_min, scan->angle_increment, scan->ranges.size(), *matched == -1},
        scan->angle_max, scan->range_min, scan->range_max, in_flight_->mount.z, *matched});
      std::atomic_store(&geometry_witness_, std::move(witness));
    }
    auto * committed = AsynchronousSlamToolbox::addScan(laser, scan, odom_pose);
    if ((capture_directory_ || in_flight_) && committed) {
      ++graph_commit_.sequence;
      graph_commit_.last_keyframe_id = committed->GetUniqueId();
      if (in_flight_) {
        last_committed_ingress_ =
          CommittedIngressReceipt{in_flight_->scan->capture, in_flight_->static_receipt};
        record_graph_commit(*committed, laser, ray_angle_sign, odom_pose);
      }
    }
    return committed;
  }

  void record_graph_commit(
    const karto::LocalizedRangeScan & committed, karto::LaserRangeFinder * laser,
    std::int8_t ray_angle_sign, const karto::Pose2 & original_odom)
  {
    // The bounded ingress already owns the outer lock. The vendor released its
    // mapper lock after applying the scan and any loop correction.
    boost::mutex::scoped_lock lock(smapper_mutex_);
    if (committed.GetLaserRangeFinder() != laser) {
      graph_commit_history_.append(graph_commit_.sequence, std::nullopt);
      retire_geometry();
      return;
    }
    const auto sensor = committed.GetSensorPose();
    const auto limit = ReadGraphCommit::Response{}.keyframes.max_size();
    auto frames = copy_keyframes(smapper_ ? smapper_->getMapper() : nullptr, limit);
    if (!std::isfinite(sensor.GetX()) || !std::isfinite(sensor.GetY()) ||
      !std::isfinite(sensor.GetHeading()) ||
      !frames || validate_snapshot(*frames, limit) != SnapshotStatus::ok ||
      std::none_of(frames->begin(), frames->end(), [this](const Keyframe & frame) {
        return frame.id == graph_commit_.last_keyframe_id;
      }))
    {
      graph_commit_history_.append(graph_commit_.sequence, std::nullopt);
      return;
    }
    graph_commit_history_.append(
      graph_commit_.sequence,
      CommittedScanCut{
        graph_commit_.last_keyframe_id, now(),
        {sensor.GetX(), sensor.GetY(), sensor.GetHeading()}, ray_angle_sign,
        in_flight_->scan, in_flight_->static_receipt, std::move(*frames),
        {original_odom.GetX(), original_odom.GetY(), original_odom.GetHeading()}});
  }

  void log_graph_commit_unavailable(
    const ReadGraphCommit::Request & request, const char * reason,
    const char * initial_active, const char * final_active) noexcept
  {
    // This service and graph mutation share the default mutually exclusive group.
    if (graph_commit_unavailable_logged_) {return;}
    graph_commit_unavailable_logged_ = true;
    try {
      // Retirement is a later atomic snapshot, not the active decision witness.
      const bool retired_after_decision = bounded_ingress_->retired_snapshot();
      RCLCPP_WARN(get_logger(),
        "ReadGraphCommit UNAVAILABLE: reason=%s request_boot=%.36s current_boot=%.36s "
        "request_sequence=%llu graph_sequence=%llu initial_active=%s final_active=%s "
        "retired_after_decision=%s raw_receipt_last_reason=N/A static_receipt_last_reason=N/A",
        reason, request.boot_id.c_str(), boot_id_.c_str(),
        static_cast<unsigned long long>(request.graph_commit_sequence),
        static_cast<unsigned long long>(graph_commit_.sequence), initial_active, final_active,
        retired_after_decision ? "true" : "false");
    } catch (...) {}  // Diagnostics must not change service delivery, even if logging fails.
  }

  void start_graph_commit_service()
  {
    read_graph_commit_ = create_service<ReadGraphCommit>(
      "/senior/slam/read_graph_commit",
      [this](const ReadGraphCommit::Request::SharedPtr request,
        ReadGraphCommit::Response::SharedPtr response)
      {
        if (request->boot_id != boot_id_) {
          log_graph_commit_unavailable(*request, "boot_mismatch", "N/A", "N/A");
          return;
        }
        const bool initial_active = bounded_ingress_->active_snapshot();
        if (!initial_active) {
          log_graph_commit_unavailable(*request, "inactive", "false", "N/A");
          return;
        }
        // Delivery and lookup stay on the graph owner; ingress eligibility is an atomic witness.
        const auto [state, cut] = graph_commit_history_.lookup(request->graph_commit_sequence);
        switch (state) {
          case CommitLookup::invalid:
            log_graph_commit_unavailable(*request, "invalid_lookup", "true", "N/A");
            return;
          case CommitLookup::pending:
            response->status = ReadGraphCommit::Response::PENDING;
            return;
          case CommitLookup::gap:
            response->status = ReadGraphCommit::Response::GAP;
            return;
          case CommitLookup::found:
            break;
        }
        ReadGraphCommit::Response result;
        result.boot_id = boot_id_;
        result.graph_commit_sequence = request->graph_commit_sequence;
        result.committed_keyframe_id = cut->keyframe_id;
        result.sample_stamp = cut->sample_stamp;
        result.map_frame = map_frame_;
        result.odom_frame = odom_frame_;
        result.corrected_sensor_x = cut->corrected_sensor.x;
        result.corrected_sensor_y = cut->corrected_sensor.y;
        result.corrected_sensor_yaw = cut->corrected_sensor.yaw;
        result.raw_ray_angle_sign = cut->raw_ray_angle_sign;
        result.original_odom_x = cut->original_odom.x;
        result.original_odom_y = cut->original_odom.y;
        result.original_odom_yaw = cut->original_odom.yaw;
        result.scan = *cut->scan;
        if (cut->static_receipt) {
          result.static_lidar_tf_receipt = *cut->static_receipt;
          result.static_lidar_tf_receipt_valid = true;
        }
        fill_keyframes(cut->keyframes, result.keyframes);
        const bool final_active = bounded_ingress_->active_snapshot();
        if (final_active) {
          result.status = ReadGraphCommit::Response::OK;
          *response = std::move(result);
        } else {
          log_graph_commit_unavailable(*request, "final_inactive", "true", "false");
        }
      });
  }

  void capture_locked_observation(CaptureGraph::Response & response)
  {
    capture_directory_->require_free_slot();
    const auto limit = response.keyframes.max_size();
    std::vector<Keyframe> frames;
    std::vector<char> graph_bytes;
    std::vector<char> dataset_bytes;
    GraphCommitWatermark watermark;
    std::optional<CommittedIngressReceipt> receipt;
    {
      const std::lock_guard<std::mutex> ingress(scan_ingress_mutex_);
      if (bounded_ingress_ && last_committed_ingress_ && !bounded_ingress_->active_snapshot()) {
        return;
      }
      boost::mutex::scoped_lock lock(smapper_mutex_);
      response.sample_stamp = now();
      watermark = graph_commit_;
      auto * mapper = smapper_ ? smapper_->getMapper() : nullptr;
      auto copied = copy_keyframes(mapper, limit);
      if (!copied) {
        return;
      }
      frames = std::move(*copied);
      switch (validate_snapshot(frames, limit)) {
        case SnapshotStatus::empty:
          response.status = CaptureGraph::Response::EMPTY;
          fill_commit_watermark(response, watermark);
          return;
        case SnapshotStatus::unavailable:
          return;
        case SnapshotStatus::ok:
          break;
      }
      if (bounded_ingress_) {
        if (!last_committed_ingress_) {
          return;
        }
        receipt = last_committed_ingress_;
      }
      graph_bytes = capture_native_archive(
        [mapper](const std::string & path) {mapper->SaveToFile(path);});
      dataset_bytes = capture_native_archive(
        [this](const std::string & path) {dataset_->SaveToFile(path);});
    }
    // Build the response before reserving the single output slot.
    decltype(response.keyframes) keyframes;
    fill_keyframes(frames, keyframes);
    StaticReceipt static_receipt;
    if (receipt && receipt->static_receipt) {
      static_receipt = *receipt->static_receipt;
    }
    const auto publication_allowed = [this]() {
        return !bounded_ingress_ || bounded_ingress_->active_snapshot();
      };
    if (!publication_allowed()) {
      return;
    }
    auto artifact = capture_directory_->write_pair(graph_bytes, dataset_bytes, publication_allowed);
    response.keyframes = std::move(keyframes);
    response.relative_prefix = std::move(artifact.relative_prefix);
    response.pair_digest = std::move(artifact.pair_digest);
    fill_commit_watermark(response, watermark);
    if (receipt) {
      response.last_committed_scan = std::move(receipt->scan);
      response.last_committed_scan_valid = true;
      if (receipt->static_receipt) {
        response.static_lidar_tf_receipt = std::move(static_receipt);
        response.static_lidar_tf_receipt_valid = true;
      }
    }
    response.status = CaptureGraph::Response::OK;
  }

  void load_candidate(
    const std::string & prefix, const std::string & digest, const std::string & attempt,
    bool stationary_diagnostic)
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
    if (stationary_diagnostic) {
      start_stationary_diagnostic(attempt, staged.digest());
    }
  }

  void start_stationary_diagnostic(const std::string & attempt, const std::string & digest)
  {
    stationary_match_service_ = create_service<MatchStationaryScan>(
      "/senior/slam/match_stationary_scan",
      [this, attempt, digest](
        const MatchStationaryScan::Request::SharedPtr request,
        MatchStationaryScan::Response::SharedPtr response)
      {
        if (request->boot_id != boot_id_ || request->attempt_id != attempt ||
          request->pair_digest != digest || request->map_frame != map_frame_)
        {
          return;
        }
        if (stationary_match_spent_) {
          response->status = MatchStationaryScan::Response::SPENT;
          return;
        }
        const StationaryMatchSeed primary{request->seed_x, request->seed_y, request->seed_yaw};
        const StationaryMatchSeed second{
          request->second_seed_x, request->second_seed_y, request->second_seed_yaw};
        const StationaryMatchQuery query{
          request->first_angle, request->angle_step,
          {request->ranges.begin(), request->ranges.end()}};
        if (!valid_stationary_match_seed(primary) || !valid_stationary_match_query(query) ||
          (request->compare_second_seed &&
          (!valid_stationary_match_seed(second) || !disjoint_stationary_match_seeds(primary, second))))
        {
          response->status = MatchStationaryScan::Response::INVALID;
          return;
        }

        stationary_match_spent_ = true;
        try {
          const auto private_name = "stationary_match_" + boot_id_;
          boost::mutex::scoped_lock lock(smapper_mutex_);
          const auto * mapper = smapper_ ? smapper_->getMapper() : nullptr;
          if (!mapper) {
            return;
          }
          for (const auto * object : dataset_->GetLasers()) {
            if (const auto * sensor = dynamic_cast<const karto::Sensor *>(object);
              sensor && sensor->GetName().ToString() == private_name)
            {
              return;
            }
          }
          const auto match = [&](const StationaryMatchSeed & seed)
            -> std::optional<StationaryMatchDiagnostic>
            {
              try {
                return stationary_match(*mapper, query, seed, private_name);
              } catch (const std::exception & error) {
                RCLCPP_WARN(get_logger(), "Candidate stationary hypothesis failed: %s", error.what());
              } catch (...) {
                RCLCPP_WARN(get_logger(), "Candidate stationary hypothesis failed");
              }
              return std::nullopt;
            };
          const auto first_result = match(primary);
          const auto second_result = request->compare_second_seed ? match(second) : std::nullopt;
          if (!first_result && !second_result) {
            return;
          }

          MatchStationaryScan::Response diagnostic;
          if (first_result) {
            diagnostic.primary_result_valid = true;
            diagnostic.response = first_result->response;
            diagnostic.fitted_x = first_result->map_sensor_pose.GetX();
            diagnostic.fitted_y = first_result->map_sensor_pose.GetY();
            diagnostic.fitted_yaw = first_result->map_sensor_pose.GetHeading();
            diagnostic.reference_ids.assign(
              first_result->reference_ids.begin(), first_result->reference_ids.end());
          }
          if (second_result) {
            diagnostic.second_result_valid = true;
            diagnostic.second_response = second_result->response;
            diagnostic.second_fitted_x = second_result->map_sensor_pose.GetX();
            diagnostic.second_fitted_y = second_result->map_sensor_pose.GetY();
            diagnostic.second_fitted_yaw = second_result->map_sensor_pose.GetHeading();
            diagnostic.second_reference_ids.assign(
              second_result->reference_ids.begin(), second_result->reference_ids.end());
          }
          diagnostic.status = MatchStationaryScan::Response::COMPLETE;
          *response = std::move(diagnostic);
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "Candidate stationary matching failed: %s", error.what());
        } catch (...) {
          RCLCPP_WARN(get_logger(), "Candidate stationary matching failed");
        }
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
        GraphCommitWatermark watermark;
        auto frames = copy_graph(limit, sample_stamp, &watermark);
        response->sample_stamp = sample_stamp;
        if (!frames) {
          response->status = ReadGraph::Response::UNAVAILABLE;
          return;
        }
        const auto status = validate_snapshot(*frames, limit);
        switch (status) {
          case SnapshotStatus::empty:
            response->status = ReadGraph::Response::EMPTY;
            if (capture_directory_ || bounded_ingress_) {
              fill_commit_watermark(*response, watermark);
            }
            return;
          case SnapshotStatus::unavailable:
            response->status = ReadGraph::Response::UNAVAILABLE;
            return;
          case SnapshotStatus::ok:
            response->status = ReadGraph::Response::OK;
            break;
        }

        fill_keyframes(*frames, response->keyframes);
        if (capture_directory_ || bounded_ingress_) {
          fill_commit_watermark(*response, watermark);
        }
        if (bounded_ingress_ && last_committed_ingress_ && bounded_ingress_->active_snapshot()) {
          response->last_committed_scan_valid = true;
          response->last_committed_scan = last_committed_ingress_->scan;
        }
      });
  }

  std::optional<std::vector<Keyframe>> copy_graph(
    std::size_t limit, rclcpp::Time & sample_stamp, GraphCommitWatermark * watermark = nullptr)
  {
    const std::lock_guard<std::mutex> ingress(scan_ingress_mutex_);
    boost::mutex::scoped_lock lock(smapper_mutex_);
    sample_stamp = now();
    if (watermark) {
      *watermark = graph_commit_;
    }
    return copy_keyframes(smapper_ ? smapper_->getMapper() : nullptr, limit);
  }

  static void fill_keyframes(
    const std::vector<Keyframe> & frames, ReadGraph::Response::_keyframes_type & keyframes)
  {
    keyframes.reserve(frames.size());
    for (const auto & frame : frames) {
      senior_interfaces::msg::Keyframe output;
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

  std::string boot_id_{boost::uuids::to_string(boost::uuids::random_generator{}())};
  std::uint64_t snapshot_sequence_{0};
  std::mutex scan_ingress_mutex_;
  std::unique_ptr<tbb::global_control> compute_budget_;
  std::unique_ptr<MappingNavigation> navigation_;
  bool production_{false};
  std::atomic<bool> accepting_{true}, stopping_{false};
  std::atomic<std::int64_t> input_floor_ns_{0};
  std::shared_ptr<const GeometryWitness> geometry_witness_;
  std::mutex queue_mutex_;
  std::deque<GraphCandidate> graph_queue_;
  rclcpp::CallbackGroup::SharedPtr ingress_group_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> ingress_executor_;
  std::thread ingress_thread_;
  rclcpp::TimerBase::SharedPtr graph_drain_, qualified_expiry_;
  rclcpp::Publisher<Qualified>::SharedPtr qualified_publisher_;
  bool current_valid_{false}, retirement_published_{false};
  std::uint64_t current_deadline_{0};
  rclcpp::Service<senior_interfaces::srv::MappingGate>::SharedPtr mapping_gate_;
  GraphCommitWatermark graph_commit_;
  std::optional<InFlightScan> in_flight_;
  std::optional<double> mount_height_;
  std::optional<CommittedIngressReceipt> last_committed_ingress_;
  GraphCommitHistory<CommittedScanCut, 8> graph_commit_history_;
  std::unique_ptr<StaticLidarTfIngress> static_lidar_tf_;
  std::unique_ptr<BoundedScanIngress> bounded_ingress_;
  std::unique_ptr<CaptureDirectory> capture_directory_;
  rclcpp::Service<CaptureGraph>::SharedPtr capture_graph_;
  rclcpp::Service<ReadGraph>::SharedPtr read_graph_;
  rclcpp::Service<ReadGraphCommit>::SharedPtr read_graph_commit_;
  bool graph_commit_unavailable_logged_{false};
  rclcpp::Service<ReadLoadedGraph>::SharedPtr candidate_receipt_;
  rclcpp::Service<MatchStationaryScan>::SharedPtr stationary_match_service_;
  bool stationary_match_spent_{false};
};
}  // namespace senior_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<senior_mapping::SlamGraphNode>(rclcpp::NodeOptions{});
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
