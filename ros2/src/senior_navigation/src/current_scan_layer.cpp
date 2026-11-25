#include "senior_navigation/current_scan_layer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <set>
#include <tuple>

#include <fcntl.h>
#include <sys/random.h>
#include <sys/stat.h>
#include <unistd.h>

#include "nlohmann/json.hpp"
#include <functional>
#include <stdexcept>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace senior_navigation
{
namespace
{
using Json = nlohmann::json;
constexpr uint64_t kLifetimeNs = 500000000;
constexpr size_t kRecordLimit = 8192;

void require(bool condition, const char * reason)
{
  if (!condition) {throw std::invalid_argument(reason);}
}

void keys(const Json & row, std::initializer_list<const char *> names)
{
  require(row.is_object() && row.size() == names.size(), "unexpected contract fields");
  for (const auto * name : names) {require(row.contains(name), "missing contract field");}
}

uint64_t number(const Json & row, const char * name)
{
  const auto & value = row.at(name);
  require(value.is_number_unsigned(), "contract integer must be uint64");
  return value.get<uint64_t>();
}

std::string text(const Json & row, const char * name, size_t limit = 128)
{
  const auto & value = row.at(name);
  require(value.is_string(), "contract text required");
  auto result = value.get<std::string>();
  require(result.size() <= limit && result.find('\0') == std::string::npos, "invalid contract text");
  return result;
}

double real(const Json & row, const char * name)
{
  const auto & value = row.at(name);
  require(value.is_number() && !value.is_boolean(), "contract finite number required");
  auto result = value.get<double>();
  require(std::isfinite(result), "nonfinite contract coordinate");
  return result;
}

Json strictJson(const std::string & bytes)
{
  std::vector<std::set<std::string>> objects;
  return Json::parse(bytes, [&objects](int depth, Json::parse_event_t event, Json & parsed) {
      require(depth <= 8, "contract nesting exceeded");
      if (event == Json::parse_event_t::object_start) {objects.emplace_back();}
      if (event == Json::parse_event_t::key) {
        require(!objects.empty() && objects.back().insert(parsed.get<std::string>()).second,
          "duplicate contract key");
      }
      if (event == Json::parse_event_t::object_end) {objects.pop_back();}
      return true;
    });
}

struct Source
{
  std::string backend_boot, bridge_boot_id, source_epoch, raw_gid, bridge_gid;
  uint64_t capture_generation{0}, clock_generation{0};
  bool operator==(const Source & other) const
  {
    return std::tie(backend_boot, bridge_boot_id, source_epoch, capture_generation,
      clock_generation, raw_gid, bridge_gid) ==
           std::tie(other.backend_boot, other.bridge_boot_id, other.source_epoch,
      other.capture_generation, other.clock_generation, other.raw_gid, other.bridge_gid);
  }
  static Source parse(const Json & row, bool key = false)
  {
    if (key) {
      keys(row, {"backend_boot", "bridge_boot_id", "source_epoch", "capture_generation",
        "clock_generation", "raw_gid", "bridge_gid", "sequence"});
    } else {
      keys(row, {"backend_boot", "bridge_boot_id", "source_epoch", "capture_generation",
        "clock_generation", "raw_gid", "bridge_gid"});
    }
    Source source{text(row, "backend_boot", 36), text(row, "bridge_boot_id", 64),
      text(row, "source_epoch", 64), text(row, "raw_gid"), text(row, "bridge_gid"),
      number(row, "capture_generation"), number(row, "clock_generation")};
    require(!source.backend_boot.empty() && !source.bridge_boot_id.empty() &&
      !source.source_epoch.empty() && !source.raw_gid.empty() && !source.bridge_gid.empty(),
      "empty selected source");
    return source;
  }
  Json wire() const
  {
    return {{"backend_boot", backend_boot}, {"bridge_boot_id", bridge_boot_id},
      {"source_epoch", source_epoch}, {"capture_generation", capture_generation},
      {"clock_generation", clock_generation}, {"raw_gid", raw_gid}, {"bridge_gid", bridge_gid}};
  }
};

struct Key
{
  Source source;
  uint64_t sequence{0};
  static Key parse(const Json & row) {return {Source::parse(row, true), number(row, "sequence")};}
  Json wire() const
  {
    auto result = source.wire();
    result["sequence"] = sequence;
    return result;
  }
};

struct Geometry
{
  uint64_t width, height;
  double resolution, origin_x, origin_y;
  static Geometry parse(const Json & row)
  {
    keys(row, {"width", "height", "resolution", "origin_x", "origin_y"});
    Geometry value{number(row, "width"), number(row, "height"), real(row, "resolution"),
      real(row, "origin_x"), real(row, "origin_y")};
    require(value.width > 0 && value.height > 0 && value.width <= 1048576 &&
      value.height <= 1048576 / value.width && value.resolution > 0 &&
      std::isfinite(value.origin_x + value.width * value.resolution) &&
      std::isfinite(value.origin_y + value.height * value.resolution), "invalid contract geometry");
    return value;
  }
  bool operator==(const Geometry & other) const
  {
    return std::tie(width, height, resolution, origin_x, origin_y) ==
           std::tie(other.width, other.height, other.resolution, other.origin_x, other.origin_y);
  }
  Json wire() const
  {
    return {{"width", width}, {"height", height}, {"resolution", resolution},
      {"origin_x", origin_x}, {"origin_y", origin_y}};
  }
};

Key scanKey(const senior_interfaces::msg::CurrentObstacles & scan)
{
  const auto & h = scan.capture;
  return {{scan.backend_boot, h.bridge_boot_id, h.source_epoch, h.raw_gid, h.bridge_gid,
      h.capture_generation, h.clock_generation}, h.sequence};
}

Geometry masterGeometry(const nav2_costmap_2d::Costmap2D & master)
{
  return {master.getSizeInCellsX(), master.getSizeInCellsY(), master.getResolution(),
    master.getOriginX(), master.getOriginY()};
}

std::string instanceNonce()
{
  std::array<unsigned char, 16> bytes{};
  require(getrandom(bytes.data(), bytes.size(), 0) == static_cast<ssize_t>(bytes.size()),
    "plugin instance entropy unavailable");
  std::string result;
  constexpr char hex[] = "0123456789abcdef";
  for (auto value : bytes) {result += hex[value >> 4]; result += hex[value & 15];}
  return result;
}

void requireSafeFootprintDependency()
{
  // Capability check against the actually linked shared checker, not its version
  // string. This private 6.4 KiB grid cannot clear or alter the real master.
  nav2_costmap_2d::Costmap2D probe(80, 80, 0.01, -0.4, -0.4, nav2_costmap_2d::FREE_SPACE);
  const auto mark = [&probe](double x, double y, unsigned char cost) {
      unsigned int mx, my;
      require(probe.worldToMap(x, y, mx, my), "dependency probe outside private grid");
      probe.setCost(mx, my, cost);
    };
  // The same S14 physical polygon (.674 x .540 plus its original .01 padding),
  // mixed edge cells, and nonlethal center. No real footprint is reconfigured.
  nav2_costmap_2d::Footprint polygon(4);
  polygon[0].x = polygon[3].x = -0.347;
  polygon[1].x = polygon[2].x = 0.347;
  polygon[0].y = polygon[1].y = -0.28;
  polygon[2].y = polygon[3].y = 0.28;
  mark(0.005, 0.005, 128);
  mark(0.005, -0.275, nav2_costmap_2d::NO_INFORMATION);
  mark(0.352, 0.005, nav2_costmap_2d::LETHAL_OBSTACLE);
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(&probe);
  for (int order = 0; order < 3; ++order) {
    require(checker.footprintCostAtPose(0.005, 0.005, 0, polygon) ==
      nav2_costmap_2d::LETHAL_OBSTACLE,
      "unsafe loaded Nav2 footprint checker: mixed unknown masks known lethal; install reviewed overlay");
    std::rotate(polygon.begin(), polygon.begin() + 1, polygon.end());
  }
}

class FileDescriptor
{
public:
  explicit FileDescriptor(int fd) : fd_(fd) {}
  ~FileDescriptor() {if (fd_ >= 0) {::close(fd_);}}
  int get() const {return fd_;}
  FileDescriptor(const FileDescriptor &) = delete;
  FileDescriptor & operator=(const FileDescriptor &) = delete;
private:
  int fd_;
};
}  // namespace

struct CurrentScanLayer::Authorization
{
  std::string owner, status, map_id, reference_generation, backend_boot, expected_grid_id;
  std::string global_permit, local_permit, selection_token, digest, canonical;
  uint64_t revision, epoch, reference_revision, floor_ns, navigation_floor_ns;
  uint64_t fixed_revision, domain, lease_ns, graph_sequence, selection_generation;
  std::optional<Source> source;
  std::optional<Key> floor;
  std::optional<Geometry> geometry;

  static std::shared_ptr<const Authorization> parse(const Json & row, const Contract & contract);
  bool sameAuthority(const Authorization & old) const
  {
    return owner == old.owner && epoch == old.epoch && map_id == old.map_id &&
           reference_revision == old.reference_revision && reference_generation == old.reference_generation &&
           backend_boot == old.backend_boot && source.has_value() == old.source.has_value() &&
           (!source || *source == *old.source) && floor_ns == old.floor_ns &&
           navigation_floor_ns == old.navigation_floor_ns && selection_token == old.selection_token &&
           selection_generation == old.selection_generation &&
           ((!floor && !old.floor) || (floor && old.floor && floor->source == old.floor->source &&
           floor->sequence == old.floor->sequence));
  }
};

struct CurrentScanLayer::Contract
{
  std::string directory, run_nonce, host_boot, role, permit, plugin_instance;
  std::string pinned_owner, last_reference, receipt_name;
  int directory_fd{-1};
  uint64_t seen_revision{0}, seen_epoch{0}, seen_domain{0}, last_sequence{0}, receipt_counter{0};
  bool seen_reference{false}, have_sequence{false}, io_healthy{false}, host_checked{false};
  std::optional<PublisherGid> publisher;
  std::optional<Json> pending;
  std::string last_status, last_reason;
  std::shared_ptr<const Authorization> last_record;
  uint64_t receipt_deadline{0};

  ~Contract() {if (directory_fd >= 0) {::close(directory_fd);}}

  void openRoot()
  {
    if (!host_checked) {
      std::ifstream boot("/proc/sys/kernel/random/boot_id");
      std::string actual;
      std::getline(boot, actual);
      require(boot.good() || boot.eof(), "host boot unavailable");
      require(actual == host_boot && !actual.empty(), "contract host boot mismatch");
      host_checked = true;
    }
    struct stat path_stat{}, fd_stat{};
    require(!directory.empty() && directory[0] == '/' &&
      ::lstat(directory.c_str(), &path_stat) == 0 && S_ISDIR(path_stat.st_mode) &&
      path_stat.st_uid == ::getuid() && (path_stat.st_mode & 0777) == 0700, "unsafe contract root");
    if (directory_fd < 0) {
      directory_fd = ::open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    }
    require(directory_fd >= 0 && ::fstat(directory_fd, &fd_stat) == 0 &&
      fd_stat.st_dev == path_stat.st_dev && fd_stat.st_ino == path_stat.st_ino, "contract root replaced");
  }

  Json readReference()
  {
    openRoot();
    FileDescriptor file(::openat(directory_fd, "reference.json", O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC));
    struct stat st{};
    require(file.get() >= 0 && ::fstat(file.get(), &st) == 0 && S_ISREG(st.st_mode) &&
      st.st_uid == ::getuid() && (st.st_mode & 0777) == 0600 &&
      st.st_size > 0 && st.st_size <= static_cast<off_t>(kRecordLimit), "unsafe reference record");
    std::array<char, kRecordLimit + 1> buffer{};
    size_t used = 0;
    while (used < buffer.size()) {
      const auto count = ::read(file.get(), buffer.data() + used, buffer.size() - used);
      if (count < 0 && errno == EINTR) {continue;}
      require(count >= 0, "reference read failed");
      if (count == 0) {break;}
      used += static_cast<size_t>(count);
    }
    require(used == static_cast<size_t>(st.st_size) && used <= kRecordLimit, "reference size changed");
    return strictJson(std::string(buffer.data(), used));
  }

  void writeCoverage(const Json & row)
  {
    openRoot();
    const auto bytes = row.dump();
    require(bytes.size() <= kRecordLimit, "coverage record too large");
    const std::string temporary = "." + receipt_name + "." + plugin_instance + ".tmp";
    FileDescriptor file(::openat(directory_fd, temporary.c_str(),
      O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600));
    require(file.get() >= 0, "coverage temporary unavailable");
    try {
      require(::fchmod(file.get(), 0600) == 0, "coverage permissions failed");
      size_t offset = 0;
      while (offset < bytes.size()) {
        const auto count = ::write(file.get(), bytes.data() + offset, bytes.size() - offset);
        if (count < 0 && errno == EINTR) {continue;}
        require(count > 0, "coverage write failed");
        offset += static_cast<size_t>(count);
      }
      require(::renameat(directory_fd, temporary.c_str(), directory_fd, receipt_name.c_str()) == 0,
        "coverage replace failed");
    } catch (...) {
      ::unlinkat(directory_fd, temporary.c_str(), 0);
      throw;
    }
  }
};

std::shared_ptr<const CurrentScanLayer::Authorization> CurrentScanLayer::Authorization::parse(
  const Json & row, const Contract & contract)
{
  keys(row, {"schema", "run_nonce", "host_boot_id", "owner_instance", "revision", "authority_epoch",
    "status", "reason", "map_id", "reference_revision", "reference_generation", "backend_boot", "source",
    "selection_token", "selection_generation", "acceptance_floor", "acceptance_floor_steady_ns",
    "navigation_floor_steady_ns", "fixed_revision", "graph_sequence", "anchor_digest", "expected_grid_id",
    "geometry", "coverage_domain_epoch", "global_instance_permit", "local_instance_permit", "lease_until_steady_ns"});
  require(number(row, "schema") == 1 && text(row, "run_nonce") == contract.run_nonce &&
    text(row, "host_boot_id", 36) == contract.host_boot, "contract run mismatch");
  auto result = std::make_shared<Authorization>();
  result->owner = text(row, "owner_instance");
  result->revision = number(row, "revision");
  result->epoch = number(row, "authority_epoch");
  result->status = text(row, "status", 16);
  text(row, "reason", 64);
  require(result->status == "bootstrap" || result->status == "selected" || result->status == "revoked",
    "unknown authorization status");
  result->map_id = text(row, "map_id");
  result->reference_revision = number(row, "reference_revision");
  result->reference_generation = text(row, "reference_generation");
  result->backend_boot = text(row, "backend_boot", 36);
  result->selection_token = text(row, "selection_token");
  result->selection_generation = number(row, "selection_generation");
  result->floor_ns = number(row, "acceptance_floor_steady_ns");
  result->navigation_floor_ns = number(row, "navigation_floor_steady_ns");
  result->fixed_revision = number(row, "fixed_revision");
  result->graph_sequence = number(row, "graph_sequence");
  result->digest = text(row, "anchor_digest", 64);
  result->expected_grid_id = text(row, "expected_grid_id");
  result->domain = number(row, "coverage_domain_epoch");
  result->global_permit = text(row, "global_instance_permit");
  result->local_permit = text(row, "local_instance_permit");
  result->lease_ns = number(row, "lease_until_steady_ns");
  if (!row.at("source").is_null()) {result->source = Source::parse(row.at("source"));}
  if (!row.at("acceptance_floor").is_null()) {result->floor = Key::parse(row.at("acceptance_floor"));}
  if (!row.at("geometry").is_null()) {result->geometry = Geometry::parse(row.at("geometry"));}
  if (result->status == "selected") {
    require(!result->owner.empty() && result->epoch > 0 && !result->map_id.empty() &&
      !result->reference_generation.empty() && !result->selection_token.empty() &&
      result->source && result->source->backend_boot == result->backend_boot && result->geometry &&
      !result->expected_grid_id.empty() && result->digest.size() == 64 &&
      std::all_of(result->digest.begin(), result->digest.end(), [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');}), "incomplete selected reference");
    require((contract.role == "global" ? result->global_permit : result->local_permit) == contract.permit,
      "role permit changed");
  }
  result->canonical = row.dump();
  return result;
}

CurrentScanLayer::CurrentScanLayer() = default;

CurrentScanLayer::~CurrentScanLayer()
{
  if (io_timer_) {io_timer_->cancel();}
  io_timer_.reset();
  subscription_.reset();
  parameter_callback_.reset();
}

void CurrentScanLayer::pollContract()
{
  // Only this mutually-exclusive I/O group touches directory_fd. The ROS node's
  // existing executor owns the callback: no worker thread, no file I/O under either mutex.
  try {
    auto candidate = Authorization::parse(contract_->readReference(), *contract_);
    std::lock_guard<std::mutex> lock(mutex_);
    auto & c = *contract_;
    require(c.pinned_owner.empty() || candidate->owner == c.pinned_owner,
      "reference owner changed within run");
    require(!c.seen_reference || (candidate->revision >= c.seen_revision &&
      candidate->epoch >= c.seen_epoch &&
      (candidate->epoch > c.seen_epoch || candidate->domain >= c.seen_domain)), "reference rolled back");
    require(!c.seen_reference || candidate->revision != c.seen_revision ||
      candidate->canonical == c.last_reference, "reference revision changed contents");
    if (c.last_record && candidate->epoch == c.last_record->epoch) {
      require(candidate->sameAuthority(*c.last_record), "authority changed without new epoch");
      require(c.last_record->status == "selected" || candidate->status != "selected",
        "retired authority reopened");
      if (candidate->geometry && c.last_record->geometry &&
        !(*candidate->geometry == *c.last_record->geometry))
      {
        require(candidate->domain > c.last_record->domain, "geometry changed without new domain");
      }
    }
    const bool new_authority = !c.last_record || !candidate->sameAuthority(*c.last_record);
    const bool new_domain = c.last_record && candidate->domain != c.last_record->domain;
    if (new_authority) {
      withdrawLocked();
      c.have_sequence = false;
      c.publisher.reset();
    } else if (new_domain) {
      // The same complete Q may cover the new domain after an actual master resize.
      ++generation_;
    }
    if (!candidate->owner.empty()) {c.pinned_owner = candidate->owner;}
    c.seen_reference = true;
    c.seen_revision = candidate->revision;
    c.seen_epoch = candidate->epoch;
    c.seen_domain = candidate->domain;
    c.last_reference = candidate->canonical;
    c.last_record = candidate;
    authorization_ = std::move(candidate);
    if (new_authority || new_domain) {
      queueCoverageLocked("invalid", new_authority ? "authority_changed" : "coverage_pending");
    }
    if (authorization_->status != "selected" || steadyNow() >= authorization_->lease_ns) {
      negativeLocked("reference_unavailable");
    }
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> lock(mutex_);
    authorization_.reset();
    contract_->io_healthy = false;
    negativeLocked("authorization_unavailable");
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Current Q authorization: %s", error.what());
  }

  std::optional<Json> pending;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (contract_->last_status == "ready" && steadyNow() >= contract_->receipt_deadline) {
      queueCoverageLocked("invalid", "coverage_expired");
    }
    pending.swap(contract_->pending);
  }
  if (!pending) {return;}
  try {
    contract_->writeCoverage(*pending);
    std::lock_guard<std::mutex> lock(mutex_);
    contract_->io_healthy = true;
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> lock(mutex_);
    contract_->io_healthy = false;
    withdrawLocked();
    // The sole writer cannot publish an older positive after a newer negative.
    // A failed replace leaves the remote slot bounded by its original deadline.
    queueCoverageLocked("invalid", "coverage_write_failed");
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Current Q coverage: %s", error.what());
  }
}

void CurrentScanLayer::receiveQ(CurrentObstacles::ConstSharedPtr scan, const rclcpp::MessageInfo & info)
{
  auto node = node_.lock();
  if (!node) {return;}
  bool owner = false;
  PublisherGid gid{};
  std::copy_n(info.get_rmw_message_info().publisher_gid.data, gid.size(), gid.begin());
  try {
    const auto writers = node->get_publishers_info_by_topic("/senior/mapping/current_obstacles");
    const auto maps = node->get_publishers_info_by_topic("/map");
    owner = writers.size() == 1 && maps.size() == 1 &&
      writers[0].topic_type() == "senior_interfaces/msg/CurrentObstacles" &&
      maps[0].topic_type() == "nav_msgs/msg/OccupancyGrid" &&
      writers[0].node_name() == maps[0].node_name() &&
      writers[0].node_namespace() == maps[0].node_namespace() && writers[0].endpoint_gid() == gid &&
      std::any_of(gid.begin(), gid.end(), [](uint8_t byte) {return byte != 0;});
  } catch (const std::exception &) {owner = false;}
  std::lock_guard<std::mutex> lock(mutex_);
  if (!owner) {negativeLocked("q_publisher_unavailable"); return;}
  // Discovery only checks the sample writer. The private original-owner grant
  // below, not the publisher's name or message contents, selects R and source.
  admitQLocked(std::move(scan), gid);
}

void CurrentScanLayer::admitQLocked(CurrentObstacles::ConstSharedPtr scan, const PublisherGid & gid)
{
  auto & c = *contract_;
  const auto & a = authorization_;
  if (!active_ || !requested_enabled_ || !c.io_healthy || !a || a->status != "selected" ||
    steadyNow() >= a->lease_ns || !a->source)
  {
    negativeLocked("reference_unavailable");
    return;
  }
  if (c.publisher && *c.publisher != gid) {negativeLocked("q_publisher_changed"); return;}
  const auto key = scanKey(*scan);
  if (!scan->valid) {
    if (key.source == *a->source && c.have_sequence && key.sequence < c.last_sequence) {return;}
    if (key.source == *a->source) {
      c.last_sequence = std::max(c.last_sequence, key.sequence);
      c.have_sequence = true;
    }
    negativeLocked("q_invalid");
    return;
  }
  if (!(key.source == *a->source) || scan->host_boot_id != c.host_boot || scan->map_id != a->map_id ||
    scan->reference_revision != a->reference_revision || scan->map_generation != a->reference_generation)
  {
    negativeLocked("q_identity_changed");
    return;
  }
  if (c.have_sequence && key.sequence <= c.last_sequence) {return;}
  const auto & h = scan->capture;
  const auto now = steadyNow();
  bool valid = h.valid && scan->frame_id == "map" && scan->points.size() <= 2048 &&
    h.stamp.sec >= 0 && h.received_ros.sec >= 0 && h.stamp.nanosec < 1000000000 &&
    h.received_ros.nanosec < 1000000000 && h.sample_steady_ns <= h.received_steady_ns &&
    h.received_steady_ns <= now && h.sample_steady_ns > a->floor_ns &&
    h.sample_steady_ns > a->navigation_floor_ns && scan->valid_until_steady_ns > now &&
    scan->valid_until_steady_ns >= h.sample_steady_ns &&
    scan->valid_until_steady_ns - h.sample_steady_ns <= kLifetimeNs;
  if (valid) {
    const uint64_t stamp = static_cast<uint64_t>(h.stamp.sec) * 1000000000 + h.stamp.nanosec;
    const uint64_t receipt = static_cast<uint64_t>(h.received_ros.sec) * 1000000000 + h.received_ros.nanosec;
    valid = receipt >= stamp && receipt - stamp == h.received_steady_ns - h.sample_steady_ns;
  }
  if (a->floor && a->floor->source == key.source && key.sequence <= a->floor->sequence) {valid = false;}
  for (const auto & p : scan->points) {
    valid = valid && std::isfinite(p.x) && std::isfinite(p.y) && p.z == 0.0F;
  }
  if (!valid) {negativeLocked("q_unqualified"); return;}
  c.publisher = gid;
  c.have_sequence = true;
  c.last_sequence = key.sequence;
  latest_ = std::move(scan);
}

void CurrentScanLayer::negativeLocked(const std::string & reason)
{
  if (latest_) {withdrawLocked();}
  const std::string status = !active_ ? "retired" : (!requested_enabled_ ? "disabled" : "invalid");
  if (contract_ && (contract_->last_status != status || contract_->last_reason != reason)) {
    queueCoverageLocked(status, reason);
  }
}

void CurrentScanLayer::queueCoverageLocked(const std::string & status, const std::string & reason,
  nav2_costmap_2d::Costmap2D * master, bool covered, std::optional<Bounds> updated)
{
  if (!contract_) {return;}
  auto & c = *contract_;
  const bool positive = status == "ready";
  const auto a = positive ? cycle_.authorization : (authorization_ ? authorization_ : c.last_record);
  const auto q = positive ? cycle_.scan : CurrentObstacles::ConstSharedPtr{};
  const auto completed = steadyNow();
  const uint64_t deadline = positive && a && q ?
    std::min({q->valid_until_steady_ns, a->lease_ns, cycle_.started_ns + kLifetimeNs}) : 0;
  if (positive && completed >= deadline) {
    current_ = false;
    queueCoverageLocked("invalid", "expired", master);
    return;
  }
  Json bounds = nullptr;
  if (updated) {bounds = Json::array({updated->min_x, updated->min_y, updated->max_x, updated->max_y});}
  Json row{
    {"schema", uint64_t{1}}, {"run_nonce", c.run_nonce}, {"host_boot_id", c.host_boot},
    {"owner_instance", a ? a->owner : ""}, {"authority_epoch", a ? a->epoch : uint64_t{0}},
    {"role", c.role}, {"role_instance_permit", c.permit}, {"plugin_instance", c.plugin_instance},
    {"cycle", ++c.receipt_counter}, {"status", status}, {"reason", reason},
    {"map_id", a ? a->map_id : ""}, {"reference_revision", a ? a->reference_revision : uint64_t{0}},
    {"reference_generation", a ? a->reference_generation : ""}, {"backend_boot", a ? a->backend_boot : ""},
    {"key", q ? scanKey(*q).wire() : Json(nullptr)},
    {"sample_steady_ns", q ? q->capture.sample_steady_ns : uint64_t{0}},
    {"scan_deadline_steady_ns", q ? q->valid_until_steady_ns : uint64_t{0}},
    {"coverage_domain_epoch", a ? a->domain : uint64_t{0}},
    {"geometry", a && a->geometry ? a->geometry->wire() : Json(nullptr)},
    {"expected_fixed_revision", a ? a->fixed_revision : uint64_t{0}},
    {"expected_grid_id", a ? a->expected_grid_id : ""},
    {"cycle_started_steady_ns", positive ? cycle_.started_ns : uint64_t{0}},
    {"cycle_completed_steady_ns", completed}, {"valid_until_steady_ns", deadline},
    {"master_frame", master ? layered_costmap_->getGlobalFrameID() : ""},
    {"master_geometry", master ? masterGeometry(*master).wire() : Json(nullptr)},
    {"updated_bounds", bounds}, {"global_all_hits_covered", covered && c.role == "global"},
    {"local_in_window_hits_covered", covered && c.role == "local"},
    {"transform_valid", master && cycle_.transform_valid},
    {"cleanup_complete", master && cycle_.cleanup_complete}};
  c.last_status = status;
  c.last_reason = reason;
  c.receipt_deadline = deadline;
  c.pending = std::move(row);
}

void CurrentScanLayer::Bounds::include(double x, double y)
{
  min_x = std::min(min_x, x);
  min_y = std::min(min_y, y);
  max_x = std::max(max_x, x);
  max_y = std::max(max_y, y);
}

void CurrentScanLayer::Bounds::mergeInto(double * x0, double * y0, double * x1, double * y1) const
{
  *x0 = std::min(*x0, min_x);
  *y0 = std::min(*y0, min_y);
  *x1 = std::max(*x1, max_x);
  *y1 = std::max(*y1, max_y);
}

uint64_t CurrentScanLayer::steadyNow()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool CurrentScanLayer::isFresh(const CurrentObstacles::ConstSharedPtr & scan) const
{
  const auto now = steadyNow();
  return contract_ && contract_->io_healthy && authorization_ &&
         authorization_->status == "selected" && now < authorization_->lease_ns &&
         scan && scan->valid && scan->frame_id == "map" &&
         scan->points.size() <= 2048 && now < scan->valid_until_steady_ns;
}

void CurrentScanLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("CurrentScanLayer lost lifecycle node");
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".enabled", requested_enabled_);
  enabled_ = requested_enabled_;
  current_ = false;
  contract_ = std::make_unique<Contract>();
  std::string mode;
  const auto get = [this, &node](const std::string & key, std::string & value,
      const std::string & fallback = "") {
      declareParameter(key, rclcpp::ParameterValue(fallback));
      node->get_parameter(name_ + "." + key, value);
    };
  get("mode", mode, "live");
  get("contract_directory", contract_->directory);
  get("run_nonce", contract_->run_nonce);
  get("host_boot_id", contract_->host_boot);
  get("role", contract_->role);
  get("role_instance_permit", contract_->permit);
  require(mode == "live" && !contract_->directory.empty() && !contract_->run_nonce.empty() &&
    !contract_->host_boot.empty() && !contract_->permit.empty() &&
    contract_->run_nonce.size() <= 128 && contract_->permit.size() <= 128 &&
    (contract_->role == "global" || contract_->role == "local"), "missing live current-layer contract");
  require((contract_->role == "global" && !layered_costmap_->isRolling() &&
    layered_costmap_->getGlobalFrameID() == "map") || (contract_->role == "local" &&
    layered_costmap_->isRolling() && (layered_costmap_->getGlobalFrameID() == "map" ||
    layered_costmap_->getGlobalFrameID() == "odom")), "current-layer role/frame mismatch");
  requireSafeFootprintDependency();
  contract_->plugin_instance = instanceNonce();
  contract_->receipt_name = contract_->role + "-coverage.json";
  std::weak_ptr<nav2_costmap_2d::Layer> weak_self;
  for (const auto & plugin : *layered_costmap_->getPlugins()) {
    if (plugin.get() == this) {weak_self = plugin; break;}
  }
  require(!weak_self.expired(), "initialize CurrentScanLayer through its LayeredCostmap owner");
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node->create_subscription<CurrentObstacles>(
    "/senior/mapping/current_obstacles", rclcpp::QoS(1).reliable().durability_volatile(),
    [weak_self](CurrentObstacles::ConstSharedPtr scan, const rclcpp::MessageInfo & info) {
      if (auto self = weak_self.lock()) {
        static_cast<CurrentScanLayer *>(self.get())->receiveQ(std::move(scan), info);
      }
    }, options);
  // The normal costmap node executor spins automatically-added groups, separate
  // from Nav2's explicitly-owned plugin callback group. No new executor/thread.
  // Weak callbacks hold the plugin alive only for an in-flight callback during
  // cleanup; asynchronous handlers never dereference the LayeredCostmap owner.
  io_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  io_timer_ = node->create_wall_timer(std::chrono::milliseconds(20), [weak_self]() {
      if (auto self = weak_self.lock()) {static_cast<CurrentScanLayer *>(self.get())->pollContract();}
    }, io_group_);
  parameter_callback_ = node->add_on_set_parameters_callback(
    [weak_self](const std::vector<rclcpp::Parameter> & values) {
      if (auto self = weak_self.lock()) {
        return static_cast<CurrentScanLayer *>(self.get())->parameters(values);
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "current layer retired";
      return result;
    });
}

rcl_interfaces::msg::SetParametersResult CurrentScanLayer::parameters(
  const std::vector<rclcpp::Parameter> & values)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & value : values) {
    const auto & name = value.get_name();
    if (name == name_ + ".mode" || name == name_ + ".contract_directory" ||
      name == name_ + ".run_nonce" || name == name_ + ".host_boot_id" ||
      name == name_ + ".role" || name == name_ + ".role_instance_permit")
    {
      result.successful = false;
      result.reason = "current-layer run/role contract requires a new lifecycle instance";
      return result;
    }
  }
  for (const auto & value : values) {
    if (value.get_name() == name_ + ".enabled" &&
      value.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
    {
      result.successful = false;
      result.reason = "enabled must be boolean";
      return result;
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & value : values) {
    if (value.get_name() == name_ + ".enabled" && value.as_bool() != requested_enabled_) {
      requested_enabled_ = value.as_bool();
      withdrawLocked();
      queueCoverageLocked(requested_enabled_ ? "invalid" : "disabled", "enabled_changed");
    }
  }
  return result;
}

void CurrentScanLayer::withdrawLocked()
{
  latest_.reset();
  ++generation_;
  // Never drop rendered_: a later bounds/costs pass must retire its contribution.
}

void CurrentScanLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  withdrawLocked();
  current_ = false;
  queueCoverageLocked("invalid", "reset");
}

void CurrentScanLayer::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = true;
  withdrawLocked();
  queueCoverageLocked("invalid", "activated");
}

void CurrentScanLayer::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  withdrawLocked();
  queueCoverageLocked("retired", "deactivated");
}

void CurrentScanLayer::matchSize()
{
  // No private raster. Preserve the already-rendered world bounds through resize;
  // the next bounds pass and StaticLayer rebuild the actual new master domain.
}

bool CurrentScanLayer::projectCycleLocked()
{
  const auto & scan = cycle_.scan;
  if (!isFresh(scan)) {
    return false;
  }
  tf2::Transform transform;
  transform.setIdentity();
  const auto & frame = layered_costmap_->getGlobalFrameID();
  if (frame != "map") {
    try {
      // Map-frame sample points are already motion-compensated by the owner.
      // Only change coordinates into this cycle's costmap frame, without re-sampling.
      tf2::fromMsg(tf_->lookupTransform(frame, "map", tf2::TimePointZero).transform, transform);
    } catch (const tf2::TransformException &) {
      return false;
    }
  }
  cycle_.points.reserve(scan->points.size());
  for (const auto & point : scan->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.z != 0.0F) {
      return false;
    }
    const auto projected = transform * tf2::Vector3(point.x, point.y, 0.0);
    geometry_msgs::msg::Point output;
    output.x = projected.x();
    output.y = projected.y();
    if (!std::isfinite(output.x) || !std::isfinite(output.y)) {
      return false;
    }
    cycle_.points.push_back(output);
  }
  return true;
}

void CurrentScanLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  enabled_ = requested_enabled_;
  current_ = false;  // Coverage is established only by the actual updateCosts master.
  cycle_ = Cycle{};
  cycle_.generation = generation_;
  cycle_.started_ns = steadyNow();
  cycle_.authorization = authorization_;
  if (enabled_ && active_) {
    cycle_.scan = latest_;
    cycle_.transform_valid = projectCycleLocked();
    if (!cycle_.transform_valid) {
      cycle_.scan.reset();
      cycle_.points.clear();
      negativeLocked("q_or_transform_unavailable");
    }
  }
  if (rendered_) {
    rendered_->mergeInto(min_x, min_y, max_x, max_y);
  }
  // A valid empty Q must still get a costs pass. Also makes withdrawal observable
  // when nothing was ever rendered. Inflation expands this minimal update window.
  Bounds bounds{robot_x, robot_y, robot_x, robot_y};
  for (const auto & point : cycle_.points) {
    bounds.include(point.x, point.y);
  }
  bounds.mergeInto(min_x, min_y, max_x, max_y);
}

void CurrentScanLayer::updateCosts(nav2_costmap_2d::Costmap2D & master,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // LayeredCostmap has reset the union window and StaticLayer has restored O/F/U.
  // This plugin only writes current lethal cells; it never writes free or unknown.
  const double resolution = master.getResolution();
  const Bounds updated{master.getOriginX() + min_i * resolution,
    master.getOriginY() + min_j * resolution, master.getOriginX() + max_i * resolution,
    master.getOriginY() + max_j * resolution};
  const Bounds domain{master.getOriginX(), master.getOriginY(),
    master.getOriginX() + master.getSizeInCellsX() * resolution,
    master.getOriginY() + master.getSizeInCellsY() * resolution};
  cycle_.cleanup_complete = true;
  if (rendered_) {
    const Bounds intersection{std::max(rendered_->min_x, domain.min_x),
      std::max(rendered_->min_y, domain.min_y), std::min(rendered_->max_x, domain.max_x),
      std::min(rendered_->max_y, domain.max_y)};
    const double epsilon = resolution * 1e-6;
    cycle_.cleanup_complete = intersection.min_x >= intersection.max_x ||
      intersection.min_y >= intersection.max_y ||
      (updated.min_x <= intersection.min_x + epsilon && updated.min_y <= intersection.min_y + epsilon &&
      updated.max_x + epsilon >= intersection.max_x && updated.max_y + epsilon >= intersection.max_y);
  }
  if (cycle_.cleanup_complete) {rendered_.reset();}
  current_ = false;
  if (!active_ || !requested_enabled_ || cycle_.generation != generation_ || !isFresh(cycle_.scan) ||
    !cycle_.authorization || !authorization_->sameAuthority(*cycle_.authorization) ||
    authorization_->domain != cycle_.authorization->domain)
  {
    negativeLocked("q_or_authority_unavailable");
    queueCoverageLocked(!active_ ? "retired" : (!requested_enabled_ ? "disabled" : "invalid"),
      "q_or_authority_unavailable", &master, false, updated);
    return;
  }
  bool covered = cycle_.cleanup_complete;
  if (contract_->role == "global") {
    const auto & expected = *cycle_.authorization->geometry;
    const auto actual = masterGeometry(master);
    // OccupancyGrid stores resolution as float32. Account only for that
    // representation error, never an uninstalled row/column or truncated Q.
    const double delta = std::abs(actual.resolution - expected.resolution);
    const double epsilon = std::max(expected.width, expected.height) * delta +
      expected.resolution * 1e-6;
    covered = covered && delta <= expected.resolution * 1e-6 &&
      actual.origin_x <= expected.origin_x + epsilon && actual.origin_y <= expected.origin_y + epsilon &&
      actual.origin_x + actual.width * actual.resolution + epsilon >=
      expected.origin_x + expected.width * expected.resolution &&
      actual.origin_y + actual.height * actual.resolution + epsilon >=
      expected.origin_y + expected.height * expected.resolution;
  }
  for (const auto & point : cycle_.points) {
    unsigned int mx, my;
    if (!master.worldToMap(point.x, point.y, mx, my)) {
      // Rolling local windows retain the full Q, with off-window hits covered by global.
      covered = covered && contract_->role == "local";
      continue;
    }
    if (static_cast<int>(mx) < min_i || static_cast<int>(my) < min_j ||
      static_cast<int>(mx) >= max_i || static_cast<int>(my) >= max_j)
    {
      covered = false;
      continue;
    }
    master.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    double x, y;
    master.mapToWorld(mx, my, x, y);
    // Save the cell's actual world extent, not the last-received Q's bounds.
    const double half = master.getResolution() * 0.5;
    if (!rendered_) {
      rendered_ = Bounds{x - half, y - half, x + half, y + half};
    } else {
      rendered_->include(x - half, y - half);
      rendered_->include(x + half, y + half);
    }
  }
  current_ = covered;
  // This receipt proves this plugin's completed contribution and actual master
  // geometry, not a content-G installation ACK from stock StaticLayer/inflation.
  queueCoverageLocked(covered ? "ready" : "pending",
    covered ? "" : (cycle_.cleanup_complete ? "coverage_pending" : "cleanup_pending"),
    &master, covered, updated);
}
}  // namespace senior_navigation

PLUGINLIB_EXPORT_CLASS(senior_navigation::CurrentScanLayer, nav2_costmap_2d::Layer)
