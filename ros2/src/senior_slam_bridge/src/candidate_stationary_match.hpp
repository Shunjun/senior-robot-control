#ifndef SENIOR_SLAM_BRIDGE__CANDIDATE_STATIONARY_MATCH_HPP_
#define SENIOR_SLAM_BRIDGE__CANDIDATE_STATIONARY_MATCH_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <karto_sdk/Mapper.h>

namespace senior_slam_bridge
{
struct StationaryMatchQuery
{
  // Beam layout is in native Karto sensor convention.
  double first_angle, angle_step;
  karto::RangeReadingsVector ranges;
};

struct StationaryMatchDiagnostic
{
  double response;
  karto::Pose2 map_sensor_pose;
  std::vector<std::int32_t> reference_ids;
};

constexpr std::size_t stationary_match_min_beams = 32;
constexpr double stationary_match_range_limit = 8.0;
constexpr double stationary_match_full_turn = 6.28318530717958647692;
constexpr double stationary_match_layout_tolerance = 1e-5;

constexpr double stationary_match_coarse_extent = 0.2;
constexpr double stationary_match_fine_extent = 0.02;

struct StationaryMatchSeed
{
  double x, y, yaw;
};

inline bool valid_stationary_match_seed(const StationaryMatchSeed & seed)
{
  constexpr double max_coordinate = 1000000.0;
  return std::abs(seed.x) <= max_coordinate && std::abs(seed.y) <= max_coordinate &&
         std::abs(seed.yaw) <= stationary_match_full_turn;
}

inline bool disjoint_stationary_match_seeds(
  const StationaryMatchSeed & first, const StationaryMatchSeed & second)
{
  // Touching boxes overlap. Include a small margin for the boundary arithmetic.
  constexpr double width = 2 * (stationary_match_coarse_extent + stationary_match_fine_extent);
  constexpr double boundary_margin = 1e-9;
  return std::abs(first.x - second.x) > width + boundary_margin ||
         std::abs(first.y - second.y) > width + boundary_margin ||
         std::abs(std::remainder(first.yaw - second.yaw, stationary_match_full_turn)) >
         width + boundary_margin;
}

inline bool valid_stationary_match_query(const StationaryMatchQuery & query)
{
  const auto count = query.ranges.size();
  if (count < stationary_match_min_beams || count > 2048 ||
    !(std::abs(query.first_angle) <= stationary_match_full_turn) ||
    !(query.angle_step > 0 && (count - 1) * query.angle_step <=
    stationary_match_full_turn + stationary_match_layout_tolerance))
  {
    return false;
  }
  std::size_t usable = 0;
  for (const auto range : query.ranges) {
    if (std::isnan(range) || range < 0) {
      return false;
    }
    if (range > 0 && range < stationary_match_range_limit) {
      ++usable;
    }
  }
  return usable >= stationary_match_min_beams;
}

inline std::optional<StationaryMatchDiagnostic> stationary_match(
  const karto::Mapper & loaded, const StationaryMatchQuery & query,
  const StationaryMatchSeed & seed, const std::string & private_query_name)
{
  constexpr std::size_t max_vertices = 16384;
  constexpr std::size_t max_references = 8;
  constexpr double reference_radius = 4.0;
  constexpr double resolution = stationary_match_fine_extent;
  const auto count = query.ranges.size();
  const auto * graph = loaded.GetGraph();
  if (!graph || private_query_name.empty() || private_query_name.size() > 128) {
    return std::nullopt;
  }

  struct Reference
  {
    const karto::LocalizedRangeScan * scan;
    karto::Pose2 sensor;
    double distance;
  };
  std::vector<Reference> references;
  references.reserve(max_references + 1);
  const karto::LaserRangeFinder * archived = nullptr;
  std::size_t vertices = 0;
  for (const auto & group : graph->GetVertices()) {
    for (const auto & entry : group.second) {
      const auto * scan = entry.second ? entry.second->GetObject() : nullptr;
      if (!scan || ++vertices > max_vertices ||
        scan->GetSensorName().ToString() == private_query_name)
      {
        return std::nullopt;
      }
      const auto * model = scan->GetLaserRangeFinder();
      if (!model || (archived && archived != model) || scan->GetUniqueId() < 0 ||
        model->GetNumberOfRangeReadings() != count || scan->GetNumberOfRangeReadings() != count ||
        !std::isfinite(model->GetMinimumAngle()) ||
        !std::isfinite(model->GetAngularResolution()) ||
        std::abs(model->GetMinimumAngle() - query.first_angle) > stationary_match_layout_tolerance ||
        std::abs(model->GetAngularResolution() - query.angle_step) > stationary_match_layout_tolerance)
      {
        return std::nullopt;
      }
      archived = model;
      const auto pose = scan->GetSensorPose();
      const double distance = std::hypot(pose.GetX() - seed.x, pose.GetY() - seed.y);
      if (!std::isfinite(distance) || !std::isfinite(pose.GetHeading())) {
        return std::nullopt;
      }
      if (distance <= reference_radius) {
        const auto nearer = [](const Reference & a, const Reference & b) {
            return a.distance == b.distance ? a.scan->GetUniqueId() < b.scan->GetUniqueId() :
                   a.distance < b.distance;
          };
        const Reference candidate{scan, pose, distance};
        const auto position = std::lower_bound(
          references.begin(), references.end(), candidate, nearer);
        if (position != references.end() || references.size() < max_references) {
          references.insert(position, candidate);
          if (references.size() > max_references) {
            references.pop_back();
          }
        }
      }
    }
  }
  if (!archived || references.empty()) {
    return std::nullopt;
  }

  const double min_range = archived->GetMinimumRange();
  const double model_range = archived->GetMaximumRange();
  const double model_threshold = archived->GetRangeThreshold();
  const double max_range = std::min({stationary_match_range_limit, model_range, model_threshold});
  if (!std::isfinite(min_range) || !std::isfinite(model_range) ||
    !std::isfinite(model_threshold) || min_range < 0 || min_range >= max_range)
  {
    return std::nullopt;
  }
  const auto valid = [&](double value) {
      return std::isfinite(value) && value >= min_range && value < max_range;
    };
  if (static_cast<std::size_t>(std::count_if(query.ranges.begin(), query.ranges.end(), valid)) <
    stationary_match_min_beams) {
    return std::nullopt;
  }
  auto ranges = query.ranges;
  for (auto & range : ranges) {
    if (!valid(range)) {
      range = std::numeric_limits<double>::quiet_NaN();
    }
  }

  const karto::Name name(private_query_name);
  auto * sensors = karto::SensorManager::GetInstance();
  //Compatible: Humble resolves query scan sensors through the process-wide registry.
  // The service supplies a fresh per-boot name in an isolated candidate.
  auto unregister = [sensors](karto::LaserRangeFinder * sensor) {
      sensors->UnregisterSensor(sensor);
      delete sensor;
    };
  std::unique_ptr<karto::LaserRangeFinder> unregistered(
    karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, name));
  if (!unregistered) {
    return std::nullopt;
  }
  unregistered->SetMinimumRange(min_range);
  unregistered->SetRangeThreshold(max_range);
  unregistered->SetMaximumRange(max_range);
  unregistered->SetMinimumAngle(query.first_angle);
  unregistered->SetMaximumAngle(query.first_angle + (count - 1) * query.angle_step);
  unregistered->SetAngularResolution(query.angle_step);
  unregistered->SetRangeThreshold(max_range);
  if (unregistered->GetNumberOfRangeReadings() != count) {
    return std::nullopt;
  }
  sensors->RegisterSensor(unregistered.get());
  std::unique_ptr<karto::LaserRangeFinder, decltype(unregister)> laser(
    unregistered.release(), unregister);

  karto::Mapper scratch;
  std::unique_ptr<karto::ScanMatcher> matcher(
    karto::ScanMatcher::Create(&scratch, 0.4, resolution, 0.04, stationary_match_range_limit));
  if (!matcher) {
    return std::nullopt;
  }
  auto & grid = *matcher->GetCorrelationGrid();
  grid.Clear();
  const auto roi = grid.GetROI();
  const double half_x = 0.5 * (roi.GetWidth() - 1) * resolution;
  const double half_y = 0.5 * (roi.GetHeight() - 1) * resolution;
  grid.GetCoordinateConverter()->SetOffset(karto::Vector2<double>(-half_x, -half_y));
  StationaryMatchDiagnostic result{};
  for (const auto & ref : references) {
    const auto * readings = ref.scan->GetRangeReadings();
    if (!readings) {
      return std::nullopt;
    }
    bool used = false;
    for (std::size_t ray = 0; ray < count; ++ray) {
      if (!valid(readings[ray])) {
        continue;
      }
      const double theta = ref.sensor.GetHeading() + archived->GetMinimumAngle() +
        ray * archived->GetAngularResolution();
      const double x = ref.sensor.GetX() - seed.x + readings[ray] * std::cos(theta);
      const double y = ref.sensor.GetY() - seed.y + readings[ray] * std::sin(theta);
      if (std::abs(x) > half_x || std::abs(y) > half_y) {
        continue;
      }
      const auto cell = grid.WorldToGrid(karto::Vector2<double>(x, y));
      if (cell.GetX() < 0 || cell.GetY() < 0 ||
        cell.GetX() >= roi.GetWidth() || cell.GetY() >= roi.GetHeight())
      {
        continue;
      }
      grid.GetDataPointer()[grid.GridIndex(cell)] = karto::GridStates_Occupied;
      grid.SmearPoint(cell);
      used = true;
    }
    if (used) {
      result.reference_ids.push_back(ref.scan->GetUniqueId());
    }
  }
  if (result.reference_ids.empty()) {
    return std::nullopt;
  }

  karto::LocalizedRangeScan scan(name, ranges);
  scan.SetCorrectedPose(karto::Pose2(0, 0, 0));
  karto::Pose2 coarse, fitted;
  karto::Matrix3 covariance;
  matcher->CorrelateScan(
    &scan, karto::Pose2(0, 0, std::remainder(seed.yaw, stationary_match_full_turn)),
    karto::Vector2<double>(stationary_match_coarse_extent, stationary_match_coarse_extent),
    karto::Vector2<double>(0.04, 0.04),
    stationary_match_coarse_extent, 0.02, false, coarse, covariance, false);
  result.response = matcher->CorrelateScan(
    &scan, coarse, karto::Vector2<double>(stationary_match_fine_extent, stationary_match_fine_extent),
    karto::Vector2<double>(resolution, resolution), stationary_match_fine_extent, 0.005, false,
    fitted, covariance, true);
  result.map_sensor_pose = karto::Pose2(
    seed.x + fitted.GetX(), seed.y + fitted.GetY(), fitted.GetHeading());
  if (!std::isfinite(result.response) || !std::isfinite(result.map_sensor_pose.GetX()) ||
    !std::isfinite(result.map_sensor_pose.GetY()) || !std::isfinite(fitted.GetHeading()))
  {
    return std::nullopt;
  }
  return result;
}
}  // namespace senior_slam_bridge
#endif  // SENIOR_SLAM_BRIDGE__CANDIDATE_STATIONARY_MATCH_HPP_
