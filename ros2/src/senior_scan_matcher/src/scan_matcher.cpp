#include "senior_scan_matcher/scan_matcher.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <utility>

#include <karto_sdk/Mapper.h>

namespace senior_scan_matcher {
Pose compose(const Pose & a, const Pose & b)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {a.x + c*b.x - s*b.y, a.y + s*b.x + c*b.y,
    std::atan2(std::sin(a.yaw+b.yaw), std::cos(a.yaw+b.yaw))};
}
Pose inverse(const Pose & p)
{
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  return {-c*p.x-s*p.y, s*p.x-c*p.y, -p.yaw};
}
Pose to_body_delta(const Pose & sensor_delta, const Pose & sensor_to_body)
{
  return compose(compose(sensor_to_body, sensor_delta), inverse(sensor_to_body));
}
namespace {
constexpr double resolution = 0.02;
constexpr double search_size = 0.4;
constexpr double smear = 0.04;
constexpr double angular_offset = 0.2;
constexpr double angular_step = 0.02;
std::atomic<unsigned> next_sensor{0};
using Point = std::pair<double, double>;

bool valid_scan(const LaserScan & scan, const MatcherLimits & limits)
{
  return std::isfinite(scan.stamp) && scan.stamp > 0 &&
    std::isfinite(scan.angle_min) && std::isfinite(scan.angle_increment) &&
    scan.angle_increment > 0 && std::isfinite(scan.range_min) &&
    std::isfinite(scan.range_max) && scan.range_min >= 0 &&
    scan.range_max > scan.range_min && limits.range_limit > scan.range_min &&
    scan.ranges.size() >= limits.min_rays && scan.ranges.size() <= 10000;
}

std::vector<Point> points(const LaserScan & scan, const MatcherLimits & limits)
{
  std::vector<Point> result;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max || r > limits.range_limit)
      continue;
    const double a = scan.angle_min + i * scan.angle_increment;
    result.emplace_back(r * std::cos(a), r * std::sin(a));
  }
  return result;
}

double spread(const std::vector<Point> & pts)
{
  if (pts.empty()) return 0;
  double mx = 0, my = 0, xx = 0, xy = 0, yy = 0;
  for (const auto & p : pts) { mx += p.first; my += p.second; }
  mx /= pts.size(); my /= pts.size();
  for (const auto & p : pts) {
    const double x = p.first - mx, y = p.second - my;
    xx += x*x; xy += x*y; yy += y*y;
  }
  xx /= pts.size(); xy /= pts.size(); yy /= pts.size();
  return 0.5 * (xx + yy - std::hypot(xx - yy, 2 * xy));
}

// Point-to-line information rejects geometry that cannot constrain a body
// translation or rotation, even when a correlation peak has high response.
// Short adjacent chords overfit range quantization. Use nine consecutive
// original valid beams and reject windows that do not form one local surface;
// gaps and object edges must not manufacture a line normal. Thresholds
// qualify geometry, not pose accuracy.
bool observable(const LaserScan & scan, const MatcherLimits & limits, MatchDiagnostics & diagnostics)
{
  double xx = 0, xy = 0, yy = 0, xr = 0, yr = 0, rr = 0;
  unsigned count = 0;
  std::deque<Point> window;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max ||
        range > limits.range_limit) {
      window.clear();
      continue;
    }
    const double angle = scan.angle_min + i * scan.angle_increment;
    const Point current{range * std::cos(angle), range * std::sin(angle)};
    window.push_back(current);
    if (window.size() > 9) window.pop_front();
    if (window.size() == 9) {
      const auto & first = window.front();
      const double dx = current.first - first.first;
      const double dy = current.second - first.second;
      const double length = std::hypot(dx, dy);
      if (length >= 0.03 && length <= 0.5) {
        const double nx = -dy / length, ny = dx / length;
        const bool planar = std::all_of(window.begin(), window.end(), [&](const Point & p) {
          return std::abs(nx*(p.first-first.first) + ny*(p.second-first.second)) <= 0.02;
        });
        if (!planar) continue;
        const double x = (current.first + first.first) * 0.5;
        const double y = (current.second + first.second) * 0.5;
        const double rotation = -nx * y + ny * x;
        xx += nx * nx; xy += nx * ny; yy += ny * ny;
        xr += nx * rotation; yr += ny * rotation; rr += rotation * rotation;
        ++count;
      }
    }
  }
  auto & geometry = *diagnostics.geometry;
  geometry.normal_windows = count;
  if (count < std::max(10u, limits.min_rays / 4)) {
    diagnostics.failures.push_back("geometry.normal_windows"); return false;
  }
  const double min_translation = 0.5 * (xx + yy - std::hypot(xx - yy, 2 * xy)) / count;
  const double determinant = xx * yy - xy * xy;
  geometry.translation_information = min_translation; geometry.determinant = determinant;
  if (min_translation < limits.min_translation_observability || determinant <= 1e-12) {
    diagnostics.failures.push_back("geometry.translation_information"); return false;
  }
  // Eliminate translation from the 3x3 information matrix; this rejects a
  // circular surface whose apparent yaw is indistinguishable from no motion.
  const double independent_yaw =
    (rr - (yy*xr*xr - 2*xy*xr*yr + xx*yr*yr) / determinant) / count;
  geometry.yaw_information = independent_yaw;
  const bool qualified = std::isfinite(independent_yaw) && independent_yaw >= limits.min_yaw_observability;
  if (!qualified) diagnostics.failures.push_back("geometry.yaw_information");
  return qualified;
}
}

class LocalMatcher::Impl {
public:
  explicit Impl(MatcherLimits value) : limits(value), name("senior_local_" + std::to_string(next_sensor++))
  {
    if (!std::isfinite(limits.max_interval) || limits.max_interval <= 0 ||
        !std::isfinite(limits.range_limit) || limits.range_limit <= 0 ||
        !std::isfinite(limits.min_response) || limits.min_response <= 0 || limits.min_response > 1 ||
        !std::isfinite(limits.max_residual) || limits.max_residual <= 0 ||
        !std::isfinite(limits.min_inlier_fraction) || limits.min_inlier_fraction <= 0 ||
        limits.min_inlier_fraction > 1 || !std::isfinite(limits.min_spread) || limits.min_spread <= 0 ||
        !std::isfinite(limits.min_translation_observability) || limits.min_translation_observability <= 0 ||
        limits.min_translation_observability > 0.5 || !std::isfinite(limits.min_yaw_observability) ||
        limits.min_yaw_observability <= 0 ||
        !std::isfinite(limits.reanchor_distance) || limits.reanchor_distance <= 0 ||
        limits.reanchor_distance >= 0.19 || !std::isfinite(limits.reanchor_yaw) ||
        limits.reanchor_yaw <= 0 || limits.reanchor_yaw >= 0.19 ||
        !std::isfinite(limits.max_variance) || limits.max_variance <= 0 || limits.min_rays < 3)
      throw std::invalid_argument("invalid matcher limits");
    matcher.reset(karto::ScanMatcher::Create(&mapper, search_size, resolution, smear, limits.range_limit));
    if (!matcher) throw std::runtime_error("cannot create scan matcher");
  }
  ~Impl() { drop_sensor(); }

  void drop_sensor()
  {
    if (registered) karto::SensorManager::GetInstance()->UnregisterSensor(sensor.get());
    registered = false;
    sensor.reset();
  }

  void configure(const LaserScan & scan)
  {
    sensor.reset(karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, karto::Name(name)));
    if (!sensor) throw std::runtime_error("cannot create laser model");
    sensor->SetMinimumRange(scan.range_min);
    sensor->SetRangeThreshold(std::min(scan.range_max, limits.range_limit));
    sensor->SetMaximumRange(std::min(scan.range_max, limits.range_limit));
    sensor->SetMinimumAngle(scan.angle_min);
    sensor->SetMaximumAngle(scan.angle_min + (scan.ranges.size() - 1) * scan.angle_increment);
    sensor->SetAngularResolution(scan.angle_increment);
    sensor->SetRangeThreshold(std::min(scan.range_max, limits.range_limit));
    if (sensor->GetNumberOfRangeReadings() != scan.ranges.size())
      throw std::runtime_error("laser model ray count mismatch");
    karto::SensorManager::GetInstance()->RegisterSensor(sensor.get());
    registered = true;
  }

  bool same_geometry(const LaserScan & scan) const
  {
    return scan.ranges.size() == previous.ranges.size() &&
      std::abs(scan.angle_min - previous.angle_min) < 1e-7 &&
      std::abs(scan.angle_increment - previous.angle_increment) < 1e-7 &&
      std::abs(scan.range_min - previous.range_min) < 1e-7 &&
      std::abs(scan.range_max - previous.range_max) < 1e-7;
  }
  void clear()
  {
    previous = LaserScan{}; base.clear(); drop_sensor();
    anchor_to_previous = {}; previous_covariance = {}; matched_previous = false;
  }

  MatcherLimits limits;
  std::string name;
  karto::Mapper mapper;
  std::unique_ptr<karto::ScanMatcher> matcher;
  std::unique_ptr<karto::LaserRangeFinder> sensor;
  bool registered = false;
  LaserScan previous;
  std::vector<Point> base;
  Pose anchor_to_previous;
  std::array<double, 9> previous_covariance{};
  bool matched_previous = false;
};

LocalMatcher::LocalMatcher(MatcherLimits limits) : impl_(std::make_unique<Impl>(limits)) {}
LocalMatcher::~LocalMatcher() = default;
void LocalMatcher::reset() { impl_->clear(); }

MatchResult LocalMatcher::update(const LaserScan & scan)
{
  MatchResult out;
  out.end = scan.stamp;
  auto & m = *impl_;
  if (!valid_scan(scan, m.limits)) { m.clear(); out.reason = "invalid_scan"; return out; }
  auto query_points = points(scan, m.limits);
  out.valid_rays = static_cast<unsigned>(query_points.size());
  out.diagnostics.geometry.emplace();
  out.diagnostics.geometry->valid_rays = static_cast<unsigned>(query_points.size());
  if (query_points.size() < m.limits.min_rays) {
    out.diagnostics.failures.push_back("geometry.valid_rays");
    m.clear();
    out.reason = "insufficient_geometry";
    return out;
  }
  const double point_spread = spread(query_points);
  out.diagnostics.geometry->spread = point_spread;
  if (!(point_spread >= m.limits.min_spread)) {
    out.diagnostics.failures.push_back("geometry.spread");
    m.clear();
    out.reason = "insufficient_geometry";
    return out;
  }
  if (!observable(scan, m.limits, out.diagnostics)) {
    m.clear(); out.reason = "unobservable_geometry"; return out;
  }
  if (!m.sensor) m.configure(scan);
  if (m.previous.ranges.empty()) {
    m.previous = scan; m.base = std::move(query_points);
    out.reason = "initializing"; return out;
  }
  out.start = m.previous.stamp;
  if (!m.same_geometry(scan)) {
    m.clear(); m.drop_sensor(); out.reason = "geometry_discontinuity"; return out;
  }
  if (scan.stamp <= m.previous.stamp || scan.stamp - m.previous.stamp > m.limits.max_interval) {
    m.clear(); out.reason = "geometry_or_time_discontinuity"; return out;
  }
  auto * grid = m.matcher->GetCorrelationGrid();
  grid->Clear();
  const auto roi = grid->GetROI();
  grid->GetCoordinateConverter()->SetOffset(karto::Vector2<double>(
      -0.5 * (roi.GetWidth() - 1) * resolution, -0.5 * (roi.GetHeight() - 1) * resolution));
  for (const auto & p : m.base) {
    auto cell = grid->WorldToGrid(karto::Vector2<double>(p.first, p.second));
    if (cell.GetX() < 0 || cell.GetY() < 0 || cell.GetX() >= roi.GetWidth() || cell.GetY() >= roi.GetHeight())
      continue;
    const auto index = grid->GridIndex(cell);
    grid->GetDataPointer()[index] = karto::GridStates_Occupied;
    grid->SmearPoint(cell);
  }
  karto::RangeReadingsVector ranges;
  ranges.reserve(scan.ranges.size());
  for (double r : scan.ranges) {
    // Karto's lookup checks NaN/inf by original ray index. Never replace with a far hit.
    ranges.push_back(std::isfinite(r) && r >= scan.range_min && r <= scan.range_max && r <= m.limits.range_limit ?
        r : std::numeric_limits<double>::quiet_NaN());
  }
  karto::LocalizedRangeScan query(karto::Name(m.name), ranges);
  query.SetCorrectedPose(karto::Pose2(0, 0, 0));
  karto::Pose2 mean;
  karto::Matrix3 cov;
  out.response = m.matcher->CorrelateScan(&query, karto::Pose2(0, 0, 0),
      karto::Vector2<double>(0.2, 0.2), karto::Vector2<double>(0.04, 0.04),
      angular_offset, angular_step, false, mean, cov, false);
  karto::Pose2 refined;
  out.response = m.matcher->CorrelateScan(&query, mean,
      karto::Vector2<double>(0.02, 0.02), karto::Vector2<double>(0.02, 0.02),
      0.02, 0.005, false, refined, cov, true);
  const Pose anchor_to_current{refined.GetX(), refined.GetY(), refined.GetHeading()};
  for (unsigned row = 0; row < 3; ++row)
    for (unsigned col = 0; col < 3; ++col) out.covariance[row*3+col] = cov(row, col);

  double squared = 0;
  unsigned inliers = 0;
  const double c = std::cos(anchor_to_current.yaw), s = std::sin(anchor_to_current.yaw);
  for (const auto & p : query_points) {
    const double x = anchor_to_current.x + c*p.first - s*p.second;
    const double y = anchor_to_current.y + s*p.first + c*p.second;
    double best = std::numeric_limits<double>::infinity();
    for (const auto & b : m.base) best = std::min(best, std::hypot(x-b.first, y-b.second));
    if (best <= m.limits.max_residual * 2) { squared += best*best; ++inliers; }
  }
  out.residual = inliers ? std::sqrt(squared / inliers) : std::numeric_limits<double>::infinity();
  out.diagnostics.match = MatchMetrics{out.response, out.residual,
    static_cast<double>(inliers) / query_points.size(), inliers,
    static_cast<unsigned>(query_points.size()), anchor_to_current, out.covariance};
  out.diagnostics.failures = qualification_failures(*out.diagnostics.match, m.limits);
  out.valid = out.diagnostics.failures.empty();
  if (!out.valid) { m.clear(); out.reason = "match_unqualified"; return out; }
  // Difference consecutive estimates against one fixed scan; replacing the
  // reference on every frame otherwise discards sub-grid physical motion.
  out.sensor_delta = compose(inverse(m.anchor_to_previous), anchor_to_current);
  const auto current_covariance = out.covariance;
  if (m.matched_previous) {
    // Do not assume the two anchor matches are independent: sum their
    // standard-deviation bounds, including position rotation by prior yaw.
    const auto linear_sigma = [](const std::array<double, 9> & covariance) {
      return std::sqrt(covariance[0] + covariance[4] + 2*std::abs(covariance[1]));
    };
    const double translation_sigma = linear_sigma(current_covariance) +
      linear_sigma(m.previous_covariance) + std::hypot(out.sensor_delta.x, out.sensor_delta.y) *
      std::sqrt(m.previous_covariance[8]);
    const double yaw_sigma = std::sqrt(current_covariance[8]) + std::sqrt(m.previous_covariance[8]);
    out.covariance[0] = out.covariance[4] = translation_sigma * translation_sigma;
    out.covariance[8] = yaw_sigma * yaw_sigma;
    out.covariance[1] = out.covariance[3] = out.covariance[0];
    out.covariance[2] = out.covariance[5] = out.covariance[6] = out.covariance[7] = 0;
    const bool covariance_ok = relative_covariance_qualified(out.covariance, m.limits.max_variance);
    out.diagnostics.relative_covariance = out.covariance;
    if (!covariance_ok) {
      out.diagnostics.failures.push_back("relative_covariance");
      m.clear(); out.valid = false; out.reason = "relative_covariance_unqualified"; return out;
    }
  }
  out.reason = "qualified";
  m.previous = scan;
  if (std::hypot(anchor_to_current.x, anchor_to_current.y) >= m.limits.reanchor_distance ||
      std::abs(anchor_to_current.yaw) >= m.limits.reanchor_yaw) {
    m.base = std::move(query_points);
    m.anchor_to_previous = {};
    m.previous_covariance = {};
    m.matched_previous = false;
  } else {
    m.anchor_to_previous = anchor_to_current;
    m.previous_covariance = current_covariance;
    m.matched_previous = true;
  }
  return out;
}
}
