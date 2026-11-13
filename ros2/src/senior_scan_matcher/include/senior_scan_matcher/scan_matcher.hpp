#pragma once

#include <array>
#include <memory>
#include <optional>
#include <utility>
#include <string>
#include <vector>

namespace senior_scan_matcher {
struct Pose { double x = 0.0; double y = 0.0; double yaw = 0.0; };
Pose compose(const Pose & a, const Pose & b);
Pose inverse(const Pose & pose);
Pose to_body_delta(const Pose & sensor_delta, const Pose & sensor_to_body);
struct LaserScan {
  double stamp = 0.0;
  double angle_min = 0.0;
  double angle_increment = 0.0;
  double range_min = 0.0;
  double range_max = 0.0;
  std::vector<double> ranges;
};
struct GeometryMetrics {
  unsigned valid_rays = 0;
  std::optional<unsigned> normal_windows;
  std::optional<double> spread, translation_information, determinant, yaw_information;
};
struct MatchMetrics {
  double response = 0.0, residual = 0.0, inlier_fraction = 0.0;
  unsigned inliers = 0, total = 0;
  Pose anchor;
  std::array<double, 9> covariance{};
};
struct MatchDiagnostics {
  std::optional<GeometryMetrics> geometry;
  std::optional<MatchMetrics> match;
  std::optional<std::array<double, 9>> relative_covariance;
  std::vector<std::string> failures;
};
struct MatchResult {
  bool valid = false;
  std::string reason;
  double start = 0.0;
  double end = 0.0;
  Pose sensor_delta;
  std::array<double, 9> covariance{};
  double response = 0.0;
  double residual = 0.0;
  unsigned valid_rays = 0;
  MatchDiagnostics diagnostics;
};
struct MatcherLimits {
  double max_interval = 0.3;
  double range_limit = 8.0;
  double min_response = 0.3;
  double max_residual = 0.09;
  double min_inlier_fraction = 0.55;
  double min_spread = 0.04;
  double min_translation_observability = 0.05;
  double min_yaw_observability = 0.01;
  double max_variance = 0.1;
  double reanchor_distance = 0.12;
  double reanchor_yaw = 0.1;
  unsigned min_rays = 100;
};
bool relative_covariance_qualified(const std::array<double, 9> &, double);
std::vector<std::string> qualification_failures(const MatchMetrics &, const MatcherLimits &);
std::vector<std::pair<std::string, std::string>> diagnostic_values(const MatchDiagnostics &);
class LocalMatcher {
public:
  explicit LocalMatcher(MatcherLimits limits = {});
  ~LocalMatcher();
  LocalMatcher(const LocalMatcher &) = delete;
  LocalMatcher & operator=(const LocalMatcher &) = delete;
  MatchResult update(const LaserScan & scan);
  void reset();
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
