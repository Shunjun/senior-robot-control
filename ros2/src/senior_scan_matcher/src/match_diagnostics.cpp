#include "senior_scan_matcher/scan_matcher.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace senior_scan_matcher {

bool relative_covariance_qualified(const std::array<double, 9> & covariance, double maximum)
{
  bool qualified = true;
  for (unsigned i : {0u, 4u, 8u})
    qualified &= std::isfinite(covariance[i]) && covariance[i] <= maximum;
  return qualified;
}
std::vector<std::string> qualification_failures(const MatchMetrics & m, const MatcherLimits & limits)
{
  std::vector<std::string> failures;
  if (!(std::isfinite(m.response) && m.response >= limits.min_response)) {
    failures.push_back("response");
  }
  bool covariance_ok = std::all_of(m.covariance.begin(), m.covariance.end(),
    [](double value) { return std::isfinite(value); });
  for (unsigned i : {0u, 4u, 8u})
    covariance_ok &= std::isfinite(m.covariance[i]) && m.covariance[i] > 0 &&
      m.covariance[i] <= limits.max_variance;
  if (!covariance_ok) failures.push_back("covariance");
  if (!(std::isfinite(m.residual) && m.residual <= limits.max_residual)) {
    failures.push_back("residual");
  }
  if (!(m.inlier_fraction >= limits.min_inlier_fraction)) failures.push_back("inlier_fraction");
  if (!(std::abs(m.anchor.x) < .19 && std::abs(m.anchor.y) < .19 && std::abs(m.anchor.yaw) < .19))
    failures.push_back("search_boundary");
  return failures;
}
std::vector<std::pair<std::string, std::string>> diagnostic_values(const MatchDiagnostics & d)
{
  std::vector<std::pair<std::string, std::string>> values;
  auto add = [&](const std::string & key, std::optional<double> value) {
    std::ostringstream stream;
    if (value) stream << std::setprecision(17) << *value;
    else stream << "not_computed";
    values.emplace_back(key, stream.str());
  };
  add("geometry.valid_rays", d.geometry ? std::optional<double>(d.geometry->valid_rays) : std::nullopt);
  add("geometry.normal_windows", d.geometry && d.geometry->normal_windows ?
    std::optional<double>(*d.geometry->normal_windows) : std::nullopt);
  for (const auto & item : std::vector<std::pair<std::string, std::optional<double>>>{
      {"geometry.spread", d.geometry ? d.geometry->spread : std::nullopt},
      {"geometry.translation_information", d.geometry ? d.geometry->translation_information : std::nullopt},
      {"geometry.determinant", d.geometry ? d.geometry->determinant : std::nullopt},
      {"geometry.yaw_information", d.geometry ? d.geometry->yaw_information : std::nullopt}}) {
    add(item.first, item.second);
  }
  const auto & m = d.match;
  add("match.response", m ? std::optional<double>(m->response) : std::nullopt);
  add("match.residual", m ? std::optional<double>(m->residual) : std::nullopt);
  add("match.inliers", m ? std::optional<double>(m->inliers) : std::nullopt);
  add("match.total", m ? std::optional<double>(m->total) : std::nullopt);
  add("match.inlier_fraction", m ? std::optional<double>(m->inlier_fraction) : std::nullopt);
  add("match.anchor_x", m ? std::optional<double>(m->anchor.x) : std::nullopt);
  add("match.anchor_y", m ? std::optional<double>(m->anchor.y) : std::nullopt);
  add("match.anchor_yaw", m ? std::optional<double>(m->anchor.yaw) : std::nullopt);
  for (unsigned i = 0; i < 9; ++i) {
    add("match.covariance." + std::to_string(i), m ? std::optional<double>(m->covariance[i]) : std::nullopt);
    add("relative_covariance." + std::to_string(i), d.relative_covariance ?
      std::optional<double>((*d.relative_covariance)[i]) : std::nullopt);
  }
  std::string failures;
  for (const auto & failure : d.failures) {
    if (!failures.empty()) failures += ",";
    failures += failure;
  }
  values.emplace_back("failed_checks", failures);
  return values;
}
}
