#include <cmath>
#include <limits>
#include <map>
#include <gtest/gtest.h>
#include "senior_scan_matcher/scan_matcher.hpp"
using namespace senior_scan_matcher;

namespace {
MatchMetrics healthy()
{
  MatchMetrics m;
  m.response = .3; m.residual = .09; m.inlier_fraction = .55;
  m.covariance = {.1, 0, 0, 0, .1, 0, 0, 0, .1};
  m.anchor = {.189999, -.189999, .189999};
  return m;
}
}
TEST(MatchDiagnostics, ExactInclusiveThresholdsAndStrictSearchBoundary)
{
  auto m = healthy();
  EXPECT_TRUE(qualification_failures(m, {}).empty());
  m.anchor.x = .19;
  EXPECT_EQ(qualification_failures(m, {}), std::vector<std::string>{"search_boundary"});
  m = healthy(); m.response = std::nextafter(.3, 0.);
  EXPECT_EQ(qualification_failures(m, {}), std::vector<std::string>{"response"});
  m = healthy(); m.residual = std::nextafter(.09, 1.);
  EXPECT_EQ(qualification_failures(m, {}), std::vector<std::string>{"residual"});
  m = healthy(); m.inlier_fraction = std::nextafter(.55, 0.);
  EXPECT_EQ(qualification_failures(m, {}), std::vector<std::string>{"inlier_fraction"});
}
TEST(MatchDiagnostics, ReportsConcurrentFailuresAndNonfiniteCovariance)
{
  auto m = healthy(); m.response = NAN; m.residual = INFINITY;
  m.inlier_fraction = .1; m.covariance[1] = NAN; m.anchor.yaw = -.19;
  EXPECT_EQ(qualification_failures(m, {}),
    (std::vector<std::string>{"response", "covariance", "residual", "inlier_fraction", "search_boundary"}));
  m = healthy(); m.covariance[0] = 0;
  EXPECT_EQ(qualification_failures(m, {}), std::vector<std::string>{"covariance"});
  m = healthy(); m.covariance[8] = std::nextafter(.1, 1.);
  EXPECT_EQ(qualification_failures(m, {}), std::vector<std::string>{"covariance"});
}
TEST(MatchDiagnostics, MissingIsNotFabricatedZero)
{
  MatchResult result;
  EXPECT_FALSE(result.diagnostics.geometry.has_value());
  EXPECT_FALSE(result.diagnostics.match.has_value());
  EXPECT_FALSE(result.diagnostics.relative_covariance.has_value());
}

TEST(MatchDiagnostics, RelativeCovarianceRetainsOriginalBoundary)
{
  std::array<double, 9> covariance{.1, 0, 0, 0, .1, 0, 0, 0, .1};
  EXPECT_TRUE(relative_covariance_qualified(covariance, .1));
  covariance[8] = std::nextafter(.1, 1.);
  EXPECT_FALSE(relative_covariance_qualified(covariance, .1));
  covariance[8] = NAN;
  EXPECT_FALSE(relative_covariance_qualified(covariance, .1));
}

TEST(MatchDiagnostics, SerializedMissingAndMeasuredZeroAreDistinct)
{
  MatchDiagnostics missing;
  auto pairs = diagnostic_values(missing);
  std::map<std::string, std::string> values(pairs.begin(), pairs.end());
  EXPECT_EQ(values.at("match.response"), "not_computed");
  EXPECT_EQ(values.at("geometry.valid_rays"), "not_computed");
  missing.match = healthy(); missing.match->residual = 0;
  pairs = diagnostic_values(missing); values = {pairs.begin(), pairs.end()};
  EXPECT_EQ(values.at("match.residual"), "0");
}

TEST(MatchDiagnostics, GeometryRejectedBeforePlaneCalculationIsExplicitlyMissing)
{
  MatchDiagnostics diagnostics; diagnostics.geometry.emplace();
  diagnostics.geometry->valid_rays = 7;
  const auto pairs = diagnostic_values(diagnostics);
  const std::map<std::string, std::string> values(pairs.begin(), pairs.end());
  EXPECT_EQ(values.at("geometry.valid_rays"), "7");
  EXPECT_EQ(values.at("geometry.normal_windows"), "not_computed");
}
