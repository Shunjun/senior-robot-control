#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <gtest/gtest.h>
#include "senior_scan_matcher/scan_matcher.hpp"

using senior_scan_matcher::LaserScan;
using senior_scan_matcher::LocalMatcher;
using senior_scan_matcher::Pose;

namespace {
constexpr double pi = 3.14159265358979323846;

// Ray cast an asymmetric rectangular room, independently of the estimator.
LaserScan room(Pose sensor, double stamp)
{
  LaserScan scan;
  scan.stamp = stamp;
  scan.angle_min = -pi;
  scan.angle_increment = 2 * pi / 720;
  scan.range_min = 0.08;
  scan.range_max = 8.0;
  for (int i = 0; i < 720; ++i) {
    double a = sensor.yaw + scan.angle_min + i * scan.angle_increment;
    double dx = std::cos(a), dy = std::sin(a);
    double distance = std::numeric_limits<double>::infinity();
    for (double wall : {-2.3, 3.1}) {
      double t = (wall - sensor.x) / dx;
      if (t > 0 && sensor.y + t * dy > -2.0 && sensor.y + t * dy < 2.7)
        distance = std::min(distance, t);
    }
    for (double wall : {-2.0, 2.7}) {
      double t = (wall - sensor.y) / dy;
      if (t > 0 && sensor.x + t * dx > -2.3 && sensor.x + t * dx < 3.1)
        distance = std::min(distance, t);
    }
    scan.ranges.push_back(distance);
  }
  return scan;
}
}  // namespace

TEST(LocalMatcher, TracksIndependentTranslationAndYaw)
{
  LocalMatcher matcher;
  EXPECT_FALSE(matcher.update(room({0, 0, 0}, 1.0)).valid);
  auto result = matcher.update(room({0.08, -0.04, 0.06}, 1.1));
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_NEAR(result.sensor_delta.x, 0.08, 0.035);
  EXPECT_NEAR(result.sensor_delta.y, -0.04, 0.035);
  EXPECT_NEAR(result.sensor_delta.yaw, 0.06, 0.025);
  EXPECT_GT(result.response, 0.0);
  EXPECT_GT(result.covariance[0], 0.0);
  EXPECT_GT(result.covariance[4], 0.0);
  EXPECT_GT(result.covariance[8], 0.0);
}

TEST(LocalMatcher, AccumulatesSubcellTranslationAndYawAcrossManyScans)
{
  LocalMatcher matcher;
  EXPECT_FALSE(matcher.update(room({0, 0, 0}, 1.0)).valid);
  Pose integrated{};
  for (int index = 1; index <= 40; ++index) {
    const Pose actual{0.002 * index, 0.0, 0.0015 * index};
    const auto result = matcher.update(room(actual, 1.0 + 0.1 * index));
    ASSERT_TRUE(result.valid) << "frame=" << index << " reason=" << result.reason;
    integrated = senior_scan_matcher::compose(integrated, result.sensor_delta);
  }
  EXPECT_NEAR(integrated.x, 0.08, 0.03);
  EXPECT_NEAR(integrated.yaw, 0.06, 0.025);
}

TEST(LocalMatcher, AccumulatesSubcellLateralMotionInBothDirections)
{
  for (int side : {-1, 1}) {
    LocalMatcher matcher;
    EXPECT_FALSE(matcher.update(room({0, 0, 0}, 1.0)).valid);
    Pose integrated{};
    for (int index = 1; index <= 40; ++index) {
      const auto result = matcher.update(room({0, side * 0.002 * index, 0}, 1.0 + 0.1 * index));
      ASSERT_TRUE(result.valid) << "side=" << side << " frame=" << index << " " << result.reason;
      integrated = senior_scan_matcher::compose(integrated, result.sensor_delta);
    }
    EXPECT_NEAR(integrated.y, side * 0.08, 0.03);
  }
}

TEST(LocalMatcher, ReanchorKeepsMeasuredContinuity)
{
  LocalMatcher matcher;
  EXPECT_FALSE(matcher.update(room({0, 0, 0}, 1.0)).valid);
  Pose integrated{};
  for (int i = 1; i <= 18; ++i) {
    const auto result = matcher.update(room({0.01 * i, 0, 0}, 1.0 + 0.1 * i));
    ASSERT_TRUE(result.valid) << "frame=" << i << " reason=" << result.reason;
    integrated = senior_scan_matcher::compose(integrated, result.sensor_delta);
  }
  EXPECT_NEAR(integrated.x, 0.18, 0.035);

}

TEST(LocalMatcher, LongStationaryAnchorThenSubcellMovement)
{
  LocalMatcher matcher;
  EXPECT_FALSE(matcher.update(room({0, 0, 0}, 1.0)).valid);
  for (int i = 1; i <= 300; ++i) {
    const auto result = matcher.update(room({0, 0, 0}, 1.0 + 0.1 * i));
    ASSERT_TRUE(result.valid) << "stationary frame=" << i << " reason=" << result.reason;
  }
  Pose integrated{};
  for (int i = 1; i <= 40; ++i) {
    const auto result = matcher.update(room({0.002 * i, 0, 0}, 31.0 + 0.1 * i));
    ASSERT_TRUE(result.valid) << "moving frame=" << i << " reason=" << result.reason;
    integrated = senior_scan_matcher::compose(integrated, result.sensor_delta);
  }
  EXPECT_NEAR(integrated.x, 0.08, 0.03);
}

TEST(LocalMatcher, TurningAcrossReanchorKeepsMeasuredYaw)
{
  LocalMatcher matcher;
  EXPECT_FALSE(matcher.update(room({0, 0, 0}, 1.0)).valid);
  Pose integrated{};
  for (int i = 1; i <= 15; ++i) {
    const auto result = matcher.update(room({0.003 * i, 0, 0.01 * i}, 1.0 + 0.1 * i));
    ASSERT_TRUE(result.valid) << "frame=" << i << " reason=" << result.reason;
    integrated = senior_scan_matcher::compose(integrated, result.sensor_delta);
  }
  EXPECT_NEAR(integrated.yaw, 0.15, 0.025);
}

TEST(LocalMatcher, InvalidAndMaskedPointsCannotBecomeOccupiedReturns)
{
  LocalMatcher matcher;
  auto base = room({0, 0, 0}, 1.0);
  for (size_t i = 0; i < base.ranges.size(); ++i)
    if (i < 80 || i > 640) base.ranges[i] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(matcher.update(base).valid);
  auto query = room({0.05, 0, 0}, 1.1);
  for (size_t i = 0; i < query.ranges.size(); ++i)
    if (i < 80 || i > 640) query.ranges[i] = std::numeric_limits<double>::quiet_NaN();
  auto match = matcher.update(query);
  ASSERT_TRUE(match.valid) << match.reason;
  EXPECT_NEAR(match.sensor_delta.x, 0.05, 0.035);
  std::fill(query.ranges.begin(), query.ranges.end(), std::numeric_limits<double>::quiet_NaN());
  query.stamp = 1.2;
  EXPECT_FALSE(matcher.update(query).valid);
}

TEST(LocalMatcher, PreExcludedDynamicReturnsDoNotBecomeHits)
{
  // These NaNs represent actual upstream exclusions, not a dynamic detector.
  // A moving object occludes different beam sectors in the two observations.
  auto base = room({0, 0, 0}, 1.0);
  auto query = room({0.05, 0, 0}, 1.1);
  for (size_t i = 0; i < base.ranges.size(); ++i) {
    if (i < 60 || i >= 660 || (i >= 250 && i < 300))
      base.ranges[i] = std::numeric_limits<double>::quiet_NaN();
    if (i < 60 || i >= 660 || (i >= 350 && i < 400))
      query.ranges[i] = std::numeric_limits<double>::quiet_NaN();
  }
  LocalMatcher matcher;
  EXPECT_FALSE(matcher.update(base).valid);
  const auto result = matcher.update(query);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_NEAR(result.sensor_delta.x, 0.05, 0.035);
}

TEST(LocalMatcher, RejectsRollbackAndDegenerateSingleWall)
{
  LocalMatcher matcher;
  auto wall = room({0, 0, 0}, 1.0);
  for (size_t i = 0; i < wall.ranges.size(); ++i)
    if (i < 300 || i > 420) wall.ranges[i] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(matcher.update(wall).valid);
  wall.stamp = 1.1;
  EXPECT_FALSE(matcher.update(wall).valid);
  auto old = room({0, 0, 0}, 0.9);
  EXPECT_FALSE(matcher.update(old).valid);
}

TEST(LocalMatcher, SensorOffsetAndYawAreRemovedFromBodyRotation)
{
  // Sensor follows the body rotating in place about the body origin.
  Pose extrinsic{0.2, 0.0, pi / 2};
  Pose sensor_delta{0.2, 0.2, pi / 2};
  // The delta above is expressed in the old sensor frame.
  auto body_delta = senior_scan_matcher::to_body_delta(sensor_delta, extrinsic);
  EXPECT_NEAR(body_delta.x, 0.0, 1e-9);
  EXPECT_NEAR(body_delta.y, 0.0, 1e-9);
  EXPECT_NEAR(body_delta.yaw, pi / 2, 1e-9);
}

TEST(LocalMatcher, RejectsParallelWallsUnobservableAlongCorridor)
{
  auto scan = room({0, 0, 0}, 1.0);
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double ray_y = std::sin(scan.angle_min + i * scan.angle_increment);
    const double distance = 2.0 / std::abs(ray_y);
    scan.ranges[i] = distance < 7.9 ? distance : std::numeric_limits<double>::quiet_NaN();
  }
  LocalMatcher matcher;
  EXPECT_FALSE(matcher.update(scan).valid);
  scan.stamp = 1.1;
  const auto result = matcher.update(scan);
  EXPECT_FALSE(result.valid) << "parallel walls cannot measure along-corridor translation";
}

TEST(LocalMatcher, RejectsCircularWallUnobservableInYaw)
{
  auto scan = room({0, 0, 0}, 1.0);
  std::fill(scan.ranges.begin(), scan.ranges.end(), 2.0);
  LocalMatcher matcher;
  EXPECT_FALSE(matcher.update(scan).valid);
  scan.stamp = 1.1;
  const auto result = matcher.update(scan);
  EXPECT_FALSE(result.valid) << "a circular wall cannot measure yaw";
  EXPECT_EQ(result.reason, "unobservable_geometry");
}

TEST(LocalMatcher, RejectsQuantizedParallelWalls)
{
  // 1 cm range quantization creates apparent short diagonal steps. They are
  // not evidence that motion along the walls can be measured.
  auto scan = room({0, 0, 0}, 1.0);
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double ray_y = std::sin(scan.angle_min + i * scan.angle_increment);
    const double distance = 0.75 / std::abs(ray_y);
    scan.ranges[i] = distance < 7.9 ? std::round(distance * 100) / 100 :
      std::numeric_limits<double>::quiet_NaN();
  }
  LocalMatcher matcher;
  EXPECT_EQ(matcher.update(scan).reason, "unobservable_geometry");
  scan.stamp = 1.1;
  EXPECT_EQ(matcher.update(scan).reason, "unobservable_geometry");
}
