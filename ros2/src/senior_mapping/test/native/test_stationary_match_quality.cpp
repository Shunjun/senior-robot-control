#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

#include <karto_sdk/Mapper.h>

namespace
{
constexpr double pi = 3.14159265358979323846;
constexpr int ray_count = 720;
constexpr double first_angle = -pi;
constexpr double angle_step = 2.0 * pi / ray_count;
constexpr double min_range = 0.08;
constexpr double max_range = 8.0;
constexpr double resolution = 0.02;
constexpr double coarse_xy_step = 0.04;
constexpr double coarse_yaw_step = 0.02;

struct Room
{
  double left, right, bottom, top;
};

karto::RangeReadingsVector scan_room(const Room & room, const karto::Pose2 & sensor)
{
  karto::RangeReadingsVector ranges;
  ranges.reserve(ray_count);
  for (int ray = 0; ray < ray_count; ++ray) {
    const double heading = sensor.GetHeading() + first_angle + ray * angle_step;
    const double dx = std::cos(heading);
    const double dy = std::sin(heading);
    const double wall_x = dx > 0.0 ? room.right : room.left;
    const double wall_y = dy > 0.0 ? room.top : room.bottom;
    ranges.push_back(std::min((wall_x - sensor.GetX()) / dx, (wall_y - sensor.GetY()) / dy));
  }
  return ranges;
}

void mark_reference(
  karto::CorrelationGrid & grid, const Room & room, const karto::Pose2 & sensor)
{
  const auto ranges = scan_room(room, sensor);
  const auto roi = grid.GetROI();
  for (int ray = 0; ray < ray_count; ++ray) {
    const double heading = sensor.GetHeading() + first_angle + ray * angle_step;
    const auto cell = grid.WorldToGrid(karto::Vector2<double>(
      sensor.GetX() + ranges[ray] * std::cos(heading),
      sensor.GetY() + ranges[ray] * std::sin(heading)));
    if (cell.GetX() < 0 || cell.GetY() < 0 ||
      cell.GetX() >= roi.GetWidth() || cell.GetY() >= roi.GetHeight())
    {
      throw std::runtime_error("fixture reference point is outside the correlation grid");
    }
    grid.GetDataPointer()[grid.GridIndex(cell)] = karto::GridStates_Occupied;
    grid.SmearPoint(cell);
  }
}

struct Match
{
  double response;
  karto::Pose2 pose;
};

Match match(
  karto::ScanMatcher & matcher, const karto::Name & sensor_name,
  const karto::RangeReadingsVector & ranges)
{
  karto::LocalizedRangeScan query(sensor_name, ranges);
  query.SetCorrectedPose(karto::Pose2(0, 0, 0));
  karto::Pose2 coarse;
  karto::Matrix3 covariance;
  matcher.CorrelateScan(
    &query, karto::Pose2(0, 0, 0),
    karto::Vector2<double>(0.2, 0.2),
    karto::Vector2<double>(coarse_xy_step, coarse_xy_step),
    0.2, coarse_yaw_step, false, coarse, covariance, false);

  Match result;
  result.response = matcher.CorrelateScan(
    &query, coarse, karto::Vector2<double>(resolution, resolution),
    karto::Vector2<double>(resolution, resolution),
    coarse_yaw_step, 0.005, false, result.pose, covariance, true);
  return result;
}
}  // namespace

int main()
{
  const karto::Name name("stationary_match_quality_lidar");
  std::unique_ptr<karto::LaserRangeFinder> laser(
    karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, name));
  if (!laser) {
    throw std::runtime_error("cannot create fixture laser");
  }
  laser->SetMinimumRange(min_range);
  laser->SetRangeThreshold(max_range);
  laser->SetMaximumRange(max_range);
  laser->SetMinimumAngle(first_angle);
  laser->SetMaximumAngle(first_angle + (ray_count - 1) * angle_step);
  laser->SetAngularResolution(angle_step);
  laser->SetRangeThreshold(max_range);
  if (laser->GetNumberOfRangeReadings() != ray_count) {
    throw std::runtime_error("fixture laser ray count mismatch");
  }
  auto * sensors = karto::SensorManager::GetInstance();
  sensors->RegisterSensor(laser.get());

  karto::Mapper mapper;
  std::unique_ptr<karto::ScanMatcher> matcher(
    karto::ScanMatcher::Create(&mapper, 0.4, resolution, 0.04, max_range));
  if (!matcher) {
    throw std::runtime_error("cannot create Karto matcher");
  }
  auto & grid = *matcher->GetCorrelationGrid();
  grid.Clear();
  const auto roi = grid.GetROI();
  grid.GetCoordinateConverter()->SetOffset(karto::Vector2<double>(
    -0.5 * (roi.GetWidth() - 1) * resolution, -0.5 * (roi.GetHeight() - 1) * resolution));

  const Room reference{-2.3, 3.1, -2.0, 2.7};
  mark_reference(grid, reference, karto::Pose2(-0.23, 0.19, -0.05));
  mark_reference(grid, reference, karto::Pose2(0.32, -0.27, 0.12));

  // Both held-out queries have the same stationary pose and seed. The larger
  // room has no matching walls anywhere within the bounded search window.
  const karto::Pose2 stationary(0.08, -0.04, 0.06);
  const Room wrong_scene{-4.2, 4.5, -3.8, 4.3};
  const auto correct = match(*matcher, name, scan_room(reference, stationary));
  const auto wrong = match(*matcher, name, scan_room(wrong_scene, stationary));
  sensors->UnregisterSensor(laser.get());

  std::cout << "Correct scene response " << correct.response << ", wrong scene response "
            << wrong.response << "; recovered sensor pose " << correct.pose.GetX() << ' '
            << correct.pose.GetY() << ' ' << correct.pose.GetHeading() << '\n';
  if (!std::isfinite(correct.response) || !std::isfinite(wrong.response) ||
    !(correct.response > wrong.response && correct.response > 0.0))
  {
    throw std::runtime_error("stationary Karto response did not distinguish the wrong scene");
  }
  // Compare against the known fixture pose to within one coarse search step.
  if (!(std::abs(correct.pose.GetX() - stationary.GetX()) <= coarse_xy_step &&
    std::abs(correct.pose.GetY() - stationary.GetY()) <= coarse_xy_step &&
    std::abs(correct.pose.GetHeading() - stationary.GetHeading()) <= coarse_yaw_step))
  {
    throw std::runtime_error("stationary match did not recover the known fixture pose");
  }
}
