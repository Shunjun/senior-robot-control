#include <senior_sensor_ingress/ros_wiring.hpp>
#ifndef SENIOR_MAPPING__MAPPING_NAVIGATION_HPP_
#define SENIOR_MAPPING__MAPPING_NAVIGATION_HPP_

#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <karto_sdk/Mapper.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "rclcpp/rclcpp.hpp"
#include "scan_receipt_admission.hpp"
#include "senior_interfaces/srv/read_mapping_grid.hpp"
#include "senior_interfaces/srv/read_mapping_pose.hpp"

namespace senior_mapping
{
// Localization-only diagnostic / one-time V1 migration raster.
// The management owner installs authoritative fixed cuts; this service never publishes /map.
// Preserve native free/occupied/unknown semantics. Never infer free from absence.
inline std::optional<nav_msgs::msg::OccupancyGrid> bounded_native_raster(
  karto::Mapper & mapper, double resolution, std::size_t maximum_cells)
{
  const auto scans = mapper.GetAllProcessedScans();
  if (scans.empty() || scans.size() > 16384 || !(resolution > 0)) {return std::nullopt;}
  double x0 = std::numeric_limits<double>::infinity(), y0 = x0;
  double x1 = -x0, y1 = -x0;
  for (const auto * scan : scans) {
    const auto & box = scan->GetBoundingBox();
    const auto minimum = box.GetMinimum(), maximum = box.GetMaximum();
    x0 = std::min(x0, minimum.GetX()); y0 = std::min(y0, minimum.GetY());
    x1 = std::max(x1, maximum.GetX()); y1 = std::max(y1, maximum.GetY());
  }
  const double width = std::ceil((x1-x0)/resolution) + 2;
  const double height = std::ceil((y1-y0)/resolution) + 2;
  if (!std::isfinite(width) || !std::isfinite(height) || width <= 0 || height <= 0 ||
    width > 65536 || height > 65536 || width * height > maximum_cells)
  {return std::nullopt;}
  std::unique_ptr<karto::OccupancyGrid> raster(karto::OccupancyGrid::CreateFromScans(
      scans, resolution, mapper.getParamMinPassThrough(), mapper.getParamOccupancyThreshold()));
  if (!raster || raster->GetWidth() <= 0 || raster->GetHeight() <= 0 ||
    static_cast<std::size_t>(raster->GetWidth()) * raster->GetHeight() > maximum_cells)
  {return std::nullopt;}
  nav_msgs::msg::OccupancyGrid result;
  result.info.width = raster->GetWidth(); result.info.height = raster->GetHeight();
  result.info.resolution = resolution;
  const auto offset = raster->GetCoordinateConverter()->GetOffset();
  result.info.origin.position.x = offset.GetX(); result.info.origin.position.y = offset.GetY();
  result.info.origin.orientation.w = 1;
  result.data.assign(result.info.width * result.info.height, -1);
  for (int y = 0; y < raster->GetHeight(); ++y) {
    for (int x = 0; x < raster->GetWidth(); ++x) {
      const auto value = raster->GetValue(karto::Vector2<kt_int32s>(x, y));
      if (value == karto::GridStates_Free) {result.data[y * result.info.width + x] = 0;}
      if (value == karto::GridStates_Occupied) {result.data[y * result.info.width + x] = 100;}
    }
  }
  return result;
}

class MappingNavigation
{
public:
  using Grid = senior_interfaces::srv::ReadMappingGrid;
  using Pose = senior_interfaces::srv::ReadMappingPose;
  using Raster = std::function<std::optional<nav_msgs::msg::OccupancyGrid>(std::uint64_t)>;
  MappingNavigation(rclcpp::Node & node, std::string boot, std::function<bool()> active, Raster raster)
  : boot_(std::move(boot)), active_(std::move(active))
  {
    grid_service_ = node.create_service<Grid>(senior_sensor_ingress::wiring::senior_slam_read_mapping_grid,
      [this, raster](Grid::Request::SharedPtr request, Grid::Response::SharedPtr response) {
        if (request->boot_id != boot_ || !active_()) {return;}
        auto result = raster(request->graph_commit_sequence);
        if (!result) {return;}
        response->grid = std::move(*result);
        response->boot_id = boot_; response->graph_commit_sequence = request->graph_commit_sequence;
        response->ok = true;
      });
    pose_service_ = node.create_service<Pose>(senior_sensor_ingress::wiring::senior_slam_read_mapping_pose,
      [this](Pose::Request::SharedPtr request, Pose::Response::SharedPtr response) {
        if (request->boot_id != boot_ || !active_() || !pose_ ||
          !scan_receipt_admission::sample_valid(pose_->capture))
        {return;}
        *response = *pose_;
      });
  }

  void observe(const senior_interfaces::msg::RawCaptureHeader & capture,
    const karto::Pose2 & odometry)
  {
    if (!std::isfinite(odometry.GetX()) || !std::isfinite(odometry.GetY()) ||
      !std::isfinite(odometry.GetHeading())) {pose_.reset(); return;}
    Pose::Response result;
    result.boot_id = boot_; result.capture = capture; result.ok = true;
    result.odom_x = odometry.GetX(); result.odom_y = odometry.GetY();
    result.odom_yaw = odometry.GetHeading();
    pose_ = std::move(result);
  }

private:
  std::string boot_;
  std::function<bool()> active_;
  std::optional<Pose::Response> pose_;
  rclcpp::Service<Grid>::SharedPtr grid_service_;
  rclcpp::Service<Pose>::SharedPtr pose_service_;
};
}  // namespace senior_mapping
#endif
