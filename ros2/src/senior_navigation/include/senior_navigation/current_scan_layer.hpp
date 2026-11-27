#ifndef SENIOR_NAVIGATION__CURRENT_SCAN_LAYER_HPP_
#define SENIOR_NAVIGATION__CURRENT_SCAN_LAYER_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "senior_interfaces/msg/current_obstacles.hpp"

namespace senior_navigation
{
// Rendering is deliberately separate from admission. Only an independently
// authorized original reference consumer may populate latest_; Q is not a grant.
class CurrentScanLayer final : public nav2_costmap_2d::Layer
{
public:
  CurrentScanLayer();
  ~CurrentScanLayer() override;
  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master, int min_i, int min_j,
    int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return true;}
  //Compatible: Humble 1.1.20 Layer provides both no-argument hooks.
  void activate() override;
  void deactivate() override;
  void matchSize() override;

private:
  friend class CurrentScanLayerTestAccess;
  using CurrentObstacles = senior_interfaces::msg::CurrentObstacles;
  struct Bounds
  {
    double min_x, min_y, max_x, max_y;
    void include(double x, double y);
    void mergeInto(double * x0, double * y0, double * x1, double * y1) const;
  };
  struct Authorization;
  struct Contract;
  using PublisherGid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;
  struct Cycle
  {
    CurrentObstacles::ConstSharedPtr scan;
    std::shared_ptr<const Authorization> authorization;
    uint64_t started_ns{0};
    bool transform_valid{false};
    bool cleanup_complete{false};
    std::vector<geometry_msgs::msg::Point> points;
    uint64_t generation{0};
  };

  static uint64_t steadyNow();
  bool isFresh(const CurrentObstacles::ConstSharedPtr & scan) const;
  void withdrawLocked();
  void pollContract();
  void receiveQ(CurrentObstacles::ConstSharedPtr scan, const rclcpp::MessageInfo & info);
  void admitQLocked(CurrentObstacles::ConstSharedPtr scan, const PublisherGid & gid);
  void queueCoverageLocked(const std::string & status, const std::string & reason,
    nav2_costmap_2d::Costmap2D * master = nullptr, bool covered = false,
    std::optional<Bounds> updated = std::nullopt);
  void negativeLocked(const std::string & reason);
  bool projectCycleLocked();
  rcl_interfaces::msg::SetParametersResult parameters(const std::vector<rclcpp::Parameter> & values);

  // Callbacks only hold this lock. Costmap hooks take the Nav2 master lock first,
  // then this lock; callbacks never acquire the master lock or do TF/file I/O.
  std::mutex mutex_;
  bool active_{true};
  bool requested_enabled_{true};
  uint64_t generation_{0};
  CurrentObstacles::ConstSharedPtr latest_;
  Cycle cycle_;
  std::shared_ptr<const Authorization> authorization_;
  std::unique_ptr<Contract> contract_;
  std::optional<Bounds> rendered_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::Subscription<CurrentObstacles>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr io_group_;
  rclcpp::TimerBase::SharedPtr io_timer_;
};
}  // namespace senior_navigation
#endif  // SENIOR_NAVIGATION__CURRENT_SCAN_LAYER_HPP_
