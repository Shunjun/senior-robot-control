// S12 renderer integration: actual Humble LayeredCostmap/StaticLayer/InflationLayer.
// The test peer supplies DDS metadata; the real private grant reader and Q admission run.
// The reference file is a frozen owner fixture, not a production catalog/IPC test.
// Compatible: the actual Humble MPPI compile_commands defines both macros.
// xtensor 0.23.10 selects its public tensor allocator with XTENSOR_USE_XSIMD;
// match that ABI before any tensor headers are included by this test translation unit.
#ifndef XTENSOR_ENABLE_XSIMD
#define XTENSOR_ENABLE_XSIMD 1
#endif
#ifndef XTENSOR_USE_XSIMD
#define XTENSOR_USE_XSIMD 1
#endif
#include <chrono>
#include <algorithm>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>
#include "nlohmann/json.hpp"
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_mppi_controller/critics/cost_critic.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "senior_navigation/current_scan_layer.hpp"

namespace senior_navigation
{
class CurrentScanLayerTestAccess
{
public:
  static void admitted(CurrentScanLayer & layer, const std::vector<geometry_msgs::msg::Point32> & points,
    uint64_t deadline, bool valid = true)
  {
    auto q = std::make_shared<senior_interfaces::msg::CurrentObstacles>();
    static uint64_t sequence = 0;
    q->backend_boot = "test-backend";
    std::ifstream boot("/proc/sys/kernel/random/boot_id");
    std::getline(boot, q->host_boot_id);
    q->map_id = "test-map";
    q->reference_revision = 1;
    q->map_generation = "test-reference";
    auto & capture = q->capture;
    capture.valid = valid;
    capture.bridge_boot_id = "test-bridge";
    capture.source_epoch = "test-source";
    capture.capture_generation = capture.clock_generation = 1;
    capture.raw_gid = std::string(48, 'a');
    capture.bridge_gid = std::string(48, 'b');
    capture.sequence = ++sequence;
    capture.sample_steady_ns = deadline >= 500000000 ? deadline - 500000000 : 0;
    capture.received_steady_ns = capture.sample_steady_ns;
    q->valid = valid;
    q->frame_id = "map";
    q->valid_until_steady_ns = deadline;
    q->points.assign(points.begin(), points.end());
    CurrentScanLayer::PublisherGid gid{};
    gid.fill(7);  // Transport metadata fixture; never used as a reference grant.
    std::lock_guard<std::mutex> lock(layer.mutex_);
    layer.admitQLocked(q, gid);
  }
  static void poll(CurrentScanLayer & layer) {layer.pollContract();}
};
}  // namespace senior_navigation

namespace
{
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using senior_navigation::CurrentScanLayer;
using senior_navigation::CurrentScanLayerTestAccess;

uint64_t nowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

class ReferenceFixture
{
public:
  ReferenceFixture()
  {
    char root[] = "/tmp/senior-current-layer-XXXXXX";
    const auto directory = ::mkdtemp(root);
    if (!directory) {throw std::runtime_error("test private directory unavailable");}
    path = directory;
    std::ifstream boot("/proc/sys/kernel/random/boot_id");
    std::getline(boot, host);
    row = {
      {"schema", 1}, {"run_nonce", "test-run"}, {"host_boot_id", host},
      {"owner_instance", "test-owner"}, {"revision", 0}, {"authority_epoch", 1},
      {"status", "selected"}, {"reason", ""}, {"map_id", "test-map"},
      {"reference_revision", 1}, {"reference_generation", "test-reference"},
      {"backend_boot", "test-backend"},
      {"source", {{"backend_boot", "test-backend"}, {"bridge_boot_id", "test-bridge"},
        {"source_epoch", "test-source"}, {"capture_generation", 1}, {"clock_generation", 1},
        {"raw_gid", std::string(48, 'a')}, {"bridge_gid", std::string(48, 'b')}}},
      {"selection_token", "test-selection"}, {"selection_generation", 1},
      {"acceptance_floor", nullptr}, {"acceptance_floor_steady_ns", 0},
      {"navigation_floor_steady_ns", 0}, {"fixed_revision", 1}, {"graph_sequence", 1},
      {"anchor_digest", std::string(64, 'a')}, {"expected_grid_id", "test-grid"},
      {"geometry", {{"width", 80}, {"height", 60}, {"resolution", 0.1},
        {"origin_x", -4.0}, {"origin_y", -3.0}}}, {"coverage_domain_epoch", 1},
      {"global_instance_permit", "test-global"}, {"local_instance_permit", "test-local"},
      {"lease_until_steady_ns", 0}};
    refresh();
  }
  ~ReferenceFixture() {std::filesystem::remove_all(path);}
  void refresh()
  {
    row["revision"] = row["revision"].get<uint64_t>() + 1;
    row["lease_until_steady_ns"] = nowNs() + 500000000;
    const auto temporary = path / ".reference.tmp";
    {std::ofstream file(temporary); file << row.dump();}
    if (::chmod(temporary.c_str(), 0600) != 0) {throw std::runtime_error("test permissions failed");}
    std::filesystem::rename(temporary, path / "reference.json");
  }
  nlohmann::json coverage(const std::string & role) const
  {
    std::ifstream file(path / (role + "-coverage.json"));
    return nlohmann::json::parse(file);
  }
  std::filesystem::path path;
  std::string host;
  nlohmann::json row;
};

geometry_msgs::msg::Point32 hit(float x, float y)
{
  geometry_msgs::msg::Point32 point;
  point.x = x;
  point.y = y;
  return point;
}

// Exposes the real StaticLayer callback for a deterministic, transport-free map receipt.
class StaticInput : public nav2_costmap_2d::StaticLayer
{
public:
  using nav2_costmap_2d::StaticLayer::incomingMap;
};

class CurrentLayerIntegration : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(CurrentLayerIntegration, ReplacesAndWithdrawsCurrentHitsWithoutClearingFixedOrUnknown)
{
  // The same input drives the global and rolling local consumers, not a capacity matrix.
  for (const bool rolling : {false, true}) {
    ReferenceFixture reference;
    const std::string role = rolling ? "local" : "global";
    auto node = std::make_shared<nav2_util::LifecycleNode>(rolling ? "local_q_test" : "global_q_test");
    node->declare_parameter("current.mode", "live");
    node->declare_parameter("current.contract_directory", reference.path.string());
    node->declare_parameter("current.run_nonce", "test-run");
    node->declare_parameter("current.host_boot_id", reference.host);
    node->declare_parameter("current.role", role);
    node->declare_parameter("current.role_instance_permit", "test-" + role);
    node->declare_parameter("map_topic", "/map");
    node->declare_parameter("track_unknown_space", true);
    node->declare_parameter("use_maximum", false);
    node->declare_parameter("lethal_cost_threshold", 100);
    node->declare_parameter("unknown_cost_value", 255);
    node->declare_parameter("trinary_costmap", true);
    node->declare_parameter("transform_tolerance", 0.0);
    node->declare_parameter("inflation.inflation_radius", 0.35);
    node->declare_parameter("inflation.cost_scaling_factor", 3.0);
    auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tf2_ros::Buffer tf(node->get_clock());
    tf2_ros::TransformListener listener(tf);  // Real managed TF executor, including rolling StaticLayer.
    nav2_costmap_2d::LayeredCostmap layered("map", rolling, true);
    layered.resizeMap(80, 60, 0.1, -4.0, -3.0);
    auto fixed = std::make_shared<StaticInput>();
    auto current = std::make_shared<CurrentScanLayer>();
    auto inflation = std::make_shared<nav2_costmap_2d::InflationLayer>();
    layered.addPlugin(fixed);
    layered.addPlugin(current);
    layered.addPlugin(inflation);
    fixed->initialize(&layered, "fixed", &tf, node, group);
    current->initialize(&layered, "current", &tf, node, group);
    CurrentScanLayerTestAccess::poll(*current);
    inflation->initialize(&layered, "inflation", &tf, node, group);
    std::vector<geometry_msgs::msg::Point> footprint(4);
    footprint[0].x = footprint[3].x = -0.337;
    footprint[1].x = footprint[2].x = 0.337;
    footprint[0].y = footprint[1].y = -0.27;
    footprint[2].y = footprint[3].y = 0.27;
    layered.setFootprint(footprint);

    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->header.frame_id = "map";
    map->info.width = 80;
    map->info.height = 60;
    map->info.resolution = 0.1;
    map->info.origin.position.x = -4.0;
    map->info.origin.position.y = -3.0;
    map->info.origin.orientation.w = 1.0;
    map->data.assign(80 * 60, 0);
    // Fixed wall at (2.05, 1.05); unknown under A and another untouched U.
    map->data[40 * 80 + 60] = 100;
    map->data[30 * 80 + 25] = -1;
    map->data[15 * 80 + 25] = -1;
    fixed->incomingMap(map);
    auto * master = layered.getCostmap();
    const auto cost = [master](double x, double y) {
        unsigned int mx, my;
        EXPECT_TRUE(master->worldToMap(x, y, mx, my));
        return master->getCost(mx, my);
      };
    const auto install = [&current, &reference](const std::vector<geometry_msgs::msg::Point32> & points) {
        reference.refresh();
        CurrentScanLayerTestAccess::poll(*current);
        CurrentScanLayerTestAccess::admitted(*current, points, nowNs() + 500000000);
      };
    const auto invariants = [&cost]() {
        EXPECT_EQ(cost(2.05, 1.05), LETHAL_OBSTACLE);
        EXPECT_EQ(cost(-1.45, -1.45), NO_INFORMATION);
      };

    install({hit(-1.45F, 0.05F), hit(2.05F, 1.05F)});  // A also overlaps the fixed wall.
    layered.updateMap(0, 0, 0);
    EXPECT_TRUE(current->isCurrent());
    CurrentScanLayerTestAccess::poll(*current);
    const auto first_receipt = reference.coverage(role);
    std::cout << "S12 role=" << role << " receipt=" << first_receipt.dump() << std::endl;
    EXPECT_EQ(first_receipt["status"], "ready");
    EXPECT_GT(first_receipt["cycle"].get<uint64_t>(), 0U);
    EXPECT_LE(first_receipt["valid_until_steady_ns"].get<uint64_t>(),
      first_receipt["scan_deadline_steady_ns"].get<uint64_t>());
    // A healthy next bounds phase must not replace the completed positive with pending.
    double bx0 = 1e30, by0 = 1e30, bx1 = -1e30, by1 = -1e30;
    current->updateBounds(0, 0, 0, &bx0, &by0, &bx1, &by1);
    CurrentScanLayerTestAccess::poll(*current);
    EXPECT_EQ(reference.coverage(role)["cycle"], first_receipt["cycle"]);
    EXPECT_EQ(cost(-1.45, 0.05), LETHAL_OBSTACLE);
    EXPECT_GT(cost(-1.25, 0.05), 0);
    EXPECT_LT(cost(-1.25, 0.05), NO_INFORMATION);
    invariants();

    install({hit(0.05F, 1.05F)});  // Not rendered: replaces latest between cycles.
    install({hit(1.05F, 0.05F)});  // B.
    layered.updateMap(0, 0, 0);
    EXPECT_EQ(cost(-1.45, 0.05), NO_INFORMATION);
    EXPECT_EQ(cost(-1.25, 0.05), 0);
    EXPECT_EQ(cost(0.05, 1.05), 0);
    EXPECT_EQ(cost(1.05, 0.05), LETHAL_OBSTACLE);
    invariants();

    current->reset();
    layered.updateMap(rolling ? 0.4 : 0.0, 0, 0);
    EXPECT_FALSE(current->isCurrent());
    EXPECT_EQ(cost(1.05, 0.05), 0);
    EXPECT_EQ(cost(1.25, 0.05), 0);
    invariants();

    install({hit(1.05F, 0.05F)});
    layered.updateMap(0, 0, 0);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("current.enabled", false)).successful);
    layered.updateMap(0, 0, 0);
    EXPECT_EQ(cost(1.05, 0.05), 0);
    EXPECT_EQ(cost(1.25, 0.05), 0);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("current.enabled", true)).successful);
    install({});  // Valid empty Q: withdrawal is not a free observation.
    layered.updateMap(0, 0, 0);
    EXPECT_TRUE(current->isCurrent());
    invariants();

    reference.refresh();
    CurrentScanLayerTestAccess::poll(*current);
    CurrentScanLayerTestAccess::admitted(*current, {hit(1.05F, 0.05F)}, nowNs() + 20000000);
    layered.updateMap(0, 0, 0);
    EXPECT_EQ(cost(1.05, 0.05), LETHAL_OBSTACLE);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    layered.updateMap(0, 0, 0);
    EXPECT_FALSE(current->isCurrent());
    EXPECT_EQ(cost(1.05, 0.05), 0);
    EXPECT_EQ(cost(1.25, 0.05), 0);
    invariants();

    install({hit(1.05F, 0.05F)});
    layered.updateMap(0, 0, 0);
    CurrentScanLayerTestAccess::admitted(*current, {}, 0, false);
    layered.updateMap(0, 0, 0);
    EXPECT_FALSE(current->isCurrent());
    EXPECT_EQ(cost(1.05, 0.05), 0);
    invariants();

    if (!rolling) {
      // A bridge ACK is deliberately not an input to the layer: only actual master coverage counts.
      install({hit(4.55F, 0.05F)});
      layered.updateMap(0, 0, 0);
      EXPECT_FALSE(current->isCurrent());
      auto expanded = std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
      expanded->info.width = 90;
      expanded->data.assign(90 * 60, -1);
      for (unsigned int y = 0; y < 60; ++y) {
        for (unsigned int x = 0; x < 80; ++x) {
          expanded->data[y * 90 + x] = map->data[y * 80 + x];
        }
      }
      reference.row["geometry"]["width"] = 90;
      reference.row["coverage_domain_epoch"] = 2;
      reference.refresh();
      CurrentScanLayerTestAccess::poll(*current);
      layered.updateMap(0, 0, 0);
      EXPECT_FALSE(current->isCurrent());  // Original bridge domain is known, master still old.
      fixed->incomingMap(expanded);
      layered.updateMap(0, 0, 0);
      EXPECT_TRUE(current->isCurrent());
      EXPECT_EQ(cost(4.55, 0.05), LETHAL_OBSTACLE);
      install({});
      layered.updateMap(0, 0, 0);
      EXPECT_EQ(cost(4.55, 0.05), NO_INFORMATION);
      invariants();
    }
  }
}

// Expose only the real protected collision decision. Configuration and score below
// use the same Humble ParametersHandler/on_configure path as the controller.
class MppiCollisionInput : public mppi::critics::CostCritic
{
public:
  using mppi::critics::CostCritic::inCollision;
};

class MppiScoreInput
{
public:
  struct Result {float cost; bool all_collide;};

  explicit MppiScoreInput(const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & wrapper)
  : node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "s14_cost_critic", scoreOptions())), parameters_(node_), master_(wrapper->getCostmap())
  {
    critic_.on_configure(node_, "FollowPath", "FollowPath.CostCritic", wrapper, &parameters_);
    parameters_.start();
    state_.reset(1, 1);
    state_.pose.header.frame_id = wrapper->getGlobalFrameID();
    state_.pose.pose.position.x = state_.pose.pose.position.y = 0.005;
    state_.pose.pose.orientation.w = 1.0;
    trajectories_.reset(1, 1);
    trajectories_.x(0, 0) = trajectories_.y(0, 0) = 0.005F;
    path_.reset(1);
    path_.x(0) = 1.2F;  // Outside the 0.5m near-goal gate; no path scoring is substituted.
    costs_ = xt::zeros<float>({1});
  }

  bool inCollision(float center_cost)
  {
    std::lock_guard<std::mutex> parameters_lock(*parameters_.getLock());
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> master_lock(*master_->getMutex());
    return critic_.inCollision(center_cost, 0.005F, 0.005F, 0.0F);
  }

  Result score()
  {
    std::lock_guard<std::mutex> parameters_lock(*parameters_.getLock());
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> master_lock(*master_->getMutex());
    costs_.fill(0.0F);
    mppi::CriticData data{state_, trajectories_, path_, costs_, model_dt_, false,
      nullptr, nullptr, std::nullopt, std::nullopt};
    critic_.score(data);  // Real shared library; not the full optimizer/closed-loop S13.
    return {costs_(0), data.fail_flag};
  }

private:
  static rclcpp::NodeOptions scoreOptions()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("FollowPath.CostCritic.enabled", true),
      rclcpp::Parameter("FollowPath.CostCritic.consider_footprint", true),
      rclcpp::Parameter("FollowPath.CostCritic.cost_power", 1),
      rclcpp::Parameter("FollowPath.CostCritic.cost_weight", 3.81),
      rclcpp::Parameter("FollowPath.CostCritic.critical_cost", 300.0),
      rclcpp::Parameter("FollowPath.CostCritic.collision_cost", 1000000.0),
      rclcpp::Parameter("FollowPath.CostCritic.near_goal_distance", 0.5)});
    return options;
  }

  // Handler/callbacks and node outlive the real critic; no executor or extra thread.
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  mppi::ParametersHandler parameters_;
  nav2_costmap_2d::Costmap2D * master_;
  MppiCollisionInput critic_;
  mppi::models::State state_;
  mppi::models::Trajectories trajectories_;
  mppi::models::Path path_;
  xt::xtensor<float, 1> costs_;
  float model_dt_{0.05F};
};

TEST_F(CurrentLayerIntegration, MixedUnknownLethalFootprintLoopAndClosingEdgeOrders)
{
  // S14: same polygon and two mixed edge cells, three cyclic starts.
  // Orders0/1 retain the original inputs; order2 puts lethal on the closing edge.
  // A failure is version evidence: do not weaken the assertion or vendor Nav2 here.
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("track_unknown_space", true),
    rclcpp::Parameter("footprint", "[[-0.337,-0.27],[0.337,-0.27],[0.337,0.27],[-0.337,0.27]]")});
  auto wrapper = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  ASSERT_EQ(wrapper->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
  auto * master = wrapper->getCostmap();
  master->resizeMap(200, 200, 0.01, -1.0, -1.0);
  std::vector<geometry_msgs::msg::Point> raw(4);
  raw[0].x = raw[3].x = -0.337;
  raw[1].x = raw[2].x = 0.337;
  raw[0].y = raw[1].y = -0.27;
  raw[2].y = raw[3].y = 0.27;
  const auto set = [master](double x, double y, unsigned char value) {
      unsigned int mx, my;
      EXPECT_TRUE(master->worldToMap(x, y, mx, my));
      master->setCost(mx, my, value);
    };
  MppiScoreInput mppi(wrapper);
  for (int order = 0; order < 3; ++order) {
    wrapper->setRobotFootprint(raw);  // Retains the wrapper's original padding.
    const auto footprint = wrapper->getRobotFootprint();
    master->resetMap(0, 0, 200, 200);
    for (unsigned int y = 0; y < 200; ++y) {
      for (unsigned int x = 0; x < 200; ++x) {
        master->setCost(x, y, nav2_costmap_2d::FREE_SPACE);
      }
    }
    // Force full-footprint examination without making the center lethal.
    set(0.005, 0.005, 128);
    // Exact edge coordinates of the padded polygon; identical geometry in both orders.
    const double bottom = std::min_element(footprint.begin(), footprint.end(),
      [](const auto & a, const auto & b) {return a.y < b.y;})->y;
    const double right = std::max_element(footprint.begin(), footprint.end(),
      [](const auto & a, const auto & b) {return a.x < b.x;})->x;
    set(0.005, bottom + 0.005, NO_INFORMATION);
    set(right + 0.005, 0.005, LETHAL_OBSTACLE);
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(master);
    const double footprint_cost = checker.footprintCostAtPose(0.005, 0.005, 0, footprint);
    nav2_smac_planner::GridCollisionChecker smac(master, 72, wrapper);
    smac.setFootprint(footprint, false, 0.0);
    // Cell 100 maps to (0.005,0.005), the same pose as checker/MPPI.
    const bool smac_collision = smac.inCollision(100.0F, 100.0F, 0.0F, true);
    const bool mppi_collision = mppi.inCollision(128.0F);
    const auto score = mppi.score();
    std::cout << "S14 order=" << order << " footprint_cost=" << footprint_cost
              << " smac_collision=" << smac_collision << " mppi_collision=" << mppi_collision
              << " score=" << score.cost << " all_collide=" << score.all_collide
              << " center_cost=128 footprint_padding=" << wrapper->get_parameter("footprint_padding").as_double()
              << std::endl;
    EXPECT_EQ(footprint_cost, LETHAL_OBSTACLE) << "order=" << order;
    EXPECT_TRUE(smac_collision) << "order=" << order;
    EXPECT_TRUE(mppi_collision) << "order=" << order;
    EXPECT_TRUE(score.all_collide) << "order=" << order;
    EXPECT_FLOAT_EQ(score.cost, 15000.0F) << "order=" << order;  // 1e6 * (3.81/254).
    std::rotate(raw.begin(), raw.begin() + 1, raw.end());
  }
  EXPECT_EQ(wrapper->on_cleanup(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
}

// Two additional semantic inputs, not a parameter/capacity matrix. They catch
// unknown being priced as near-collision, and a broad reduction of known costs.
class MppiUniformCostmap
{
public:
  explicit MppiUniformCostmap(unsigned char cost)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("track_unknown_space", true),
      rclcpp::Parameter("footprint", "[[-0.337,-0.27],[0.337,-0.27],[0.337,0.27],[-0.337,0.27]]")});
    wrapper = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    if (wrapper->on_configure(rclcpp_lifecycle::State()) != nav2_util::CallbackReturn::SUCCESS) {
      throw std::runtime_error("S14 real score costmap configuration failed");
    }
    auto * master = wrapper->getCostmap();
    master->resizeMap(200, 200, 0.01, -1.0, -1.0);
    for (unsigned int y = 0; y < 200; ++y) {
      for (unsigned int x = 0; x < 200; ++x) {master->setCost(x, y, cost);}
    }
  }
  ~MppiUniformCostmap()
  {
    EXPECT_EQ(wrapper->on_cleanup(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
  }
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> wrapper;
};

TEST_F(CurrentLayerIntegration, AllowsInGridUnknownWithoutCriticalRepulsion)
{
  MppiUniformCostmap map(NO_INFORMATION);
  MppiScoreInput mppi(map.wrapper);
  EXPECT_FALSE(mppi.inCollision(NO_INFORMATION));
  const auto score = mppi.score();
  std::cout << "S14 pure_unknown score=" << score.cost << " all_collide=" << score.all_collide << std::endl;
  EXPECT_FALSE(score.all_collide);
  EXPECT_FLOAT_EQ(score.cost, 0.0F);  // Original same-version scoring is RED at 4.5.
}

TEST_F(CurrentLayerIntegration, PreservesKnownInscribedCriticalRepulsion)
{
  MppiUniformCostmap map(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  MppiScoreInput mppi(map.wrapper);
  EXPECT_FALSE(mppi.inCollision(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE));
  const auto score = mppi.score();
  std::cout << "S14 known253 score=" << score.cost << " all_collide=" << score.all_collide << std::endl;
  EXPECT_FALSE(score.all_collide);
  EXPECT_FLOAT_EQ(score.cost, 4.5F);  // Known critical300 * (3.81/254) is unchanged.
}
}  // namespace
