#include <gtest/gtest.h>
#include "senior_sensor_ingress/static_tf_receipt.hpp"

using senior_sensor_ingress::validate_static_tf;
using tf2_msgs::msg::TFMessage;

TEST(StaticTfReceipt, ZeroStampIsValidAndDuplicateChildIsRejected)
{
  TFMessage message;
  geometry_msgs::msg::TransformStamped edge;
  edge.header.frame_id = "base_link";
  edge.child_frame_id = "camera";
  edge.transform.rotation.w = 1.0;
  message.transforms.push_back(edge);
  EXPECT_TRUE(validate_static_tf(message, "base_link", "camera"));
  message.transforms.push_back(edge);
  EXPECT_FALSE(validate_static_tf(message, "base_link", "camera"));
}

TEST(StaticTfReceipt, CyclesAndNonUnitQuaternionAreRejected)
{
  TFMessage message;
  geometry_msgs::msg::TransformStamped edge;
  edge.header.frame_id = "base_link";
  edge.child_frame_id = "camera";
  edge.transform.rotation.w = 2.0;
  message.transforms.push_back(edge);
  EXPECT_FALSE(validate_static_tf(message, "base_link", "camera"));
  message.transforms[0].transform.rotation.w = 1.0;
  edge.header.frame_id = "camera";
  edge.child_frame_id = "base_link";
  edge.transform.rotation.w = 1.0;
  message.transforms.push_back(edge);
  EXPECT_FALSE(validate_static_tf(message, "base_link", "camera"));
}

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <senior_motion_msgs/msg/bounded_static_tf.hpp>

using namespace std::chrono_literals;

namespace {
bool wait_for_tf(const std::function<bool()> & predicate)
{
  const auto end = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < end) {
    if (predicate()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

TEST(StaticTfReceipt, AuthenticPublisherZeroStampAndCurrentInvalidation)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("input_topic", "/d2a_tf_test/static"),
    rclcpp::Parameter("output_topic", "/d2a_tf_test/receipt"),
    rclcpp::Parameter("base_frame", "base_link"),
    rclcpp::Parameter("camera_frame", "camera"),
  });
  auto bridge = std::make_shared<senior_sensor_ingress::StaticTfReceipt>(options);
  auto source = std::make_shared<rclcpp::Node>("robot_state_publisher");
  auto observer = std::make_shared<rclcpp::Node>("d2a_tf_observer");
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  auto publisher = source->create_publisher<TFMessage>("/d2a_tf_test/static", qos);
  std::mutex mutex;
  std::vector<senior_motion_msgs::msg::BoundedStaticTf> received;
  auto sub = observer->create_subscription<senior_motion_msgs::msg::BoundedStaticTf>(
    "/d2a_tf_test/receipt", qos, [&](senior_motion_msgs::msg::BoundedStaticTf::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(*message);
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bridge);
  executor.add_node(source);
  executor.add_node(observer);
  std::thread spin([&] {executor.spin();});
  auto cleanup = [&](int *) {
    executor.cancel();
    if (spin.joinable()) spin.join();
    rclcpp::shutdown();
  };
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_for_tf([&] {return publisher->get_subscription_count() > 0;}));
  TFMessage message;
  geometry_msgs::msg::TransformStamped edge;
  edge.header.frame_id = "base_link";
  edge.child_frame_id = "camera";
  edge.transform.rotation.w = 1.0;
  edge.transform.translation.x = 0.2;
  message.transforms.push_back(edge);
  publisher->publish(message);
  ASSERT_TRUE(wait_for_tf([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : received) if (value.valid && value.admitted) return true;
    return false;
  }));
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto & last = received.back();
    ASSERT_TRUE(last.valid);
    ASSERT_EQ(last.edges.size(), 1u);
    EXPECT_EQ(last.edges[0].stamp.sec, 0);
    EXPECT_EQ(last.edges[0].stamp.nanosec, 0u);
    EXPECT_EQ(last.edges[0].translation[0], 0.2);
    EXPECT_FALSE(last.raw_gid.empty());
    EXPECT_FALSE(last.bridge_gid.empty());
    EXPECT_FALSE(last.bridge_boot_id.empty());
    EXPECT_GT(last.received_steady, 0.0);
  }
  uint64_t first_sequence = 0;
  builtin_interfaces::msg::Time first_ros;
  double first_steady = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex);
    first_sequence = received.back().first_sequence;
    first_ros = received.back().first_received_ros;
    first_steady = received.back().first_received_steady;
  }
  EXPECT_GT(first_sequence, 0u);
  EXPECT_GT(first_steady, 0.0);
  publisher->publish(message);
  ASSERT_TRUE(wait_for_tf([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : received) if (value.valid && value.sequence > first_sequence) return true;
    return false;
  }));
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto & repeated = received.back();
    EXPECT_EQ(repeated.first_sequence, first_sequence);
    EXPECT_EQ(repeated.first_received_ros, first_ros);
    EXPECT_DOUBLE_EQ(repeated.first_received_steady, first_steady);
    EXPECT_GT(repeated.received_steady, first_steady);
  }
  message.transforms.push_back(edge);
  publisher->publish(message);
  EXPECT_TRUE(wait_for_tf([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : received) if (!value.valid) return true;
    return false;
  }));
}
}  // namespace

#include <rosgraph_msgs/msg/clock.hpp>

TEST(StaticTfReceipt, StartupZeroReceiptIsPreservedAcrossClockAdmission)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("use_sim_time", true),
    rclcpp::Parameter("input_topic", "/d2a_zero_tf/static"),
    rclcpp::Parameter("output_topic", "/d2a_zero_tf/receipt"),
    rclcpp::Parameter("base_frame", "base_link"),
    rclcpp::Parameter("camera_frame", "camera"),
  });
  auto bridge = std::make_shared<senior_sensor_ingress::StaticTfReceipt>(options);
  auto source = std::make_shared<rclcpp::Node>("robot_state_publisher");
  auto observer = std::make_shared<rclcpp::Node>("d2a_zero_observer");
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  auto publisher = source->create_publisher<TFMessage>("/d2a_zero_tf/static", qos);
  auto clock = observer->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));
  std::mutex mutex;
  std::vector<senior_motion_msgs::msg::BoundedStaticTf> received;
  auto sub = observer->create_subscription<senior_motion_msgs::msg::BoundedStaticTf>(
    "/d2a_zero_tf/receipt", qos,
    [&](senior_motion_msgs::msg::BoundedStaticTf::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(*message);
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bridge);
  executor.add_node(source);
  executor.add_node(observer);
  std::thread spin([&] {executor.spin();});
  auto cleanup = [&](int *) {
    executor.cancel();
    if (spin.joinable()) spin.join();
    rclcpp::shutdown();
  };
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_for_tf([&] {return publisher->get_subscription_count() && clock->get_subscription_count();}));
  TFMessage message;
  geometry_msgs::msg::TransformStamped edge;
  edge.header.frame_id = "base_link";
  edge.child_frame_id = "camera";
  edge.transform.rotation.w = 1.0;
  message.transforms.push_back(edge);
  publisher->publish(message);
  ASSERT_TRUE(wait_for_tf([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) if (item.valid && !item.admitted) return true;
    return false;
  }));
  rosgraph_msgs::msg::Clock tick;
  tick.clock.sec = 1;
  clock->publish(tick);
  ASSERT_TRUE(wait_for_tf([&] {return bridge->get_clock()->now().nanoseconds() == 1000000000LL;}));
  std::this_thread::sleep_for(80ms);  // allow the 20 ms monitor to observe clock 1
  tick.clock.sec = 2;
  clock->publish(tick);
  ASSERT_TRUE(wait_for_tf([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) if (item.valid && item.admitted) return true;
    return false;
  }));
  std::lock_guard<std::mutex> lock(mutex);
  const auto & admitted = received.back();
  EXPECT_EQ(admitted.received_ros.sec, 0);
  EXPECT_EQ(admitted.received_ros.nanosec, 0u);
  EXPECT_GT(admitted.received_steady, 0.0);
  EXPECT_GT(admitted.admitted_from_ros.sec, 0);
  EXPECT_GT(admitted.admitted_from_steady, admitted.received_steady);
  EXPECT_EQ(admitted.edges[0].stamp.sec, 0);
}

TEST(StaticTfReceipt, AdditionalPublisherRevokesCurrentBinding)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("input_topic", "/d2a_conflict_tf/static"),
    rclcpp::Parameter("output_topic", "/d2a_conflict_tf/receipt"),
    rclcpp::Parameter("base_frame", "base_link"),
    rclcpp::Parameter("camera_frame", "camera"),
  });
  auto bridge = std::make_shared<senior_sensor_ingress::StaticTfReceipt>(options);
  auto source = std::make_shared<rclcpp::Node>("robot_state_publisher");
  auto observer = std::make_shared<rclcpp::Node>("d2a_conflict_observer");
  auto intruder = std::make_shared<rclcpp::Node>("another_tf_writer");
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  auto publisher = source->create_publisher<TFMessage>("/d2a_conflict_tf/static", qos);
  std::mutex mutex;
  std::vector<senior_motion_msgs::msg::BoundedStaticTf> received;
  auto sub = observer->create_subscription<senior_motion_msgs::msg::BoundedStaticTf>(
    "/d2a_conflict_tf/receipt", qos,
    [&](senior_motion_msgs::msg::BoundedStaticTf::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(*message);
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bridge);
  executor.add_node(source);
  executor.add_node(observer);
  executor.add_node(intruder);
  std::thread spin([&] {executor.spin();});
  auto cleanup = [&](int *) {
    executor.cancel();
    if (spin.joinable()) spin.join();
    rclcpp::shutdown();
  };
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_for_tf([&] {return publisher->get_subscription_count() > 0;}));
  TFMessage message;
  geometry_msgs::msg::TransformStamped edge;
  edge.header.frame_id = "base_link";
  edge.child_frame_id = "camera";
  edge.transform.rotation.w = 1.0;
  message.transforms.push_back(edge);
  publisher->publish(message);
  ASSERT_TRUE(wait_for_tf([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) if (item.valid && item.admitted) return true;
    return false;
  }));
  auto conflicting = intruder->create_publisher<TFMessage>("/d2a_conflict_tf/static", qos);
  ASSERT_TRUE(wait_for_tf([&] {
    return bridge->get_publishers_info_by_topic("/d2a_conflict_tf/static").size() == 2;
  }));
  EXPECT_TRUE(wait_for_tf([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) if (!item.valid && item.reason == "source_lost") return true;
    return false;
  }));
}

#include <condition_variable>

TEST(StaticTfReceipt, QueuedRetiredPublisherCannotRevokeSuccessor)
{
  rclcpp::init(0, nullptr);
  std::atomic<bool> armed{false};
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false, resume = false;
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("input_topic", "/d2a_old_tf/static"),
    rclcpp::Parameter("output_topic", "/d2a_old_tf/receipt"),
    rclcpp::Parameter("base_frame", "base_link"),
    rclcpp::Parameter("camera_frame", "camera"),
  });
  auto bridge = std::make_shared<senior_sensor_ingress::StaticTfReceipt>(
    options, [&](const rmw_gid_t &) {
      if (!armed.exchange(false)) return;
      std::unique_lock<std::mutex> lock(mutex);
      entered = true;
      condition.notify_all();
      condition.wait_for(lock, 5s, [&] {return resume;});
    }, true);
  auto source = std::make_shared<rclcpp::Node>("robot_state_publisher");
  auto observer = std::make_shared<rclcpp::Node>("d2a_old_observer");
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  auto publisher = source->create_publisher<TFMessage>("/d2a_old_tf/static", qos);
  std::vector<senior_motion_msgs::msg::BoundedStaticTf> received;
  auto sub = observer->create_subscription<senior_motion_msgs::msg::BoundedStaticTf>(
    "/d2a_old_tf/receipt", qos,
    [&](senior_motion_msgs::msg::BoundedStaticTf::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(*message);
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bridge);
  executor.add_node(source);
  executor.add_node(observer);
  std::thread spin([&] {executor.spin();});
  auto cleanup = [&](int *) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      resume = true;
      condition.notify_all();
    }
    executor.cancel();
    if (spin.joinable()) spin.join();
    rclcpp::shutdown();
  };
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_for_tf([&] {return publisher->get_subscription_count() > 0;}));
  TFMessage message;
  geometry_msgs::msg::TransformStamped edge;
  edge.header.frame_id = "base_link";
  edge.child_frame_id = "camera";
  edge.transform.rotation.w = 1.0;
  message.transforms.push_back(edge);
  publisher->publish(message);
  ASSERT_TRUE(wait_for_tf([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) if (item.valid && item.admitted) return true;
    return false;
  }));
  std::string old_gid;
  {
    std::lock_guard<std::mutex> lock(mutex);
    old_gid = received.back().raw_gid;
  }
  armed = true;
  publisher->publish(message);
  ASSERT_TRUE(wait_for_tf([&] {std::lock_guard<std::mutex> lock(mutex); return entered;}));
  publisher.reset();
  auto successor = source->create_publisher<TFMessage>("/d2a_old_tf/static", qos);
  ASSERT_TRUE(wait_for_tf([&] {
    return bridge->get_publishers_info_by_topic("/d2a_old_tf/static").size() == 1 &&
      successor->get_subscription_count() > 0;
  }));
  successor->publish(message);
  ASSERT_TRUE(wait_for_tf([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received)
      if (item.valid && item.admitted && item.raw_gid != old_gid) return true;
    return false;
  }));
  size_t after_successor = 0;
  {
    std::lock_guard<std::mutex> lock(mutex);
    after_successor = received.size();
    resume = true;
    condition.notify_all();
  }
  std::this_thread::sleep_for(150ms);
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (size_t i = after_successor; i < received.size(); ++i)
      EXPECT_TRUE(received[i].valid);
  }
}

TEST(StaticTfReceipt, NeverAdmittedQueuedOldPublisherCannotRevokeSuccessor)
{
  rclcpp::init(0, nullptr);
  std::atomic<bool> armed{true};
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false, resume = false;
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("input_topic", "/d2a_unseen_old_tf/static"),
    rclcpp::Parameter("output_topic", "/d2a_unseen_old_tf/receipt"),
    rclcpp::Parameter("base_frame", "base_link"),
    rclcpp::Parameter("camera_frame", "camera"),
  });
  auto bridge = std::make_shared<senior_sensor_ingress::StaticTfReceipt>(
    options, [&](const rmw_gid_t &) {
      if (!armed.exchange(false)) return;
      std::unique_lock<std::mutex> lock(mutex);
      entered = true;
      condition.notify_all();
      condition.wait_for(lock, 5s, [&] {return resume;});
    }, true);
  auto source = std::make_shared<rclcpp::Node>("robot_state_publisher");
  auto observer = std::make_shared<rclcpp::Node>("d2a_unseen_old_observer");
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  auto publisher = source->create_publisher<TFMessage>("/d2a_unseen_old_tf/static", qos);
  std::vector<senior_motion_msgs::msg::BoundedStaticTf> received;
  auto sub = observer->create_subscription<senior_motion_msgs::msg::BoundedStaticTf>(
    "/d2a_unseen_old_tf/receipt", qos,
    [&](senior_motion_msgs::msg::BoundedStaticTf::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(*message);
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bridge); executor.add_node(source); executor.add_node(observer);
  std::thread spin([&] {executor.spin();});
  auto cleanup = [&](int *) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      resume = true;
      condition.notify_all();
    }
    executor.cancel(); if (spin.joinable()) spin.join(); rclcpp::shutdown();
  };
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_for_tf([&] {return publisher->get_subscription_count() > 0;}));
  TFMessage message;
  geometry_msgs::msg::TransformStamped edge;
  edge.header.frame_id = "base_link"; edge.child_frame_id = "camera";
  edge.transform.rotation.w = 1.0;
  message.transforms.push_back(edge);
  publisher->publish(message);
  ASSERT_TRUE(wait_for_tf([&] {std::lock_guard<std::mutex> lock(mutex); return entered;}));
  publisher.reset();
  auto successor = source->create_publisher<TFMessage>("/d2a_unseen_old_tf/static", qos);
  ASSERT_TRUE(wait_for_tf([&] {
    return bridge->get_publishers_info_by_topic("/d2a_unseen_old_tf/static").size() == 1 &&
      successor->get_subscription_count() > 0;
  }));
  successor->publish(message);
  ASSERT_TRUE(wait_for_tf([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) if (item.valid && item.admitted) return true;
    return false;
  }));
  size_t after_successor;
  {
    std::lock_guard<std::mutex> lock(mutex);
    after_successor = received.size();
    resume = true;
    condition.notify_all();
  }
  std::this_thread::sleep_for(150ms);
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (size_t i = after_successor; i < received.size(); ++i) EXPECT_TRUE(received[i].valid);
  }
}

TEST(StaticTfReceipt, DelayedInspectorSampleDoesNotFakeClockRollback)
{
  rclcpp::init(0, nullptr);
  std::atomic<bool> armed{true};
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false, resume = false;
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("input_topic", "/d2a_inspect_race/static"),
    rclcpp::Parameter("output_topic", "/d2a_inspect_race/receipt"),
    rclcpp::Parameter("base_frame", "base_link"),
    rclcpp::Parameter("camera_frame", "camera"),
  });
  auto bridge = std::make_shared<senior_sensor_ingress::StaticTfReceipt>(
    options, senior_sensor_ingress::StaticTfReceipt::BeforeVerify{}, true, [&] {
      if (!armed.exchange(false)) return;
      std::unique_lock<std::mutex> lock(mutex);
      entered = true;
      condition.notify_all();
      condition.wait_for(lock, 5s, [&] {return resume;});
    });
  auto source = std::make_shared<rclcpp::Node>("robot_state_publisher");
  auto observer = std::make_shared<rclcpp::Node>("d2a_inspect_race_observer");
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  auto publisher = source->create_publisher<TFMessage>("/d2a_inspect_race/static", qos);
  std::vector<senior_motion_msgs::msg::BoundedStaticTf> received;
  auto sub = observer->create_subscription<senior_motion_msgs::msg::BoundedStaticTf>(
    "/d2a_inspect_race/receipt", qos,
    [&](senior_motion_msgs::msg::BoundedStaticTf::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); received.push_back(*value);
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bridge); executor.add_node(source); executor.add_node(observer);
  std::thread spin([&] {executor.spin();});
  auto cleanup = [&](int *) {
    {std::lock_guard<std::mutex> lock(mutex); resume = true; condition.notify_all();}
    executor.cancel(); if (spin.joinable()) spin.join(); rclcpp::shutdown();
  };
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_for_tf([&] {std::lock_guard<std::mutex> lock(mutex); return entered;}));
  ASSERT_TRUE(wait_for_tf([&] {return publisher->get_subscription_count() > 0;}));
  TFMessage message;
  geometry_msgs::msg::TransformStamped edge;
  edge.header.frame_id = "base_link"; edge.child_frame_id = "camera";
  edge.transform.rotation.w = 1.0;
  message.transforms.push_back(edge);
  publisher->publish(message);
  ASSERT_TRUE(wait_for_tf([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : received) if (value.valid && value.admitted) return true;
    return false;
  }));
  size_t after_valid;
  {std::lock_guard<std::mutex> lock(mutex); after_valid = received.size(); resume = true; condition.notify_all();}
  std::this_thread::sleep_for(100ms);
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (size_t i = after_valid; i < received.size(); ++i)
      EXPECT_NE(received[i].reason, "clock_invalid");
  }
}
