#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "senior_sensor_ingress/scan_ledger.hpp"

using senior_interfaces::msg::BoundedRawLaserScan;
using senior_sensor_ingress::ScanLedger;

TEST(ScanLedger, RawSafetyAndFilteredBeamsRemainBoundToIssuedFrame)
{
  ScanLedger ledger(2, 1.0);
  BoundedRawLaserScan original;
  original.capture.bridge_boot_id = "scan-boot";
  original.capture.capture_generation = 1;
  original.capture.source_epoch = "scan-source";
  original.capture.sequence = 1;
  original.capture.received_steady_ns = 10000000000ULL;
  original.capture.sample_steady_ns = 10000000000ULL;
  original.capture.clock_generation = 1;
  original.capture.valid = true;
  original.ranges = {1.0f, std::numeric_limits<float>::quiet_NaN(), 3.0f};
  ASSERT_TRUE(ledger.observe(original, 10.0));
  auto filtered = original.ranges;
  EXPECT_TRUE(ledger.same_ranges(original.capture, original.ranges, 10.1));
  filtered[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ledger.filtered_ranges(original.capture, filtered, 10.1));
  filtered[2] = 8.0f;
  EXPECT_FALSE(ledger.filtered_ranges(original.capture, filtered, 10.1));
  EXPECT_FALSE(ledger.same_ranges(original.capture, filtered, 10.1));
  auto forged = original.capture;
  forged.received_steady_ns += 10000000ULL;
  EXPECT_FALSE(ledger.same_ranges(forged, original.ranges, 10.1));
  EXPECT_EQ(ledger.match(original.capture, 14.0), ScanLedger::Match::exact);
  ASSERT_TRUE(ledger.observe(original, 14.0));  // Retention is not safety qualification.
}

TEST(ScanLedger, OldGenerationAndCapacityDoNotRevokeSuccessor)
{
  ScanLedger ledger(2, 1.0);
  BoundedRawLaserScan scan;
  scan.capture.bridge_boot_id = "scan-boot";
  scan.capture.capture_generation = 1;
  scan.capture.source_epoch = "scan-source";
  scan.capture.received_steady_ns = 10000000000ULL;
  scan.capture.sample_steady_ns = 10000000000ULL;
  scan.capture.clock_generation = 1;
  scan.capture.valid = true;
  scan.ranges = {1.0f};
  scan.capture.sequence = 1;
  ASSERT_TRUE(ledger.observe(scan, 10.0));
  const auto first = scan.capture;
  scan.capture.sequence = 2;
  scan.capture.stamp.nanosec = 2;
  scan.capture.received_ros.nanosec = 2;
  scan.capture.received_steady_ns = 10000000000ULL;
  scan.capture.sample_steady_ns = scan.capture.received_steady_ns;
  ASSERT_TRUE(ledger.observe(scan, 10.0));
  scan.capture.sequence = 3;
  scan.capture.stamp.nanosec = 3;
  scan.capture.received_ros.nanosec = 3;
  scan.capture.received_steady_ns = 10000000000ULL;
  scan.capture.sample_steady_ns = scan.capture.received_steady_ns;
  ASSERT_TRUE(ledger.observe(scan, 10.0));
  EXPECT_EQ(ledger.match(first, 10.1), ScanLedger::Match::missing);
  const auto retired = scan.capture;
  scan.capture.capture_generation = 2;
  scan.capture.sequence = 1;
  ASSERT_TRUE(ledger.observe(scan, 10.1));
  EXPECT_EQ(ledger.match(retired, 10.1), ScanLedger::Match::old_generation);
  EXPECT_EQ(ledger.match(scan.capture, 10.1), ScanLedger::Match::exact);
}
