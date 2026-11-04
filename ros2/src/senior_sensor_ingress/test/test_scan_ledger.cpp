#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "senior_sensor_ingress/scan_ledger.hpp"

using senior_motion_msgs::msg::BoundedRawLaserScan;
using senior_sensor_ingress::ScanLedger;

TEST(ScanLedger, RawSafetyAndFilteredBeamsRemainBoundToIssuedFrame)
{
  ScanLedger ledger(2, 1.0);
  BoundedRawLaserScan original;
  original.capture.bridge_boot_id = "scan-boot";
  original.capture.capture_generation = 1;
  original.capture.source_epoch = "scan-source";
  original.capture.sequence = 1;
  original.capture.received_steady = 10.0;
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
  forged.received_steady += 0.01;
  EXPECT_FALSE(ledger.same_ranges(forged, original.ranges, 10.1));
  EXPECT_EQ(ledger.match(original.capture, 11.01), ScanLedger::Match::retired);
}

TEST(ScanLedger, RetiredGenerationAndCapacityDoNotRevokeSuccessor)
{
  ScanLedger ledger(2, 1.0);
  BoundedRawLaserScan scan;
  scan.capture.bridge_boot_id = "scan-boot";
  scan.capture.capture_generation = 1;
  scan.capture.source_epoch = "scan-source";
  scan.capture.received_steady = 10.0;
  scan.capture.valid = true;
  scan.ranges = {1.0f};
  scan.capture.sequence = 1;
  ASSERT_TRUE(ledger.observe(scan, 10.0));
  const auto first = scan.capture;
  scan.capture.sequence = 2;
  ASSERT_TRUE(ledger.observe(scan, 10.0));
  scan.capture.sequence = 3;
  ASSERT_TRUE(ledger.observe(scan, 10.0));
  EXPECT_EQ(ledger.match(first, 10.1), ScanLedger::Match::retired);
  const auto retired = scan.capture;
  scan.capture.capture_generation = 2;
  scan.capture.sequence = 1;
  ASSERT_TRUE(ledger.observe(scan, 10.1));
  EXPECT_EQ(ledger.match(retired, 10.1), ScanLedger::Match::retired);
  EXPECT_EQ(ledger.match(scan.capture, 10.1), ScanLedger::Match::exact);
}
