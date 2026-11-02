#include <gtest/gtest.h>

#include "senior_sensor_ingress/capture_ledger.hpp"

using senior_motion_msgs::msg::RawCaptureHeader;
using senior_sensor_ingress::CaptureLedger;

TEST(CaptureLedger, MatchesExactIssuedHeaderAndNeverExtendsFirstReceipt)
{
  CaptureLedger ledger(2, 1.0);
  RawCaptureHeader first;
  first.bridge_boot_id = "boot";
  first.capture_generation = 1;
  first.source_epoch = "depth";
  first.sequence = 1;
  first.raw_gid = "0101";
  first.bridge_gid = "0202";
  first.frame_id = "optical";
  first.received_steady = 10.0;
  first.valid = true;
  ASSERT_TRUE(ledger.observe(0, first, 10.0));
  EXPECT_EQ(ledger.match(0, first, 10.5), CaptureLedger::Match::exact);
  auto changed = first;
  changed.received_steady = 10.1;
  EXPECT_EQ(ledger.match(0, changed, 10.5), CaptureLedger::Match::missing);
  EXPECT_EQ(ledger.match(0, first, 11.001), CaptureLedger::Match::retired);
}

TEST(CaptureLedger, OldGenerationIsRetiredAndCapacityIsBounded)
{
  CaptureLedger ledger(2, 1.0);
  RawCaptureHeader value;
  value.bridge_boot_id = "boot";
  value.capture_generation = 1;
  value.source_epoch = "depth";
  value.raw_gid = "0101";
  value.bridge_gid = "0202";
  value.frame_id = "optical";
  value.received_steady = 10.0;
  value.valid = true;
  value.sequence = 1;
  ASSERT_TRUE(ledger.observe(0, value, 10.0));
  auto old = value;
  value.sequence = 2;
  ASSERT_TRUE(ledger.observe(0, value, 10.0));
  value.sequence = 3;
  ASSERT_TRUE(ledger.observe(0, value, 10.0));
  EXPECT_EQ(ledger.match(0, old, 10.1), CaptureLedger::Match::retired);
  auto retired = value;
  value.capture_generation = 2;
  value.sequence = 4;
  ASSERT_TRUE(ledger.observe(0, value, 10.1));
  EXPECT_EQ(ledger.match(0, retired, 10.1), CaptureLedger::Match::retired);
  EXPECT_EQ(ledger.match(0, value, 10.1), CaptureLedger::Match::exact);
}
