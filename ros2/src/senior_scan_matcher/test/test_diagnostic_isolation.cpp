#include <new>
#include <type_traits>
#include <gtest/gtest.h>
#include "senior_scan_matcher/diagnostic_isolation.hpp"
using namespace senior_scan_matcher;
static_assert(std::is_nothrow_move_assignable<MatchDiagnostics>::value,
  "Diagnostic handoff must not introduce a throwing copy in on_scan");
TEST(DiagnosticIsolation, PublisherFailuresCannotInvalidateOrMutateMotion)
{
  senior_motion_msgs::msg::ScanMotion motion;
  motion.valid = true;
  motion.reason = "qualified";
  motion.source_epoch = "current-epoch";
  motion.body_delta.x = .012;
  EXPECT_FALSE(emit_scan_diagnostics(motion, {}, [](const auto &) { throw std::bad_alloc(); }));
  EXPECT_TRUE(motion.valid);
  EXPECT_EQ(motion.reason, "qualified");
  EXPECT_EQ(motion.source_epoch, "current-epoch");
  EXPECT_EQ(motion.body_delta.x, .012);
  EXPECT_FALSE(emit_scan_diagnostics(motion, {}, [](const auto &) { throw 7; }));
}
TEST(DiagnosticIsolation, NormalAndInvalidMotionRetainOriginalDiagnosticIdentity)
{
  senior_motion_msgs::msg::ScanMotion motion;
  motion.source_epoch = "new-epoch";
  motion.reason = "initializing";
  int publications = 0;
  EXPECT_TRUE(emit_scan_diagnostics(motion, {}, [&](const auto & message) {
    ++publications;
    EXPECT_EQ(message.status.front().hardware_id, "new-epoch");
    EXPECT_EQ(message.status.front().message, "initializing");
  }));
  EXPECT_EQ(publications, 1);
}
