#pragma once
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <senior_motion_msgs/msg/scan_motion.hpp>
#include "senior_scan_matcher/scan_matcher.hpp"
namespace senior_scan_matcher {
// Independent observation side effect: never changes the motion result or owner.
template<typename Publish>
bool emit_scan_diagnostics(const senior_motion_msgs::msg::ScanMotion & output,
                          const MatchDiagnostics & diagnostics, Publish publish) noexcept
{
  try {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header = output.header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "scan_matcher";
    status.hardware_id = output.source_epoch;
    status.level = output.valid ? 0 : 1;
    status.message = output.reason;
    for (const auto & item : diagnostic_values(diagnostics)) {
      diagnostic_msgs::msg::KeyValue value;
      value.key = item.first;
      value.value = item.second;
      status.values.push_back(value);
    }
    array.status.push_back(status);
    publish(array);
    return true;
  } catch (...) {
    return false;
  }
}
}
