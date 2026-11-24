// Offline stdin replay only: no ROS node, publishers, truth or device access.
#include "senior_motion_estimation/scan_matcher.hpp"
#include <chrono>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
using namespace senior_motion_estimation::scan_matching;
namespace {
void number(std::ostream & output, double value)
{
  if (std::isfinite(value)) output << value;
  else output << "null";
}
// The input clock token is exact decimal seconds, never a floating epoch.
// Accept nonnegative int64 nanosecond coordinates with at most nine decimals.
int64_t stamp_ns(const std::string & token)
{
  const auto dot = token.find('.');
  const auto seconds = token.substr(0, dot);
  auto fraction = dot == std::string::npos ? std::string() : token.substr(dot + 1);
  const auto digits = [](const std::string & value) {
    return std::all_of(value.begin(), value.end(), [](char c) {return c >= '0' && c <= '9';});
  };
  if (seconds.empty() || seconds.size() > 10 || !digits(seconds) ||
      fraction.size() > 9 || !digits(fraction))
    throw std::runtime_error("invalid decimal ROS stamp");
  fraction.append(9 - fraction.size(), '0');
  const auto whole = std::stoll(seconds), nanos = std::stoll(fraction);
  const auto maximum = std::numeric_limits<int64_t>::max();
  if (whole > (maximum - nanos) / 1000000000LL)
    throw std::runtime_error("ROS stamp exceeds int64 nanoseconds");
  return whole * 1000000000LL + nanos;
}

}
int main()
{
  try {
    // Karto writes lifecycle logs to cout; keep them separate from JSON stdout.
    std::ostream output(std::cout.rdbuf());
    std::cout.rdbuf(std::cerr.rdbuf());
    output << std::setprecision(17);
    std::string command;
    MatcherLimits limits;
    if (!(std::cin >> command) || command != "L") throw std::runtime_error("limits header required");
    if (!(std::cin >> limits.max_interval_ms >> limits.range_limit >> limits.min_response >>
        limits.max_residual >> limits.min_inlier_fraction >> limits.min_spread >>
        limits.min_translation_observability >> limits.min_yaw_observability >>
        limits.max_variance >> limits.reanchor_distance >> limits.reanchor_yaw >> limits.min_rays))
      throw std::runtime_error("invalid limits header");
    LocalMatcher matcher(limits);
    std::optional<int64_t> origin_ns, previous_ns;
    while (std::cin >> command) {
      if (command == "R") {
        matcher.reset();
        origin_ns.reset(); previous_ns.reset();
        continue;
      }
      if (command != "S") throw std::runtime_error("unknown record");
      LaserScan scan;
      size_t count;
      std::string clock_token;
      if (!(std::cin >> clock_token >> scan.angle_min >> scan.angle_increment >>
          scan.range_min >> scan.range_max >> count) || count > 10000)
        throw std::runtime_error("invalid scan header");
      const auto current_ns = stamp_ns(clock_token);
      if (!origin_ns) origin_ns = current_ns;
      scan.stamp = static_cast<double>(current_ns - *origin_ns) / 1e9;
      for (size_t i = 0; i < count; ++i) {
        std::string token;
        if (!(std::cin >> token)) throw std::runtime_error("incomplete scan");
        size_t used = 0;
        double value = std::stod(token, &used);
        if (used != token.size()) throw std::runtime_error("invalid beam");
        scan.ranges.push_back(value);
      }
      const auto begin = std::chrono::steady_clock::now();
      const auto result = matcher.update(scan);
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
      output << "{\"stamp_ns\":" << current_ns << ",\"start_ns\":";
      if (result.valid && previous_ns) output << *previous_ns;
      else output << "null";
      // Numeric seconds remain diagnostic/geometry projections, not identity.
      output << ",\"stamp\":" << static_cast<double>(current_ns) / 1e9
        << ",\"start\":" << (previous_ns ? static_cast<double>(*previous_ns) / 1e9 : 0.0)
        << ",\"valid\":" << (result.valid ? "true" : "false") << ",\"reason\":" << std::quoted(result.reason)
        << ",\"seconds\":" << elapsed << ",\"sensor_delta\":[";
      number(output, result.sensor_delta.x);
      output << ',';
      number(output, result.sensor_delta.y);
      output << ',';
      number(output, result.sensor_delta.yaw);
      output << ']';
#ifdef MATCH_DIAGNOSTICS
      output << ",\"diagnostics\":{";
      bool first = true;
      for (const auto & item : diagnostic_values(result.diagnostics)) {
        if (!first) output << ',';
        first = false;
        output << std::quoted(item.first) << ':' << std::quoted(item.second);
      }
      output << '}';
#endif
      output << "}\n";
      if (result.valid || result.reason == "initializing") previous_ns = current_ns;
      else {origin_ns.reset(); previous_ns.reset();}
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
