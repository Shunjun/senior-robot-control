// Offline stdin replay only: no ROS node, publishers, truth or device access.
#include "senior_scan_matcher/scan_matcher.hpp"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
using namespace senior_scan_matcher;
namespace {
void number(std::ostream & output, double value)
{
  if (std::isfinite(value)) output << value;
  else output << "null";
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
    if (!(std::cin >> limits.max_interval >> limits.range_limit >> limits.min_response >>
        limits.max_residual >> limits.min_inlier_fraction >> limits.min_spread >>
        limits.min_translation_observability >> limits.min_yaw_observability >>
        limits.max_variance >> limits.reanchor_distance >> limits.reanchor_yaw >> limits.min_rays))
      throw std::runtime_error("invalid limits header");
    LocalMatcher matcher(limits);
    while (std::cin >> command) {
      if (command == "R") {
        matcher.reset();
        continue;
      }
      if (command != "S") throw std::runtime_error("unknown record");
      LaserScan scan;
      size_t count;
      if (!(std::cin >> scan.stamp >> scan.angle_min >> scan.angle_increment >>
          scan.range_min >> scan.range_max >> count) || count > 10000)
        throw std::runtime_error("invalid scan header");
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
      output << "{\"stamp\":" << scan.stamp << ",\"start\":" << result.start
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
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
