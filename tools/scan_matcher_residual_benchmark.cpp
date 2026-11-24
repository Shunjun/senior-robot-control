// Native benchmark of the production residual search, not the Karto matcher.
// Build: clang++ -std=c++17 -O3 -DNDEBUG -Iros2/src/senior_motion_estimation/src \
//   tools/scan_matcher_residual_benchmark.cpp -o /tmp/scan-residual-benchmark
#include "point_index.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using senior_motion_estimation::scan_matching::detail::PointIndex;
using Point = PointIndex::Point;
using Clock = std::chrono::steady_clock;

namespace {
constexpr unsigned frames = 12;
constexpr unsigned repetitions = 7;
volatile double consumed = 0;

struct Measurement {
  double milliseconds;
  double squared;
  unsigned inliers;
};

template<typename Nearest>
Measurement measure(const std::vector<Point> & queries, Nearest nearest)
{
  const auto start = Clock::now();
  double squared = 0;
  unsigned inliers = 0;
  for (const auto & query : queries) {
    const double best = nearest(query.first, query.second);
    if (best <= 0.18) {squared += best * best; ++inliers;}
  }
  consumed = squared;
  return {std::chrono::duration<double, std::milli>(Clock::now() - start).count(),
    squared, inliers};
}

double median(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}

void benchmark(unsigned count)
{
  std::vector<Point> anchor, queries;
  constexpr double pi = 3.14159265358979323846;
  for (unsigned i = 0; i < count; ++i) {
    const double angle = -pi + 2 * pi * i / count;
    const double dx = std::cos(angle), dy = std::sin(angle);
    const double x_hit = (dx < 0 ? -2.3 : 3.1) / dx;
    const double y_hit = (dy < 0 ? -2.0 : 2.7) / dy;
    const double range = std::min(x_hit, y_hit);
    anchor.emplace_back(range * dx, range * dy);
  }
  // Precompute queries so neither measurement includes synthetic data creation.
  // The movement crosses the existing 0.18m inlier cutoff at some endpoints.
  for (unsigned frame = 1; frame <= frames; ++frame) {
    const double yaw = 0.004 * frame, c = std::cos(yaw), s = std::sin(yaw);
    for (const auto & p : anchor)
      queries.emplace_back(0.008 * frame + c * p.first - s * p.second,
        -0.002 * frame + s * p.first + c * p.second);
  }
  const auto brute = [&](double x, double y) {
      double best = std::numeric_limits<double>::infinity();
      for (const auto & p : anchor) best = std::min(best, std::hypot(x - p.first, y - p.second));
      return best;
    };
  PointIndex index;
  index.reset(anchor);
  const auto indexed = [&](double x, double y) {return index.nearest(x, y);};
  measure(queries, brute);
  measure(queries, indexed);
  std::vector<double> before, after, builds, amortized;
  for (unsigned repetition = 0; repetition < repetitions; ++repetition) {
    const auto start = Clock::now();
    index.reset(anchor);
    const double build = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    Measurement old_result{}, new_result{};
    // Alternate timing order to reduce systematic first/second-run bias.
    if (repetition % 2 == 0) {
      old_result = measure(queries, brute);
      new_result = measure(queries, indexed);
    } else {
      new_result = measure(queries, indexed);
      old_result = measure(queries, brute);
    }
    if (old_result.inliers != new_result.inliers || old_result.squared != new_result.squared)
      throw std::runtime_error("residual/inlier mismatch; timing is invalid");
    before.push_back(old_result.milliseconds / frames);
    after.push_back(new_result.milliseconds / frames);
    builds.push_back(build);
    amortized.push_back((build + new_result.milliseconds) / frames);
  }
  std::cout << "{\"points\":" << count << ",\"frames_per_anchor\":" << frames
    << ",\"repetitions\":" << repetitions
    << ",\"baseline_ms_per_frame\":" << median(before)
    << ",\"index_ms_per_frame\":" << median(after)
    << ",\"index_build_ms\":" << median(builds)
    << ",\"index_with_build_ms_per_frame\":" << median(amortized)
    << ",\"speedup_with_build\":" << median(before) / median(amortized)
    << ",\"equivalent\":true}\n";
}
}  // namespace

int main()
{
  try {
    std::cout << std::setprecision(9);
    for (unsigned count : {720u, 1440u, 2048u}) benchmark(count);
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
